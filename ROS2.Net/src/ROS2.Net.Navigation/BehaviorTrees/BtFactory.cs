using System.Xml.Linq;

namespace ROS2.Net.Navigation.BehaviorTrees;

/// <summary>Raised for malformed behavior tree XML or unknown node types.</summary>
public sealed class BehaviorTreeException(string message) : Exception(message);

/// <summary>
/// Creates nodes by registration ID and loads BehaviorTree.CPP v3/v4 XML
/// (<c>&lt;root BTCPP_format="4" main_tree_to_execute="..."&gt;&lt;BehaviorTree ID="..."&gt;</c>).
/// Both <c>&lt;Action ID="X"/&gt;</c> and the compact <c>&lt;X/&gt;</c> forms are accepted.
/// </summary>
public sealed class BtFactory
{
    private enum Kind
    {
        Leaf,
        Control,
        Decorator,
    }

    private readonly Dictionary<string, (Kind Kind, Func<BtNodeConfig, BtNode> Create)> _builders = new(StringComparer.Ordinal);

    public BtFactory()
    {
        RegisterControl("Sequence", c => new SequenceNode(c));
        RegisterControl("SequenceWithMemory", c => new SequenceNode(c));
        RegisterControl("ReactiveSequence", c => new ReactiveSequenceNode(c));
        RegisterControl("Fallback", c => new FallbackNode(c));
        RegisterControl("ReactiveFallback", c => new ReactiveFallbackNode(c));
        RegisterControl("Parallel", c => new ParallelNode(c));
        RegisterControl("PipelineSequence", c => new PipelineSequenceNode(c));
        RegisterControl("RoundRobin", c => new RoundRobinNode(c));
        RegisterControl("RecoveryNode", c => new RecoveryNode(c));
        RegisterDecorator("Inverter", c => new InverterNode(c));
        RegisterDecorator("ForceSuccess", c => new ForceSuccessNode(c));
        RegisterDecorator("ForceFailure", c => new ForceFailureNode(c));
        RegisterDecorator("RetryUntilSuccessful", c => new RetryNode(c));
        RegisterDecorator("Retry", c => new RetryNode(c));
        RegisterDecorator("KeepRunningUntilFailure", c => new KeepRunningUntilFailureNode(c));
        RegisterDecorator("RateController", c => new RateControllerNode(c));
        RegisterDecorator("Repeat", c => new RepeatNode(c));
        RegisterAction("AlwaysSuccess", _ => NodeStatus.Success);
        RegisterAction("AlwaysFailure", _ => NodeStatus.Failure);
    }

    /// <summary>Registers a leaf node type.</summary>
    public BtFactory Register(string id, Func<BtNodeConfig, BtNode> create)
    {
        _builders[id] = (Kind.Leaf, create);
        return this;
    }

    public BtFactory RegisterControl(string id, Func<BtNodeConfig, ControlNode> create)
    {
        _builders[id] = (Kind.Control, create);
        return this;
    }

    public BtFactory RegisterDecorator(string id, Func<BtNodeConfig, DecoratorNode> create)
    {
        _builders[id] = (Kind.Decorator, create);
        return this;
    }

    /// <summary>Registers a synchronous action.</summary>
    public BtFactory RegisterAction(string id, Func<BtNode, NodeStatus> tick) => Register(id, c => new ActionNode(c, tick));

    /// <summary>Registers a condition.</summary>
    public BtFactory RegisterCondition(string id, Func<BtNode, bool> condition) => Register(id, c => new ConditionNode(c, condition));

    /// <summary>Registers an asynchronous action (Running until the task completes; halting cancels it).</summary>
    public BtFactory RegisterAsyncAction(string id, Func<BtNode, CancellationToken, Task<NodeStatus>> run) =>
        Register(id, c => new DelegateAsyncActionNode(c, run));

    public bool IsRegistered(string id) => _builders.ContainsKey(id);

    /// <summary>Loads the main tree from XML text.</summary>
    public BehaviorTree CreateTreeFromText(string xml, Blackboard? blackboard = null, string? treeId = null)
    {
        var doc = XDocument.Parse(xml);
        var root = doc.Root ?? throw new BehaviorTreeException("Empty behavior tree XML.");
        if (root.Name.LocalName != "root")
        {
            throw new BehaviorTreeException($"The XML root must be <root>, found <{root.Name.LocalName}>.");
        }

        var trees = root.Elements("BehaviorTree").ToDictionary(e => (string?)e.Attribute("ID") ?? "MainTree", e => e, StringComparer.Ordinal);
        if (trees.Count == 0)
        {
            throw new BehaviorTreeException("No <BehaviorTree> element found.");
        }

        var main = treeId ?? (string?)root.Attribute("main_tree_to_execute") ?? trees.Keys.First();
        if (!trees.TryGetValue(main, out var mainTree))
        {
            throw new BehaviorTreeException($"Tree '{main}' is not defined.");
        }

        blackboard ??= new Blackboard();
        var rootNode = BuildTree(mainTree, trees, blackboard, new HashSet<string>(StringComparer.Ordinal) { main });
        return new BehaviorTree(rootNode, blackboard);
    }

    public BehaviorTree CreateTreeFromFile(string path, Blackboard? blackboard = null) =>
        CreateTreeFromText(File.ReadAllText(path), blackboard);

    private BtNode BuildTree(XElement tree, Dictionary<string, XElement> trees, Blackboard bb, HashSet<string> stack)
    {
        var children = tree.Elements().ToList();
        if (children.Count != 1)
        {
            throw new BehaviorTreeException($"BehaviorTree '{tree.Attribute("ID")?.Value}' must have exactly one root node.");
        }

        return Build(children[0], trees, bb, stack);
    }

    private BtNode Build(XElement element, Dictionary<string, XElement> trees, Blackboard bb, HashSet<string> stack)
    {
        var tag = element.Name.LocalName;
        var id = tag is "Action" or "Condition" or "Control" or "Decorator" or "SubTree" ? (string?)element.Attribute("ID") ?? throw new BehaviorTreeException($"<{tag}> requires an ID attribute.") : tag;
        var ports = element.Attributes()
            .Where(a => a.Name.LocalName is not "ID" and not "name")
            .ToDictionary(a => a.Name.LocalName, a => a.Value, StringComparer.Ordinal);
        var config = new BtNodeConfig((string?)element.Attribute("name") ?? id, ports, bb);

        if (tag == "SubTree")
        {
            if (!trees.TryGetValue(id, out var sub))
            {
                throw new BehaviorTreeException($"SubTree '{id}' is not defined.");
            }

            if (!stack.Add(id))
            {
                throw new BehaviorTreeException($"SubTree '{id}' is recursive.");
            }

            var node = new SubTreeNode(config) { Child = BuildTree(sub, trees, bb, stack) };
            stack.Remove(id);
            return node;
        }

        if (!_builders.TryGetValue(id, out var builder))
        {
            throw new BehaviorTreeException($"Node type '{id}' is not registered.");
        }

        var created = builder.Create(config);
        var childElements = element.Elements().ToList();
        switch (created)
        {
            case ControlNode control:
                if (childElements.Count == 0)
                {
                    throw new BehaviorTreeException($"Control node '{id}' has no children.");
                }

                foreach (var c in childElements)
                {
                    control.Children.Add(Build(c, trees, bb, stack));
                }

                break;
            case DecoratorNode decorator:
                if (childElements.Count != 1)
                {
                    throw new BehaviorTreeException($"Decorator '{id}' must have exactly one child.");
                }

                decorator.Child = Build(childElements[0], trees, bb, stack);
                break;
            default:
                if (childElements.Count > 0)
                {
                    throw new BehaviorTreeException($"Leaf node '{id}' cannot have children.");
                }

                break;
        }

        return created;
    }
}

/// <summary>Behavior trees shipped with ROS2.Net.Navigation.</summary>
public static class NavTrees
{
    /// <summary>
    /// Equivalent of Nav2's <c>navigate_to_pose_w_replanning_and_recovery.xml</c>: replan at 1 Hz while
    /// following the path; on failure cycle through clearing, spinning, waiting and backing up.
    /// </summary>
    public const string NavigateToPoseWithReplanningAndRecovery = """
        <root BTCPP_format="4" main_tree_to_execute="NavigateToPoseWReplanningAndRecovery">
          <BehaviorTree ID="NavigateToPoseWReplanningAndRecovery">
            <RecoveryNode number_of_retries="6" name="NavigateRecovery">
              <PipelineSequence name="NavigateWithReplanning">
                <RateController hz="1.0">
                  <RecoveryNode number_of_retries="1" name="ComputePathToPose">
                    <ComputePathToPose goal="{goal}" path="{path}" planner_id="GridBased"/>
                    <ClearEntireCostmap name="ClearGlobalCostmap-Context" service_name="global_costmap/clear_entirely_global_costmap"/>
                  </RecoveryNode>
                </RateController>
                <RecoveryNode number_of_retries="1" name="FollowPath">
                  <FollowPath path="{path}" controller_id="FollowPath"/>
                  <ClearEntireCostmap name="ClearLocalCostmap-Context" service_name="local_costmap/clear_entirely_local_costmap"/>
                </RecoveryNode>
              </PipelineSequence>
              <ReactiveFallback name="RecoveryFallback">
                <GoalUpdated/>
                <RoundRobin name="RecoveryActions">
                  <Sequence name="ClearingActions">
                    <ClearEntireCostmap name="ClearLocalCostmap-Subtree" service_name="local_costmap/clear_entirely_local_costmap"/>
                    <ClearEntireCostmap name="ClearGlobalCostmap-Subtree" service_name="global_costmap/clear_entirely_global_costmap"/>
                  </Sequence>
                  <Spin spin_dist="1.57"/>
                  <Wait wait_duration="5"/>
                  <BackUp backup_dist="0.30" backup_speed="0.05"/>
                </RoundRobin>
              </ReactiveFallback>
            </RecoveryNode>
          </BehaviorTree>
        </root>
        """;

    /// <summary>Plan once, then follow (Nav2's <c>navigate_w_replanning_only_if_path_becomes_invalid</c> simplified).</summary>
    public const string NavigateToPoseSimple = """
        <root BTCPP_format="4" main_tree_to_execute="NavigateToPoseSimple">
          <BehaviorTree ID="NavigateToPoseSimple">
            <Sequence>
              <ComputePathToPose goal="{goal}" path="{path}"/>
              <FollowPath path="{path}"/>
            </Sequence>
          </BehaviorTree>
        </root>
        """;
}
