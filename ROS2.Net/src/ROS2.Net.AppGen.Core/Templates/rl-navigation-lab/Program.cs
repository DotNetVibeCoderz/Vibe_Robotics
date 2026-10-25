// {{ProjectName}} — reinforcement learning on the ROS2.Net simulator (ROS2.Net template).
using ROS2.Net.AI.Gym;
using ROS2.Net.Simulation;

var env = new RobotGymEnv(new RobotGymOptions
{
    Scenario = ScenarioKind.Empty,
    Sectors = 0,                      // goal bearing/distance only: a small tabular problem
    Start = new Pose2D(10, 10, 0),
    Goal = new Vec2(12.5, 11.0),
    MaxEpisodeSteps = 150,
    Seed = 7,
});

var random = new Random(1);
var baseline = RlTrainer.Evaluate(env, _ => random.Next(env.Options.ActionCount), episodes: 20, seed: 1);
Console.WriteLine($"Random policy: success {baseline.Count(e => e.Terminated && e.Return > 0) * 5}%, mean return {baseline.Average(e => e.Return):0.0}");

var agent = AgentFactory.CreateQTable(env, bins: 6, seed: 1);
var history = RlTrainer.Train(env, agent, episodes: 300, onEpisode: s =>
{
    if (s.Episode % 50 == 0)
    {
        Console.WriteLine($"episode {s.Episode,4}: return {s.Return,7:0.0} in {s.Steps,3} steps");
    }
}, seed: 1);

var eval = RlTrainer.Evaluate(env, agent, episodes: 20, seed: 2);
Console.WriteLine($"Trained policy: success {eval.Count(e => e.Terminated && e.Return > 0) * 5}%, mean return {eval.Average(e => e.Return):0.0}, mean steps {eval.Average(e => e.Steps):0}");
Console.WriteLine("Next: deploy the agent with PolicyRunnerNode (obs => agent.SelectAction(obs, true)) on a real or simulated robot.");
