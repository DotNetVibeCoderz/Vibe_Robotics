using RLNet.Agents;
using RLNet.Environments;
using RLNet.Spaces;

namespace ROS2.Net.AI.Gym;

/// <summary>Statistics of one training or evaluation episode.</summary>
public sealed record EpisodeStats(int Episode, float Return, int Steps, bool Terminated);

/// <summary>Creates RLNet agents sized for an environment.</summary>
public static class AgentFactory
{
    /// <summary>A DQN agent (double-Q, dueling) for continuous observations.</summary>
    public static DqnAgent CreateDqn(IDiscreteEnvironment env, int[]? hiddenSizes = null, int totalSteps = 50_000, int? seed = null) =>
        new(env.ObservationSpace, env.ActionSpace, new DqnOptions
        {
            HiddenSizes = hiddenSizes ?? [64, 64],
            LearningRate = 1e-3f,
            Gamma = 0.98f,
            BatchSize = 64,
            LearningStarts = Math.Min(1000, totalSteps / 10),
            TargetUpdateInterval = 500,
            BufferCapacity = Math.Max(10_000, totalSteps),
            Epsilon = Schedule.Linear(1.0f, 0.05f, 0.5f),
            DoubleQ = true,
            Dueling = true,
        }, seed: seed);

    /// <summary>A tabular Q-learning agent; each observation dimension is bucketed into <paramref name="bins"/> bins.</summary>
    public static QTableAgent CreateQTable(IDiscreteEnvironment env, int bins = 5, float learningRate = 0.5f, float gamma = 0.95f, int? seed = null)
    {
        if (env.ObservationSpace is not BoxSpace box)
        {
            throw new ArgumentException("Q-table agents need a box observation space.", nameof(env));
        }

        var encoder = StateDiscretizer.ForBox(box, Enumerable.Repeat(bins, box.FlatSize).ToArray());
        return new QTableAgent(env.ActionSpace, encoder, new QTableOptions
        {
            LearningRate = learningRate,
            Gamma = gamma,
            Epsilon = Schedule.Linear(1.0f, 0.05f, 0.6f),
        }, seed);
    }
}

/// <summary>Minimal training and evaluation loops for RLNet discrete agents.</summary>
public static class RlTrainer
{
    /// <summary>Trains for <paramref name="episodes"/> episodes and returns each episode's return.</summary>
    public static IReadOnlyList<EpisodeStats> Train(IDiscreteEnvironment env, IDiscreteAgent agent, int episodes, Action<EpisodeStats>? onEpisode = null, int? seed = null, CancellationToken cancellationToken = default)
    {
        var stats = new List<EpisodeStats>(episodes);
        var obs = new float[env.ObservationSpace.FlatSize];
        var next = new float[obs.Length];
        for (var e = 0; e < episodes && !cancellationToken.IsCancellationRequested; e++)
        {
            agent.SetProgress((float)e / episodes);
            env.Reset(seed is { } s ? s + e : null);
            env.Observation.CopyTo(obs);
            float total = 0;
            var steps = 0;
            bool terminated;
            while (true)
            {
                var action = agent.SelectAction(obs);
                var result = env.Step(action);
                env.Observation.CopyTo(next);
                agent.Observe(obs, action, result.Reward, next, result.Terminated, result.Truncated);
                total += result.Reward;
                steps++;
                (obs, next) = (next, obs);
                if (result.Done)
                {
                    terminated = result.Terminated;
                    break;
                }
            }

            agent.OnEpisodeEnd();
            var stat = new EpisodeStats(e, total, steps, terminated);
            stats.Add(stat);
            onEpisode?.Invoke(stat);
        }

        return stats;
    }

    /// <summary>Runs a policy without learning and returns each episode's return.</summary>
    public static IReadOnlyList<EpisodeStats> Evaluate(IDiscreteEnvironment env, Func<ReadOnlySpan<float>, int> policy, int episodes, int? seed = null)
    {
        var stats = new List<EpisodeStats>(episodes);
        for (var e = 0; e < episodes; e++)
        {
            env.Reset(seed is { } s ? s + e : null);
            float total = 0;
            var steps = 0;
            while (true)
            {
                var result = env.Step(policy(env.Observation));
                total += result.Reward;
                steps++;
                if (result.Done)
                {
                    stats.Add(new EpisodeStats(e, total, steps, result.Terminated));
                    break;
                }
            }
        }

        return stats;
    }

    /// <summary>Greedy evaluation of an agent.</summary>
    public static IReadOnlyList<EpisodeStats> Evaluate(IDiscreteEnvironment env, IDiscreteAgent agent, int episodes, int? seed = null) =>
        Evaluate(env, obs => agent.SelectAction(obs, deterministic: true), episodes, seed);
}
