using ROS2.Net.Rmw.InProcess;

namespace ROS2.Net.Tests.Core;

/// <summary>Each test gets its own in-process domain so parallel tests never see each other's endpoints.</summary>
internal static class TestDomains
{
    private static int s_next = 100;

    public static int Next() => Interlocked.Increment(ref s_next) % 230;

    public static Ros2Context Create(bool serialize = false, IEnumerable<string>? args = null, int? domain = null)
    {
        var options = new Ros2ContextOptions
        {
            DomainId = domain ?? Next(),
            Rmw = new InProcessRmwFactory(),
            Arguments = args,
        };
        if (serialize)
        {
            options.RmwSettings["serialize"] = "true";
        }

        return Ros2Context.Create(options);
    }

    public static async Task<T> WithTimeout<T>(this Task<T> task, int ms = 5000) => await task.WaitAsync(TimeSpan.FromMilliseconds(ms));

    public static async Task WithTimeout(this Task task, int ms = 5000) => await task.WaitAsync(TimeSpan.FromMilliseconds(ms));
}
