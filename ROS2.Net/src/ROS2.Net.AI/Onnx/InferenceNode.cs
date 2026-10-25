using Microsoft.Extensions.Logging;

namespace ROS2.Net.AI.Onnx;

/// <summary>
/// Generic model-inference node: subscribes <typeparamref name="TIn"/>, converts each message to a
/// float tensor, runs the model and publishes the converted <typeparamref name="TOut"/>. When the
/// model is slower than the input, older messages are skipped (keep-last 1), so latency stays bounded.
/// </summary>
public sealed class InferenceNode<TIn, TOut> : Node
    where TIn : class, IRosMessage<TIn>, new()
    where TOut : class, IRosMessage<TOut>, new()
{
    private readonly Func<TIn, (float[] Data, int[] Shape)> _preprocess;
    private readonly Func<float[], int[], float[]> _model;
    private readonly Func<float[], TIn, TOut?> _postprocess;
    private readonly Publisher<TOut> _publisher;
    private readonly CancellationTokenSource _cts = new();
    private readonly Task _loop;
    private long _processed;

    /// <param name="context">Context.</param>
    /// <param name="name">Node name.</param>
    /// <param name="inputTopic">Input topic.</param>
    /// <param name="outputTopic">Output topic.</param>
    /// <param name="preprocess">Message → tensor data and shape.</param>
    /// <param name="model">Inference function taking data and shape, e.g. <c>(x, s) => runner.Run(x, s)</c>.</param>
    /// <param name="postprocess">Model output (+ the input message) → message to publish, or null to skip.</param>
    /// <param name="inputQos">QoS of the input subscription (default sensor data, depth 1).</param>
    public InferenceNode(Ros2Context context, string name, string inputTopic, string outputTopic,
        Func<TIn, (float[] Data, int[] Shape)> preprocess, Func<float[], int[], float[]> model, Func<float[], TIn, TOut?> postprocess, QosProfile? inputQos = null)
        : base(context, name)
    {
        _preprocess = preprocess;
        _model = model;
        _postprocess = postprocess;
        _publisher = CreatePublisher<TOut>(outputTopic);
        var sub = CreateSubscription<TIn>(inputTopic, inputQos ?? QosProfile.SensorData.KeepLast(1));
        _loop = Task.Run(() => LoopAsync(sub, _cts.Token));
    }

    /// <summary>Creates a node that runs an <see cref="OnnxModelRunner"/>.</summary>
    public static InferenceNode<TIn, TOut> ForOnnx(Ros2Context context, string name, string inputTopic, string outputTopic, OnnxModelRunner runner,
        Func<TIn, (float[] Data, int[] Shape)> preprocess, Func<float[], TIn, TOut?> postprocess) =>
        new(context, name, inputTopic, outputTopic, preprocess, (x, s) => runner.Run(x, s), postprocess);

    /// <summary>Messages processed so far.</summary>
    public long ProcessedCount => Interlocked.Read(ref _processed);

    /// <summary>Duration of the latest inference.</summary>
    public TimeSpan LastInferenceTime { get; private set; }

    private async Task LoopAsync(Subscription<TIn> sub, CancellationToken ct)
    {
        try
        {
            await foreach (var msg in sub.ReadAllAsync(ct).ConfigureAwait(false))
            {
                try
                {
                    var start = System.Diagnostics.Stopwatch.GetTimestamp();
                    var (data, shape) = _preprocess(msg);
                    var output = _model(data, shape);
                    LastInferenceTime = System.Diagnostics.Stopwatch.GetElapsedTime(start);
                    if (_postprocess(output, msg) is { } result)
                    {
                        _publisher.Publish(result);
                    }

                    Interlocked.Increment(ref _processed);
                }
                catch (Exception ex) when (ex is not OperationCanceledException)
                {
                    Logger.LogError(ex, "Inference failed");
                }
            }
        }
        catch (OperationCanceledException)
        {
        }
    }

    protected override void Dispose(bool disposing)
    {
        if (disposing)
        {
            _cts.Cancel();
        }

        base.Dispose(disposing);
    }
}
