using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;

namespace ROS2.Net.AI.Onnx;

/// <summary>
/// Runs an ONNX model with ONNX Runtime on float tensors. Thread-safe for concurrent <c>Run</c>
/// calls (ONNX Runtime sessions are). Use for policies exported from PyTorch/TF/RLNet, detectors,
/// or any perception model.
/// </summary>
public sealed class OnnxModelRunner : IDisposable
{
    private readonly InferenceSession _session;

    public OnnxModelRunner(string modelPath, SessionOptions? options = null)
        : this(new InferenceSession(modelPath, options ?? new SessionOptions()))
    {
    }

    public OnnxModelRunner(byte[] model, SessionOptions? options = null)
        : this(new InferenceSession(model, options ?? new SessionOptions()))
    {
    }

    private OnnxModelRunner(InferenceSession session)
    {
        _session = session;
        InputName = session.InputMetadata.Keys.First();
        OutputName = session.OutputMetadata.Keys.First();
        InputShape = session.InputMetadata[InputName].Dimensions;
    }

    public string InputName { get; }

    public string OutputName { get; }

    /// <summary>Declared input dimensions (-1 for dynamic axes).</summary>
    public IReadOnlyList<int> InputShape { get; }

    public IReadOnlyList<string> InputNames => _session.InputMetadata.Keys.ToList();

    public IReadOnlyList<string> OutputNames => _session.OutputMetadata.Keys.ToList();

    /// <summary>
    /// Runs the model on one input and returns the first output flattened. When
    /// <paramref name="shape"/> is omitted, dynamic axes of the declared shape become 1 except the
    /// last, which absorbs the data length.
    /// </summary>
    public float[] Run(ReadOnlySpan<float> input, ReadOnlySpan<int> shape = default)
    {
        var dims = shape.IsEmpty ? ResolveShape(input.Length) : shape.ToArray();
        var tensor = new DenseTensor<float>(input.ToArray(), dims);
        using var results = _session.Run([NamedOnnxValue.CreateFromTensor(InputName, tensor)]);
        var output = results.First(r => r.Name == OutputName);
        return output.AsEnumerable<float>().ToArray();
    }

    /// <summary>Runs the model with several named inputs and returns every float output.</summary>
    public IReadOnlyDictionary<string, float[]> Run(IReadOnlyDictionary<string, (float[] Data, int[] Shape)> inputs)
    {
        var values = inputs.Select(kv => NamedOnnxValue.CreateFromTensor(kv.Key, new DenseTensor<float>(kv.Value.Data, kv.Value.Shape))).ToList();
        using var results = _session.Run(values);
        return results.ToDictionary(r => r.Name, r => r.AsEnumerable<float>().ToArray());
    }

    private int[] ResolveShape(int length)
    {
        var dims = InputShape.ToArray();
        if (dims.Length == 0)
        {
            return [length];
        }

        var known = 1;
        var dynamicIndex = -1;
        for (var i = 0; i < dims.Length; i++)
        {
            if (dims[i] <= 0)
            {
                dims[i] = 1;
                dynamicIndex = i;
            }

            known *= dims[i];
        }

        if (known != length && dynamicIndex >= 0)
        {
            dims[dynamicIndex] = length / (known / dims[dynamicIndex]);
        }

        return dims;
    }

    public void Dispose() => _session.Dispose();
}

/// <summary>A discrete or continuous policy backed by an ONNX model (observation in, logits/Q-values or actions out).</summary>
public sealed class OnnxPolicy(OnnxModelRunner runner) : IDisposable
{
    public OnnxModelRunner Runner { get; } = runner;

    public static OnnxPolicy Load(string modelPath) => new(new OnnxModelRunner(modelPath));

    /// <summary>Arg-max of the model output (Q-values or logits).</summary>
    public int SelectAction(ReadOnlySpan<float> observation)
    {
        var output = Runner.Run(observation);
        var best = 0;
        for (var i = 1; i < output.Length; i++)
        {
            if (output[i] > output[best])
            {
                best = i;
            }
        }

        return best;
    }

    /// <summary>Raw continuous action vector.</summary>
    public float[] Act(ReadOnlySpan<float> observation) => Runner.Run(observation);

    public void Dispose() => Runner.Dispose();
}
