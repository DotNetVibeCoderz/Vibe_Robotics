using Microsoft.ML.OnnxRuntime;
using Microsoft.ML.OnnxRuntime.Tensors;
using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Perception.Images;
using ROS2.Net.Perception.Vision;

namespace ROS2.Net.Perception.Onnx;

/// <summary>Options for <see cref="YoloDetector"/>.</summary>
public sealed record YoloOptions
{
    /// <summary>Square model input size (pixels).</summary>
    public int InputSize { get; init; } = 640;

    public float ScoreThreshold { get; init; } = 0.25f;

    public float IouThreshold { get; init; } = 0.45f;

    /// <summary>Output layout: <c>v8</c> ([1, 4+C, N], also YOLO11) or <c>v5</c> ([1, N, 5+C]). Auto-detected when null.</summary>
    public string? Version { get; init; }

    /// <summary>Configures the ONNX Runtime session (execution providers, threads).</summary>
    public Action<SessionOptions>? ConfigureSession { get; init; }
}

/// <summary>
/// YOLO object detector on ONNX Runtime: letterboxes the image to the model input, runs the model,
/// decodes boxes and applies NMS. Works with Ultralytics exports (<c>yolo export format=onnx</c>).
/// </summary>
public sealed class YoloDetector : IObjectDetector
{
    private readonly InferenceSession _session;
    private readonly string _inputName;
    private readonly YoloOptions _options;

    public YoloDetector(string modelPath, IReadOnlyList<string> labels, YoloOptions? options = null)
    {
        _options = options ?? new YoloOptions();
        using var sessionOptions = new SessionOptions();
        _options.ConfigureSession?.Invoke(sessionOptions);
        _session = new InferenceSession(modelPath, sessionOptions);
        _inputName = _session.InputMetadata.Keys.First();
        Labels = labels;
    }

    public IReadOnlyList<string> Labels { get; }

    public ValueTask<IReadOnlyList<Detection>> DetectAsync(Image image, CancellationToken cancellationToken = default)
    {
        cancellationToken.ThrowIfCancellationRequested();
        var size = _options.InputSize;
        var input = Preprocess(image, size, out var letterbox);
        using var results = _session.Run([NamedOnnxValue.CreateFromTensor(_inputName, input)]);
        var output = results.First().AsTensor<float>();
        var dims = output.Dimensions.ToArray();
        var data = output.ToArray();

        List<Detection> raw;
        var v5 = _options.Version == "v5" || (_options.Version is null && dims.Length == 3 && dims[2] == Labels.Count + 5);
        if (v5)
        {
            raw = DetectionPostprocessing.DecodeYoloV5(data, Labels.Count, dims[1], _options.ScoreThreshold, Labels);
        }
        else
        {
            var transposed = dims.Length == 3 && dims[2] == Labels.Count + 4 && dims[1] != Labels.Count + 4;
            var boxes = transposed ? dims[1] : dims[2];
            raw = DetectionPostprocessing.DecodeYoloV8(data, Labels.Count, boxes, _options.ScoreThreshold, Labels, transposed);
        }

        var kept = DetectionPostprocessing.NonMaxSuppression(raw, _options.IouThreshold)
            .Select(d => d with { Box = letterbox.ToImage(d.Box) })
            .ToList();
        return ValueTask.FromResult<IReadOnlyList<Detection>>(kept);
    }

    /// <summary>Letterboxes an image into a normalized RGB CHW tensor [1, 3, size, size] (gray 114 padding).</summary>
    public static DenseTensor<float> Preprocess(Image image, int size, out Letterbox letterbox)
    {
        var rgb = image.Encoding == ImageEncodings.Rgb8 ? image : ImageOps.ConvertTo(image, ImageEncodings.Rgb8);
        letterbox = Letterbox.Compute((int)rgb.Width, (int)rgb.Height, size);
        var w = Math.Max(1, (int)Math.Round(rgb.Width * letterbox.Scale));
        var h = Math.Max(1, (int)Math.Round(rgb.Height * letterbox.Scale));
        var resized = ImageOps.Resize(rgb, w, h);
        var tensor = new DenseTensor<float>([1, 3, size, size]);
        tensor.Buffer.Span.Fill(114f / 255f);
        var px = (int)letterbox.PadX;
        var py = (int)letterbox.PadY;
        var data = resized.Data;
        var step = (int)resized.Step;
        for (var y = 0; y < h && y + py < size; y++)
        {
            for (var x = 0; x < w && x + px < size; x++)
            {
                var o = (y * step) + (x * 3);
                tensor[0, 0, y + py, x + px] = data[o] / 255f;
                tensor[0, 1, y + py, x + px] = data[o + 1] / 255f;
                tensor[0, 2, y + py, x + px] = data[o + 2] / 255f;
            }
        }

        return tensor;
    }

    public void Dispose() => _session.Dispose();
}
