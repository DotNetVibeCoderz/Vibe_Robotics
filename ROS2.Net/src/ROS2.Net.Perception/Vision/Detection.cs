using ROS2.Net.Msgs.SensorMsgs;
using ROS2.Net.Msgs.StdMsgs;
using ROS2.Net.Msgs.VisionMsgs;

namespace ROS2.Net.Perception.Vision;

/// <summary>An axis-aligned box in pixels: top-left corner plus size.</summary>
public readonly record struct BoxF(float X, float Y, float Width, float Height)
{
    public float Right => X + Width;

    public float Bottom => Y + Height;

    public float Area => Math.Max(0, Width) * Math.Max(0, Height);

    public float CenterX => X + (Width / 2);

    public float CenterY => Y + (Height / 2);

    public static BoxF FromCenter(float cx, float cy, float w, float h) => new(cx - (w / 2), cy - (h / 2), w, h);

    /// <summary>Intersection over union.</summary>
    public float IoU(BoxF other)
    {
        var ix = Math.Max(0, Math.Min(Right, other.Right) - Math.Max(X, other.X));
        var iy = Math.Max(0, Math.Min(Bottom, other.Bottom) - Math.Max(Y, other.Y));
        var inter = ix * iy;
        var union = Area + other.Area - inter;
        return union <= 0 ? 0 : inter / union;
    }
}

/// <summary>One detected object.</summary>
public sealed record Detection(int ClassId, string Label, float Score, BoxF Box);

/// <summary>Runs object detection on images. Implementations: <c>ROS2.Net.Perception.Onnx.YoloDetector</c>, or your own.</summary>
public interface IObjectDetector : IDisposable
{
    IReadOnlyList<string> Labels { get; }

    ValueTask<IReadOnlyList<Detection>> DetectAsync(Image image, CancellationToken cancellationToken = default);
}

/// <summary>Letterbox geometry: how an image was scaled and padded into a square model input.</summary>
public readonly record struct Letterbox(float Scale, float PadX, float PadY, int InputSize)
{
    public static Letterbox Compute(int imageWidth, int imageHeight, int inputSize)
    {
        var scale = Math.Min((float)inputSize / imageWidth, (float)inputSize / imageHeight);
        return new Letterbox(scale, (inputSize - (imageWidth * scale)) / 2, (inputSize - (imageHeight * scale)) / 2, inputSize);
    }

    /// <summary>Maps a box from model-input pixels back to original image pixels.</summary>
    public BoxF ToImage(BoxF box) => new((box.X - PadX) / Scale, (box.Y - PadY) / Scale, box.Width / Scale, box.Height / Scale);
}

/// <summary>Post-processing shared by detectors: YOLO output decoding and non-maximum suppression.</summary>
public static class DetectionPostprocessing
{
    /// <summary>Greedy NMS; per class unless <paramref name="classAgnostic"/>.</summary>
    public static List<Detection> NonMaxSuppression(IEnumerable<Detection> detections, float iouThreshold = 0.45f, bool classAgnostic = false, int maxDetections = 300)
    {
        var sorted = detections.OrderByDescending(d => d.Score).ToList();
        var kept = new List<Detection>();
        foreach (var d in sorted)
        {
            if (kept.Count >= maxDetections)
            {
                break;
            }

            if (!kept.Any(k => (classAgnostic || k.ClassId == d.ClassId) && k.Box.IoU(d.Box) > iouThreshold))
            {
                kept.Add(d);
            }
        }

        return kept;
    }

    /// <summary>
    /// Decodes a YOLOv8/v11 head output of shape [1, 4 + classes, boxes] (cx, cy, w, h, class scores),
    /// in model-input pixels. Set <paramref name="transposed"/> for [1, boxes, 4 + classes].
    /// </summary>
    public static List<Detection> DecodeYoloV8(ReadOnlySpan<float> output, int classes, int boxes, float scoreThreshold, IReadOnlyList<string>? labels = null, bool transposed = false)
    {
        var result = new List<Detection>();
        var attrs = 4 + classes;
        float At(ReadOnlySpan<float> o, int a, int b) => transposed ? o[(b * attrs) + a] : o[(a * boxes) + b];
        for (var b = 0; b < boxes; b++)
        {
            var best = -1;
            var score = scoreThreshold;
            for (var c = 0; c < classes; c++)
            {
                var s = At(output, 4 + c, b);
                if (s > score)
                {
                    score = s;
                    best = c;
                }
            }

            if (best < 0)
            {
                continue;
            }

            var box = BoxF.FromCenter(At(output, 0, b), At(output, 1, b), At(output, 2, b), At(output, 3, b));
            result.Add(new Detection(best, labels is not null && best < labels.Count ? labels[best] : best.ToString(System.Globalization.CultureInfo.InvariantCulture), score, box));
        }

        return result;
    }

    /// <summary>Decodes a YOLOv5/v7 output [1, boxes, 5 + classes] (cx, cy, w, h, objectness, class scores).</summary>
    public static List<Detection> DecodeYoloV5(ReadOnlySpan<float> output, int classes, int boxes, float scoreThreshold, IReadOnlyList<string>? labels = null)
    {
        var result = new List<Detection>();
        var attrs = 5 + classes;
        for (var b = 0; b < boxes; b++)
        {
            var row = output.Slice(b * attrs, attrs);
            var obj = row[4];
            if (obj < scoreThreshold)
            {
                continue;
            }

            var best = 0;
            for (var c = 1; c < classes; c++)
            {
                if (row[5 + c] > row[5 + best])
                {
                    best = c;
                }
            }

            var score = obj * row[5 + best];
            if (score >= scoreThreshold)
            {
                result.Add(new Detection(best, labels is not null && best < labels.Count ? labels[best] : best.ToString(System.Globalization.CultureInfo.InvariantCulture), score, BoxF.FromCenter(row[0], row[1], row[2], row[3])));
            }
        }

        return result;
    }

    /// <summary>Converts detections to <c>vision_msgs/Detection2DArray</c>.</summary>
    public static Detection2DArray ToMessage(IEnumerable<Detection> detections, Header header)
    {
        var msg = new Detection2DArray { Header = header.Clone() };
        msg.Detections = detections.Select((d, i) =>
        {
            var det = new Msgs.VisionMsgs.Detection2D { Header = header.Clone(), Id = i.ToString(System.Globalization.CultureInfo.InvariantCulture) };
            det.Bbox.Center.Position.X = d.Box.CenterX;
            det.Bbox.Center.Position.Y = d.Box.CenterY;
            det.Bbox.SizeX = d.Box.Width;
            det.Bbox.SizeY = d.Box.Height;
            det.Results = [new ObjectHypothesisWithPose { Hypothesis = new ObjectHypothesis { ClassId = d.Label, Score = d.Score } }];
            return det;
        }).ToArray();
        return msg;
    }
}
