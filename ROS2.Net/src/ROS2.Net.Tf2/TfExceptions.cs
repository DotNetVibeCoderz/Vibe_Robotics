namespace ROS2.Net.Tf2;

/// <summary>Base of every TF lookup failure (<c>tf2::TransformException</c>).</summary>
public class TransformException(string message) : Exception(message);

/// <summary>A frame does not exist in the buffer (<c>tf2::LookupException</c>).</summary>
public sealed class LookupException(string message) : TransformException(message);

/// <summary>The two frames are not connected by any chain of transforms (<c>tf2::ConnectivityException</c>).</summary>
public sealed class ConnectivityException(string message) : TransformException(message);

/// <summary>The requested time is outside the cached data (<c>tf2::ExtrapolationException</c>).</summary>
public sealed class ExtrapolationException(string message) : TransformException(message);

/// <summary>A frame name or argument is invalid (<c>tf2::InvalidArgumentException</c>).</summary>
public sealed class InvalidTransformArgumentException(string message) : TransformException(message);

/// <summary>Waiting for a transform timed out.</summary>
public sealed class TransformTimeoutException(string message) : TransformException(message);
