namespace ROS2.Net.Control;

/// <summary>
/// A discrete PID controller like <c>control_toolbox::Pid</c>, with integral clamping, conditional
/// integration (anti-windup), optional derivative-on-measurement and a first-order derivative filter.
/// Allocation-free; not thread-safe (one instance per loop).
/// </summary>
public sealed class PidController
{
    private double _integral;
    private double _previousError = double.NaN;
    private double _previousMeasurement = double.NaN;
    private double _derivative;

    public PidController(double p = 0, double i = 0, double d = 0)
    {
        P = p;
        I = i;
        D = d;
    }

    public double P { get; set; }

    public double I { get; set; }

    public double D { get; set; }

    /// <summary>Clamp of the integral term's contribution (same units as the output).</summary>
    public double IntegralMax { get; set; } = double.PositiveInfinity;

    public double IntegralMin { get; set; } = double.NegativeInfinity;

    public double OutputMax { get; set; } = double.PositiveInfinity;

    public double OutputMin { get; set; } = double.NegativeInfinity;

    /// <summary>Differentiate the measurement instead of the error, avoiding derivative kick on setpoint steps.</summary>
    public bool DerivativeOnMeasurement { get; set; }

    /// <summary>Time constant (s) of the derivative low-pass filter; 0 disables filtering.</summary>
    public double DerivativeFilterTime { get; set; }

    /// <summary>Integral term (already multiplied by <see cref="I"/>).</summary>
    public double IntegralTerm => _integral;

    /// <summary>Last output.</summary>
    public double Output { get; private set; }

    public void Reset()
    {
        _integral = 0;
        _previousError = double.NaN;
        _previousMeasurement = double.NaN;
        _derivative = 0;
        Output = 0;
    }

    /// <summary>Computes the command for a setpoint and measurement.</summary>
    public double Compute(double setpoint, double measurement, double dt)
    {
        var error = setpoint - measurement;
        if (dt <= 0 || double.IsNaN(error))
        {
            return Output;
        }

        double rawDerivative;
        if (DerivativeOnMeasurement)
        {
            rawDerivative = double.IsNaN(_previousMeasurement) ? 0 : -(measurement - _previousMeasurement) / dt;
        }
        else
        {
            rawDerivative = double.IsNaN(_previousError) ? 0 : (error - _previousError) / dt;
        }

        _derivative = DerivativeFilterTime > 0
            ? _derivative + ((rawDerivative - _derivative) * (dt / (DerivativeFilterTime + dt)))
            : rawDerivative;

        _previousError = error;
        _previousMeasurement = measurement;

        var proportional = P * error;
        var derivative = D * _derivative;
        var candidateIntegral = Math.Clamp(_integral + (I * error * dt), IntegralMin, IntegralMax);
        var unsaturated = proportional + candidateIntegral + derivative;
        var output = Math.Clamp(unsaturated, OutputMin, OutputMax);

        // Conditional integration: stop integrating while saturated in the direction of the error.
        var saturatedSameSign = output != unsaturated && Math.Sign(error) == Math.Sign(unsaturated);
        if (!saturatedSameSign)
        {
            _integral = candidateIntegral;
        }

        Output = Math.Clamp(proportional + _integral + derivative, OutputMin, OutputMax);
        return Output;
    }

    /// <summary>Computes the command from an error directly.</summary>
    public double ComputeFromError(double error, double dt) => Compute(error, 0, dt);
}
