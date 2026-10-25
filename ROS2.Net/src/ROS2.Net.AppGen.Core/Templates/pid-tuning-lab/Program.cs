// {{ProjectName}} — PID tuning lab on a first-order + integrator DC motor model (ROS2.Net template).
using ROS2.Net.Control;

var gainSets = new (string Name, double P, double I, double D)[]
{
    ("P only", 2.0, 0.0, 0.0),
    ("PI", 2.0, 1.5, 0.0),
    ("PID", 3.0, 2.0, 0.15),
    ("aggressive", 8.0, 6.0, 0.05),
};

const double Target = 1.0;      // rad/s
const double Dt = 0.001;
const double Tau = 0.25;        // motor time constant [s]
const double Gain = 2.0;        // (rad/s) per volt

Console.WriteLine($"{"gains",-12} {"rise [s]",9} {"overshoot",10} {"settle [s]",11} {"ss error",9}");
foreach (var g in gainSets)
{
    var pid = new PidController(g.P, g.I, g.D) { OutputMax = 12, OutputMin = -12, IntegralMax = 10, IntegralMin = -10, DerivativeOnMeasurement = true };
    double speed = 0, peak = 0, rise = double.NaN, settle = 0;
    for (var t = 0.0; t < 5; t += Dt)
    {
        var volts = pid.Compute(Target, speed, Dt);
        speed += (Gain * volts - speed) / Tau * Dt;    // first-order motor dynamics
        peak = Math.Max(peak, speed);
        if (double.IsNaN(rise) && speed >= 0.9 * Target)
        {
            rise = t;
        }

        if (Math.Abs(speed - Target) > 0.02 * Target)
        {
            settle = t;
        }
    }

    Console.WriteLine($"{g.Name,-12} {rise,9:0.000} {(peak - Target) / Target,10:P1} {settle,11:0.000} {Math.Abs(Target - speed),9:0.0000}");
}

Console.WriteLine("Try your own gains: edit gainSets and re-run.");
