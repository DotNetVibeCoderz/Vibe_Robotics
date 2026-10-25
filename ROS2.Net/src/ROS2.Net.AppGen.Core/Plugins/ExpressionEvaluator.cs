using System.Globalization;

namespace ROS2.Net.AppGen.Plugins;

/// <summary>
/// A small, safe arithmetic evaluator (no reflection, no code execution): numbers, <c>+ - * / % ^</c>,
/// parentheses, unary signs, constants <c>pi</c>, <c>e</c>, <c>tau</c> and functions
/// <c>sin cos tan asin acos atan atan2 sinh cosh tanh sqrt cbrt abs log log10 log2 ln exp pow min max
/// round floor ceiling trunc sign deg rad hypot clamp</c>. Trigonometry uses radians.
/// </summary>
public static class ExpressionEvaluator
{
    public static double Evaluate(string expression)
    {
        ArgumentNullException.ThrowIfNull(expression);
        var parser = new Parser(expression);
        var value = parser.ParseExpression();
        parser.SkipSpaces();
        if (!parser.AtEnd)
        {
            throw new FormatException($"Unexpected '{parser.Current}' at position {parser.Position + 1}.");
        }

        return value;
    }

    private sealed class Parser(string text)
    {
        private int _pos;
        private int _depth;

        public int Position => _pos;

        public bool AtEnd => _pos >= text.Length;

        public char Current => text[_pos];

        public void SkipSpaces()
        {
            while (!AtEnd && char.IsWhiteSpace(Current))
            {
                _pos++;
            }
        }

        private bool Accept(char c)
        {
            SkipSpaces();
            if (!AtEnd && Current == c)
            {
                _pos++;
                return true;
            }

            return false;
        }

        public double ParseExpression()
        {
            if (++_depth > 200)
            {
                throw new FormatException("Expression is nested too deeply.");
            }

            var value = ParseTerm();
            while (true)
            {
                if (Accept('+'))
                {
                    value += ParseTerm();
                }
                else if (Accept('-'))
                {
                    value -= ParseTerm();
                }
                else
                {
                    _depth--;
                    return value;
                }
            }
        }

        private double ParseTerm()
        {
            var value = ParsePower();
            while (true)
            {
                if (Accept('*'))
                {
                    value *= ParsePower();
                }
                else if (Accept('/'))
                {
                    var d = ParsePower();
                    value /= d;
                }
                else if (Accept('%'))
                {
                    value %= ParsePower();
                }
                else
                {
                    return value;
                }
            }
        }

        private double ParsePower()
        {
            var b = ParseUnary();
            return Accept('^') ? Math.Pow(b, ParsePowerRhs()) : b;
        }

        // Right-associative: 2^3^2 = 2^(3^2); a unary minus binds looser than ^ on the left (-2^2 = -4).
        private double ParsePowerRhs()
        {
            var b = ParseUnary();
            return Accept('^') ? Math.Pow(b, ParsePowerRhs()) : b;
        }

        private double ParseUnary()
        {
            if (Accept('-'))
            {
                return -ParsePower();
            }

            if (Accept('+'))
            {
                return ParsePower();
            }

            return ParsePrimary();
        }

        private double ParsePrimary()
        {
            SkipSpaces();
            if (AtEnd)
            {
                throw new FormatException("Unexpected end of expression.");
            }

            if (Accept('('))
            {
                var v = ParseExpression();
                if (!Accept(')'))
                {
                    throw new FormatException("Missing ')'.");
                }

                return v;
            }

            if (char.IsDigit(Current) || Current == '.')
            {
                var start = _pos;
                while (!AtEnd && (char.IsDigit(Current) || Current == '.'))
                {
                    _pos++;
                }

                if (!AtEnd && (Current == 'e' || Current == 'E') && _pos + 1 < text.Length &&
                    (char.IsDigit(text[_pos + 1]) || ((text[_pos + 1] == '-' || text[_pos + 1] == '+') && _pos + 2 < text.Length && char.IsDigit(text[_pos + 2]))))
                {
                    _pos += 2;
                    while (!AtEnd && char.IsDigit(Current))
                    {
                        _pos++;
                    }
                }

                var s = text[start.._pos];
                return double.Parse(s, NumberStyles.Float, CultureInfo.InvariantCulture);
            }

            if (char.IsLetter(Current))
            {
                var start = _pos;
                while (!AtEnd && (char.IsLetterOrDigit(Current) || Current == '_'))
                {
                    _pos++;
                }

                var name = text[start.._pos].ToLowerInvariant();
                if (Accept('('))
                {
                    var args = new List<double>();
                    if (!Accept(')'))
                    {
                        do
                        {
                            args.Add(ParseExpression());
                        }
                        while (Accept(','));

                        if (!Accept(')'))
                        {
                            throw new FormatException($"Missing ')' after arguments of {name}.");
                        }
                    }

                    return Call(name, args);
                }

                return name switch
                {
                    "pi" => Math.PI,
                    "e" => Math.E,
                    "tau" => Math.Tau,
                    "inf" or "infinity" => double.PositiveInfinity,
                    _ => throw new FormatException($"Unknown constant '{name}'."),
                };
            }

            throw new FormatException($"Unexpected '{Current}' at position {_pos + 1}.");
        }

        private static double Call(string name, List<double> a)
        {
            double Arg(int i) => i < a.Count ? a[i] : throw new FormatException($"{name} needs {i + 1} argument(s).");
            return name switch
            {
                "sin" => Math.Sin(Arg(0)),
                "cos" => Math.Cos(Arg(0)),
                "tan" => Math.Tan(Arg(0)),
                "asin" => Math.Asin(Arg(0)),
                "acos" => Math.Acos(Arg(0)),
                "atan" => Math.Atan(Arg(0)),
                "atan2" => Math.Atan2(Arg(0), Arg(1)),
                "sinh" => Math.Sinh(Arg(0)),
                "cosh" => Math.Cosh(Arg(0)),
                "tanh" => Math.Tanh(Arg(0)),
                "sqrt" => Math.Sqrt(Arg(0)),
                "cbrt" => Math.Cbrt(Arg(0)),
                "abs" => Math.Abs(Arg(0)),
                "ln" => Math.Log(Arg(0)),
                "log" => a.Count > 1 ? Math.Log(Arg(0), Arg(1)) : Math.Log10(Arg(0)),
                "log10" => Math.Log10(Arg(0)),
                "log2" => Math.Log2(Arg(0)),
                "exp" => Math.Exp(Arg(0)),
                "pow" => Math.Pow(Arg(0), Arg(1)),
                "min" => a.Count > 0 ? a.Min() : throw new FormatException("min needs arguments."),
                "max" => a.Count > 0 ? a.Max() : throw new FormatException("max needs arguments."),
                "round" => Math.Round(Arg(0), a.Count > 1 ? (int)Arg(1) : 0, MidpointRounding.AwayFromZero),
                "floor" => Math.Floor(Arg(0)),
                "ceiling" or "ceil" => Math.Ceiling(Arg(0)),
                "trunc" => Math.Truncate(Arg(0)),
                "sign" => Math.Sign(Arg(0)),
                "deg" => Arg(0) * 180.0 / Math.PI,
                "rad" => Arg(0) * Math.PI / 180.0,
                "hypot" => Math.Sqrt(a.Sum(x => x * x)),
                "clamp" => Math.Clamp(Arg(0), Arg(1), Arg(2)),
                _ => throw new FormatException($"Unknown function '{name}'."),
            };
        }
    }
}
