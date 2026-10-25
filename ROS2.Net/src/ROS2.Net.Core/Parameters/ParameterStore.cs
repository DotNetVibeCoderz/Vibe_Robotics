namespace ROS2.Net;

/// <summary>A parameter change, as delivered to <see cref="Node.ParametersChanged"/>.</summary>
public sealed record ParameterChange(string Name, ParameterValue? OldValue, ParameterValue NewValue)
{
    public bool IsNew => OldValue is null;

    public bool IsDeleted => !NewValue.IsSet;
}

/// <summary>The parameters of one node: declaration, typed access, validation and change notification.</summary>
internal sealed class ParameterStore
{
    private readonly object _gate = new();
    private readonly Dictionary<string, (ParameterDescriptor Descriptor, ParameterValue Value)> _parameters = new(StringComparer.Ordinal);
    private readonly Dictionary<string, ParameterValue> _overrides;
    private readonly List<Func<IReadOnlyList<Parameter>, SetParametersResult>> _validators = [];
    private readonly bool _allowUndeclared;

    public ParameterStore(IReadOnlyDictionary<string, ParameterValue> overrides, bool allowUndeclared)
    {
        _overrides = new Dictionary<string, ParameterValue>(overrides, StringComparer.Ordinal);
        _allowUndeclared = allowUndeclared;
    }

    public event Action<IReadOnlyList<ParameterChange>>? Changed;

    public IReadOnlyDictionary<string, ParameterValue> Overrides => _overrides;

    public ParameterValue Declare(string name, ParameterValue defaultValue, ParameterDescriptor? descriptor, bool ignoreOverride)
    {
        ValidateName(name);
        descriptor = (descriptor ?? new ParameterDescriptor()) with { Name = name };
        if (descriptor.Type == ParameterType.NotSet && defaultValue.IsSet && !descriptor.DynamicTyping)
        {
            descriptor = descriptor with { Type = defaultValue.Type };
        }

        ParameterValue value;
        lock (_gate)
        {
            if (_parameters.ContainsKey(name))
            {
                throw new ParameterAlreadyDeclaredException(name);
            }

            value = !ignoreOverride && _overrides.TryGetValue(name, out var o) ? Coerce(o, descriptor) : defaultValue;
            var check = CheckConstraints(descriptor, value, declaring: true);
            if (!check.Successful)
            {
                throw new InvalidParameterValueException($"Parameter '{name}': {check.Reason}");
            }

            var validation = RunValidators([new Parameter(name, value)]);
            if (!validation.Successful)
            {
                throw new InvalidParameterValueException($"Parameter '{name}' rejected: {validation.Reason}");
            }

            _parameters[name] = (descriptor, value);
        }

        Changed?.Invoke([new ParameterChange(name, null, value)]);
        return value;
    }

    public void Undeclare(string name)
    {
        ParameterValue old;
        lock (_gate)
        {
            if (!_parameters.TryGetValue(name, out var entry))
            {
                throw new ParameterNotDeclaredException(name);
            }

            if (entry.Descriptor.ReadOnly)
            {
                throw new InvalidParameterValueException($"Parameter '{name}' is read-only and cannot be undeclared.");
            }

            old = entry.Value;
            _parameters.Remove(name);
        }

        Changed?.Invoke([new ParameterChange(name, old, ParameterValue.NotSet)]);
    }

    public bool Has(string name)
    {
        lock (_gate)
        {
            return _parameters.ContainsKey(name);
        }
    }

    public bool TryGet(string name, out ParameterValue value)
    {
        lock (_gate)
        {
            if (_parameters.TryGetValue(name, out var entry))
            {
                value = entry.Value;
                return true;
            }
        }

        value = ParameterValue.NotSet;
        return false;
    }

    public ParameterValue Get(string name) =>
        TryGet(name, out var v) ? v : throw new ParameterNotDeclaredException(name);

    public ParameterDescriptor Describe(string name)
    {
        lock (_gate)
        {
            return _parameters.TryGetValue(name, out var e) ? e.Descriptor : throw new ParameterNotDeclaredException(name);
        }
    }

    public IReadOnlyList<string> Names
    {
        get
        {
            lock (_gate)
            {
                return _parameters.Keys.Order(StringComparer.Ordinal).ToList();
            }
        }
    }

    public IReadOnlyList<Parameter> All
    {
        get
        {
            lock (_gate)
            {
                return _parameters.Select(kv => new Parameter(kv.Key, kv.Value.Value)).OrderBy(p => p.Name, StringComparer.Ordinal).ToList();
            }
        }
    }

    public void AddValidator(Func<IReadOnlyList<Parameter>, SetParametersResult> validator)
    {
        lock (_gate)
        {
            _validators.Add(validator);
        }
    }

    public void RemoveValidator(Func<IReadOnlyList<Parameter>, SetParametersResult> validator)
    {
        lock (_gate)
        {
            _validators.Remove(validator);
        }
    }

    /// <summary>Sets parameters; when <paramref name="atomic"/>, either all succeed or none are applied.</summary>
    public IReadOnlyList<SetParametersResult> Set(IReadOnlyList<Parameter> parameters, bool atomic)
    {
        var results = new SetParametersResult[parameters.Count];
        var changes = new List<ParameterChange>();
        lock (_gate)
        {
            var accepted = new List<(Parameter Parameter, ParameterDescriptor Descriptor, ParameterValue? Old)>();
            for (var i = 0; i < parameters.Count; i++)
            {
                var p = parameters[i];
                if (!_parameters.TryGetValue(p.Name, out var entry))
                {
                    if (!_allowUndeclared)
                    {
                        results[i] = SetParametersResult.Failure($"parameter '{p.Name}' has not been declared");
                        continue;
                    }

                    entry = (new ParameterDescriptor { Name = p.Name, DynamicTyping = true }, ParameterValue.NotSet);
                }

                var value = p.Value.IsSet ? Coerce(p.Value, entry.Descriptor) : p.Value;
                var check = CheckConstraints(entry.Descriptor, value, declaring: false);
                results[i] = check;
                if (check.Successful)
                {
                    accepted.Add((p with { Value = value }, entry.Descriptor, entry.Value.IsSet ? entry.Value : null));
                }
            }

            if (accepted.Count > 0)
            {
                var validation = RunValidators(accepted.Select(a => a.Parameter).ToList());
                if (!validation.Successful)
                {
                    for (var i = 0; i < results.Length; i++)
                    {
                        if (results[i].Successful)
                        {
                            results[i] = validation;
                        }
                    }

                    accepted.Clear();
                }
            }

            if (atomic && results.Any(r => !r.Successful))
            {
                var reason = results.First(r => !r.Successful).Reason;
                return results.Select(_ => SetParametersResult.Failure(reason)).ToList();
            }

            foreach (var (parameter, descriptor, old) in accepted)
            {
                if (!parameter.Value.IsSet)
                {
                    _parameters.Remove(parameter.Name);
                }
                else
                {
                    _parameters[parameter.Name] = (descriptor, parameter.Value);
                }

                changes.Add(new ParameterChange(parameter.Name, old, parameter.Value));
            }
        }

        if (changes.Count > 0)
        {
            Changed?.Invoke(changes);
        }

        return results;
    }

    private SetParametersResult RunValidators(IReadOnlyList<Parameter> parameters)
    {
        foreach (var v in _validators)
        {
            var r = v(parameters);
            if (!r.Successful)
            {
                return r;
            }
        }

        return SetParametersResult.Success;
    }

    private static ParameterValue Coerce(ParameterValue value, ParameterDescriptor d)
    {
        // Integers written where a double was declared ("gain: 1") are accepted, as rclcpp does for YAML.
        if (d.Type == ParameterType.Double && value.Type == ParameterType.Integer)
        {
            return ParameterValue.From((double)value.AsInteger());
        }

        if (d.Type == ParameterType.DoubleArray && value.Type == ParameterType.IntegerArray)
        {
            return ParameterValue.From(value.AsDoubleArray());
        }

        return value;
    }

    private static SetParametersResult CheckConstraints(ParameterDescriptor d, ParameterValue value, bool declaring)
    {
        if (!declaring && d.ReadOnly)
        {
            return SetParametersResult.Failure($"parameter '{d.Name}' is read-only");
        }

        if (!value.IsSet)
        {
            return SetParametersResult.Success;
        }

        if (!d.DynamicTyping && d.Type != ParameterType.NotSet && value.Type != d.Type)
        {
            return SetParametersResult.Failure($"wrong parameter type, parameter {{{d.Name}}} is of type {{{d.Type.ToString().ToLowerInvariant()}}}, setting it to {{{value.Type.ToString().ToLowerInvariant()}}} is not allowed.");
        }

        if (d.IntegerRange is { } ir && value.Type == ParameterType.Integer)
        {
            var v = value.AsInteger();
            if (v < ir.From || v > ir.To || (ir.Step > 0 && v != ir.To && (ulong)(v - ir.From) % ir.Step != 0))
            {
                return SetParametersResult.Failure($"parameter '{d.Name}' value {v} is outside [{ir.From}, {ir.To}] step {ir.Step}");
            }
        }

        if (d.FloatingPointRange is { } fr && value.Type == ParameterType.Double)
        {
            var v = value.AsDouble();
            const double tol = 1e-9;
            var onStep = fr.Step <= 0 || Math.Abs(v - fr.To) < tol ||
                         Math.Abs(Math.IEEERemainder(v - fr.From, fr.Step)) < tol;
            if (v < fr.From - tol || v > fr.To + tol || !onStep)
            {
                return SetParametersResult.Failure($"parameter '{d.Name}' value {v} is outside [{fr.From}, {fr.To}] step {fr.Step}");
            }
        }

        return SetParametersResult.Success;
    }

    private static void ValidateName(string name)
    {
        if (string.IsNullOrWhiteSpace(name))
        {
            throw new ArgumentException("Parameter name must not be empty.", nameof(name));
        }
    }
}
