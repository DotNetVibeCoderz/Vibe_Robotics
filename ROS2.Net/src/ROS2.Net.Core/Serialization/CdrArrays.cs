namespace ROS2.Net.Serialization;

/// <summary>Array helpers used by generated message code. They reuse existing arrays to keep decoding allocation-free.</summary>
public static class CdrArrays
{
    /// <summary>Returns <paramref name="existing"/> when it has <paramref name="length"/> elements, otherwise a new array. String arrays are filled with <see cref="string.Empty"/>.</summary>
    public static T[] Ensure<T>(T[]? existing, int length)
    {
        if (existing is not null && existing.Length == length)
        {
            return existing;
        }

        if (length == 0)
        {
            return [];
        }

        var result = new T[length];
        if (typeof(T) == typeof(string))
        {
            ((string[])(object)result).AsSpan().Fill(string.Empty);
        }

        return result;
    }

    /// <summary>Like <see cref="Ensure{T}"/>, but every element is a live message instance (existing instances are reused).</summary>
    public static T[] EnsureMessages<T>(T[]? existing, int length)
        where T : class, IRosMessage, new()
    {
        if (existing is not null && existing.Length == length)
        {
            for (var i = 0; i < existing.Length; i++)
            {
                existing[i] ??= new T();
            }

            return existing;
        }

        if (length == 0)
        {
            return [];
        }

        var result = new T[length];
        var reuse = existing is null ? 0 : Math.Min(existing.Length, length);
        for (var i = 0; i < length; i++)
        {
            result[i] = i < reuse && existing![i] is { } e ? e : new T();
        }

        return result;
    }

    /// <summary>Shallow copy of a primitive or string array.</summary>
    public static T[] Copy<T>(T[]? source) => source is null || source.Length == 0 ? [] : (T[])source.Clone();

    /// <summary>Deep copy of a message array.</summary>
    public static T[] CloneMessages<T>(T[]? source)
        where T : class, IRosMessage
    {
        if (source is null || source.Length == 0)
        {
            return [];
        }

        var result = new T[source.Length];
        for (var i = 0; i < source.Length; i++)
        {
            result[i] = (T)source[i].CloneMessage();
        }

        return result;
    }
}
