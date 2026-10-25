using K4os.Compression.LZ4.Streams;

namespace ROS2.Net.Bag.Mcap;

/// <summary>Chunk compression codecs (zstd via ZstdSharp, lz4 frame format via K4os).</summary>
internal static class McapCompressionCodec
{
    public static byte[] Compress(McapCompression compression, ReadOnlySpan<byte> data)
    {
        switch (compression)
        {
            case McapCompression.None:
                return data.ToArray();
            case McapCompression.Zstd:
            {
                using var compressor = new ZstdSharp.Compressor(3);
                return compressor.Wrap(data).ToArray();
            }

            case McapCompression.Lz4:
            {
                using var ms = new MemoryStream(data.Length / 2 + 64);
                using (var lz = LZ4Stream.Encode(ms, leaveOpen: true))
                {
                    lz.Write(data);
                }

                return ms.ToArray();
            }

            default:
                throw new ArgumentOutOfRangeException(nameof(compression));
        }
    }

    public static byte[] Decompress(string compression, ReadOnlySpan<byte> data, ulong uncompressedSize)
    {
        switch (compression)
        {
            case "":
                return data.ToArray();
            case "zstd":
            {
                var result = new byte[uncompressedSize];
                using var decompressor = new ZstdSharp.Decompressor();
                var written = decompressor.Unwrap(data, result);
                if ((ulong)written != uncompressedSize)
                {
                    throw new McapException($"zstd chunk decompressed to {written} bytes, expected {uncompressedSize}.");
                }

                return result;
            }

            case "lz4":
            {
                var result = new byte[uncompressedSize];
                using var src = new MemoryStream(data.ToArray(), writable: false);
                using var lz = LZ4Stream.Decode(src);
                lz.ReadExactly(result);
                return result;
            }

            default:
                throw new McapException($"Unsupported chunk compression '{compression}'.");
        }
    }
}
