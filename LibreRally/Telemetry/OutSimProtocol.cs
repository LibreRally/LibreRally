using System;
using System.Buffers.Binary;
using LibreRally.Vehicle;
using Stride.Core.Mathematics;

namespace LibreRally.Telemetry;

internal sealed record OutSimSnapshot
{
    public uint Time { get; init; }
    public Vector3 AngularVelocity { get; init; }
    public float Heading { get; init; }
    public float Pitch { get; init; }
    public float Roll { get; init; }
    public Vector3 Acceleration { get; init; }
    public Vector3 Velocity { get; init; }
    public int PositionX { get; init; }
    public int PositionY { get; init; }
    public int PositionZ { get; init; }
}

internal static class OutSimProtocol
{
    internal const int PacketSizeWithoutId = 64;
    internal const int PacketSizeWithId = 68;

    internal static OutSimSnapshot FromCar(RallyCarComponent car, uint timeMilliseconds, in Vector3 acceleration)
    {
        var body = car.CarBody.Get<Stride.BepuPhysics.BodyComponent>();
        var transform = car.CarBody.Transform;
        transform.UpdateWorldMatrix();
        var world = transform.WorldMatrix;

        var forward = world.Backward;
        var up = world.Up;

        var heading = MathF.Atan2(forward.X, forward.Z);
        var pitch = MathF.Asin(Math.Clamp(forward.Y, -1f, 1f));
        var roll = MathF.Atan2(up.X, up.Y);
        var position = world.TranslationVector;

        return new OutSimSnapshot
        {
            Time = timeMilliseconds,
            AngularVelocity = body?.AngularVelocity ?? Vector3.Zero,
            Heading = heading,
            Pitch = pitch,
            Roll = roll,
            Acceleration = acceleration,
            Velocity = body?.LinearVelocity ?? Vector3.Zero,
            PositionX = ConvertPositionMillimeters(position.X),
            PositionY = ConvertPositionMillimeters(position.Y),
            PositionZ = ConvertPositionMillimeters(position.Z),
        };
    }

    internal static byte[] Encode(OutSimSnapshot snapshot, int outSimId = 0)
    {
        var includeId = outSimId != 0;
        var packet = new byte[includeId ? PacketSizeWithId : PacketSizeWithoutId];
        var span = packet.AsSpan();
        var offset = 0;

        WriteUInt32(span, ref offset, snapshot.Time);
        WriteVector3(span, ref offset, snapshot.AngularVelocity);
        WriteSingle(span, ref offset, snapshot.Heading);
        WriteSingle(span, ref offset, snapshot.Pitch);
        WriteSingle(span, ref offset, snapshot.Roll);
        WriteVector3(span, ref offset, snapshot.Acceleration);
        WriteVector3(span, ref offset, snapshot.Velocity);
        WriteInt32(span, ref offset, snapshot.PositionX);
        WriteInt32(span, ref offset, snapshot.PositionY);
        WriteInt32(span, ref offset, snapshot.PositionZ);

        if (includeId)
        {
            WriteInt32(span, ref offset, outSimId);
        }

        return packet;
    }

    private static int ConvertPositionMillimeters(float positionMeters)
    {
        var positionMm = positionMeters * 1000f;
        if (positionMm >= int.MaxValue)
        {
            return int.MaxValue;
        }

        if (positionMm <= int.MinValue)
        {
            return int.MinValue;
        }

        return (int)MathF.Round(positionMm);
    }

    private static void WriteUInt32(Span<byte> packet, ref int offset, uint value)
    {
        BinaryPrimitives.WriteUInt32LittleEndian(packet[offset..], value);
        offset += sizeof(uint);
    }

    private static void WriteInt32(Span<byte> packet, ref int offset, int value)
    {
        BinaryPrimitives.WriteInt32LittleEndian(packet[offset..], value);
        offset += sizeof(int);
    }

    private static void WriteSingle(Span<byte> packet, ref int offset, float value)
    {
        BinaryPrimitives.WriteInt32LittleEndian(packet[offset..], BitConverter.SingleToInt32Bits(value));
        offset += sizeof(float);
    }

    private static void WriteVector3(Span<byte> packet, ref int offset, in Vector3 value)
    {
        WriteSingle(packet, ref offset, value.X);
        WriteSingle(packet, ref offset, value.Y);
        WriteSingle(packet, ref offset, value.Z);
    }
}
