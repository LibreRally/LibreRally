using System.Buffers.Binary;
using System.Text;
using LibreRally.Telemetry;
using Xunit;

namespace LibreRally.Tests;

public class OutGaugeProtocolTests
{
    [Fact]
    public void Encode_UsesLfsOutGaugePacketLayout_WithoutOptionalId()
    {
        var snapshot = new OutGaugeSnapshot
        {
            Time = 1234,
            Car = "beam",
            Flags = 0xC000,
            Gear = 3,
            Plid = 7,
            Speed = 12.5f,
            Rpm = 3456f,
            Turbo = 0.7f,
            EngineTemp = 91f,
            Fuel = 0.42f,
            OilPressure = 3.2f,
            OilTemp = 88f,
            DashLights = 0x1234u,
            ShowLights = 0x5678u,
            Throttle = 0.8f,
            Brake = 0.1f,
            Clutch = 0.25f,
            Display1 = "Fuel",
            Display2 = "Setup",
        };

        var packet = OutGaugeProtocol.Encode(snapshot);

        Assert.Equal(OutGaugeProtocol.PacketSizeWithoutId, packet.Length);
        Assert.Equal(1234u, BinaryPrimitives.ReadUInt32LittleEndian(packet.AsSpan(0, 4)));
        Assert.Equal("beam", Encoding.ASCII.GetString(packet, 4, 4));
        Assert.Equal(0xC000, BinaryPrimitives.ReadUInt16LittleEndian(packet.AsSpan(8, 2)));
        Assert.Equal(3, packet[10]);
        Assert.Equal(7, packet[11]);
        Assert.Equal(12.5f, ReadSingle(packet, 12), 4);
        Assert.Equal(3456f, ReadSingle(packet, 16), 4);
        Assert.Equal("Fuel", ReadFixedAscii(packet, 60, 16));
        Assert.Equal("Setup", ReadFixedAscii(packet, 76, 16));
    }

    [Fact]
    public void Encode_AppendsIdentifier_WhenOutGaugeIdIsConfigured()
    {
        var packet = OutGaugeProtocol.Encode(new OutGaugeSnapshot(), outGaugeId: 77);

        Assert.Equal(OutGaugeProtocol.PacketSizeWithId, packet.Length);
        Assert.Equal(77, BinaryPrimitives.ReadInt32LittleEndian(packet.AsSpan(92, 4)));
    }

    [Theory]
    [InlineData(0, 0)]
    [InlineData(1, 2)]
    [InlineData(2, 3)]
    public void MapGear_ConvertsGearIndicesToOutGaugeConvention(int rallyGear, byte outGaugeGear)
    {
        Assert.Equal(outGaugeGear, OutGaugeProtocol.MapGear(rallyGear));
    }

    private static float ReadSingle(byte[] packet, int offset)
    {
        return BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(packet.AsSpan(offset, 4)));
    }

    private static string ReadFixedAscii(byte[] packet, int offset, int length)
    {
        return Encoding.ASCII.GetString(packet, offset, length).TrimEnd('\0');
    }
}
