using System.Buffers.Binary;
using LibreRally.Telemetry;
using Stride.Core.Mathematics;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the out sim protocol behavior.
	/// </summary>
	public class OutSimProtocolTests
	{
		/// <summary>
		/// Verifies that encode uses lfs out sim packet layout without optional id.
		/// </summary>
		[Fact]
		public void Encode_UsesLfsOutSimPacketLayout_WithoutOptionalId()
		{
			var snapshot = new OutSimSnapshot
			{
				Time = 1234,
				AngularVelocity = new Vector3(1.1f, 2.2f, 3.3f),
				Heading = 0.4f,
				Pitch = -0.5f,
				Roll = 0.6f,
				Acceleration = new Vector3(-4f, 5f, -6f),
				Velocity = new Vector3(7f, 8f, 9f),
				PositionX = 100,
				PositionY = -200,
				PositionZ = 300,
			};

			var packet = OutSimProtocol.Encode(snapshot);

			Assert.Equal(OutSimProtocol.PacketSizeWithoutId, packet.Length);
			Assert.Equal(1234u, BinaryPrimitives.ReadUInt32LittleEndian(packet.AsSpan(0, 4)));
			Assert.Equal(1.1f, ReadSingle(packet, 4), 4);
			Assert.Equal(3.3f, ReadSingle(packet, 12), 4);
			Assert.Equal(0.4f, ReadSingle(packet, 16), 4);
			Assert.Equal(0.6f, ReadSingle(packet, 24), 4);
			Assert.Equal(-4f, ReadSingle(packet, 28), 4);
			Assert.Equal(9f, ReadSingle(packet, 48), 4);
			Assert.Equal(100, BinaryPrimitives.ReadInt32LittleEndian(packet.AsSpan(52, 4)));
			Assert.Equal(-200, BinaryPrimitives.ReadInt32LittleEndian(packet.AsSpan(56, 4)));
			Assert.Equal(300, BinaryPrimitives.ReadInt32LittleEndian(packet.AsSpan(60, 4)));
		}

		/// <summary>
		/// Verifies that encode appends identifier when out sim id is configured.
		/// </summary>
		[Fact]
		public void Encode_AppendsIdentifier_WhenOutSimIdIsConfigured()
		{
			var packet = OutSimProtocol.Encode(new OutSimSnapshot(), outSimId: 55);

			Assert.Equal(OutSimProtocol.PacketSizeWithId, packet.Length);
			Assert.Equal(55, BinaryPrimitives.ReadInt32LittleEndian(packet.AsSpan(64, 4)));
		}

		private static float ReadSingle(byte[] packet, int offset)
		{
			return BitConverter.Int32BitsToSingle(BinaryPrimitives.ReadInt32LittleEndian(packet.AsSpan(offset, 4)));
		}
	}
}
