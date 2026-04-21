using System;
using System.Buffers.Binary;
using LibreRally.Vehicle;

namespace LibreRally.Telemetry
{
	[Flags]
	internal enum OutGaugeFlags : ushort
	{
		Turbo = 1 << 13,
		Kilometers = 1 << 14,
		Bar = 1 << 15,
	}

	[Flags]
	internal enum OutGaugeDashLights : uint
	{
		None = 0,
		Shift = 1u << 0,
		Handbrake = 1u << 2,
		TractionControl = 1u << 4,
		Abs = 1u << 10,
	}

	internal sealed record OutGaugeSnapshot
	{
		public uint Time { get; init; }
		public string Car { get; init; } = "libr";
		public ushort Flags { get; init; } = (ushort)(OutGaugeFlags.Kilometers | OutGaugeFlags.Bar);
		public byte Gear { get; init; }
		public byte Plid { get; init; }
		public float Speed { get; init; }
		public float Rpm { get; init; }
		public float Turbo { get; init; }
		public float EngineTemp { get; init; }
		public float Fuel { get; init; }
		public float OilPressure { get; init; }
		public float OilTemp { get; init; }
		public uint DashLights { get; init; }
		public uint ShowLights { get; init; }
		public float Throttle { get; init; }
		public float Brake { get; init; }
		public float Clutch { get; init; }
		public string Display1 { get; init; } = string.Empty;
		public string Display2 { get; init; } = string.Empty;
	}

	internal static class OutGaugeProtocol
	{
		internal const int PacketSizeWithoutId = 92;
		internal const int PacketSizeWithId = 96;

		internal static OutGaugeSnapshot FromCar(RallyCarComponent car, uint timeMilliseconds)
		{
			var availableDashLights = OutGaugeDashLights.Handbrake | OutGaugeDashLights.TractionControl | OutGaugeDashLights.Abs | OutGaugeDashLights.Shift;
			var showLights = OutGaugeDashLights.None;

			if (car.HandbrakeEngaged)
			{
				showLights |= OutGaugeDashLights.Handbrake;
			}

			if (car.TractionControlActive)
			{
				showLights |= OutGaugeDashLights.TractionControl;
			}

			if (car.AbsActive)
			{
				showLights |= OutGaugeDashLights.Abs;
			}

			if (car.MaxRpm > 0f && car.EngineRpm >= car.MaxRpm * 0.9f)
			{
				showLights |= OutGaugeDashLights.Shift;
			}

			return new OutGaugeSnapshot
			{
				Time = timeMilliseconds,
				Gear = MapGear(car.CurrentGear),
				Speed = car.SpeedKmh / 3.6f,
				Rpm = car.EngineRpm,
				Turbo = car.TurboBoostBar,
				EngineTemp = car.EngineTemp,
				Fuel = car.FuelCapacityLiters > 0f ? car.FuelLiters / car.FuelCapacityLiters : 0f,
				OilPressure = car.OilPressure,
				OilTemp = car.OilTemp,
				DashLights = (uint)availableDashLights,
				ShowLights = (uint)showLights,
				Throttle = Math.Clamp(car.ThrottleInput, 0f, 1f),
				Brake = Math.Clamp(car.BrakeInput, 0f, 1f),
				Clutch = 0f,
			};
		}

		internal static byte[] Encode(OutGaugeSnapshot snapshot, int outGaugeId = 0)
		{
			var includeId = outGaugeId != 0;
			var packet = new byte[includeId ? PacketSizeWithId : PacketSizeWithoutId];
			var span = packet.AsSpan();
			var offset = 0;

			WriteUInt32(span, ref offset, snapshot.Time);
			WriteFixedAscii(span, ref offset, 4, snapshot.Car);
			WriteUInt16(span, ref offset, snapshot.Flags);
			span[offset++] = snapshot.Gear;
			span[offset++] = snapshot.Plid;
			WriteSingle(span, ref offset, snapshot.Speed);
			WriteSingle(span, ref offset, snapshot.Rpm);
			WriteSingle(span, ref offset, snapshot.Turbo);
			WriteSingle(span, ref offset, snapshot.EngineTemp);
			WriteSingle(span, ref offset, snapshot.Fuel);
			WriteSingle(span, ref offset, snapshot.OilPressure);
			WriteSingle(span, ref offset, snapshot.OilTemp);
			WriteUInt32(span, ref offset, snapshot.DashLights);
			WriteUInt32(span, ref offset, snapshot.ShowLights);
			WriteSingle(span, ref offset, snapshot.Throttle);
			WriteSingle(span, ref offset, snapshot.Brake);
			WriteSingle(span, ref offset, snapshot.Clutch);
			WriteFixedAscii(span, ref offset, 16, snapshot.Display1);
			WriteFixedAscii(span, ref offset, 16, snapshot.Display2);

			if (includeId)
			{
				BinaryPrimitives.WriteInt32LittleEndian(span[offset..], outGaugeId);
			}

			return packet;
		}

		internal static byte MapGear(int currentGear)
		{
			if (currentGear <= 0)
			{
				return 0;
			}

			return (byte)Math.Min(currentGear + 1, byte.MaxValue);
		}

		private static void WriteUInt16(Span<byte> packet, ref int offset, ushort value)
		{
			BinaryPrimitives.WriteUInt16LittleEndian(packet[offset..], value);
			offset += sizeof(ushort);
		}

		private static void WriteUInt32(Span<byte> packet, ref int offset, uint value)
		{
			BinaryPrimitives.WriteUInt32LittleEndian(packet[offset..], value);
			offset += sizeof(uint);
		}

		private static void WriteSingle(Span<byte> packet, ref int offset, float value)
		{
			BinaryPrimitives.WriteInt32LittleEndian(packet[offset..], BitConverter.SingleToInt32Bits(value));
			offset += sizeof(float);
		}

		private static void WriteFixedAscii(Span<byte> packet, ref int offset, int width, string? value)
		{
			var destination = packet.Slice(offset, width);
			destination.Clear();

			if (!string.IsNullOrEmpty(value))
			{
				var charCount = Math.Min(value.Length, width);
				for (var i = 0; i < charCount; i++)
				{
					var c = value[i];
					destination[i] = c <= 127 ? (byte)c : (byte)'?';
				}
			}

			offset += width;
		}
	}
}
