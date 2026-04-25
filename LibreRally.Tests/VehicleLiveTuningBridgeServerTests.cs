using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using LibreRally.Telemetry;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the localhost live tuning bridge protocol independently of the Stride game loop.
	/// </summary>
	public sealed class VehicleLiveTuningBridgeServerTests
	{
		private static readonly JsonSerializerOptions SerializerOptions = new(JsonSerializerDefaults.Web);

		/// <summary>
		/// Verifies that snapshot requests are returned after the game-thread pump services the queue.
		/// </summary>
		[Fact]
		public async Task GetSnapshot_ReturnsSnapshotFromPump()
		{
			using var bridge = new VehicleLiveTuningBridgeServer(0);
			bridge.Start();

			var requestTask = SendRequestAsync(bridge.Port, new LiveTuningBridgeRequest { Command = "get_snapshot" });
			await PumpUntilCompletedAsync(
				requestTask,
				bridge,
				() => new LiveTuningSnapshot { VehicleLoaded = true, VehicleName = "Sunburst 2", SpeedKmh = 87.5f },
				_ => new LiveTuningBridgeResponse { Succeeded = true, Command = "apply_patch", Summary = "applied" },
				() => new LiveTuningBridgeResponse { Succeeded = true, Command = "reload_vehicle", Summary = "reloaded" });

			var response = await requestTask;

			Assert.True(response.Succeeded);
			Assert.Equal("Sunburst 2", response.Snapshot!.VehicleName);
			Assert.Equal(87.5f, response.Snapshot.SpeedKmh);
		}

		/// <summary>
		/// Verifies that patch requests are forwarded through the queue with their payload intact.
		/// </summary>
		[Fact]
		public async Task ApplyPatch_ForwardsPatchPayloadToPump()
		{
			using var bridge = new VehicleLiveTuningBridgeServer(0);
			bridge.Start();

			LiveTuningPatch? observedPatch = null;
			var requestTask = SendRequestAsync(bridge.Port, new LiveTuningBridgeRequest
			{
				Command = "apply_patch",
				Patch = new LiveTuningPatch { TyreModelMode = "brush_only", PeakFrictionCoefficient = 0.92f },
			});
			await PumpUntilCompletedAsync(
				requestTask,
				bridge,
				() => new LiveTuningSnapshot(),
				patch =>
				{
					observedPatch = patch;
					return new LiveTuningBridgeResponse
					{
						Succeeded = true,
						Command = "apply_patch",
						Summary = "Applied live tuning changes.",
					};
				},
				() => new LiveTuningBridgeResponse { Succeeded = true, Command = "reload_vehicle", Summary = "reloaded" });

			var response = await requestTask;

			Assert.True(response.Succeeded);
			Assert.NotNull(observedPatch);
			Assert.Equal("brush_only", observedPatch!.TyreModelMode);
			Assert.Equal(0.92f, observedPatch.PeakFrictionCoefficient);
		}

		private static async Task<LiveTuningBridgeResponse> SendRequestAsync(int port, LiveTuningBridgeRequest request)
		{
			using var client = new TcpClient();
			await client.ConnectAsync("127.0.0.1", port);
			using var stream = client.GetStream();
			using var writer = new StreamWriter(stream, new UTF8Encoding(false), leaveOpen: true) { AutoFlush = true };
			using var reader = new StreamReader(stream, Encoding.UTF8, detectEncodingFromByteOrderMarks: false, leaveOpen: true);

			await writer.WriteLineAsync(JsonSerializer.Serialize(request, SerializerOptions));
			var line = await reader.ReadLineAsync();
			Assert.False(string.IsNullOrWhiteSpace(line));

			return JsonSerializer.Deserialize<LiveTuningBridgeResponse>(line!, SerializerOptions)
			       ?? throw new InvalidOperationException("Bridge response was not valid JSON.");
		}

		private static async Task PumpUntilCompletedAsync(
			Task<LiveTuningBridgeResponse> requestTask,
			VehicleLiveTuningBridgeServer bridge,
			Func<LiveTuningSnapshot> getSnapshot,
			Func<LiveTuningPatch, LiveTuningBridgeResponse> applyPatch,
			Func<LiveTuningBridgeResponse> reloadVehicle)
		{
			using var timeout = new CancellationTokenSource(TimeSpan.FromSeconds(5));
			while (!requestTask.IsCompleted)
			{
				bridge.Pump(getSnapshot, applyPatch, reloadVehicle);
				await Task.Delay(20, timeout.Token);
			}
		}
	}
}
