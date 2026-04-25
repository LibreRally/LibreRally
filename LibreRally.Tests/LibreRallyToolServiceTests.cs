using System.Text.RegularExpressions;
using LibreRally.McpServer;
using LibreRally.Telemetry;
using ModelContextProtocol.Server;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the SDK-backed MCP tool service logic independently of the hosting layer.
	/// </summary>
	public sealed class LibreRallyToolServiceTests
	{
		private static string GetVehicleFolder(string vehicleFolderName)
		{
			DirectoryInfo? directory = new(AppContext.BaseDirectory);
			while (directory != null)
			{
				var candidate = Path.Combine(directory.FullName, "LibreRally.sln");
				if (File.Exists(candidate))
				{
					return Path.Combine(directory.FullName, "LibreRally", "Resources", "BeamNG Vehicles", vehicleFolderName);
				}

				directory = directory.Parent;
			}

			throw new DirectoryNotFoundException($"Could not locate the repository root for '{vehicleFolderName}' tests.");
		}

		/// <summary>
		/// Verifies that tuning sessions can be created and advanced through the shared tool service.
		/// </summary>
		[Fact]
		public void TuningSessionTools_CreateAndAdvanceSession()
		{
			var service = new LibreRallyToolService(new FakeLiveTuningBridgeClient());
			var startResult = service.StartTuningSession(
				"Sunburst 2",
				"Baseline",
				"Variant A",
				vehicleFolderPath: GetVehicleFolder("sunburst2"),
				configFileName: "rally_am_asphalt.pc");

			Assert.False(string.IsNullOrWhiteSpace(startResult.SessionId));
			Assert.Equal("launch-grip", startResult.CurrentPrompt.Id);

			var updatedResult = service.RecordTuningVerdict(startResult.SessionId, "better", "cleaner launch", advance: true);

			Assert.Equal(1, updatedResult.CompletedPromptCount);
			Assert.Equal("corner-entry", updatedResult.CurrentPrompt.Id);
			Assert.Contains(updatedResult.Responses, response => response.Verdict == "Better");
		}

		/// <summary>
		/// Verifies that live snapshot calls are proxied through the bridge client.
		/// </summary>
		[Fact]
		public void LiveSnapshot_ReturnsBridgeSnapshot()
		{
			var service = new LibreRallyToolService(new FakeLiveTuningBridgeClient());

			var response = service.GetLiveGameSnapshot();

			Assert.True(response.Succeeded);
			Assert.Equal("Sunburst 2", response.Snapshot!.VehicleName);
			Assert.Equal("brush_only", response.Snapshot.TyreModelMode);
		}

		/// <summary>
		/// Verifies that live tuning patches are forwarded to the bridge client.
		/// </summary>
		[Fact]
		public void ApplyLiveTuningPatch_ForwardsPatchValues()
		{
			var fakeClient = new FakeLiveTuningBridgeClient();
			var service = new LibreRallyToolService(fakeClient);

			var response = service.ApplyLiveTuningPatch(new LiveTuningPatch
			{
				TyreModelMode = "pacejka_only",
				PeakFrictionCoefficient = 0.93f,
			});

			Assert.True(response.Succeeded);
			Assert.NotNull(fakeClient.LastRequest);
			Assert.Equal("apply_patch", fakeClient.LastRequest!.Command);
			Assert.Equal("pacejka_only", fakeClient.LastRequest.Patch!.TyreModelMode);
			Assert.Equal(0.93f, fakeClient.LastRequest.Patch.PeakFrictionCoefficient);
		}

		/// <summary>
		/// Verifies that exported MCP tool names stay within the Copilot CLI naming constraints.
		/// </summary>
		[Fact]
		public void ToolNames_MatchCopilotCliPattern()
		{
			var validToolName = new Regex("^[a-zA-Z0-9_-]+$");
			var toolNames = typeof(LibreRallyTools)
				.GetMethods()
				.Select(method => method.GetCustomAttributes(typeof(McpServerToolAttribute), inherit: false)
					.OfType<McpServerToolAttribute>()
					.SingleOrDefault()?.Name)
				.Where(name => !string.IsNullOrWhiteSpace(name))
				.Cast<string>()
				.ToArray();

			Assert.NotEmpty(toolNames);
			Assert.All(toolNames, name => Assert.Matches(validToolName, name));
		}

		private sealed class FakeLiveTuningBridgeClient : ILiveTuningBridgeClient
		{
			public LiveTuningBridgeRequest? LastRequest { get; private set; }

			public LiveTuningBridgeResponse Send(string host, int port, LiveTuningBridgeRequest request)
			{
				LastRequest = request;
				return new LiveTuningBridgeResponse
				{
					Succeeded = true,
					Command = request.Command,
					Summary = "Bridge response OK.",
					Snapshot = new LiveTuningSnapshot
					{
						VehicleLoaded = true,
						VehicleName = "Sunburst 2",
						ConfigFileName = "rally_am_asphalt.pc",
						TyreModelMode = request.Patch?.TyreModelMode ?? "brush_only",
						SpeedKmh = 64.2f,
					},
				};
			}
		}
	}
}
