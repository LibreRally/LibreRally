using LibreRally.HUD;
using LibreRally.Vehicle;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the local A/B tuning-session workflow.
	/// </summary>
	public class TuningSessionModelTests
	{
		/// <summary>
		/// Verifies that tuning sessions create structured driving prompts from a troubleshooting report.
		/// </summary>
		[Fact]
		public void Create_BuildsStructuredPromptsAndTelemetrySummary()
		{
			var report = new VehicleTroubleshootingReport
			{
				VehicleFolderPath = @"C:\vehicles\sunburst2",
				ConfigPath = "rally_am_asphalt.pc",
				Observations =
				[
					"Brush and Pacejka stay close in the launch probe.",
					"Neutral tyre control does not dramatically outperform the loaded tyre data.",
				],
			};

			var session = TuningSessionModel.CreateFromTroubleshootingReport(
				vehicleName: "Sunburst 2",
				statusText: "A/B launch comparison",
				baselineLabel: "Baseline",
				candidateLabel: "Variant A",
				report: report);

			Assert.Equal("Sunburst 2", session.VehicleName);
			Assert.Equal("Baseline", session.BaselineLabel);
			Assert.Equal("Variant A", session.CandidateLabel);
			Assert.Equal(4, session.Prompts.Count);
			Assert.Contains("Brush and Pacejka stay close", session.TelemetrySummary);
			Assert.Equal("launch-grip", session.CurrentPrompt.Id);
			Assert.Contains("better, worse, or the same", session.CurrentPrompt.ComparisonQuestion);
		}

		/// <summary>
		/// Verifies that tuning sessions record responses and summarize progress.
		/// </summary>
		[Fact]
		public void RecordCurrentPromptResponse_StoresVerdictsAndBuildsSummary()
		{
			var session = TuningSessionModel.Create(
				vehicleName: "Basic Car",
				statusText: "Flat-tarmac check",
				baselineLabel: "Baseline",
				candidateLabel: "Variant B");

			session.RecordCurrentPromptResponse(TuningSessionVerdict.Better, "hooked up sooner");
			Assert.True(session.MoveNextPrompt());
			session.RecordCurrentPromptResponse(TuningSessionVerdict.Same);

			var summary = session.CreateSessionSummary();

			Assert.Equal(2, session.CompletedPromptCount);
			Assert.Equal(2, session.PendingPromptCount);
			Assert.Contains("Launch grip: Better (hooked up sooner)", summary);
			Assert.Contains("Corner entry rotation: Same", summary);
			Assert.Contains("Mid-corner balance: Pending", summary);
		}
	}
}
