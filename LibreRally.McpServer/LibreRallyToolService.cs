using LibreRally.HUD;
using LibreRally.Telemetry;
using LibreRally.Vehicle;

namespace LibreRally.McpServer;

internal sealed class LibreRallyToolService
{
	private const string DefaultLiveBridgeHost = "127.0.0.1";
	private const int DefaultLiveBridgePort = 18765;
	private readonly Dictionary<string, TuningSessionModel> _sessions = new(StringComparer.OrdinalIgnoreCase);
	private readonly ILiveTuningBridgeClient _liveTuningBridgeClient;

	public LibreRallyToolService(ILiveTuningBridgeClient liveTuningBridgeClient)
	{
		_liveTuningBridgeClient = liveTuningBridgeClient;
	}

	public VehicleConfigListResult ListVehicleConfigs(string vehicleFolderPath)
	{
		if (!Directory.Exists(vehicleFolderPath))
		{
			throw new DirectoryNotFoundException($"Vehicle folder not found: '{vehicleFolderPath}'.");
		}

		var configs = Directory.EnumerateFiles(vehicleFolderPath, "*.pc", SearchOption.TopDirectoryOnly)
			.Select(Path.GetFileName)
			.OrderBy(name => name, StringComparer.OrdinalIgnoreCase)
			.Where(name => !string.IsNullOrWhiteSpace(name))
			.ToArray()!;

		return new VehicleConfigListResult
		{
			VehicleFolderPath = vehicleFolderPath,
			Configs = configs,
		};
	}

	public TroubleshootingReportResult BuildTroubleshootingReport(string vehicleFolderPath, string? configFileName = null)
	{
		var report = VehicleTroubleshootingReportBuilder.Build(vehicleFolderPath, configFileName);
		return new TroubleshootingReportResult
		{
			VehicleFolderPath = report.VehicleFolderPath,
			ConfigPath = report.ConfigPath,
			FormattedReport = report.Format(),
			Observations = report.Observations.ToArray(),
		};
	}

	public TuningSessionResult StartTuningSession(
		string vehicleName,
		string baselineLabel,
		string candidateLabel,
		string? statusText = null,
		string? vehicleFolderPath = null,
		string? configFileName = null)
	{
		statusText ??= "A/B tuning session";
		VehicleTroubleshootingReport? report = null;
		TuningSessionModel session;
		if (!string.IsNullOrWhiteSpace(vehicleFolderPath))
		{
			report = VehicleTroubleshootingReportBuilder.Build(vehicleFolderPath, configFileName);
			var telemetrySummary = report.Observations.Count > 0
				? string.Join(" ", report.Observations.Take(2))
				: "Troubleshooting report attached.";
			session = TuningSessionModel.Create(vehicleName, statusText, baselineLabel, candidateLabel, telemetrySummary);
		}
		else
		{
			session = TuningSessionModel.Create(vehicleName, statusText, baselineLabel, candidateLabel);
		}

		var sessionId = Guid.NewGuid().ToString("N");
		_sessions[sessionId] = session;
		return CreateSessionResult(sessionId, session, report);
	}

	public TuningSessionResult GetTuningSession(string sessionId)
	{
		var session = GetSession(sessionId);
		return CreateSessionResult(sessionId, session, report: null);
	}

	public TuningSessionResult RecordTuningVerdict(string sessionId, string verdict, string? note = null, bool advance = true)
	{
		var session = GetSession(sessionId);
		session.RecordCurrentPromptResponse(ParseVerdict(verdict), note);
		if (advance)
		{
			session.MoveNextPrompt();
		}

		return CreateSessionResult(sessionId, session, report: null);
	}

	public LiveTuningBridgeResponse GetLiveGameSnapshot(string? host = null, int? port = null)
	{
		return SendLiveBridgeRequest(host, port, new LiveTuningBridgeRequest { Command = "get_snapshot" });
	}

	public LiveTuningBridgeResponse ApplyLiveTuningPatch(LiveTuningPatch patch, string? host = null, int? port = null)
	{
		return SendLiveBridgeRequest(host, port, new LiveTuningBridgeRequest
		{
			Command = "apply_patch",
			Patch = patch,
		});
	}

	public LiveTuningBridgeResponse ReloadLiveVehicle(string? host = null, int? port = null)
	{
		return SendLiveBridgeRequest(host, port, new LiveTuningBridgeRequest { Command = "reload_vehicle" });
	}

	private LiveTuningBridgeResponse SendLiveBridgeRequest(string? host, int? port, LiveTuningBridgeRequest request)
	{
		return _liveTuningBridgeClient.Send(host ?? DefaultLiveBridgeHost, port ?? DefaultLiveBridgePort, request);
	}

	private TuningSessionModel GetSession(string sessionId)
	{
		if (_sessions.TryGetValue(sessionId, out var session))
		{
			return session;
		}

		throw new InvalidOperationException($"Unknown tuning session '{sessionId}'.");
	}

	private static TuningSessionVerdict ParseVerdict(string value)
	{
		return value.Trim().ToLowerInvariant() switch
		{
			"better" => TuningSessionVerdict.Better,
			"worse" => TuningSessionVerdict.Worse,
			"same" => TuningSessionVerdict.Same,
			"needs_more_time" => TuningSessionVerdict.NeedsMoreTime,
			_ => throw new InvalidOperationException($"Unsupported verdict '{value}'."),
		};
	}

	private static TuningSessionResult CreateSessionResult(string sessionId, TuningSessionModel session, VehicleTroubleshootingReport? report)
	{
		return new TuningSessionResult
		{
			SessionId = sessionId,
			VehicleName = session.VehicleName,
			StatusText = session.StatusText,
			BaselineLabel = session.BaselineLabel,
			CandidateLabel = session.CandidateLabel,
			TelemetrySummary = session.TelemetrySummary,
			CurrentPromptIndex = session.CurrentPromptIndex,
			CompletedPromptCount = session.CompletedPromptCount,
			PendingPromptCount = session.PendingPromptCount,
			CurrentPrompt = CreatePromptResult(session.CurrentPrompt),
			Responses = session.Responses.Values.Select(CreateResponseResult).ToArray(),
			Summary = session.CreateSessionSummary(),
			TroubleshootingReport = report == null ? null : new TroubleshootingReportResult
			{
				VehicleFolderPath = report.VehicleFolderPath,
				ConfigPath = report.ConfigPath,
				FormattedReport = report.Format(),
				Observations = report.Observations.ToArray(),
			},
		};
	}

	private static TuningPromptResult CreatePromptResult(TuningSessionPrompt prompt)
	{
		return new TuningPromptResult
		{
			Id = prompt.Id,
			Title = prompt.Title,
			DrivingScript = prompt.DrivingScript,
			ComparisonQuestion = prompt.ComparisonQuestion,
			TelemetryFocus = prompt.TelemetryFocus,
		};
	}

	private static TuningResponseResult CreateResponseResult(TuningSessionResponse response)
	{
		return new TuningResponseResult
		{
			PromptId = response.PromptId,
			Verdict = response.Verdict.ToString(),
			Note = response.Note,
		};
	}
}

internal sealed class VehicleConfigListResult
{
	public string VehicleFolderPath { get; init; } = string.Empty;
	public string[] Configs { get; init; } = [];
}

internal sealed class TroubleshootingReportResult
{
	public string VehicleFolderPath { get; init; } = string.Empty;
	public string? ConfigPath { get; init; }
	public string FormattedReport { get; init; } = string.Empty;
	public string[] Observations { get; init; } = [];
}

internal sealed class TuningSessionResult
{
	public string SessionId { get; init; } = string.Empty;
	public string VehicleName { get; init; } = string.Empty;
	public string StatusText { get; init; } = string.Empty;
	public string BaselineLabel { get; init; } = string.Empty;
	public string CandidateLabel { get; init; } = string.Empty;
	public string TelemetrySummary { get; init; } = string.Empty;
	public int CurrentPromptIndex { get; init; }
	public int CompletedPromptCount { get; init; }
	public int PendingPromptCount { get; init; }
	public TuningPromptResult CurrentPrompt { get; init; } = new();
	public TuningResponseResult[] Responses { get; init; } = [];
	public string Summary { get; init; } = string.Empty;
	public TroubleshootingReportResult? TroubleshootingReport { get; init; }
}

internal sealed class TuningPromptResult
{
	public string Id { get; init; } = string.Empty;
	public string Title { get; init; } = string.Empty;
	public string DrivingScript { get; init; } = string.Empty;
	public string ComparisonQuestion { get; init; } = string.Empty;
	public string TelemetryFocus { get; init; } = string.Empty;
}

internal sealed class TuningResponseResult
{
	public string PromptId { get; init; } = string.Empty;
	public string Verdict { get; init; } = string.Empty;
	public string? Note { get; init; }
}
