using System.ComponentModel;
using LibreRally.Telemetry;
using ModelContextProtocol.Server;

namespace LibreRally.McpServer;

[McpServerToolType]
internal sealed class LibreRallyTools
{
	private readonly LibreRallyToolService _toolService;

	public LibreRallyTools(LibreRallyToolService toolService)
	{
		_toolService = toolService;
	}

	[McpServerTool(Name = "libreRally_list_vehicle_configs"), Description("List available .pc config files for a BeamNG vehicle folder.")]
	public VehicleConfigListResult ListVehicleConfigs(
		[Description("Absolute or repo-relative path to the BeamNG vehicle folder.")] string vehicleFolderPath)
		=> _toolService.ListVehicleConfigs(vehicleFolderPath);

	[McpServerTool(Name = "libreRally_build_troubleshooting_report"), Description("Build the LibreRally troubleshooting report for a vehicle folder and optional config.")]
	public TroubleshootingReportResult BuildTroubleshootingReport(
		[Description("Absolute or repo-relative path to the BeamNG vehicle folder.")] string vehicleFolderPath,
		[Description("Optional .pc config file name to load.")] string? configFileName = null)
		=> _toolService.BuildTroubleshootingReport(vehicleFolderPath, configFileName);

	[McpServerTool(Name = "libreRally_start_tuning_session"), Description("Create a structured A/B tuning session and return the first driving prompt.")]
	public TuningSessionResult StartTuningSession(
		[Description("Display name of the vehicle under test.")] string vehicleName,
		[Description("Label for the current baseline setup.")] string baselineLabel,
		[Description("Label for the candidate setup being compared.")] string candidateLabel,
		[Description("Optional user-facing status text for the session.")] string? statusText = null,
		[Description("Optional vehicle folder path for attaching a troubleshooting report.")] string? vehicleFolderPath = null,
		[Description("Optional .pc config file name for the troubleshooting report.")] string? configFileName = null)
		=> _toolService.StartTuningSession(vehicleName, baselineLabel, candidateLabel, statusText, vehicleFolderPath, configFileName);

	[McpServerTool(Name = "libreRally_get_tuning_session"), Description("Get the current prompt, progress, and summary for an existing tuning session.")]
	public TuningSessionResult GetTuningSession(
		[Description("Existing tuning session id.")] string sessionId)
		=> _toolService.GetTuningSession(sessionId);

	[McpServerTool(Name = "libreRally_record_tuning_verdict"), Description("Record a better/worse/same verdict for the current tuning prompt and optionally advance.")]
	public TuningSessionResult RecordTuningVerdict(
		[Description("Existing tuning session id.")] string sessionId,
		[Description("One of better, worse, same, or needs_more_time.")] string verdict,
		[Description("Optional note from the driver.")] string? note = null,
		[Description("Whether to move to the next prompt after recording the verdict.")] bool advance = true)
		=> _toolService.RecordTuningVerdict(sessionId, verdict, note, advance);

	[McpServerTool(Name = "libreRally_get_live_game_snapshot"), Description("Fetch a live telemetry and tuning snapshot from the running LibreRally instance.")]
	public LiveTuningBridgeResponse GetLiveGameSnapshot(
		[Description("Optional bridge host. Defaults to 127.0.0.1.")] string? host = null,
		[Description("Optional bridge port. Defaults to 18765.")] int? port = null)
		=> _toolService.GetLiveGameSnapshot(host, port);

	[McpServerTool(Name = "libreRally_apply_live_tuning_patch"), Description("Apply live tuning changes to the running LibreRally instance without reloading the game.")]
	public LiveTuningBridgeResponse ApplyLiveTuningPatch(
		[Description("Optional bridge host. Defaults to 127.0.0.1.")] string? host = null,
		[Description("Optional bridge port. Defaults to 18765.")] int? port = null,
		[Description("Optional tyre model mode: auto, brush_only, or pacejka_only.")] string? tyreModelMode = null,
		[Description("Optional peak friction coefficient.")] float? peakFrictionCoefficient = null,
		[Description("Optional load sensitivity coefficient.")] float? loadSensitivity = null,
		[Description("Optional TCS enabled flag.")] bool? tractionControlEnabled = null,
		[Description("Optional TCS slip ratio target.")] float? tractionControlSlipRatioTarget = null,
		[Description("Optional TCS slip ratio window.")] float? tractionControlSlipRatioWindow = null,
		[Description("Optional TCS minimum activation speed in km/h.")] float? tractionControlMinimumSpeedKmh = null,
		[Description("Optional TCS apply rate.")] float? tractionControlApplyRate = null,
		[Description("Optional TCS release rate.")] float? tractionControlReleaseRate = null,
		[Description("Optional TCS minimum torque scale.")] float? tractionControlMinTorqueScale = null,
		[Description("Optional ABS enabled flag.")] bool? absEnabled = null,
		[Description("Optional ABS slip ratio target.")] float? absSlipRatioTarget = null,
		[Description("Optional ABS slip ratio window.")] float? absSlipRatioWindow = null,
		[Description("Optional ABS minimum brake torque scale.")] float? absMinBrakeTorqueScale = null,
		[Description("Optional ABS minimum activation speed in km/h.")] float? absMinimumSpeedKmh = null,
		[Description("Optional ABS apply rate.")] float? absApplyRate = null,
		[Description("Optional ABS release rate.")] float? absReleaseRate = null,
		[Description("Optional auto-clutch launch rpm.")] float? autoClutchLaunchRpm = null,
		[Description("Optional auto-clutch wheelspin window rpm.")] float? autoClutchWheelspinWindowRpm = null,
		[Description("Optional auto-clutch minimum torque scale.")] float? autoClutchMinTorqueScale = null,
		[Description("Optional automatic shift-up rpm.")] float? shiftUpRpm = null,
		[Description("Optional automatic shift-down rpm.")] float? shiftDownRpm = null)
		=> _toolService.ApplyLiveTuningPatch(
			new LiveTuningPatch
			{
				TyreModelMode = tyreModelMode,
				PeakFrictionCoefficient = peakFrictionCoefficient,
				LoadSensitivity = loadSensitivity,
				TractionControlEnabled = tractionControlEnabled,
				TractionControlSlipRatioTarget = tractionControlSlipRatioTarget,
				TractionControlSlipRatioWindow = tractionControlSlipRatioWindow,
				TractionControlMinimumSpeedKmh = tractionControlMinimumSpeedKmh,
				TractionControlApplyRate = tractionControlApplyRate,
				TractionControlReleaseRate = tractionControlReleaseRate,
				TractionControlMinTorqueScale = tractionControlMinTorqueScale,
				AbsEnabled = absEnabled,
				AbsSlipRatioTarget = absSlipRatioTarget,
				AbsSlipRatioWindow = absSlipRatioWindow,
				AbsMinBrakeTorqueScale = absMinBrakeTorqueScale,
				AbsMinimumSpeedKmh = absMinimumSpeedKmh,
				AbsApplyRate = absApplyRate,
				AbsReleaseRate = absReleaseRate,
				AutoClutchLaunchRpm = autoClutchLaunchRpm,
				AutoClutchWheelspinWindowRpm = autoClutchWheelspinWindowRpm,
				AutoClutchMinTorqueScale = autoClutchMinTorqueScale,
				ShiftUpRpm = shiftUpRpm,
				ShiftDownRpm = shiftDownRpm,
			},
			host,
			port);

	[McpServerTool(Name = "libreRally_reload_live_vehicle"), Description("Reload the currently selected vehicle inside the running LibreRally instance.")]
	public LiveTuningBridgeResponse ReloadLiveVehicle(
		[Description("Optional bridge host. Defaults to 127.0.0.1.")] string? host = null,
		[Description("Optional bridge port. Defaults to 18765.")] int? port = null)
		=> _toolService.ReloadLiveVehicle(host, port);
}
