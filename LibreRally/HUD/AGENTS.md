# HUD — AGENTS.md

## OVERVIEW
Myra.Stride UI overlay system. All in-game 2D interfaces: driving HUD, pause menu, vehicle/track selection, setup tuning, telemetry controls, physics calibration.

## STRUCTURE
```
HUD/
├── DrivingHudOverlay.cs           Speed, RPM, gear, surface, telemetry
├── PauseMenuOverlay.cs            Pause/resume, reset, menu navigation
├── VehicleSelectionOverlay.cs     Vehicle catalog browser
├── TrackSelectionOverlay.cs       Track/world selection grid
├── SetupUiShellOverlay.cs         Garage setup/tuning shell
├── SetupUiShellModel.cs           Tuning data model + logic
├── TuningSessionModel.cs          In-session tuning state
├── PhysicsCalibrationOverlay.cs   Real-time physics parameter tuning
├── TelemetryOverlay.cs            Telemetry send/receive controls
├── LoadingOverlay.cs              Vehicle load progress display
├── MenuScrollHelper.cs            Scroll input handling utilities
└── GamePadPromptWidgets.cs        Gamepad button prompt icons
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Speed/RPM/gear display | `DrivingHudOverlay.cs` | Main in-game overlay |
| Pause menu | `PauseMenuOverlay.cs` | Resume, garage, vehicle/track/telemetry/physics select |
| Vehicle selection | `VehicleSelectionOverlay.cs` | BeamNG vehicle catalog browser |
| Track selection | `TrackSelectionOverlay.cs` | Track/scene selection |
| Setup / tuning | `SetupUiShellOverlay.cs` + `SetupUiShellModel.cs` | Suspension, gearing, tyre tuning |
| Live tuning | `TuningSessionModel.cs` | TCP bridge tuning state |
| Physics calibration | `PhysicsCalibrationOverlay.cs` | Real-time physics parameter sliders (1680 lines) |
| Telemetry controls | `TelemetryOverlay.cs` | OutGauge/OutSim toggle, connect, disconnect |
| Loading screen | `LoadingOverlay.cs` | Vehicle load progress |
| Gamepad prompts | `GamePadPromptWidgets.cs` | Xbox button icons |
| Scroll input | `MenuScrollHelper.cs` | D-pad/stick scroll helpers |

## CONVENTIONS
- All code in `LibreRally.HUD` namespace
- UI framework: **Myra.Stride** (v1.5.11), not Stride.UI
- Overlays are `SyncScript` subclasses attached to scene entities in `VehicleSpawner.Start()`
- Overlay visibility toggled via `Visible` / `Enabled` properties on the parent entity
- Pause/resume managed through `Game.IsPaused` toggle
- Gamepad input handled via `InputManager` with Xbox Series button prompts
- Tuning overrides sent through `VehicleLiveTuningBridgeServer` (TCP on 127.0.0.1:18765)
- Scene-switching orchestrated via `VehicleSpawner` + `CodeFirstSceneFactory`

## ANTI-PATTERNS
- Do NOT use Stride.UI — use Myra.Stride for all 2D UI
- Do NOT create overlays in scene files — they're created code-first in `VehicleSpawner.Start()`
- Do NOT keep overlays active when hidden — toggle `Enabled` to stop rendering/input
