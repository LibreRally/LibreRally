# Vehicle — AGENTS.md

## OVERVIEW
Vehicle loading, physics simulation, and rendering pipeline. BeamNG `.jbeam` format → BEPU physics → Stride rendering.

## STRUCTURE
```
Vehicle/
├── Content/              Vehicle catalog + discovery
│   ├── BeamNgVehicleCatalog.cs    BeamNG vehicle registry
│   └── IVehicleCatalog.cs         Catalog interface
├── JBeam/                BeamNG .jbeam file parsing
│   ├── JBeamParser.cs             Main parser (1861 lines)
│   ├── JBeamAssembler.cs          Node/beam graph assembly
│   ├── JBeamData.cs               Data model types
│   └── PcConfigLoader.cs          .pc config file loader
├── Physics/              BEPU-based vehicle dynamics
│   ├── VehicleDynamicsSystem.cs   Force application loop
│   ├── VehiclePhysicsBuilder.cs   Physics setup from JBeam data
│   ├── TyreModel.cs               Magic formula / brush model
│   ├── TyreMagicFormulaSet.cs     Formula parameter sets
│   ├── Differential.cs            Drivetrain torque split
│   ├── ChassisBodyRollSystem.cs   Roll/pitch torque
│   ├── SurfaceDefinition.cs       Surface friction types
│   ├── WheelSettings.cs           Wheel config structs
│   └── BreakablePartComponent.cs  Destructible vehicle parts
├── Rendering/            Stride visual mesh + material loading
│   ├── ColladaLoader.cs           .dae mesh loading
│   ├── DtsLoader.cs               Torque .dts mesh format
│   ├── BeamNGMaterialLoader.cs    BeamNG material → Stride
│   ├── BeamNgPaintPaletteResolver.cs  Paint/skin system
│   └── SuspensionVisualKinematicsRig.cs  Visual suspension animation
├── VehicleLoader.cs               Orchestrates JBeam→physics→rendering load
├── RallyCarComponent.cs           High-level car control component
├── VehicleEngineSimulator.cs      Engine RPM, torque, throttle
├── VehicleTransmissionController.cs  Auto/manual transmission
├── VehicleInputState.cs           Input state struct
├── VehicleDefinition.cs           Vehicle description model
├── VehiclePowertrainResolver.cs   Engine + drivetrain config
├── VehicleTyreSpecResolver.cs     Tyre specification lookup
├── VehicleSetupCatalog.cs         Setup/tuning catalog
├── VehicleSetupOverrides.cs       Tuning override model
├── VehicleSetupValueResolver.cs   Setup value computation
├── VehicleModelSourceResolver.cs  Model file path resolution
├── VehicleLoadProgress.cs         Load progress tracking
├── VehicleScript.cs               Script wrapper for vehicle lifecycle
├── VehicleTroubleshootingReportBuilder.cs  Diagnostic report generation
└── IVehicleLoader.cs              Loader interface
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Parse .jbeam files | `JBeam/JBeamParser.cs` | ~1900 lines, recursive JSON structure |
| Build physics from JBeam | `Physics/VehiclePhysicsBuilder.cs` | BEPU rigid bodies, constraints, colliders |
| Apply per-frame forces | `Physics/VehicleDynamicsSystem.cs` | Main physics update loop |
| Tyre grip/slip | `Physics/TyreModel.cs` | Magic formula + brush model |
| Body roll | `Physics/ChassisBodyRollSystem.cs` | Roll/pitch torque from suspension |
| Differential | `Physics/Differential.cs` | Torque distribution across wheels |
| Load vehicles end-to-end | `VehicleLoader.cs` | Orchestrates parse→physics→render |
| Car controls | `RallyCarComponent.cs` | Steering, throttle, brake, handbrake |
| Engine simulation | `VehicleEngineSimulator.cs` | RPM curve, torque calculation |
| Transmission | `VehicleTransmissionController.cs` | Manual/auto gear changes |
| Visual mesh loading | `Rendering/ColladaLoader.cs`, `Rendering/DtsLoader.cs` | .dae and .dts formats |
| Materials | `Rendering/BeamNGMaterialLoader.cs` | BeamNG → Stride material mapping |
| Paint/skins | `Rendering/BeamNgPaintPaletteResolver.cs` | Vehicle paint system |
| Vehicle catalog | `Content/BeamNgVehicleCatalog.cs` | Discovery, enumeration, metadata |
| Driving Input state | `VehicleInputState.cs` | Keyboard/gamepad → vehicle inputs |
| Tuning/setup | `VehicleSetupCatalog.cs`, `VehicleSetupOverrides.cs` | Suspension, gearing, tyre tuning |

## CONVENTIONS
- All code in `LibreRally.Vehicle` namespace (sub-namespaces per folder)
- BEPU physics via `Stride.BepuPhysics` — forces/impulses only, no direct transform writes
- Per-frame physics in `VehicleDynamicsSystem` called from `RallyCarComponent`
- Avoid allocation in physics loops — use `struct`, `readonly`, cached references
- `.jbeam` format: JSON-based vehicle definitions from BeamNG.drive, parsed recursively
- Tyre model uses Pacejka magic formula parameters from beamNG tyre metadata

## ANTI-PATTERNS
- Do NOT use `==` for float comparison — use `MathF.Abs(x - y) <= 1e-6f` (see TyreModel fixes in `a780df8`, `b4781db`)
- Do NOT write transforms directly in physics code — always use forces/impulses through BEPU
- Do NOT allocate in frame loops; cache entity references in `Start()`, use pre-allocated fixed-size arrays
- Do NOT mix Stride physics with direct BEPU access — use `Stride.BepuPhysics` wrappers
- Never edit `.jbeam` GUIDs manually — they're JSON structure references
- Guard against NaN/Inf in force vectors — use `SanitizeForce` / `SafeNormalize` / `float.IsFinite`
- Guard against division-by-zero — use `MathF.Max(denominator, epsilon)` pattern
- Prefer `readonly record struct` for per-wheel/tyre state data; pre-allocate arrays for wheel-count-based data
