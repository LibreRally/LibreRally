# Tests — AGENTS.md

## OVERVIEW
xUnit test suite (v2.9.3) covering vehicle physics, JBeam loading, rendering, telemetry protocols, Cesium integration, UI models, and MCP tools.

## STRUCTURE
```
LibreRally.Tests/
├── TyreModelTests.cs                              Pacejka/brush tyre model (largest: 1618 lines)
├── RallyCarComponentControlTests.cs               Steering, throttle, drivetrain controls
├── ChassisBodyRollSystemTests.cs                  Roll/pitch torque computation
├── VehicleDynamicsSystemAccelerationTests.cs      Acceleration behaviour
├── VehiclePhysicsBuilderTests.cs                  JBeam → BEPU physics construction
├── DifferentialAndPowertrainTests.cs              Torque split + engine/drivetrain
├── BasicCarPipelineTests.cs                       Full vehicle assembly integration
├── JBeamParserTests.cs                            .jbeam format parsing
├── ColladaLoaderTests.cs                          .dae mesh loading
├── DtsLoaderTests.cs                              .dts mesh format loading
├── VehicleLoaderTests.cs                          End-to-end vehicle loading
├── BeamNgMaterialLoaderTests.cs                   BeamNG material → Stride
├── BeamNgPaintPaletteResolverTests.cs             Paint/skin system
├── BeamNgVehicleCatalogTests.cs                   Vehicle discovery/enumeration
├── BeamNgBrakeControlMetadataTests.cs             Brake metadata parsing
├── BeamNgTyreAndTractionMetadataTests.cs          Tyre traction metadata
├── OutGaugeProtocolTests.cs                       OutGauge UDP binary encoding
├── OutSimProtocolTests.cs                         OutSim UDP binary encoding
├── VehicleLiveTuningBridgeServerTests.cs          TCP live tuning bridge
├── TuningSessionModelTests.cs                     In-session tuning state
├── SetupUiShellModelTests.cs                      Garage tuning model
├── MenuScrollHelperTests.cs                       Scroll input logic
├── VehicleSpawnerInputTests.cs                    Input handler state machine
├── VehicleSpawnerPauseMenuTests.cs                Pause menu state transitions
├── VehicleSetupValueResolverTests.cs              Setup value computation
├── VehicleTroubleshootingReportBuilderTests.cs    Diagnostic reports
├── CodeFirstSceneFactoryTests.cs                  Scene factory + Cesium stage config
├── CesiumAnchorComponentTests.cs                  Cesium anchor behaviour
├── CesiumLocalGeoreferenceTests.cs                Georeference transforms
├── CesiumTilesetGeometryTests.cs                  3D Tiles geometry parsing
├── CesiumTilesetScriptTests.cs                    Tileset streaming logic
├── CesiumTextureDecoderTests.cs                   Terrain texture decoding
└── LibreRallyToolServiceTests.cs                  MCP tool service (only file with manual Fake)
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Tyre physics tests | `TyreModelTests.cs` | Magic formula lookup, slip ratio/angle, longitudinal/lateral |
| Roll/pitch tests | `ChassisBodyRollSystemTests.cs` | Has inlined mirror methods (ComputeRollTorque, etc.) |
| Full vehicle pipeline | `BasicCarPipelineTests.cs` | pc → jbeam → physics → rendering integration |
| Protocol encoding | `OutGaugeProtocolTests.cs` | Binary packet layout verification |
| Model tests | `SetupUiShellModelTests.cs`, `TuningSessionModelTests.cs` | Pure logic tests, no UI framework dependency |
| MCP + Fake | `LibreRallyToolServiceTests.cs` | Only test using manual Fake pattern |

## CONVENTIONS
- **Framework**: xUnit 2.9.3 + `Microsoft.NET.Test.Sdk` 17.14.1 — no mocking frameworks
- **No Moq/NSubstitute**: Use manual `private sealed class FakeX : IX` when stubbing needed
- **File/class naming**: `{Target}Tests.cs` / `{Target}Tests` in namespace `LibreRally.Tests`
- **Method naming**: `{Method}_{Scenario}_Expects{Result}` or `_{Scenario}_Produces{Result}`
- **Attributes**: Prefer `[Fact]`; use `[Theory]`/`[InlineData]` only for multi-case parameterized tests
- **Documentation**: Every test class and test method MUST have `/// <summary>` XML doc
- **No test base classes / fixtures**: Tests are standalone; inlined helpers preferred over shared infrastructure
- **Resource access**: Use `GetVehicleFolder()` pattern — walk up from `AppContext.BaseDirectory` to find `LibreRally.sln`, then descend to `LibreRally/Resources/...`
- **Physics test helpers**: Private static methods that mirror production logic (e.g., `ComputeRollTorque` in `ChassisBodyRollSystemTests`)
- `global using Xunit` in `.csproj` via `<Using Include="Xunit" />`
- **`AllowUnsafeBlocks=true`** for Cesium/native interop tests
- **Target**: `net10.0-windows` with `EnableWindowsTargeting=true`
- References both `LibreRally.csproj` and `LibreRally.McpServer.csproj` (InternalsVisibleTo grant on MCP server)

## ANTI-PATTERNS
- Do NOT add NuGet mocking frameworks — use manual Fake classes instead
- Do NOT create shared fixture base classes — tests are self-contained by design
- Do NOT skip XML doc on tests — required for both class and method level
- Do NOT use `[ClassData]` or `[MemberData]` — use `[Theory]`/`[InlineData]` for parameterized tests
