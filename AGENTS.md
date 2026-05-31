# LibreRally — Project Knowledge Base

**Generated:** 2026-05-04
**Commit:** `0ed51cd`
**Branch:** `cesium`

## OVERVIEW
Open-source rally racing simulation. Stride 4.3 game engine, C# on .NET 10, BEPU physics. Loads BeamNG.drive `.jbeam` vehicles. Code-first scene setup with optional Cesium 3D globe integration.

## STRUCTURE
```
LibreRally.sln                       6 projects
├── LibreRally/                      Core library — all game logic
│   ├── Assets/                      Stride assets (.sdscene, .sdmat, .sdtex)
│   ├── Camera/                      Rally camera script
│   ├── HUD/                         Myra.Stride UI overlays → HUD/AGENTS.md
│   ├── Race/                        Checkpoints, timer, controller
│   ├── Resources/                   Raw files (BeamNG vehicles, tracks, fonts)
│   ├── Telemetry/                   OutGauge/OutSim UDP + LiveTuning TCP bridge
│   ├── Track/                       (empty, future track system)
│   ├── Vehicle/                     Vehicle loading, physics, rendering → Vehicle/AGENTS.md
│   ├── VehicleSpawner.cs            Primary game startup script (orchestrates scene)
│   ├── CodeFirstSceneFactory.cs     Scene construction + LibreRallyGame class
│   └── CesiumTilesetScript.cs       Cesium 3D terrain integration
├── LibreRally.Windows/              Windows entry point (startup project)
├── LibreRally.Linux/                Linux entry point
├── LibreRally.macOS/                macOS entry point
├── LibreRally.Tests/                xUnit tests → Tests/AGENTS.md
└── LibreRally.McpServer/            MCP protocol server (AI tooling bridge)
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| Game entry / boot | `LibreRally.Windows/LibreRallyApp.cs` → `LibreRally/CodeFirstSceneFactory.cs` | `LibreRallyGame : Game` |
| Scene startup | `LibreRally/VehicleSpawner.cs` | SyncScript — vehicles, HUD, telemetry init |
| Vehicle loading | `LibreRally/Vehicle/VehicleLoader.cs` | BeamNG `.jbeam` → BEPU physics |
| Vehicle physics | `LibreRally/Vehicle/Physics/VehicleDynamicsSystem.cs` | Force application loop |
| Tyre model | `LibreRally/Vehicle/Physics/TyreModel.cs` | Magic formula, slip angle/ratio |
| JBeam parsing | `LibreRally/Vehicle/JBeam/JBeamParser.cs` | BeamNG vehicle file format |
| HUD / UI | `LibreRally/HUD/` | Myra.Stride overlays |
| Telemetry | `LibreRally/Telemetry/VehicleLiveTuningBridgeServer.cs` | TCP on 127.0.0.1:18765 |
| Camera | `LibreRally/Camera/RallyCameraScript.cs` | |
| Race logic | `LibreRally/Race/RaceController.cs` | Checkpoints, timing |
| Cesium terrain | `LibreRally/CesiumTilesetScript.cs` | 3D Tiles streaming |
| Tests | `LibreRally.Tests/` | xUnit, covers physics/loading/UI/telemetry |
| MCP tools | `LibreRally.McpServer/LibreRallyTools.cs` | AI-accessible game controls |
| CI/CD | `.github/workflows/dotnet-build.yml` | Matrix: win-x64, win-arm64, linux-x64, linux-arm64 |

## CONVENTIONS
- **Tabs**, not spaces. `indent_style = tab`, `tab_width = 4`
- **Explicit types** — `var` is off (`csharp_style_var_for_built_in_types = false`, etc.)
- **Block-scoped namespaces** (`namespace Foo { }`), not file-scoped
- **Allman braces** (`csharp_new_line_before_open_brace = all`)
- **Usings outside namespace** (`csharp_using_directive_placement = outside_namespace`)
- **System usings NOT sorted first** (`dotnet_sort_system_directives_first = false`)
- **No `this.` qualification** (all `dotnet_style_qualification_for_* = false`)
- **`string.Empty`** over `""` (ReSharper: `resharper_empty_string = string_empty`)
- **No space after cast** (`csharp_space_after_cast = false`)
- **CRLF** line endings; insert final newline (ReSharper override)
- **`Nullable=enable`** in core, tests, MCP server; disabled in platform projects
- **Comments**: capitalized, end with punctuation, required at `protected`+ visibility (CommentSense)
- **`AllowUnsafeBlocks=true`** in core + tests (BEPU physics interop)
- **Namespace matches folder** (`dotnet_style_namespace_match_folder = true`)
- **Modifier order**: `public,private,protected,internal,static,extern,new,virtual,abstract,sealed,override,readonly,unsafe,volatile,async` — `readonly` after `override` (non-standard)

## ANTI-PATTERNS
- Do NOT use `==` for float comparison — use `MathF.Abs(x - y) <= threshold`
- Do NOT allocate in `Update()` / frame loops — cache references in `Start()`, use pre-allocated arrays
- Do NOT use `var` — explicit types required
- Do NOT use `this.` qualification
- Do NOT use `""` — prefer `string.Empty`
- Do NOT use file-scoped namespaces — `namespace Foo { }` only
- Do NOT sort system usings first
- Do NOT manually edit Stride asset GUIDs (`.sdscene`, `.sdmat`, etc.)
- Do NOT multiply mouse delta by deltaTime — mouse deltas are frame-rate independent
- Do NOT write transforms directly — always use forces/impulses through BEPU
- Do NOT check in `*.sdpkg.user` files

## UNIQUE STYLES
- Core library dual-targets `net10.0;net10.0-windows`
- Platform projects redirect output to `../Bin/{Platform}/$(Configuration)/`
- `DisableFastUpToDateCheck=true` on all platform projects (Stride asset pipeline)
- `AppendTargetFrameworkToOutputPath=false` on all platform projects
- `InternalsVisibleTo` grants: core → tests + MCP server; MCP server → tests
- Game entry via `new LibreRallyGame().Run()` in platform projects — class defined in core lib
- Scene mode selected by env var (`LIBRERALLY_CESIUM_STAGE`, etc.)
- Stride scripts: subclass `SyncScript` (Start/Update) or `AsyncScript` (async Task Execute)
- deltaTime from `(float)Game.UpdateTime.Elapsed.TotalSeconds`
- Mouse deltas are frame-rate independent — do NOT multiply by deltaTime
- Prefer `struct`, `readonly`, cached references; no per-frame allocations
- Physics via forces/impulses through BEPU, not direct transform writes

## COMMANDS
```bash
# Build
dotnet build LibreRally.sln
dotnet build LibreRally.sln -c Release

# Run (Windows — primary target)
dotnet run --project LibreRally.Windows

# Run (Linux / macOS)
dotnet run --project LibreRally.Linux
dotnet run --project LibreRally.macOS

# Test
dotnet test LibreRally.Tests

# Publish (CI-style)
dotnet publish LibreRally.Windows/LibreRally.Windows.csproj -c Release -r win-x64 --self-contained false

# Documentation
dotnet tool install --global docfx --version 2.78.5
docfx docfx.json

# MCP server
dotnet run --project LibreRally.McpServer
```

## NOTES
- BeamNG vehicle catalog in `LibreRally/Resources/BeamNG Vehicles/` — only `basic_car/` is tracked in git; other vehicles are gitignored
- Cesium integration requires environment variables (see `LibreRally.Windows/Properties/launchSettings.json`)
- `LibreRally.sdpkg` is a Stride asset package, not a NuGet package
- `*.sdpkg.user` files are machine-specific and gitignored
- MCP server communicates with the game via TCP live-tuning bridge, not in-process
- Qodana static analysis runs in Docker (`jetbrains/qodana-cdnet:2026.1-eap`)
- ReSharper/Rider custom dictionary includes "jbeam"
- CI builds with `--tl:off` (terminal logger off) due to Stride asset compiler compatibility
- No `global.json` — SDK version managed by CI workflow
