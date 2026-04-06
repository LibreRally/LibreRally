# LibreRally — Copilot Instructions

## Project Overview

LibreRally is a rally racing game built with the [Stride game engine](https://www.stride3d.net/) (v4.3.0.2507) in C#, targeting .NET 10 on Windows.

## Solution Structure

```
LibreRally.sln
├── LibreRally/           # Core game library — scripts, assets, resources
│   ├── Assets/           # Stride assets (.sdscene, .sdmat, .sdpromodel, .sdsky, .sdtex)
│   ├── Resources/        # Raw resource files (textures, etc.)
│   └── *.cs              # Game scripts (SyncScript / AsyncScript subclasses)
└── LibreRally.Windows/   # Windows platform entry point
    └── LibreRallyApp.cs  # Top-level program: `new Game().Run()`
```

Build output goes to `Bin\Windows\<Configuration>\`.

## Build & Run

```bash
# Build the solution
dotnet build LibreRally.sln

# Run the game (Windows)
dotnet run --project LibreRally.Windows

# Release build
dotnet build LibreRally.sln -c Release
```

The startup project is `LibreRally.Windows`. The `LibreRally` project is a library referenced by it.

## Stride Engine Conventions

### Scripts
- Game logic scripts live in `LibreRally/` and must subclass `SyncScript` or `AsyncScript` from `Stride.Engine`.
- `SyncScript` requires overriding `Update()` (called every frame); optionally override `Start()`.
- `AsyncScript` uses `async Task Execute()` for coroutine-style logic.
- Scripts are attached to scene entities and serialized in `.sdscene` files; public properties appear in the Stride Editor.

### Assets
- All game assets are in `LibreRally/Assets/` and use YAML-based Stride formats.
- The main scene is `Assets/MainScene.sdscene`; it is the default scene in `GameSettings.sdgamesettings`.
- Asset references use the pattern `<guid>:<AssetName>` (e.g., `e53b226b-...:Sphere`).
- Do not hand-edit GUIDs in asset files; let the Stride Editor manage them.

### Namespace
All game code uses the `LibreRally` namespace.

### Input handling
- Use `Input.IsKeyDown()` / `Input.IsMouseButtonDown()` inside `Update()` for continuous input.
- Always multiply movement/rotation by `deltaTime` (`(float)Game.UpdateTime.Elapsed.TotalSeconds`) to stay frame-rate independent. Mouse/touch deltas are already frame-rate independent and must **not** be multiplied by delta time.
- Normalize direction vectors before scaling by speed when combining multiple axes.

### Graphics settings
- Default resolution: 1280×720, HDR rendering, DirectX Feature Level 11.2, Linear colour space.
- The graphics compositor is defined in `Assets/GraphicsCompositor.sdgfxcomp`.

### Package metadata
- `LibreRally.sdpkg` declares the Stride package (asset folders, resource folders). It is not a NuGet package.
- `*.sdpkg.user` files are machine-local and excluded from version control.
