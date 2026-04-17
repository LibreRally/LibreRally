# LibreRally — Copilot Instructions

## Project Overview

LibreRally is an open-source rally racing simulation built with the **Stride game engine** (v4.3+) in **C#**, targeting **.NET 10**.

The project focuses on **realistic rally vehicle behaviour** on multiple surfaces (gravel, tarmac, snow) and aims for believable vehicle dynamics rather than arcade handling.

Copilot should prioritise **clean architecture, deterministic physics behaviour, and maintainable gameplay systems.**

---

# Solution Structure

```
LibreRally.sln
├── LibreRally/           # Core game library — scripts, assets, resources
│   ├── Assets/           # Stride assets (.sdscene, .sdmat, .sdpromodel, .sdsky, .sdtex)
│   ├── Resources/        # Raw resource files (textures, data files)
│   └── *.cs              # Game scripts (SyncScript / AsyncScript subclasses)
└── LibreRally.Windows/   # Windows platform entry point
    └── LibreRallyApp.cs
```

Build output goes to:

```
Bin/Windows/<Configuration>/
```

---

# Build & Run

```bash
dotnet build LibreRally.sln
dotnet run --project LibreRally.Windows
dotnet build LibreRally.sln -c Release
```

Startup project: **LibreRally.Windows**

The **LibreRally project is a library** referenced by the platform project.

---

# Stride Engine Conventions

## Scripts

Gameplay code lives in the `LibreRally` project and uses Stride scripts.

Scripts must subclass:

```
SyncScript
AsyncScript
```

### SyncScript

Used for gameplay systems that update every frame.

Override:

```
Start()
Update()
```

### AsyncScript

Used for coroutine-style behaviour.

Override:

```
async Task Execute()
```

Scripts are attached to entities in `.sdscene` files and public properties appear in the Stride editor.

---

# Input Handling

Use Stride input APIs:

```
Input.IsKeyDown()
Input.IsMouseButtonDown()
Input.MouseDelta
```

Guidelines:

* Multiply movement and physics by `deltaTime`
* Obtain deltaTime from:

```
(float)Game.UpdateTime.Elapsed.TotalSeconds
```

* **Mouse deltas are already frame-rate independent and must NOT be scaled by deltaTime**

* When combining axis input, **normalize direction vectors** before applying speed.

---

# Asset System

All assets are located in:

```
LibreRally/Assets/
```

Stride assets use YAML-based formats:

```
.sdscene
.sdmat
.sdpromodel
.sdsky
.sdtex
```

Main scene:

```
Assets/MainScene.sdscene
```

Asset references follow:

```
<guid>:<AssetName>
```

Example:

```
e53b226b-...:Sphere
```

Do **not manually edit GUIDs** — the Stride editor manages them.

---

# Namespace

All code must use the namespace:

```
LibreRally
```

---

# Graphics Settings

Default rendering configuration:

* 1280×720 resolution
* HDR rendering
* DirectX Feature Level 11.2
* Linear colour space

Graphics compositor:

```
Assets/GraphicsCompositor.sdgfxcomp
```

---

# Gameplay Architecture Guidelines

When generating gameplay code, prefer **component-based architecture**.

Typical entity structure:

```
VehicleEntity
 ├── VehicleController
 ├── SuspensionComponent
 ├── WheelComponent
 ├── EngineComponent
 ├── DifferentialComponent
 └── VehicleVisualComponent
```

Each gameplay system should exist as an isolated **script or component** rather than monolithic classes.

---

# Vehicle Physics Guidelines

LibreRally aims for **believable rally vehicle dynamics**.

Copilot should prefer **physically motivated systems** rather than arcade approximations.

Key concepts include:

* tyre slip angle
* weight transfer
* suspension compression
* chassis roll and pitch
* drivetrain torque distribution

### Weight Transfer

Weight shifts during acceleration, braking and cornering.

Important variables:

```
vehicleMass
centreOfGravityHeight
wheelBase
trackWidth
```

### Body Roll

Body roll results from suspension compression differences.

Typical approximation:

```
rollAngle = (leftCompression - rightCompression) * rollStiffness
```

This generates torque applied to the chassis.

### Tyre Behaviour

Tyre forces depend on:

* slip angle
* slip ratio
* vertical load
* surface type

Simplified tyre models may be used initially, but systems should be designed so that **more advanced tyre models can be integrated later**.

---

# Performance Guidelines

Game systems should avoid:

* per-frame allocations
* unnecessary LINQ
* excessive object creation

Prefer:

```
struct
readonly
cached references
```

Physics code should remain **deterministic where possible**.

---

# Package Metadata

`LibreRally.sdpkg` defines the Stride asset package.

It is **not a NuGet package**.

Machine-specific files:

```
*.sdpkg.user
```

must not be committed to version control.
