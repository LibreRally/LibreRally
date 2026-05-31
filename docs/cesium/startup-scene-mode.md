# Cesium startup scene mode

LibreRally includes a code-first Cesium startup path behind `LIBRERALLY_SCENE_MODE=cesium`.

## Default behavior

- no environment variables: boots the existing local rally sandbox scene
- `LIBRERALLY_SCENE_MODE=cesium`: boots the Stride-native Cesium scene bootstrap
- `LIBRERALLY_CESIUM_STAGE` selects the Cesium preset:
  - `sample` (default): Cesium's public `TilesetWithRequestVolume` sample
  - `url`: load a contributor-supplied `tileset.json` URL
  - `ion`: load a Cesium ion asset

## Supported Cesium environment variables

| Variable | Purpose |
| --- | --- |
| `LIBRERALLY_SCENE_MODE` | Set to `cesium` to boot the Cesium scene path. |
| `LIBRERALLY_CESIUM_STAGE` | Select `sample`, `url`, or `ion`. |
| `LIBRERALLY_CESIUM_URL` | Overrides the tileset URL for the `url` preset. |
| `LIBRERALLY_CESIUM_ION_ASSET_ID` | Cesium ion asset id for the `ion` preset. |
| `LIBRERALLY_CESIUM_ION_ACCESS_TOKEN` | Cesium ion access token for the `ion` preset. |
| `LIBRERALLY_CESIUM_ORIGIN_LONGITUDE` | Override the local georeference longitude. |
| `LIBRERALLY_CESIUM_ORIGIN_LATITUDE` | Override the local georeference latitude. |
| `LIBRERALLY_CESIUM_ORIGIN_HEIGHT` | Override the local georeference height in meters. |
| `LIBRERALLY_CESIUM_MAX_SCREEN_SPACE_ERROR` | Override Cesium Native screen-space error budget. |
| `LIBRERALLY_CESIUM_MAX_TILE_LOADS` | Override maximum simultaneous tile loads. |

`LIBRERALLY_CESIUM_ORIGIN_LONGITUDE` and `LIBRERALLY_CESIUM_ORIGIN_LATITUDE` use the standard geographic ordering:

- **longitude**: east/west, range `-180` to `180`
- **latitude**: north/south, range `-90` to `90`

For example, central Melbourne is approximately:

```powershell
$env:LIBRERALLY_CESIUM_ORIGIN_LONGITUDE = "144.9673305"
$env:LIBRERALLY_CESIUM_ORIGIN_LATITUDE = "-37.8175087"
```

## Examples

Run the public sample:

```powershell
$env:LIBRERALLY_SCENE_MODE = "cesium"
dotnet run --project LibreRally.Windows
```

Run a custom tileset URL:

```powershell
$env:LIBRERALLY_SCENE_MODE = "cesium"
$env:LIBRERALLY_CESIUM_STAGE = "url"
$env:LIBRERALLY_CESIUM_URL = "https://example.test/tileset.json"
dotnet run --project LibreRally.Windows
```

Run a Cesium ion asset:

```powershell
$env:LIBRERALLY_SCENE_MODE = "cesium"
$env:LIBRERALLY_CESIUM_STAGE = "ion"
$env:LIBRERALLY_CESIUM_ION_ASSET_ID = "1"
$env:LIBRERALLY_CESIUM_ION_ACCESS_TOKEN = "<token>"
dotnet run --project LibreRally.Windows
```

## Cesium camera controls

- The Cesium startup camera now begins aimed at the local georeference origin.
- Use `W`, `A`, `S`, `D`, `Q`, and `E` to fly.
- Hold the **right mouse button** and move the mouse to look around.

## Current scope

This startup path is intended to validate Stride-native Cesium scene assembly, stage selection, and dataset loading. For the current MVP it also enables coarse, camera-local tile collision proxies for nearby streamed content, but it does **not** yet attempt full rally vehicle-on-globe physics integration.
