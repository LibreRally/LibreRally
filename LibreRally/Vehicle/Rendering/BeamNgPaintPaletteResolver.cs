using System;
using System.Collections.Generic;
using System.IO;
using System.Text;
using System.Text.Json;
using LibreRally.Vehicle.Content;
using Stride.Core.Mathematics;

namespace LibreRally.Vehicle.Rendering;

internal readonly record struct BeamNgPaintColor(
    string Name,
    Color4 BaseColor);

internal readonly record struct BeamNgPaintPalette(
    BeamNgPaintColor Paint1,
    BeamNgPaintColor Paint2,
    BeamNgPaintColor Paint3);

internal static class BeamNgPaintPaletteResolver
{
    private static readonly JsonDocumentOptions ParseOptions = new()
    {
        AllowTrailingCommas = true,
        CommentHandling = JsonCommentHandling.Skip,
    };

    private static readonly Lazy<IReadOnlyDictionary<string, Color4>> SharedPaintColors = new(LoadSharedPaintColors);

    private static readonly IReadOnlyDictionary<string, Color4> KnownFallbackPaintColors =
        new Dictionary<string, Color4>(StringComparer.OrdinalIgnoreCase)
        {
            ["Almond Pearl"] = new Color4(0.663f, 0.594f, 0.533f, 1.2f),
            ["Anthracite"] = new Color4(0.22f, 0.22f, 0.24f, 1.2f),
            ["Aqua Steel"] = new Color4(0.18f, 0.48f, 0.5f, 1.2f),
            ["Black Diamond"] = new Color4(0.01f, 0.01f, 0.01f, 1.5f),
            ["Chrome"] = new Color4(0.9f, 0.9f, 0.92f, 1.2f),
            ["Dusk"] = new Color4(0.06f, 0.04f, 0.16f, 1.2f),
            ["Electric Blue"] = new Color4(0.1f, 0.4f, 0.8f, 1.2f),
            ["Fire Red"] = new Color4(0.689f, 0.118f, 0.118f, 0.85f),
            ["Matte Black"] = new Color4(0.01f, 0.01f, 0.01f, 2f),
            ["Obsidian Gray"] = new Color4(0.23f, 0.23f, 0.23f, 1.2f),
            ["Palladium Gray"] = new Color4(0.623f, 0.623f, 0.623f, 1.2f),
            ["Pine Shadow"] = new Color4(0.08f, 0.14f, 0.09f, 1.2f),
            ["Silica White"] = new Color4(0.83f, 0.83f, 0.83f, 1.2f),
            ["Submarine Blue"] = new Color4(0.01f, 0.06f, 0.16f, 1.2f),
            ["Sunny Flare"] = new Color4(0.95f, 0.62f, 0.08f, 1.2f),
            ["Tangerine Rush"] = new Color4(0.96f, 0.36f, 0.05f, 1.2f),
            ["Technetium"] = new Color4(0.25f, 0.35f, 0.41f, 1.2f),
            ["Yellowjacket"] = new Color4(0.92f, 0.78f, 0.12f, 1.2f),
        };

    public static BeamNgPaintPalette? LoadDefaultPalette(string vehicleFolder, string? configFilePath)
    {
        if (string.IsNullOrWhiteSpace(vehicleFolder) || !Directory.Exists(vehicleFolder))
        {
            return null;
        }

        var vehiclePaintColors = LoadVehiclePaintColors(vehicleFolder);
        var sharedPaintColors = SharedPaintColors.Value;
        var paintNames = LoadPaintNames(vehicleFolder, configFilePath);
        if (paintNames == null)
        {
            return null;
        }

        return new BeamNgPaintPalette(
            ResolvePaintColor(paintNames.Value.Paint1, vehiclePaintColors, sharedPaintColors),
            ResolvePaintColor(paintNames.Value.Paint2, vehiclePaintColors, sharedPaintColors),
            ResolvePaintColor(paintNames.Value.Paint3, vehiclePaintColors, sharedPaintColors));
    }

    private static BeamNgPaintColor ResolvePaintColor(
        string paintName,
        IReadOnlyDictionary<string, Color4> vehiclePaintColors,
        IReadOnlyDictionary<string, Color4> sharedPaintColors)
    {
        if (vehiclePaintColors.TryGetValue(paintName, out var vehicleColor))
        {
            return new BeamNgPaintColor(paintName, vehicleColor);
        }

        if (sharedPaintColors.TryGetValue(paintName, out var sharedColor))
        {
            return new BeamNgPaintColor(paintName, sharedColor);
        }

        if (KnownFallbackPaintColors.TryGetValue(paintName, out var fallbackColor))
        {
            return new BeamNgPaintColor(paintName, fallbackColor);
        }

        return new BeamNgPaintColor(paintName, Color4.White);
    }

    private static (string Paint1, string Paint2, string Paint3)? LoadPaintNames(string vehicleFolder, string? configFilePath)
    {
        string? configInfoPath = null;
        if (!string.IsNullOrWhiteSpace(configFilePath))
        {
            var configBaseName = Path.GetFileNameWithoutExtension(configFilePath);
            if (!string.IsNullOrWhiteSpace(configBaseName))
            {
                var candidatePath = Path.Combine(vehicleFolder, $"info_{configBaseName}.json");
                if (File.Exists(candidatePath))
                {
                    configInfoPath = candidatePath;
                }
            }
        }

        if (TryLoadPaintNamesFromInfoFile(configInfoPath, out var configPaintNames))
        {
            return configPaintNames;
        }

        var vehicleInfoPath = Path.Combine(vehicleFolder, "info.json");
        if (TryLoadPaintNamesFromInfoFile(vehicleInfoPath, out var vehiclePaintNames))
        {
            return vehiclePaintNames;
        }

        return null;
    }

    private static bool TryLoadPaintNamesFromInfoFile(string? infoFilePath, out (string Paint1, string Paint2, string Paint3) paintNames)
    {
        paintNames = default;
        if (string.IsNullOrWhiteSpace(infoFilePath) || !File.Exists(infoFilePath))
        {
            return false;
        }

        using var document = ParseInfoDocument(infoFilePath);
        var root = document.RootElement;

        if (root.TryGetProperty("defaultMultiPaintSetup", out var multiPaintSetup) &&
            multiPaintSetup.ValueKind == JsonValueKind.Object)
        {
            var paint1 = GetOptionalStringProperty(multiPaintSetup, "paint1");
            if (!string.IsNullOrWhiteSpace(paint1))
            {
                var paint2 = GetOptionalStringProperty(multiPaintSetup, "paint2") ?? paint1;
                var paint3 = GetOptionalStringProperty(multiPaintSetup, "paint3") ?? paint2;
                paintNames = (paint1, paint2, paint3);
                return true;
            }
        }

        var defaultPaint1 = GetOptionalStringProperty(root, "defaultPaintName1");
        if (string.IsNullOrWhiteSpace(defaultPaint1))
        {
            return false;
        }

        var defaultPaint2 = GetOptionalStringProperty(root, "defaultPaintName2") ?? defaultPaint1;
        var defaultPaint3 = GetOptionalStringProperty(root, "defaultPaintName3") ?? defaultPaint2;
        paintNames = (defaultPaint1, defaultPaint2, defaultPaint3);
        return true;
    }

    private static Dictionary<string, Color4> LoadVehiclePaintColors(string vehicleFolder)
    {
        var paintColors = new Dictionary<string, Color4>(StringComparer.OrdinalIgnoreCase);
        var infoPath = Path.Combine(vehicleFolder, "info.json");
        if (!File.Exists(infoPath))
        {
            return paintColors;
        }

        using var document = ParseInfoDocument(infoPath);
        if (!document.RootElement.TryGetProperty("paints", out var paintsElement) ||
            paintsElement.ValueKind != JsonValueKind.Object)
        {
            return paintColors;
        }

        foreach (var paintProperty in paintsElement.EnumerateObject())
        {
            if (!TryGetBaseColor(paintProperty.Value, out var baseColor))
            {
                continue;
            }

            paintColors[paintProperty.Name] = baseColor;
        }

        return paintColors;
    }

    private static JsonDocument ParseInfoDocument(string infoFilePath)
    {
        var raw = File.ReadAllText(infoFilePath);
        var normalized = NormalizeLenientJson(raw);
        return JsonDocument.Parse(normalized, ParseOptions);
    }

    private static string NormalizeLenientJson(string input)
    {
        var sb = new StringBuilder(input.Length + input.Length / 8);
        var index = 0;
        var afterValue = false;

        void MaybeComma()
        {
            if (!afterValue)
            {
                return;
            }

            sb.Append(',');
            afterValue = false;
        }

        while (index < input.Length)
        {
            var c = input[index];

            if (char.IsWhiteSpace(c))
            {
                sb.Append(c);
                index++;
                continue;
            }

            if (c == '/' && index + 1 < input.Length && input[index + 1] == '*')
            {
                index += 2;
                while (index < input.Length - 1 && !(input[index] == '*' && input[index + 1] == '/'))
                {
                    index++;
                }

                if (index < input.Length - 1)
                {
                    index += 2;
                }

                continue;
            }

            if (c == '/' && index + 1 < input.Length && input[index + 1] == '/')
            {
                while (index < input.Length && input[index] != '\n')
                {
                    index++;
                }

                if (index < input.Length)
                {
                    sb.Append('\n');
                    index++;
                }

                continue;
            }

            if (c == '"')
            {
                MaybeComma();
                sb.Append(c);
                index++;

                var escaped = false;
                while (index < input.Length)
                {
                    var stringChar = input[index];
                    sb.Append(stringChar);
                    index++;

                    if (escaped)
                    {
                        escaped = false;
                        continue;
                    }

                    if (stringChar == '\\')
                    {
                        escaped = true;
                        continue;
                    }

                    if (stringChar == '"')
                    {
                        break;
                    }
                }

                afterValue = true;
                continue;
            }

            if (c == '[' || c == '{')
            {
                MaybeComma();
                sb.Append(c);
                index++;
                afterValue = false;
                continue;
            }

            if (c == ']' || c == '}')
            {
                sb.Append(c);
                index++;
                afterValue = true;
                continue;
            }

            if (c == ',')
            {
                if (afterValue)
                {
                    sb.Append(',');
                }

                afterValue = false;
                index++;
                continue;
            }

            if (c == ':')
            {
                sb.Append(c);
                index++;
                afterValue = false;
                continue;
            }

            if (char.IsDigit(c) || c == '-' || (c == '.' && index + 1 < input.Length && char.IsDigit(input[index + 1])))
            {
                MaybeComma();
                var tokenStart = index;
                while (index < input.Length)
                {
                    var tokenChar = input[index];
                    if (char.IsDigit(tokenChar) || tokenChar == '.' || tokenChar == 'e' || tokenChar == 'E' || tokenChar == '+' || tokenChar == '-')
                    {
                        index++;
                        continue;
                    }

                    break;
                }

                sb.Append(input, tokenStart, index - tokenStart);
                afterValue = true;
                continue;
            }

            if (char.IsLetter(c) || c == '$' || c == '_')
            {
                MaybeComma();
                while (index < input.Length && (char.IsLetterOrDigit(input[index]) || input[index] == '_' || input[index] == '$'))
                {
                    sb.Append(input[index]);
                    index++;
                }

                afterValue = true;
                continue;
            }

            sb.Append(c);
            index++;
        }

        return sb.ToString();
    }

    private static IReadOnlyDictionary<string, Color4> LoadSharedPaintColors()
    {
        var paintColors = new Dictionary<string, Color4>(StringComparer.OrdinalIgnoreCase);
        foreach (var defaultTilesPath in EnumerateDefaultTilesPaths())
        {
            try
            {
                using var document = JsonDocument.Parse(File.ReadAllText(defaultTilesPath), ParseOptions);
                CollectFactoryPaints(document.RootElement, paintColors);
            }
            catch
            {
                // Ignore missing or malformed external BeamNG UI data and fall back to the baked defaults.
            }
        }

        foreach (var (paintName, baseColor) in KnownFallbackPaintColors)
        {
            paintColors.TryAdd(paintName, baseColor);
        }

        return paintColors;
    }

    private static IEnumerable<string> EnumerateDefaultTilesPaths()
    {
        var seen = new HashSet<string>(StringComparer.OrdinalIgnoreCase);

        foreach (var beamNgRoot in EnumerateBeamNgRoots())
        {
            var candidatePath = Path.Combine(
                beamNgRoot,
                "lua",
                "ge",
                "extensions",
                "freeroam",
                "configuratorOptions",
                "defaultTiles.json");
            if (File.Exists(candidatePath) && seen.Add(candidatePath))
            {
                yield return candidatePath;
            }
        }
    }

    private static IEnumerable<string> EnumerateBeamNgRoots()
    {
        if (BeamNgVehicleCatalog.DetectBeamNgContentVehiclesRoot() is { } detectedVehiclesRoot)
        {
            var contentDirectory = Directory.GetParent(detectedVehiclesRoot);
            var beamNgRoot = contentDirectory?.Parent?.FullName;
            if (!string.IsNullOrWhiteSpace(beamNgRoot))
            {
                yield return beamNgRoot;
            }
        }

        yield return @"C:\Program Files (x86)\Steam\steamapps\common\BeamNG.drive";
        yield return @"C:\Program Files\Steam\steamapps\common\BeamNG.drive";
    }

    private static void CollectFactoryPaints(JsonElement element, Dictionary<string, Color4> paintColors)
    {
        switch (element.ValueKind)
        {
            case JsonValueKind.Object:
                foreach (var property in element.EnumerateObject())
                {
                    if (property.NameEquals("factoryPaints") && property.Value.ValueKind == JsonValueKind.Array)
                    {
                        foreach (var paintElement in property.Value.EnumerateArray())
                        {
                            if (paintElement.ValueKind != JsonValueKind.Object ||
                                !TryGetBaseColor(paintElement, out var baseColor))
                            {
                                continue;
                            }

                            var paintName = GetOptionalStringProperty(paintElement, "name");
                            if (!string.IsNullOrWhiteSpace(paintName))
                            {
                                paintColors[paintName] = baseColor;
                            }
                        }
                    }

                    CollectFactoryPaints(property.Value, paintColors);
                }

                break;

            case JsonValueKind.Array:
                foreach (var arrayElement in element.EnumerateArray())
                {
                    CollectFactoryPaints(arrayElement, paintColors);
                }

                break;
        }
    }

    private static bool TryGetBaseColor(JsonElement element, out Color4 baseColor)
    {
        baseColor = default;
        if (!element.TryGetProperty("baseColor", out var baseColorElement) ||
            baseColorElement.ValueKind != JsonValueKind.Array)
        {
            return false;
        }

        float[] components = [1f, 1f, 1f, 1f];
        var index = 0;
        foreach (var component in baseColorElement.EnumerateArray())
        {
            if (index >= components.Length || component.ValueKind != JsonValueKind.Number)
            {
                break;
            }

            components[index++] = component.GetSingle();
        }

        baseColor = new Color4(components[0], components[1], components[2], components[3]);
        return true;
    }

    private static string? GetOptionalStringProperty(JsonElement element, string propertyName)
    {
        if (!element.TryGetProperty(propertyName, out var property) || property.ValueKind != JsonValueKind.String)
        {
            return null;
        }

        return property.GetString();
    }
}
