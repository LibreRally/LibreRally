using System;
using System.Collections.Generic;
using System.IO;
using System.Text.Json;

namespace LibreRally.Vehicle.JBeam;

/// <summary>
/// A BeamNG .pc (parts config) file parsed into parts selection and variable overrides.
/// </summary>
public class PcConfig
{
    /// <summary>The root part name (e.g. "sunburst2").</summary>
    public string MainPartName { get; init; } = "";

    /// <summary>
    /// Slot-type → chosen part name.
    /// Blank entries preserve the JBeam slot default when one exists.
    /// </summary>
    public Dictionary<string, string> Parts { get; init; } = new(StringComparer.OrdinalIgnoreCase);

    /// <summary>
    /// Variable overrides: variable name (without leading $) → float value.
    /// E.g. "spring_F_asphalt" → 60000.
    /// </summary>
    public Dictionary<string, float> Vars { get; init; } = new(StringComparer.OrdinalIgnoreCase);
}

/// <summary>
/// Parses BeamNG .pc files (standard JSON with trailing commas allowed) into <see cref="PcConfig"/>.
/// </summary>
public static class PcConfigLoader
{
    private static readonly JsonDocumentOptions Options = new()
    {
        CommentHandling = JsonCommentHandling.Skip,
        AllowTrailingCommas = true,
    };

    /// <summary>Loads and parses a .pc configuration file from the specified path.</summary>
    /// <param name="pcFilePath">The absolute path to the .pc file.</param>
    /// <returns>A <see cref="PcConfig"/> object containing the parsed settings.</returns>
    public static PcConfig Load(string pcFilePath)
    {
        var text = File.ReadAllText(pcFilePath);
        using var doc = JsonDocument.Parse(text, Options);
        var root = doc.RootElement;

        var mainPart = "";
        if (root.TryGetProperty("mainPartName", out var mpn) && mpn.ValueKind == JsonValueKind.String)
        {
	        mainPart = mpn.GetString() ?? "";
        }

        var parts = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
        if (root.TryGetProperty("parts", out var partsEl) && partsEl.ValueKind == JsonValueKind.Object)
        {
            foreach (var prop in partsEl.EnumerateObject())
            {
                var value = prop.Value.ValueKind == JsonValueKind.String
                    ? prop.Value.GetString() ?? ""
                    : "";
                parts[prop.Name] = value;
            }
        }

        var vars = new Dictionary<string, float>(StringComparer.OrdinalIgnoreCase);
        if (root.TryGetProperty("vars", out var varsEl) && varsEl.ValueKind == JsonValueKind.Object)
        {
            foreach (var prop in varsEl.EnumerateObject())
            {
                var val = prop.Value.ValueKind switch
                {
                    JsonValueKind.Number => prop.Value.GetSingle(),
                    JsonValueKind.String => float.TryParse(prop.Value.GetString(), out var v) ? v : 0f,
                    _ => 0f,
                };
                // Strip leading $ if present
                var key = prop.Name.StartsWith("$") ? prop.Name[1..] : prop.Name;
                vars[key] = val;
            }
        }

        return new PcConfig { MainPartName = mainPart, Parts = parts, Vars = vars };
    }

    /// <summary>
    /// Finds the best .pc file to use from a vehicle folder.
    /// Priority: exact name match → "rally_pro_asphalt" → first .pc file found.
    /// </summary>
    /// <param name="vehicleFolder">Vehicle folder to scan for <c>.pc</c> files.</param>
    /// <param name="preferredName">Optional preferred config filename or basename.</param>
    /// <returns>The selected config path when one is found; otherwise <see langword="null"/>.</returns>
    public static string? FindBestConfig(string vehicleFolder, string? preferredName = null)
    {
        var pcFiles = Directory.GetFiles(vehicleFolder, "*.pc", SearchOption.TopDirectoryOnly);
        if (pcFiles.Length == 0)
        {
	        return null;
        }

        if (!string.IsNullOrEmpty(preferredName))
        {
            // Exact name (with or without .pc extension)
            var nameWithExt = preferredName.EndsWith(".pc", StringComparison.OrdinalIgnoreCase)
                ? preferredName : preferredName + ".pc";
            var exact = Array.Find(pcFiles, f =>
                string.Equals(Path.GetFileName(f), nameWithExt, StringComparison.OrdinalIgnoreCase));
            if (exact != null)
            {
	            return exact;
            }
        }

        // Prefer rally_pro_asphalt as a sensible default for development
        var rallyPro = Array.Find(pcFiles, f =>
            Path.GetFileNameWithoutExtension(f).Equals("rally_pro_asphalt", StringComparison.OrdinalIgnoreCase));
        if (rallyPro != null)
        {
	        return rallyPro;
        }

        return pcFiles[0];
    }
}
