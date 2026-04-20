using System;
using System.IO;
using LibreRally.Vehicle;
using LibreRally.Vehicle.Rendering;
using Stride.Core.Mathematics;

namespace LibreRally.Tests;

public class BeamNgPaintPaletteResolverTests
{
    private static string CombineRelativePath(string basePath, string relativePath)
    {
        if (Path.IsPathRooted(relativePath))
        {
            throw new ArgumentException("Path must be relative.", nameof(relativePath));
        }

        return Path.Combine(basePath, relativePath);
    }

    private static string GetVehicleFolder(string vehicleFolderName)
    {
        DirectoryInfo? directory = new(AppContext.BaseDirectory);
        while (directory != null)
        {
            string candidate = Path.Combine(directory.FullName, "LibreRally.sln");
            if (File.Exists(candidate))
            {
                return Path.Combine(directory.FullName, "LibreRally", "Resources", "BeamNG Vehicles", vehicleFolderName);
            }

            directory = directory.Parent;
        }

        throw new DirectoryNotFoundException($"Could not locate the repository root for '{vehicleFolderName}' paint tests.");
    }

    [Fact]
    public void RallyProGravel_ShouldResolveDefaultPaintPalette()
    {
        string vehicleFolder = GetVehicleFolder("sunburst2");
        string configPath = CombineRelativePath(vehicleFolder, "rally_pro_gravel.pc");

        BeamNgPaintPalette? palette = BeamNgPaintPaletteResolver.LoadDefaultPalette(vehicleFolder, configPath);

        Assert.True(palette.HasValue);
        Assert.Equal("Rally Blue", palette.Value.Paint1.Name);
        Assert.Equal("Tangerine Rush", palette.Value.Paint2.Name);
        Assert.Equal("Sunny Flare", palette.Value.Paint3.Name);
        Assert.True(palette.Value.Paint1.BaseColor.B > palette.Value.Paint1.BaseColor.R);
        Assert.True(palette.Value.Paint2.BaseColor.R > palette.Value.Paint2.BaseColor.B);
        Assert.True(palette.Value.Paint3.BaseColor.R > palette.Value.Paint3.BaseColor.B);
    }

    [Fact]
    public void ComposeInstanceDiffuseColor_ShouldTintMaskedChannels()
    {
        var palette = new BeamNgPaintPalette(
            new BeamNgPaintColor("Paint1", new Color4(1f, 0f, 0f, 1f)),
            new BeamNgPaintColor("Paint2", new Color4(0f, 1f, 0f, 1f)),
            new BeamNgPaintColor("Paint3", new Color4(0f, 0f, 1f, 1f)));
        var baseColor = new Color(128, 128, 128, 255);

        Assert.Equal(
            new Color(128, 0, 0, 255),
            VehicleLoader.ComposeInstanceDiffuseColor(baseColor, new Color(255, 0, 0, 255), palette));
        Assert.Equal(
            new Color(0, 128, 0, 255),
            VehicleLoader.ComposeInstanceDiffuseColor(baseColor, new Color(0, 255, 0, 255), palette));
        Assert.Equal(
            baseColor,
            VehicleLoader.ComposeInstanceDiffuseColor(baseColor, new Color(0, 0, 0, 255), palette));
    }

    [Fact]
    public void LoadDefaultPalette_ShouldHandleLenientInfoJson()
    {
        string vehicleFolder = Path.Combine(Path.GetTempPath(), Path.GetRandomFileName());
        Directory.CreateDirectory(vehicleFolder);

        try
        {
            File.WriteAllText(
                Path.Combine(vehicleFolder, "info.json"),
                """
                {
                  "Name":"FGX Sedan",
                  "Years":{
                    "min":2016
                    "max":2022,
                  },
                  "defaultPaintName1":"Silver",
                  "defaultPaintName2":"Army Green",
                  "defaultPaintName3":"Bone White",
                  "paints":{
                    "Silver":{"baseColor":[0.5,0.5,0.5,1]},
                    "Army Green":{"baseColor":[0.275,0.302,0.25,1]},
                    "Bone White":{"baseColor":[0.89,0.855,0.788,1]}
                  }
                }
                """);

            BeamNgPaintPalette? palette = BeamNgPaintPaletteResolver.LoadDefaultPalette(vehicleFolder, configFilePath: null);

            Assert.True(palette.HasValue);
            Assert.Equal("Silver", palette.Value.Paint1.Name);
            Assert.Equal("Army Green", palette.Value.Paint2.Name);
            Assert.Equal("Bone White", palette.Value.Paint3.Name);
            Assert.Equal(0.5f, palette.Value.Paint1.BaseColor.R, 3);
            Assert.Equal(0.302f, palette.Value.Paint2.BaseColor.G, 3);
            Assert.Equal(0.788f, palette.Value.Paint3.BaseColor.B, 3);
        }
        finally
        {
            Directory.Delete(vehicleFolder, recursive: true);
        }
    }
}
