using System;
using System.IO;
using LibreRally.Vehicle.Rendering;
using Xunit;

namespace LibreRally.Tests;

public sealed class BeamNGMaterialLoaderTests
{
    [Fact]
    public void LoadMaterialTextures_ResolvesSunburstDdsTexturesReferencedAsPng()
    {
        var vehiclesRoot = GetBeamNgVehiclesRoot();
        var vehicleFolder = Path.Combine(vehiclesRoot, "sunburst2");

        var textures = BeamNGMaterialLoader.LoadMaterialTextures(vehicleFolder, vehiclesRoot);

        Assert.True(textures.TryGetValue("sunburst2_main", out var mainTexture));
        Assert.EndsWith(".dds", mainTexture, StringComparison.OrdinalIgnoreCase);
        Assert.True(File.Exists(mainTexture));
        Assert.True(textures.TryGetValue("sunburst2_suspension", out var suspensionTexture));
        Assert.EndsWith(".dds", suspensionTexture, StringComparison.OrdinalIgnoreCase);
        Assert.True(File.Exists(suspensionTexture));
    }

    [Fact]
    public void LoadMaterialTextures_RegistersMapToAliasWhenJsonKeyDiffers()
    {
        var root = Path.Combine(AppContext.BaseDirectory, "BeamNgMaterialLoaderTests", Guid.NewGuid().ToString("N"));
        var vehiclesRoot = Path.Combine(root, "vehicles");
        var vehicleFolder = Path.Combine(vehiclesRoot, "testcar");
        Directory.CreateDirectory(vehicleFolder);

        try
        {
            var texturePath = Path.Combine(vehicleFolder, "body_d.dds");
            File.WriteAllBytes(texturePath, []);
            File.WriteAllText(
                Path.Combine(vehicleFolder, "main.materials.json"),
                """
                {
                  "body_skin": {
                    "name": "body_skin",
                    "mapTo": "body_material",
                    "Stages": [
                      {
                        "baseColorMap": "vehicles/testcar/body_d.dds"
                      }
                    ]
                  }
                }
                """);

            var textures = BeamNGMaterialLoader.LoadMaterialTextures(vehicleFolder, vehiclesRoot);

            Assert.True(textures.TryGetValue("body_skin", out var keyTexture));
            Assert.Equal(texturePath, keyTexture);
            Assert.True(textures.TryGetValue("body_material", out var aliasTexture));
            Assert.Equal(texturePath, aliasTexture);
        }
        finally
        {
            if (Directory.Exists(root))
            {
                Directory.Delete(root, recursive: true);
            }
        }
    }

    private static string GetBeamNgVehiclesRoot()
    {
        DirectoryInfo? directory = new(AppContext.BaseDirectory);
        while (directory != null)
        {
            var candidate = Path.Combine(directory.FullName, "LibreRally.sln");
            if (File.Exists(candidate))
            {
                return Path.Combine(directory.FullName, "LibreRally", "Resources", "BeamNG Vehicles");
            }

            directory = directory.Parent;
        }

        throw new DirectoryNotFoundException("Could not locate the repository root for BeamNG material tests.");
    }
}
