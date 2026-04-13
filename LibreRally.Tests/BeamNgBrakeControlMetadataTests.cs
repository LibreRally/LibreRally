using System;
using System.IO;
using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;

namespace LibreRally.Tests;

public class BeamNgBrakeControlMetadataTests
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

        throw new DirectoryNotFoundException($"Could not locate the repository root for '{vehicleFolderName}' tests.");
    }

    [Fact]
    public void BasicCar_DoesNotEnableAbsByDefault()
    {
        string vehicleFolder = GetVehicleFolder("basic_car");
        PcConfig config = PcConfigLoader.Load(CombineRelativePath(vehicleFolder, "basic_car.pc"));
        VehicleDefinition definition = JBeamAssembler.Assemble(vehicleFolder, config);

        Assert.Null(definition.BrakeControl);
        Assert.False(VehicleLoader.IsAbsEnabled(definition.BrakeControl));
    }

    [Fact]
    public void Sunburst2_ModernAbsPart_ParsesEnableFlag()
    {
        string vehicleFolder = GetVehicleFolder("sunburst2");
        JBeamPart absPart = Assert.Single(
            JBeamParser.ParseFile(CombineRelativePath(vehicleFolder, "sunburst2_dse_abs.jbeam")),
            part => part.Name == "sunburst2_DSE_ABS");

        JBeamBrakeControlDefinition brakeControl = Assert.IsType<JBeamBrakeControlDefinition>(absPart.BrakeControl);
        Assert.True(brakeControl.EnableAbs == true);
        Assert.True(VehicleLoader.IsAbsEnabled(brakeControl));
    }

    [Fact]
    public void Sunburst2_BrakeParts_ParseAbsTargetAndLegacyController()
    {
        string vehicleFolder = GetVehicleFolder("sunburst2");
        var parts = JBeamParser.ParseFile(CombineRelativePath(vehicleFolder, "sunburst2_brakes.jbeam"));

        JBeamBrakeControlDefinition targetPart = Assert.IsType<JBeamBrakeControlDefinition>(
            Assert.Single(parts, part => part.BrakeControl?.AbsSlipRatioTarget == 0.15f).BrakeControl);
        JBeamBrakeControlDefinition legacyPart = Assert.IsType<JBeamBrakeControlDefinition>(
            Assert.Single(parts, part => part.BrakeControl?.HasLegacyAbsController == true).BrakeControl);

        Assert.Equal(0.15f, VehicleLoader.ResolveAbsSlipRatioTarget(targetPart), 3);
        Assert.True(legacyPart.HasLegacyAbsController);
        Assert.True(VehicleLoader.IsAbsEnabled(legacyPart));
    }
}
