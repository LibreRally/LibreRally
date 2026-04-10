using System.IO;
using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;
using Stride.BepuPhysics.Constraints;
using Stride.Core.Mathematics;
using Stride.Engine;
using Xunit;

namespace LibreRally.Tests;

public class BasicCarPipelineTests
{
    private static string GetVehicleFolder()
    {
        DirectoryInfo? directory = new(AppContext.BaseDirectory);
        while (directory != null)
        {
            string candidate = Path.Combine(directory.FullName, "LibreRally.sln");
            if (File.Exists(candidate))
                return Path.Combine(directory.FullName, "LibreRally", "Resources", "BeamNG Vehicles", "basic_car");

            directory = directory.Parent;
        }

        throw new DirectoryNotFoundException("Could not locate the repository root for the basic_car tests.");
    }

    [Fact]
    public void BasicCar_ShouldParseNodes()
    {
        PcConfig config = PcConfigLoader.Load(Path.Combine(GetVehicleFolder(), "basic_car.pc"));
        JBeamPart bodyPart = Assert.Single(JBeamParser.ParseFile(Path.Combine(GetVehicleFolder(), "body.jbeam"), config.Vars));

        Assert.Equal("basic_car_body", bodyPart.Name);
        Assert.True(bodyPart.Nodes.Count >= 10);
        Assert.Contains(bodyPart.Nodes, node => node.Id == "bc2");
        Assert.Equal("bc2", bodyPart.RefNodes["ref"]);
    }

    [Fact]
    public void BasicCar_ShouldParseBeams()
    {
        PcConfig config = PcConfigLoader.Load(Path.Combine(GetVehicleFolder(), "basic_car.pc"));
        JBeamPart wheelsPart = Assert.Single(JBeamParser.ParseFile(Path.Combine(GetVehicleFolder(), "wheels.jbeam"), config.Vars));

        Assert.True(wheelsPart.Beams.Count >= 8);
        Assert.Contains(wheelsPart.Nodes, node => node.Properties.Groups.Contains("wheel_FL"));
        Assert.Contains(wheelsPart.Nodes, node => node.Properties.Groups.Contains("wheel_RR"));
    }

    [Fact]
    public void BasicCar_ShouldAssembleVehicle()
    {
        PcConfig config = PcConfigLoader.Load(Path.Combine(GetVehicleFolder(), "basic_car.pc"));
        VehicleDefinition definition = JBeamAssembler.Assemble(GetVehicleFolder(), config);

        Assert.Equal("basic_car", definition.VehicleName);
        Assert.True(definition.Nodes.Count >= 20);
        Assert.True(definition.Beams.Count >= 40);
        Assert.Equal("bc2", definition.RefNodes["ref"]);
        Assert.Equal(52000f, definition.Vars["spring_F_asphalt"]);

        AssembledNode wheelNode = definition.Nodes["wfl1"];
        Assert.Contains("wheel_FL", wheelNode.Groups);
    }

    [Fact]
    public void BasicCar_ShouldBuildPhysics()
    {
        PcConfig config = PcConfigLoader.Load(Path.Combine(GetVehicleFolder(), "basic_car.pc"));
        VehicleDefinition definition = JBeamAssembler.Assemble(GetVehicleFolder(), config);
        VehicleBuilderResult result = VehiclePhysicsBuilder.Build(definition);

        WheelSettings? wheelFlSettings = result.WheelFL.Get<WheelSettings>();
        WheelSettings? wheelFrSettings = result.WheelFR.Get<WheelSettings>();
        WheelSettings? wheelRlSettings = result.WheelRL.Get<WheelSettings>();
        WheelSettings? wheelRrSettings = result.WheelRR.Get<WheelSettings>();
        var wheelFlBody = result.WheelFL.Get<Stride.BepuPhysics.BodyComponent>();

        Assert.Equal("basic_car", result.RootEntity.Name);
        Assert.Equal("chassis", result.ChassisEntity.Name);
        Assert.NotNull(wheelFlSettings);
        Assert.NotNull(wheelFrSettings);
        Assert.NotNull(wheelRlSettings);
        Assert.NotNull(wheelRrSettings);
        Assert.NotNull(wheelFlBody);
        Assert.Equal("wheel_FL", result.WheelFL.Name);
        Assert.Equal("wheel_FR", result.WheelFR.Name);
        Assert.Equal("wheel_RL", result.WheelRL.Name);
        Assert.Equal("wheel_RR", result.WheelRR.Name);
        Assert.Equal(0.05f, wheelFlBody!.FrictionCoefficient, 3);

        foreach (WheelSettings wheelSettings in new[] { wheelFlSettings!, wheelFrSettings!, wheelRlSettings!, wheelRrSettings! })
        {
            Assert.Equal(Vector3.Zero, wheelSettings.SuspensionLocalOffsetB);
            Assert.Equal(Vector3.UnitY, wheelSettings.SuspensionLocalAxis);
            Assert.Equal(0f, wheelSettings.SuspensionTargetOffset);
            Assert.True(wheelSettings.SuspensionMinimumOffset < 0f);
            Assert.True(wheelSettings.SuspensionMaximumOffset > 0f);
        }

        LinearAxisLimitConstraintComponent? flLimit = result.WheelFL.Get<LinearAxisLimitConstraintComponent>();
        Assert.NotNull(flLimit);
        Assert.Equal(wheelFlSettings!.SuspensionLocalOffsetA, flLimit!.LocalOffsetA);
        Assert.Equal(wheelFlSettings.SuspensionLocalOffsetB, flLimit.LocalOffsetB);
        Assert.Equal(wheelFlSettings.SuspensionLocalAxis, flLimit.LocalAxis);
        Assert.Equal(wheelFlSettings.SuspensionMinimumOffset, flLimit.MinimumOffset);
        Assert.Equal(wheelFlSettings.SuspensionMaximumOffset, flLimit.MaximumOffset);
    }
}
