using System.IO;
using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;
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

        Assert.Equal("basic_car", result.RootEntity.Name);
        Assert.Equal("chassis", result.ChassisEntity.Name);
        Assert.NotNull(result.WheelFL.Get<WheelSettings>());
        Assert.NotNull(result.WheelFR.Get<WheelSettings>());
        Assert.NotNull(result.WheelRL.Get<WheelSettings>());
        Assert.NotNull(result.WheelRR.Get<WheelSettings>());
        Assert.Equal("wheel_FL", result.WheelFL.Name);
        Assert.Equal("wheel_FR", result.WheelFR.Name);
        Assert.Equal("wheel_RL", result.WheelRL.Name);
        Assert.Equal("wheel_RR", result.WheelRR.Name);
    }
}
