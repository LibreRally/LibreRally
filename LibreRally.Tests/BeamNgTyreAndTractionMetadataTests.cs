using System.Collections.Generic;
using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;
using Xunit;

namespace LibreRally.Tests;

public class BeamNgTyreAndTractionMetadataTests
{
    [Fact]
    public void ParsePressureWheels_ParsesBeamNgTyreOptionObjects()
    {
        const string jbeam = """
{
  "test_tire": {
    "slotType": "tire_F_19x8",
    "pressureWheels": [
      ["name", "hubGroup", "group", "node1:", "node2:", "nodeS", "nodeArm:", "wheelDir"],
      { "hasTire": true },
      { "radius": 0.32 },
      { "tireWidth": 0.200 },
      { "pressurePSI": "$tirepressure_F" },
      { "frictionCoef": 1.0 },
      { "slidingFrictionCoef": 1.0 },
      { "treadCoef": 0.5 },
      { "noLoadCoef": 1.6 },
      { "loadSensitivitySlope": 0.000165 },
      { "fullLoadCoef": 0.5 },
      { "softnessCoef": 0.8 },
      ["FL", "hub", "group", "n1", "n2", "nS", "arm", 1]
    ]
  }
}
""";

        JBeamParser.SetVars(new Dictionary<string, float> { ["tirepressure_F"] = 30f });
        try
        {
            var part = Assert.Single(JBeamParser.Parse(jbeam));
            var options = Assert.IsType<JBeamPressureWheelOptions>(part.PressureWheelOptions);

            Assert.True(options.HasTire);
            Assert.Equal(0.32f, options.Radius.GetValueOrDefault(), 3);
            Assert.Equal(0.200f, options.TireWidth.GetValueOrDefault(), 3);
            Assert.Equal(30f, options.PressurePsi.GetValueOrDefault(), 3);
            Assert.Equal(1.6f, options.NoLoadCoef.GetValueOrDefault(), 3);
            Assert.Equal(0.000165f, options.LoadSensitivitySlope.GetValueOrDefault(), 6);
            Assert.Equal(0.5f, options.FullLoadCoef.GetValueOrDefault(), 3);
            Assert.Equal(0.8f, options.SoftnessCoef.GetValueOrDefault(), 3);
        }
        finally
        {
            JBeamParser.SetVars(null);
        }
    }

    [Fact]
    public void ParsePart_ParsesTopLevelGearRatio()
    {
        const string jbeam = """
{
  "fgx_finaldrive_R_391": {
    "slotType": "fgx_finaldrive_R",
    "gearRatio": 3.91
  }
}
""";

        var part = Assert.Single(JBeamParser.Parse(jbeam));
        Assert.Equal(3.91f, part.GearRatio.GetValueOrDefault(), 3);
    }

    [Fact]
    public void ParseTractionControlDefinition_ParsesSlipThresholdAndWindow()
    {
        const string jbeam = """
{
  "test_tc": {
    "slotType": "drivingAid",
    "tractionControl": {
      "tractionControlledMotors": ["mainEngine"]
    },
    "motorTorqueControl": {
      "useForTractionControl": true,
      "tractionControl": {
        "wheelGroupSettings": [
          ["motorName", "slipThreshold", "kP"],
          ["mainEngine", 0.15, 0.9]
        ]
      }
    },
    "brakeControl": {
      "useForTractionControl": true,
      "tractionControl": {
        "wheelGroupSettings": [
          ["motorName", "slipThreshold", "slipRangeThreshold", "maxVelocity"],
          ["mainEngine", 0.10, 0.20, 30]
        ]
      }
    }
  }
}
""";

        var part = Assert.Single(JBeamParser.Parse(jbeam));
        var tractionControl = Assert.IsType<JBeamTractionControlDefinition>(part.TractionControl);

        Assert.True(tractionControl.EnableTractionControl);
        Assert.Equal(0.15f, tractionControl.SlipThreshold.GetValueOrDefault(), 3);
        Assert.Equal(0.20f, tractionControl.SlipRangeThreshold.GetValueOrDefault(), 3);
        Assert.Equal(30f, tractionControl.MaxVelocity.GetValueOrDefault(), 3);
        Assert.True(VehicleLoader.IsTractionControlEnabled(tractionControl));
        Assert.Equal(0.15f, VehicleLoader.ResolveTractionControlSlipRatioTarget(tractionControl), 3);
        Assert.Equal(0.20f, VehicleLoader.ResolveTractionControlSlipRatioWindow(tractionControl), 3);
    }

    [Fact]
    public void ResolveTyreSpec_UsesAxleSpecificBeamNgTyreMetadata()
    {
        var definition = new VehicleDefinition
        {
            PressureWheelOptions = new List<AssembledPressureWheelOptions>
            {
                new(
                    "tire_F_225_35_19_sport",
                    "tire_F_19x8",
                    new JBeamPressureWheelOptions
                    {
                        HasTire = true,
                        Radius = 0.32f,
                        TireWidth = 0.200f,
                        PressurePsi = 30f,
                        FrictionCoef = 1.0f,
                        NoLoadCoef = 1.6f,
                        LoadSensitivitySlope = 0.000165f,
                        FullLoadCoef = 0.5f,
                    }),
                new(
                    "tire_R_225_45_17_sport",
                    "tire_R_17x8",
                    new JBeamPressureWheelOptions
                    {
                        HasTire = true,
                        Radius = 0.315f,
                        TireWidth = 0.205f,
                        PressurePsi = 32f,
                        FrictionCoef = 1.0f,
                        NoLoadCoef = 1.51f,
                        LoadSensitivitySlope = 0.000175f,
                        FullLoadCoef = 0.5f,
                    }),
            },
        };

        var frontSpec = VehicleTyreSpecResolver.Resolve(definition, front: true);
        var rearSpec = VehicleTyreSpecResolver.Resolve(definition, front: false);

        Assert.Equal(0.32f, frontSpec.Radius, 3);
        Assert.Equal(0.200f, frontSpec.Width, 3);
        Assert.Equal(30f * 6.894757f, frontSpec.PressureKpa, 3);
        Assert.Equal(TyreModel.ComputeBeamNgLoadCoefficient(3000f, 1.6f, 0.5f, 0.000165f), frontSpec.PeakFrictionCoefficient, 3);

        Assert.Equal(0.315f, rearSpec.Radius, 3);
        Assert.Equal(0.205f, rearSpec.Width, 3);
        Assert.Equal(32f * 6.894757f, rearSpec.PressureKpa, 3);
        Assert.Equal(TyreModel.ComputeBeamNgLoadCoefficient(3000f, 1.51f, 0.5f, 0.000175f), rearSpec.PeakFrictionCoefficient, 3);
    }
}
