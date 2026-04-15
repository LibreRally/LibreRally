using System.Collections.Generic;
using System.Numerics;
using LibreRally.Vehicle;
using LibreRally.Vehicle.Rendering;
using Xunit;

namespace LibreRally.Tests;

public class VehicleLoaderTests
{
    [Fact]
    public void SelectChassisMeshes_ForDtsSources_KeepsOnlyActiveNonWheelFlexbodies()
    {
        List<ColladaMesh> sourceMeshes =
        [
            new ColladaMesh { GeometryName = "fgx_body" },
            new ColladaMesh { GeometryName = "fgx_interior" },
            new ColladaMesh { GeometryName = "unused_bodykit" },
            new ColladaMesh { GeometryName = "wheel_18x8" },
        ];
        HashSet<string> wheelMeshNames = ["wheel_18x8"];

        var chassisMeshes = VehicleLoader.SelectChassisMeshes(
            sourceMeshes,
            "fgx.dts",
            ["fgx_body", "fgx_interior"],
            wheelMeshNames);

        Assert.Collection(
            chassisMeshes,
            mesh => Assert.Equal("fgx_body", mesh.GeometryName),
            mesh => Assert.Equal("fgx_interior", mesh.GeometryName));
    }

    [Fact]
    public void SelectChassisMeshes_ForNonDtsSources_PreservesFallbackMeshesWhenNamesDoNotMatch()
    {
        List<ColladaMesh> sourceMeshes =
        [
            new ColladaMesh { GeometryName = "FormulaBeeBody" },
            new ColladaMesh { GeometryName = "FormulaBeeSuspension" },
            new ColladaMesh { GeometryName = "steelwheel_11a_13x5" },
        ];
        HashSet<string> wheelMeshNames = ["steelwheel_11a_13x5"];

        var chassisMeshes = VehicleLoader.SelectChassisMeshes(
            sourceMeshes,
            "FormulaBeeModel.dae",
            ["TutoFBee_Spaceframe"],
            wheelMeshNames);

        Assert.Collection(
            chassisMeshes,
            mesh => Assert.Equal("FormulaBeeBody", mesh.GeometryName),
            mesh => Assert.Equal("FormulaBeeSuspension", mesh.GeometryName));
    }

    [Fact]
    public void SelectSupplementalChassisMeshes_KeepsOnlyRequestedNonWheelMeshes()
    {
        List<ColladaMesh> sourceMeshes =
        [
            new ColladaMesh { GeometryName = "autobello_muffler_b" },
            new ColladaMesh { GeometryName = "autobello_shifter_5M" },
            new ColladaMesh { GeometryName = "autobello_engineblock" },
            new ColladaMesh { GeometryName = "steelwheel_11a_13x5" },
        ];
        HashSet<string> wheelMeshNames = ["steelwheel_11a_13x5"];

        var chassisMeshes = VehicleLoader.SelectSupplementalChassisMeshes(
            sourceMeshes,
            ["autobello_muffler_b", "autobello_shifter_5M"],
            wheelMeshNames);

        Assert.Collection(
            chassisMeshes,
            mesh => Assert.Equal("autobello_muffler_b", mesh.GeometryName),
            mesh => Assert.Equal("autobello_shifter_5M", mesh.GeometryName));
    }

    [Fact]
    public void ShouldTreatWheelGeometryAsPrePositioned_RequiresGeometryToMatchFlexbodyPosition()
    {
        Vector3 expectedPosition = new(0.655f, -1.127f, 0.295f);
        List<ColladaMesh> sourceMeshes =
        [
            new ColladaMesh
            {
                GeometryName = "steelwheel_11a_13x5",
                HasBakedTransform = true,
                Vertices =
                [
                    new ColladaVertex(new Vector3(-0.15f, -0.15f, 0f), Vector3.UnitY, Vector2.Zero),
                    new ColladaVertex(new Vector3(0.15f, 0.15f, 0f), Vector3.UnitY, Vector2.Zero),
                ],
            },
        ];

        Assert.False(VehicleLoader.ShouldTreatWheelGeometryAsPrePositioned(sourceMeshes, expectedPosition));
    }

    [Fact]
    public void ShouldTreatWheelGeometryAsPrePositioned_AcceptsGeometryAlreadyAtFlexbodyPosition()
    {
        Vector3 expectedPosition = new(0.655f, -1.127f, 0.295f);
        List<ColladaMesh> sourceMeshes =
        [
            new ColladaMesh
            {
                GeometryName = "steelwheel_11a_13x5",
                HasBakedTransform = true,
                Vertices =
                [
                    new ColladaVertex(expectedPosition + new Vector3(-0.15f, 0f, 0f), Vector3.UnitY, Vector2.Zero),
                    new ColladaVertex(expectedPosition + new Vector3(0.15f, 0f, 0f), Vector3.UnitY, Vector2.Zero),
                ],
            },
        ];

        Assert.True(VehicleLoader.ShouldTreatWheelGeometryAsPrePositioned(sourceMeshes, expectedPosition));
    }
}
