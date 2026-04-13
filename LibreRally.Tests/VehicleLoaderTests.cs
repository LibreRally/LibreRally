using System.Collections.Generic;
using System.Numerics;
using LibreRally.Vehicle;
using LibreRally.Vehicle.Rendering;
using Xunit;

namespace LibreRally.Tests;

public class VehicleLoaderTests
{
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
