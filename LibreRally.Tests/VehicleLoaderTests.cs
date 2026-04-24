using System.Numerics;
using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;
using LibreRally.Vehicle.Rendering;
using StrideMatrix = Stride.Core.Mathematics.Matrix;
using StrideVector3 = Stride.Core.Mathematics.Vector3;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the vehicle loader behavior.
	/// </summary>
	public class VehicleLoaderTests
	{
		private static string GetVehicleFolder()
		{
			DirectoryInfo? directory = new(AppContext.BaseDirectory);
			while (directory != null)
			{
				string candidate = Path.Combine(directory.FullName, "LibreRally.sln");
				if (File.Exists(candidate))
				{
					return Path.Combine(directory.FullName, "LibreRally", "Resources", "BeamNG Vehicles", "basic_car");
				}

				directory = directory.Parent;
			}

			throw new DirectoryNotFoundException("Could not locate the repository root for the basic_car tests.");
		}

		/// <summary>
		/// Verifies that select chassis meshes for dts sources keeps only active non wheel flexbodies.
		/// </summary>
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

		/// <summary>
		/// Verifies that select chassis meshes for non dts sources preserves fallback meshes when names do not match.
		/// </summary>
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

		/// <summary>
		/// Verifies that select supplemental chassis meshes keeps only requested non wheel meshes.
		/// </summary>
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

		/// <summary>
		/// Verifies that should treat wheel geometry as pre positioned requires geometry to match flexbody position.
		/// </summary>
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

		/// <summary>
		/// Verifies that should treat wheel geometry as pre positioned accepts geometry already at flexbody position.
		/// </summary>
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

		/// <summary>
		/// Verifies that is suspension kinematic flex body classifies moving link meshes.
		/// </summary>
		[Theory]
		[InlineData("autobello_upperarm_wide_F", true)]
		[InlineData("autobello_coilover_R", true)]
		[InlineData("autobello_halfshaft_R", true)]
		[InlineData("autobello_brakedisk_track_FR", false)]
		[InlineData("steelwheel_11a_13x5", false)]
		public void IsSuspensionKinematicFlexBody_ClassifiesMovingLinkMeshes(string meshName, bool expected)
		{
			var flexBody = new AssembledFlexBody(meshName, []);

			Assert.Equal(expected, SuspensionVisualKinematicsRigBuilder.IsSuspensionKinematicFlexBody(flexBody));
		}

		/// <summary>
		/// Verifies that build link specs basic car creates suspension and rear halfshaft links.
		/// </summary>
		[Fact]
		public void BuildLinkSpecs_BasicCarCreatesSuspensionAndRearHalfshaftLinks()
		{
			PcConfig config = PcConfigLoader.Load(Path.Combine(GetVehicleFolder(), "basic_car.pc"));
			VehicleDefinition definition = JBeamAssembler.Assemble(GetVehicleFolder(), config);
			VehicleBuilderResult result = VehiclePhysicsBuilder.Build(definition);

			var specs = SuspensionVisualKinematicsRigBuilder.BuildLinkSpecs(definition, result);

			Assert.Contains(specs, spec => spec.Name == "front_upperarm_wheel_FL");
			Assert.Contains(specs, spec => spec.Name == "front_upperarm_wheel_FR");
			Assert.Contains(specs, spec => spec.Name == "front_coilover_wheel_FL");
			Assert.Contains(specs, spec => spec.Name == "rear_trailingarm_wheel_RL");
			Assert.Contains(specs, spec => spec.Name == "rear_trailingarm_wheel_RR");
			Assert.Contains(specs, spec => spec.Name == "rear_halfshaft_wheel_RL");
			Assert.Contains(specs, spec => spec.Name == "rear_halfshaft_wheel_RR");
			Assert.Contains(specs, spec => spec.EndUsesNonSpinTransform);
			Assert.All(specs, spec => Assert.True(spec.Radius > 0f));
			Assert.All(specs.Where(spec => spec.StartEntity == result.ChassisEntity), spec =>
				Assert.NotEqual(spec.StartLocalPosition, spec.EndLocalPosition));
		}

		/// <summary>
		/// Verifies that transform wheel local position without spin ignores wheel spin around axle.
		/// </summary>
		[Fact]
		public void TransformWheelLocalPositionWithoutSpin_IgnoresWheelSpinAroundAxle()
		{
			var chassisWorld = StrideMatrix.Identity;
			var localAnchor = new StrideVector3(0f, 0.25f, 0f);
			var translation = new StrideVector3(1f, 2f, 3f);
			var wheelWithoutSpin = StrideMatrix.Identity;
			wheelWithoutSpin.TranslationVector = translation;
			var wheelWithSpin = StrideMatrix.RotationX(1.3f);
			wheelWithSpin.TranslationVector = translation;

			var anchorWithoutSpin = SuspensionVisualKinematicsRig.TransformWheelLocalPositionWithoutSpin(
				chassisWorld,
				wheelWithoutSpin,
				localAnchor);
			var anchorWithSpinRemoved = SuspensionVisualKinematicsRig.TransformWheelLocalPositionWithoutSpin(
				chassisWorld,
				wheelWithSpin,
				localAnchor);
			var anchorWithRawSpin = StrideVector3.TransformCoordinate(localAnchor, wheelWithSpin);

			Assert.Equal(anchorWithoutSpin, anchorWithSpinRemoved);
			Assert.NotEqual(anchorWithoutSpin, anchorWithRawSpin);
		}
	}
}
