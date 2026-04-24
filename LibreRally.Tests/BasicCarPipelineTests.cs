using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;
using LibreRally.Vehicle.Physics;
using LibreRally.Vehicle.Rendering;
using Stride.BepuPhysics.Constraints;
using Stride.BepuPhysics.Definitions;
using Stride.Core.Mathematics;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the basic car pipeline behavior.
	/// </summary>
	public class BasicCarPipelineTests
	{
		private static string CombineRelativePath(string basePath, string relativePath)
		{
			if (Path.IsPathRooted(relativePath))
			{
				throw new ArgumentException("Path must be relative.", nameof(relativePath));
			}

			return Path.Combine(basePath, relativePath);
		}

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

		/// <summary>
		/// Verifies that basic car should select template part tree.
		/// </summary>
		[Fact]
		public void BasicCar_ShouldSelectTemplatePartTree()
		{
			PcConfig config = PcConfigLoader.Load(CombineRelativePath(GetVehicleFolder(), "basic_car.pc"));

			Assert.Equal("TutoFormulaBee", config.MainPartName);
			Assert.Equal("TutoFormulaBee_Chassis", config.Parts["TutoFormulaBee_Chassis"]);
			Assert.Equal("TutoFormulaBee_transaxle_5M", config.Parts["TutoFormulaBee_transaxle"]);
			Assert.Equal("TutoFormulaBee_swaybar_F_race", config.Parts["TutoFormulaBee_swaybar_F"]);
			Assert.Equal(string.Empty, config.Parts["wheel_F_4"]);
			Assert.Equal(3.12f, config.Vars["gear_1"]);
		}

		/// <summary>
		/// Verifies that basic car should parse template nodes and wheel groups.
		/// </summary>
		[Fact]
		public void BasicCar_ShouldParseTemplateNodesAndWheelGroups()
		{
			PcConfig config = PcConfigLoader.Load(CombineRelativePath(GetVehicleFolder(), "basic_car.pc"));
			JBeamPart chassisPart = Assert.Single(
				JBeamParser.ParseFile(CombineRelativePath(GetVehicleFolder(), "TutoFormulaBee_chassis.jbeam"), config.Vars),
				part => part.Name == "TutoFormulaBee_Chassis");
			JBeamPart frontBrakes = Assert.Single(
				JBeamParser.ParseFile(CombineRelativePath(GetVehicleFolder(), "TutoFormulaBee_brakes.jbeam"), config.Vars),
				part => part.Name == "TutoFormulaBee_brake_F");
			JBeamPart frontWheelData = Assert.Single(
				JBeamParser.ParseFile(CombineRelativePath(GetVehicleFolder(), "TutoFormulaBee_suspension_F.jbeam"), config.Vars),
				part => part.Name == "TutoFormulaBee_wheeldata_F");

			Assert.Equal("TutoFormulaBee_Chassis", chassisPart.Name);
			Assert.True(chassisPart.Nodes.Count >= 1);
			Assert.Equal("fr4", chassisPart.RefNodes["ref"]);
			Assert.DoesNotContain(chassisPart.FlexBodies, flexBody => flexBody.Mesh == "TutoFBee_Spaceframe");

			Assert.True(frontBrakes.FlexBodies.Count >= 2);
			Assert.Contains(frontBrakes.FlexBodies, flexBody => flexBody.Groups.Contains("wheel_FL"));
			Assert.Contains(frontBrakes.FlexBodies, flexBody => flexBody.Groups.Contains("wheel_FR"));
			Assert.Contains(frontWheelData.PressureWheels, wheel => wheel.WheelKey == "wheel_FL" && wheel.NodeArm == "fh5l");
			Assert.Contains(frontWheelData.PressureWheels, wheel => wheel.WheelKey == "wheel_FR" && wheel.NodeArm == "fh5r");
		}

		/// <summary>
		/// Verifies that basic car should assemble vehicle.
		/// </summary>
		[Fact]
		public void BasicCar_ShouldAssembleVehicle()
		{
			PcConfig config = PcConfigLoader.Load(CombineRelativePath(GetVehicleFolder(), "basic_car.pc"));
			VehicleDefinition definition = JBeamAssembler.Assemble(GetVehicleFolder(), config);

			Assert.Equal("TutoFormulaBee", definition.VehicleName);
			Assert.True(definition.Nodes.Count >= 20);
			Assert.True(definition.Beams.Count >= 40);
			Assert.True(definition.FlexBodies.Count >= 10);
			Assert.Equal("fr4", definition.RefNodes["ref"]);
			Assert.Equal(52000f, definition.Vars["spring_F_asphalt"]);
			Assert.Equal(30000f, definition.Vars["spring_F"]);
			Assert.Equal(20000f, definition.Vars["arb_spring_F"]);
			Assert.Equal(4.125f, definition.Vars["finaldrive_R"], 3);
			Assert.DoesNotContain(definition.FlexBodies, flexBody => flexBody.MeshName == "TutoFBee_Spaceframe");
			Assert.Contains(definition.FlexBodies, flexBody => flexBody.MeshName == "autobello_brakedisk_track_FR");
			Assert.Contains(definition.FlexBodies, flexBody => flexBody.MeshName == "autobello_muffler_b");
			Assert.Contains(definition.FlexBodies, flexBody => flexBody.MeshName == "autobello_shifter_5M");
			Assert.NotNull(definition.Engine);
			Assert.NotNull(definition.Gearbox);
			Assert.NotNull(definition.VehicleController);
			Assert.Contains(definition.PowertrainDevices, device => device.Name == "wheelaxleRL" && device.ConnectedWheel == "RL");
			Assert.Contains(definition.PressureWheels, wheel => wheel.WheelKey == "wheel_FL");
			Assert.Contains(definition.PressureWheels, wheel => wheel.WheelKey == "wheel_RR");

			AssembledNode hubNode = definition.Nodes["fh1l"];
			Assert.Contains("hub_F", hubNode.Groups);
		}

		/// <summary>
		/// Verifies that basic car should resolve template rear drive powertrain.
		/// </summary>
		[Fact]
		public void BasicCar_ShouldResolveTemplateRearDrivePowertrain()
		{
			PcConfig config = PcConfigLoader.Load(CombineRelativePath(GetVehicleFolder(), "basic_car.pc"));
			VehicleDefinition definition = JBeamAssembler.Assemble(GetVehicleFolder(), config);
			VehiclePowertrainSetup powertrain = VehiclePowertrainResolver.Resolve(definition);

			Assert.False(powertrain.DriveFrontAxle);
			Assert.True(powertrain.DriveRearAxle);
			Assert.Equal(new[] { "wheel_RL", "wheel_RR" }, powertrain.DrivenWheelKeys);
			Assert.Equal(4.125f, powertrain.FinalDrive, 3);
			Assert.Equal(5050f, powertrain.MaxRpm, 0);
			Assert.Equal(875f, powertrain.IdleRpm, 0);
			Assert.Equal(2200f, powertrain.AutoClutchLaunchRpm, 0);
			Assert.Equal(4300f, powertrain.ShiftUpRpm, 0);
			Assert.InRange(powertrain.ShiftDownRpm, 1700f, 2200f);
			Assert.Equal(6, powertrain.GearRatios.Length);
			Assert.Equal(3.60f, powertrain.GearRatios[0], 2);
			Assert.Equal(3.12f, powertrain.GearRatios[1], 2);
			Assert.Equal(0.89f, powertrain.GearRatios[^1], 2);
		}

		/// <summary>
		/// Verifies that basic car should build physics.
		/// </summary>
		[Fact]
		public void BasicCar_ShouldBuildPhysics()
		{
			PcConfig config = PcConfigLoader.Load(CombineRelativePath(GetVehicleFolder(), "basic_car.pc"));
			VehicleDefinition definition = JBeamAssembler.Assemble(GetVehicleFolder(), config);
			VehicleBuilderResult result = VehiclePhysicsBuilder.Build(definition);

			WheelSettings? wheelFlSettings = result.WheelFL.Get<WheelSettings>();
			WheelSettings? wheelFrSettings = result.WheelFR.Get<WheelSettings>();
			WheelSettings? wheelRlSettings = result.WheelRL.Get<WheelSettings>();
			WheelSettings? wheelRrSettings = result.WheelRR.Get<WheelSettings>();
			var wheelFlBody = result.WheelFL.Get<Stride.BepuPhysics.BodyComponent>();
			var chassisBody = result.ChassisEntity.Get<Stride.BepuPhysics.BodyComponent>();

			Assert.Equal("TutoFormulaBee", result.RootEntity.Name);
			Assert.Equal("chassis", result.ChassisEntity.Name);
			Assert.NotNull(wheelFlSettings);
			Assert.NotNull(wheelFrSettings);
			Assert.NotNull(wheelRlSettings);
			Assert.NotNull(wheelRrSettings);
			Assert.NotNull(wheelFlBody);
			Assert.NotNull(chassisBody);
			Assert.Equal("wheel_FL", result.WheelFL.Name);
			Assert.Equal("wheel_FR", result.WheelFR.Name);
			Assert.Equal("wheel_RL", result.WheelRL.Name);
			Assert.Equal("wheel_RR", result.WheelRR.Name);
			Assert.Equal(0.05f, wheelFlBody.FrictionCoefficient, 3);
			Assert.Equal(InterpolationMode.Interpolated, wheelFlBody.InterpolationMode);
			Assert.Equal(InterpolationMode.Interpolated, chassisBody.InterpolationMode);

			foreach (WheelSettings wheelSettings in new[] { wheelFlSettings, wheelFrSettings, wheelRlSettings, wheelRrSettings })
			{
				Assert.Equal(Vector3.Zero, wheelSettings.SuspensionLocalOffsetB);
				Assert.InRange(wheelSettings.SuspensionLocalAxis.Length(), 0.99f, 1.01f);
				Assert.Equal(0f, wheelSettings.SuspensionTargetOffset, 3);
				Assert.True(wheelSettings.SuspensionMinimumOffset < 0f);
				Assert.True(wheelSettings.SuspensionMaximumOffset > 0f);
			}

			LinearAxisLimitConstraintComponent? flLimit = result.WheelFL.Get<LinearAxisLimitConstraintComponent>();
			Assert.NotNull(flLimit);
			Assert.Equal(wheelFlSettings.SuspensionLocalOffsetA, flLimit.LocalOffsetA);
			Assert.Equal(wheelFlSettings.SuspensionLocalOffsetB, flLimit.LocalOffsetB);
			Assert.Equal(wheelFlSettings.SuspensionLocalAxis, flLimit.LocalAxis);
			Assert.Equal(wheelFlSettings.SuspensionMinimumOffset, flLimit.MinimumOffset);
			Assert.Equal(wheelFlSettings.SuspensionMaximumOffset, flLimit.MaximumOffset);
		}

		/// <summary>
		/// Verifies that basic car should apply signed spring height to suspension target offset.
		/// </summary>
		[Fact]
		public void BasicCar_ShouldApplySignedSpringHeightToSuspensionTargetOffset()
		{
			PcConfig config = PcConfigLoader.Load(CombineRelativePath(GetVehicleFolder(), "basic_car.pc"));
			VehicleDefinition definition = JBeamAssembler.Assemble(GetVehicleFolder(), config);
			definition.Vars["springheight_F"] = -0.03f;
			definition.Vars["springheight_R"] = -0.02f;
			VehicleBuilderResult result = VehiclePhysicsBuilder.Build(definition);

			var fl = Assert.IsType<WheelSettings>(result.WheelFL.Get<WheelSettings>());
			var fr = Assert.IsType<WheelSettings>(result.WheelFR.Get<WheelSettings>());
			var rl = Assert.IsType<WheelSettings>(result.WheelRL.Get<WheelSettings>());
			var rr = Assert.IsType<WheelSettings>(result.WheelRR.Get<WheelSettings>());

			Assert.Equal(0.03f, fl.SuspensionTargetOffset, 3);
			Assert.Equal(0.03f, fr.SuspensionTargetOffset, 3);
			Assert.Equal(0.02f, rl.SuspensionTargetOffset, 3);
			Assert.Equal(0.02f, rr.SuspensionTargetOffset, 3);
		}

		/// <summary>
		/// Verifies that basic car should include formula bee collada mesh.
		/// </summary>
		[Fact]
		public void BasicCar_ShouldIncludeFormulaBeeColladaMesh()
		{
			string daePath = CombineRelativePath(GetVehicleFolder(), "FormulaBeeModel.dae");

			Assert.True(File.Exists(daePath));
			Assert.NotEmpty(ColladaLoader.Load(daePath));
		}

		/// <summary>
		/// Verifies that basic car should match brake disc flexbody names to collada geometry.
		/// </summary>
		[Fact]
		public void BasicCar_ShouldMatchBrakeDiscFlexbodyNamesToColladaGeometry()
		{
			string daePath = CombineRelativePath(GetVehicleFolder(), "FormulaBeeModel.dae");
			var meshes = ColladaLoader.Load(daePath);

			Assert.Contains(meshes, mesh => VehicleLoader.MatchesColladaMesh(mesh, "autobello_brakedisk_track_FL"));
			Assert.Contains(meshes, mesh => VehicleLoader.MatchesColladaMesh(mesh, "autobello_brakedisk_track_FR"));
			Assert.Contains(meshes, mesh => VehicleLoader.MatchesColladaMesh(mesh, "autobello_brakedisk_track_RL"));
			Assert.Contains(meshes, mesh => VehicleLoader.MatchesColladaMesh(mesh, "autobello_brakedisk_track_RR"));
		}
	}
}
