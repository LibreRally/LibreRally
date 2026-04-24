using LibreRally.HUD;
using LibreRally.Vehicle;
using LibreRally.Vehicle.JBeam;
using Stride.Engine;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the setup ui shell model behavior.
	/// </summary>
	public class SetupUiShellModelTests
	{
		/// <summary>
		/// Verifies that create from vehicle setup uses overrides and creates apply payload.
		/// </summary>
		[Fact]
		public void CreateFromVehicleSetup_UsesOverridesAndCreatesApplyPayload()
		{
			var definition = new VehicleDefinition
			{
				VehicleName = "Test Car",
				SetupVariables =
				[
					new JBeamVariableDefinition
					{
						Name = "spring_F",
						Category = "Suspension",
						SubCategory = "Front",
						Title = "Spring Rate",
						Description = "Front axle spring stiffness.",
						DefaultValue = 60000f,
						MinValue = 40000f,
						MaxValue = 80000f,
						StepDisplayValue = 500f,
					},
				],
				PressureWheelOptions =
				[
					new AssembledPressureWheelOptions(
						"wheeldata_front",
						"wheeldata_F",
						new JBeamPressureWheelOptions { PressurePsi = 27f, }),
				],
			};
			definition.Vars["spring_F_asphalt"] = 62000f;

			var overrides = new VehicleSetupOverrides();
			overrides.VariableOverrides["spring_F_asphalt"] = 63000f;
			overrides.PressureWheelOverrides[VehicleSetupAxle.Front] = new VehiclePressureWheelOverrides { PressurePsi = 29f, };

			var shell = SetupUiShellModel.CreateFromVehicleSetup(CreateLoadedVehicle(definition), overrides, "Test Car", "Ready");
			var suspensionField = Assert.Single(shell.Categories.Single(category => category.Title == "Suspension").Fields);
			var pressureField = Assert.Single(shell.Categories.Single(category => category.Title == "Tyres & Pressures").Fields);

			Assert.Equal(63000f, suspensionField.RawNumericValue, 3);
			Assert.Equal(29f, pressureField.RawNumericValue, 3);

			suspensionField.SetNumericValue(65000f);
			pressureField.SetNumericValue(30f);
			var payload = shell.CreateApplyPayload();

			Assert.Equal(65000f, payload.VariableOverrides["spring_F_asphalt"], 3);
			Assert.Equal(30f, payload.PressureOverrides[VehicleSetupAxle.Front], 3);
		}

		/// <summary>
		/// Verifies that create from vehicle setup uses display range hints for percent like variables.
		/// </summary>
		[Fact]
		public void CreateFromVehicleSetup_UsesDisplayRangeHintsForPercentLikeVariables()
		{
			var definition = new VehicleDefinition
			{
				VehicleName = "Brake Test",
				SetupVariables =
				[
					new JBeamVariableDefinition
					{
						Name = "brakestrength",
						Category = "Brakes",
						Title = "Brake Force",
						DefaultValue = 1f,
						MinValue = 0.2f,
						MaxValue = 1f,
						MinDisplayValue = 20f,
						MaxDisplayValue = 100f,
					},
				],
			};
			definition.Vars["brakestrength"] = 0.9f;

			var shell = SetupUiShellModel.CreateFromVehicleSetup(CreateLoadedVehicle(definition), new VehicleSetupOverrides());
			var field = Assert.Single(shell.Categories.Single(category => category.Title == "Brakes").Fields);

			Assert.Equal(90f, field.NumericValue, 3);
			Assert.Equal(20f, field.Minimum, 3);
			Assert.Equal(100f, field.Maximum, 3);
			Assert.Equal("90 %", field.DisplayValue);
			Assert.Equal(0.9f, field.RawNumericValue, 3);
		}

		private static LoadedVehicle CreateLoadedVehicle(VehicleDefinition definition)
		{
			return new LoadedVehicle(
				definition,
				new Entity("root"),
				new RallyCarComponent(),
				new Entity("chassis"),
				new Entity("wheel_fl"),
				new Entity("wheel_fr"),
				new Entity("wheel_rl"),
				new Entity("wheel_rr"),
				new VehicleLoadDiagnostics(definition.FolderPath, null, 0f));
		}
	}
}
