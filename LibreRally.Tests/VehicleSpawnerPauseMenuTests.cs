namespace LibreRally.Tests;

public class VehicleSpawnerPauseMenuTests
{
    [Fact]
    public void PauseMenuEntries_ExposeResetCarRecoveryAction()
    {
        var entries = VehicleSpawner.GetPauseMenuEntries();

        Assert.Collection(
            entries,
            entry =>
            {
                Assert.Equal(PauseMenuAction.ResumeDriving, entry.Action);
                Assert.Equal("Resume Driving", entry.Item.Title);
            },
            entry =>
            {
                Assert.Equal(PauseMenuAction.ResetCar, entry.Action);
                Assert.Equal("Reset Car", entry.Item.Title);
                Assert.Contains("spawn point", entry.Item.Description, StringComparison.OrdinalIgnoreCase);
            },
            entry => Assert.Equal(PauseMenuAction.GarageSetup, entry.Action),
            entry => Assert.Equal(PauseMenuAction.VehicleSelect, entry.Action),
            entry => Assert.Equal(PauseMenuAction.ToggleTelemetryOverlay, entry.Action));
    }

    [Fact]
    public void ResolvePauseMenuAction_ReturnsResetActionForRecoverySlot()
    {
        var action = VehicleSpawner.ResolvePauseMenuAction(1);

        Assert.Equal(PauseMenuAction.ResetCar, action);
    }

    [Fact]
    public void DriverAssistWheelEntries_ExposeAbsTcsAndEscControls()
    {
        var entries = VehicleSpawner.GetDriverAssistWheelEntries();

        Assert.Collection(
            entries,
            entry =>
            {
                Assert.Equal(DriverAssistControl.Abs, entry.Control);
                Assert.Equal("ABS", entry.Title);
            },
            entry =>
            {
                Assert.Equal(DriverAssistControl.TractionControl, entry.Control);
                Assert.Equal("TCS", entry.Title);
            },
            entry =>
            {
                Assert.Equal(DriverAssistControl.StabilityControl, entry.Control);
                Assert.Equal("ESC", entry.Title);
            });
    }

    [Fact]
    public void ResolveDriverAssistAction_ReturnsEscForFinalWheelSlot()
    {
        var action = VehicleSpawner.ResolveDriverAssistAction(2);

        Assert.Equal(DriverAssistControl.StabilityControl, action);
    }
}
