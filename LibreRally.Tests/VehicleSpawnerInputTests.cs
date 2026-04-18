namespace LibreRally.Tests;

public class VehicleSpawnerInputTests
{
    [Fact]
    public void IsVehicleMenuToggleRequested_AcceptsControllerStart()
    {
        bool requested = VehicleSpawner.IsVehicleMenuToggleRequested(
            keyboardTogglePressed: false,
            controllerStartPressed: true);

        Assert.True(requested);
    }

    [Fact]
    public void IsVehicleMenuConfirmRequested_AcceptsControllerA()
    {
        bool requested = VehicleSpawner.IsVehicleMenuConfirmRequested(
            keyboardConfirmPressed: false,
            controllerConfirmPressed: true);

        Assert.True(requested);
    }

    [Fact]
    public void IsVehicleMenuCancelRequested_AcceptsControllerB()
    {
        bool requested = VehicleSpawner.IsVehicleMenuCancelRequested(
            keyboardCancelPressed: false,
            controllerCancelPressed: true);

        Assert.True(requested);
    }

    [Fact]
    public void IsVehicleMenuMoveDownRequested_AcceptsControllerDPadDown()
    {
        bool requested = VehicleSpawner.IsVehicleMenuMoveDownRequested(
            keyboardDownPressed: false,
            controllerDownPressed: true);

        Assert.True(requested);
    }

    [Fact]
    public void IsVehicleMenuMoveUpRequested_AcceptsControllerDPadUp()
    {
        bool requested = VehicleSpawner.IsVehicleMenuMoveUpRequested(
            keyboardUpPressed: false,
            controllerUpPressed: true);

        Assert.True(requested);
    }
}
