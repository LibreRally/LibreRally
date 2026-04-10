using System;
using LibreRally.Vehicle;
using LibreRally.Vehicle.Physics;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Graphics;

namespace LibreRally.HUD;

public sealed class VehicleTelemetryOverlay : GameSystemBase
{
    private static readonly string[] WheelNames = ["FL", "FR", "RL", "RR"];

    private const float PanelX = 18f;
    private const float PanelY = 54f;
    private const float PanelWidth = 640f;
    private const float PanelHeight = 254f;
    private const float BarX = PanelX + 112f;
    private const float BarWidth = 220f;
    private const float BarHeight = 10f;

    public RallyCarComponent? Car { get; set; }
    public string StatusText { get; set; } = "Waiting for vehicle...";
    public bool OverlayVisible { get; set; } = true;

    private Game? _game;
    private SpriteBatch? _spriteBatch;
    private Texture? _pixel;
    private SpriteFont? _font;

    public VehicleTelemetryOverlay(IServiceRegistry services) : base(services)
    {
        Enabled = true;
        Visible = true;
        DrawOrder = 9995;
        UpdateOrder = 9995;
    }

    public override void Initialize()
    {
        base.Initialize();

        _game = (Game)Services.GetService<IGame>();
        _spriteBatch = new SpriteBatch(_game.GraphicsDevice);
        _pixel = CreateWhitePixel(_game.GraphicsDevice);
        _font = _game.Content.Load<SpriteFont>("StrideDefaultFont");
    }

    protected override void Destroy()
    {
        _spriteBatch?.Dispose();
        _pixel?.Dispose();
        base.Destroy();
    }

    public override void Draw(GameTime gameTime)
    {
        if (!OverlayVisible || _game == null || _spriteBatch == null || _pixel == null || _font == null)
        {
	        return;
        }

        var backBuffer = _game.GraphicsDevice.Presenter?.BackBuffer;
        if (backBuffer == null)
        {
	        return;
        }

        var context = _game.GraphicsContext;
        context.CommandList.SetRenderTargetAndViewport(null, backBuffer);

        _spriteBatch.Begin(context, SpriteSortMode.Deferred, BlendStates.AlphaBlend);
        DrawBackdrop();
        DrawInputBars();
        DrawTelemetryText();
        _spriteBatch.End();
    }

    private void DrawBackdrop()
    {
        DrawRect(new RectangleF(PanelX + 4f, PanelY + 5f, PanelWidth, PanelHeight), new Color(0, 0, 0, 56));
        DrawRect(new RectangleF(PanelX, PanelY, PanelWidth, PanelHeight), new Color(12, 14, 18, 198));
        DrawRect(new RectangleF(PanelX + 2f, PanelY + 2f, PanelWidth - 4f, PanelHeight - 4f), new Color(22, 26, 32, 146));
        DrawRect(new RectangleF(PanelX, PanelY, PanelWidth, 2f), new Color(138, 144, 154, 52));
        DrawRect(new RectangleF(PanelX, PanelY + PanelHeight - 2f, PanelWidth, 2f), new Color(0, 0, 0, 150));
        DrawRect(new RectangleF(PanelX, PanelY, 2f, PanelHeight), new Color(88, 94, 104, 32));
        DrawRect(new RectangleF(PanelX + PanelWidth - 2f, PanelY, 2f, PanelHeight), new Color(0, 0, 0, 112));
    }

    private void DrawInputBars()
    {
        float barBaseY = PanelY + 86f;
        DrawBar(barBaseY, Car?.ThrottleInput ?? 0f, new Color(68, 218, 102, 220));
        DrawBar(barBaseY + 18f, Car?.BrakeInput ?? 0f, new Color(255, 92, 70, 220));

        float steerFraction = Car == null ? 0.5f : Math.Clamp((Car.SteeringInput + 1f) * 0.5f, 0f, 1f);
        DrawCenteredBar(barBaseY + 36f, steerFraction, new Color(118, 214, 255, 220));
    }

    private void DrawBar(float y, float value, Color fillColor)
    {
        DrawRect(new RectangleF(BarX, y, BarWidth, BarHeight), new Color(16, 18, 22, 194));
        DrawRect(new RectangleF(BarX + 1f, y + 1f, BarWidth - 2f, BarHeight - 2f), new Color(42, 46, 54, 205));

        float clamped = Math.Clamp(value, 0f, 1f);
        if (clamped > 0.001f)
        {
            float innerWidth = (BarWidth - 2f) * clamped;
            DrawRect(new RectangleF(BarX + 1f, y + 1f, innerWidth, BarHeight - 2f), fillColor);
        }

        DrawRect(new RectangleF(BarX + 1f, y + 1f, BarWidth - 2f, 1f), new Color(255, 255, 255, 24));
    }

    private void DrawCenteredBar(float y, float value, Color fillColor)
    {
        DrawRect(new RectangleF(BarX, y, BarWidth, BarHeight), new Color(16, 18, 22, 194));
        DrawRect(new RectangleF(BarX + 1f, y + 1f, BarWidth - 2f, BarHeight - 2f), new Color(42, 46, 54, 205));

        float midX = BarX + (BarWidth * 0.5f);
        DrawRect(new RectangleF(midX - 1f, y + 1f, 2f, BarHeight - 2f), new Color(255, 255, 255, 42));

        float clamped = Math.Clamp((value - 0.5f) * 2f, -1f, 1f);
        if (Math.Abs(clamped) > 0.001f)
        {
            float halfWidth = (BarWidth - 2f) * 0.5f;
            float fillWidth = halfWidth * Math.Abs(clamped);
            float fillX = clamped >= 0f ? midX : midX - fillWidth;
            DrawRect(new RectangleF(fillX, y + 1f, fillWidth, BarHeight - 2f), fillColor);
        }

        DrawRect(new RectangleF(BarX + 1f, y + 1f, BarWidth - 2f, 1f), new Color(255, 255, 255, 24));
    }

    private void DrawTelemetryText()
    {
        int x = (int)PanelX + 14;
        int y = (int)PanelY + 12;
        const int line = 16;

        DrawText("Vehicle Telemetry (F3)", x, y, new Color(236, 240, 247, 255));
        y += line;
        DrawText(StatusText, x, y, new Color(184, 192, 203, 255));
        y += line;

        if (Car == null)
        {
            DrawText("Telemetry unavailable: vehicle not loaded.", x, y, Color.White);
            return;
        }

        DrawText(
            $"Speed {Car.SpeedKmh,6:F1} km/h | Gear {FormatGear(Car.CurrentGear)} | Engine {Car.EngineRpm,6:F0} rpm | Driveline {Car.DrivelineRpm,6:F0} rpm",
            x,
            y,
            Color.White);
        y += line;

        DrawText(
            $"Body fwd {Car.ForwardSpeedMs:F2} m/s | lat {Car.LateralSpeedMs:F2} m/s | yaw {Car.YawRateRad:F2} rad/s | HB {(Car.HandbrakeEngaged ? "ON" : "OFF")}",
            x,
            y,
            Color.White);
        y += line + 8;

        DrawText($"Throttle {Car.ThrottleInput:F2}", x, y, new Color(150, 242, 173, 255));
        y += 18;
        DrawText($"Brake    {Car.BrakeInput:F2}", x, y, new Color(255, 148, 132, 255));
        y += 18;
        DrawText($"Steer    {Car.SteeringInput:F2} | Rack {Car.SteeringRack:F2}", x, y, new Color(160, 227, 255, 255));
        y += 22;

        DrawText(
            $"Inputs D {Car.DriveInput:F2} | SB {Car.ServiceBrakeInput:F2} | T {Car.ThrottleInput:F2} | B {Car.BrakeInput:F2}",
            x,
            y,
            new Color(214, 219, 227, 255));
        y += line;

        var dynamics = Car.Dynamics;
        if (dynamics == null)
        {
            DrawText("Wheel dynamics unavailable.", x, y, Color.White);
            return;
        }

        DrawText("g=grounded load/Fx/Fy=kN tq=Nm om=rad/s sus=mm", x, y, new Color(214, 219, 227, 255));
        y += line;

        for (int i = 0; i < VehicleDynamicsSystem.WheelCount; i++)
        {
            DrawText(
                $"{WheelNames[i]} g:{(dynamics.WheelGrounded[i] ? "Y" : "N")} " +
                $"load:{dynamics.CurrentNormalLoads[i] / 1000f,5:F2} " +
                $"fx:{dynamics.LongitudinalForces[i] / 1000f,6:F2} " +
                $"fy:{dynamics.LateralForces[i] / 1000f,6:F2} " +
                $"tq:{dynamics.WheelDriveTorques[i],6:F0} " +
                $"om:{dynamics.WheelStates[i].AngularVelocity,6:F1} " +
                $"sus:{dynamics.SuspensionCompression[i] * 1000f,6:F0}",
                x,
                y,
                Color.White);
            y += line;
        }
    }

    private void DrawText(string message, int x, int y, Color color)
    {
        _spriteBatch!.DrawString(_font!, message, new Vector2(x, y), color, TextAlignment.Left);
    }

    private void DrawRect(RectangleF rect, Color color)
    {
        _spriteBatch!.Draw(_pixel!, new Vector2(rect.X, rect.Y), null, color, 0f, Vector2.Zero, new Vector2(rect.Width, rect.Height));
    }

    private static string FormatGear(int currentGear)
    {
        return currentGear <= 0 ? "R" : currentGear.ToString();
    }

    private static Texture CreateWhitePixel(GraphicsDevice device)
    {
        using var image = Image.New2D(1, 1, 1, PixelFormat.R8G8B8A8_UNorm);
        var pixels = image.GetPixelBuffer(0, 0).GetPixels<byte>();
        pixels[0] = 255;
        pixels[1] = 255;
        pixels[2] = 255;
        pixels[3] = 255;
        return Texture.New(device, image);
    }
}
