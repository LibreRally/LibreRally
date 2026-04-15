using System;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Graphics;

namespace LibreRally.HUD;

/// <summary>
/// Draws a compact row of engine auxiliary gauges (turbo, engine temp, fuel, oil)
/// below the existing <see cref="SpeedoGauge"/> cluster.
/// Register with <c>((Game)Game).GameSystems.Add(strip)</c> from a SyncScript.
/// </summary>
public class EngineGaugeStrip : GameSystemBase
{
    // ── Data sources (set from VehicleSpawner each frame) ────────────────────
    public float TurboBoostBar { get; set; }
    public float TurboMaxBoostBar { get; set; }
    public float EngineTempC { get; set; }
    public float EngineTempDamageC { get; set; } = 180f;
    public float EngineTempTargetC { get; set; } = 85f;
    public float FuelLiters { get; set; }
    public float FuelCapacityLiters { get; set; }
    public float OilPressureBar { get; set; }
    public float OilTempC { get; set; }
    public bool HasTurbo { get; set; }
    public bool HasFuel { get; set; }
    public bool HasOil { get; set; }

    // ── Layout constants ────────────────────────────────────────────────────
    private const float StripHeight = 14f;
    private const float GaugeWidth = 60f;
    private const float GaugeSpacing = 6f;
    private const float LabelWidth = 40f;
    private const float BarHeight = 8f;
    private const float PanelPadding = 8f;
    private const float PanelTopMargin = 6f;

    private SpriteBatch? _sb;
    private Texture? _pixel;
    private SpriteFont? _font;
    private Game? _game;

    public EngineGaugeStrip(IServiceRegistry services) : base(services)
    {
        Enabled = true;
        Visible = true;
        DrawOrder = 9991; // just after SpeedoGauge (9990)
        UpdateOrder = 9991;
    }

    public override void Initialize()
    {
        base.Initialize();
        _game = (Game?)Services.GetService<IGame>();
        if (_game is null)
        {
            return;
        }

        _sb = new SpriteBatch(_game.GraphicsDevice);
        _pixel = CreateWhitePixel(_game.GraphicsDevice);
        _font = _game.Content.Load<SpriteFont>("StrideDefaultFont");
    }

    protected override void Destroy()
    {
        _sb?.Dispose();
        _pixel?.Dispose();
        base.Destroy();
    }

    public override void Draw(GameTime gameTime)
    {
        if (_game == null || _sb == null || _pixel == null || _font == null)
        {
            return;
        }

        var gd = _game.GraphicsDevice;
        var ctx = _game.GraphicsContext;
        var cmd = ctx.CommandList;
        var back = gd.Presenter?.BackBuffer;
        if (back == null)
        {
            return;
        }

        // Count how many gauges we show
        var gaugeCount = CountActiveGauges();
        if (gaugeCount == 0)
        {
            return;
        }

        var sw = back.Width;
        var sh = back.Height;

        // Position: directly beneath the SpeedoGauge backdrop.
        // SpeedoGauge uses: speedCx = sw - 255, rpmCx = sw - 110, cy = sh - 132
        // The SpeedoGauge backdrop extends to approximately cy + BezelRadius(74) + 36 = sh - 22
        var speedCx = sw - 255f;
        var rpmCx = sw - 110f;
        var gaugeClusterLeft = speedCx - 74f - 28f; // matches SpeedoGauge panelLeft
        var gaugeClusterRight = rpmCx + 74f + 16f;  // matches SpeedoGauge panelRight

        // Strip sits just below the speedo backdrop
        var stripTop = sh - 22f + PanelTopMargin;
        var totalGaugeWidth = gaugeCount * (LabelWidth + GaugeWidth) + (gaugeCount - 1) * GaugeSpacing;
        var panelWidth = totalGaugeWidth + PanelPadding * 2f;
        var panelCenterX = (gaugeClusterLeft + gaugeClusterRight) * 0.5f;
        var panelLeft = panelCenterX - panelWidth * 0.5f;
        var panelHeight = StripHeight + PanelPadding * 2f;

        cmd.SetRenderTargetAndViewport(null, back);
        _sb.Begin(ctx, SpriteSortMode.Deferred, BlendStates.AlphaBlend);

        // Backdrop
        DrawRect(new RectangleF(panelLeft + 3f, stripTop + 3f, panelWidth, panelHeight), new Color(0, 0, 0, 50));
        DrawRect(new RectangleF(panelLeft, stripTop, panelWidth, panelHeight), new Color(12, 14, 18, 178));
        DrawRect(new RectangleF(panelLeft + 1f, stripTop + 1f, panelWidth - 2f, panelHeight - 2f), new Color(22, 26, 32, 110));
        DrawRect(new RectangleF(panelLeft, stripTop, panelWidth, 1f), new Color(132, 138, 148, 40));
        DrawRect(new RectangleF(panelLeft, stripTop + panelHeight - 1f, panelWidth, 1f), new Color(6, 7, 10, 160));

        // Draw each active gauge in order
        var x = panelLeft + PanelPadding;
        var y = stripTop + PanelPadding;
        var slotIndex = 0;

        if (HasTurbo)
        {
            DrawGauge(x + SlotOffset(slotIndex), y, "BOOST", TurboBoostBar, TurboMaxBoostBar, GetBoostColor);
            slotIndex++;
        }

        DrawGauge(x + SlotOffset(slotIndex), y, "TEMP", EngineTempC, EngineTempDamageC, GetTempColor);
        slotIndex++;

        if (HasFuel)
        {
            var fuelFrac = FuelCapacityLiters > 0f ? FuelLiters / FuelCapacityLiters : 0f;
            DrawGauge(x + SlotOffset(slotIndex), y, "FUEL", fuelFrac, 1f, GetFuelColor);
            slotIndex++;
        }

        if (HasOil)
        {
            DrawGauge(x + SlotOffset(slotIndex), y, "OIL", OilPressureBar, 5f, GetOilColor);
        }

        _sb.End();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Gauge drawing
    // ─────────────────────────────────────────────────────────────────────────

    private void DrawGauge(float x, float y, string label, float value, float maxValue, Func<float, Color> colorFunc)
    {
        var fraction = maxValue > 0f ? Math.Clamp(value / maxValue, 0f, 1f) : 0f;

        // Label
        _sb!.DrawString(_font!, label, new Vector2(x, y), new Color(170, 176, 186, 200));

        // Bar background
        var barX = x + LabelWidth;
        var barY = y + (StripHeight - BarHeight) * 0.5f;
        DrawRect(new RectangleF(barX, barY, GaugeWidth, BarHeight), new Color(16, 18, 22, 200));
        DrawRect(new RectangleF(barX + 1f, barY + 1f, GaugeWidth - 2f, BarHeight - 2f), new Color(44, 48, 56, 210));

        // Fill
        if (fraction > 0.005f)
        {
            var fillWidth = (GaugeWidth - 2f) * fraction;
            var fillColor = colorFunc(fraction);
            DrawRect(new RectangleF(barX + 1f, barY + 1f, fillWidth, BarHeight - 2f), fillColor);
            // Highlight on top edge
            DrawRect(new RectangleF(barX + 1f, barY + 1f, fillWidth, 1f), new Color(255, 255, 255, 30));
        }

        // Value text (right-aligned inside the bar)
        var valueStr = FormatValue(label, value, maxValue);
        var textX = barX + GaugeWidth - 2f;
        _sb!.DrawString(_font!, valueStr, new Vector2(textX, y), new Color(220, 224, 232, 220), TextAlignment.Right);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Color functions
    // ─────────────────────────────────────────────────────────────────────────

    private static Color GetBoostColor(float fraction)
    {
        // Low boost = blue, high boost = amber/red
        var blue = new Color(88, 178, 255, 230);
        var amber = new Color(244, 196, 68, 230);
        var red = new Color(255, 92, 68, 230);

        if (fraction <= 0.6f)
        {
            return blue;
        }

        if (fraction <= 0.85f)
        {
            return LerpColor(blue, amber, (fraction - 0.6f) / 0.25f);
        }

        return LerpColor(amber, red, (fraction - 0.85f) / 0.15f);
    }

    private Color GetTempColor(float fraction)
    {
        // fraction = temp / damageThreshold
        // Normal operating temp is roughly 85/180 ≈ 0.47
        // Below 0.4 = cold (blue), 0.4–0.7 = normal (green), >0.7 = warm (amber), >0.85 = hot (red)
        var blue = new Color(88, 148, 255, 230);
        var green = new Color(72, 220, 112, 230);
        var amber = new Color(244, 196, 68, 230);
        var red = new Color(255, 76, 60, 230);

        if (fraction <= 0.4f)
        {
            return LerpColor(blue, green, fraction / 0.4f);
        }

        if (fraction <= 0.7f)
        {
            return green;
        }

        if (fraction <= 0.85f)
        {
            return LerpColor(green, amber, (fraction - 0.7f) / 0.15f);
        }

        return LerpColor(amber, red, (fraction - 0.85f) / 0.15f);
    }

    private static Color GetFuelColor(float fraction)
    {
        // Full = green, low = amber, critical = red
        var green = new Color(72, 220, 112, 230);
        var amber = new Color(244, 196, 68, 230);
        var red = new Color(255, 76, 60, 230);

        if (fraction >= 0.3f)
        {
            return green;
        }

        if (fraction >= 0.1f)
        {
            return LerpColor(red, amber, (fraction - 0.1f) / 0.2f);
        }

        return red;
    }

    private static Color GetOilColor(float fraction)
    {
        // fraction = pressure / 5 bar
        // <0.3 (< 1.5 bar) = danger (red), 0.3–0.7 = normal (green), >0.7 = high (amber)
        var red = new Color(255, 76, 60, 230);
        var green = new Color(72, 220, 112, 230);
        var amber = new Color(244, 196, 68, 230);

        if (fraction <= 0.2f)
        {
            return red;
        }

        if (fraction <= 0.3f)
        {
            return LerpColor(red, green, (fraction - 0.2f) / 0.1f);
        }

        if (fraction <= 0.8f)
        {
            return green;
        }

        return LerpColor(green, amber, (fraction - 0.8f) / 0.2f);
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Helpers
    // ─────────────────────────────────────────────────────────────────────────

    private int CountActiveGauges()
    {
        var count = 1; // engine temp is always shown
        if (HasTurbo)
        {
            count++;
        }

        if (HasFuel)
        {
            count++;
        }

        if (HasOil)
        {
            count++;
        }

        return count;
    }

    private float SlotOffset(int slotIndex)
    {
        return slotIndex * (LabelWidth + GaugeWidth + GaugeSpacing);
    }

    private static string FormatValue(string label, float value, float maxValue)
    {
        return label switch
        {
            "BOOST" => $"{value:F2}",
            "TEMP" => $"{value:F0}°",
            "FUEL" => $"{(maxValue > 0f ? value / maxValue * 100f : 0f):F0}%",
            "OIL" => $"{value:F1}",
            _ => $"{value:F1}",
        };
    }

    private void DrawRect(RectangleF rect, Color color)
    {
        _sb!.Draw(_pixel!, new Vector2(rect.X, rect.Y), null, color, 0f, Vector2.Zero, new Vector2(rect.Width, rect.Height));
    }

    private static Color LerpColor(Color a, Color b, float t)
    {
        t = Math.Clamp(t, 0f, 1f);
        return new Color(
            LerpByte(a.R, b.R, t),
            LerpByte(a.G, b.G, t),
            LerpByte(a.B, b.B, t),
            LerpByte(a.A, b.A, t));
    }

    private static byte LerpByte(byte a, byte b, float t)
    {
        return (byte)MathF.Round(a + ((b - a) * Math.Clamp(t, 0f, 1f)));
    }

    private static Texture CreateWhitePixel(GraphicsDevice device)
    {
        using var image = Image.New2D(1, 1, 1, PixelFormat.R8G8B8A8_UNorm);
        var ptr = image.PixelBuffer[0].DataPointer;
        System.Runtime.InteropServices.Marshal.WriteByte(ptr, 0, 255);
        System.Runtime.InteropServices.Marshal.WriteByte(ptr, 1, 255);
        System.Runtime.InteropServices.Marshal.WriteByte(ptr, 2, 255);
        System.Runtime.InteropServices.Marshal.WriteByte(ptr, 3, 255);
        return Texture.New(device, image);
    }
}
