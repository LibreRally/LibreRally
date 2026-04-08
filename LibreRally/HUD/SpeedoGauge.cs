using System;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Graphics;

namespace LibreRally.HUD;

/// <summary>
/// Draws a polished vector-art dual-dial gauge cluster to the back-buffer each frame.
/// Register with <c>((Game)Game).GameSystems.Add(gauge)</c> from a SyncScript.
/// </summary>
public class SpeedoGauge : GameSystemBase
{
    // ── Data sources (set from SyncScript each frame) ────────────────────────
    public float SpeedKmh { get; set; }
    public float MaxSpeedKmh { get; set; } = 220f;
    public float EngineRpm { get; set; }
    public float MaxRpm { get; set; } = 7500f;
    public int CurrentGear { get; set; } = 1;
    public float ThrottleInput { get; set; }
    public float BrakeInput { get; set; }

    // ── Gauge geometry (screen coordinates, updated in Draw from backbuffer size) ──
    private const float StartDeg = 225f;
    private const float SweepDeg = -270f;
    private const int SpeedTicks = 11;
    private const int RpmTicks = 8;
    private const float SpeedCautionFrac = 0.55f;
    private const float SpeedWarningFrac = 0.80f;
    private const float RpmAmberFrac = 0.72f;
    private const float RpmRedlineFrac = 0.86f;
    private const float BezelRadius = 74f;
    private const float FaceRadius = 64f;
    private const float TrackRadius = 56f;
    private const float TrackThickness = 11f;
    private const float TickOuterRadius = 68f;
    private const float MajorTickInnerRadius = 54f;
    private const float MinorTickInnerRadius = 60f;
    private const float NeedleLength = 56f;
    private const float NeedleTailLength = 14f;

    private SpriteBatch? _sb;
    private Texture? _pixel;
    private Game? _game;

    public SpeedoGauge(IServiceRegistry services) : base(services)
    {
        Enabled = true;
        Visible = true;
        DrawOrder = 9990; // after scene, before DebugText (which is 10000)
        UpdateOrder = 9990;
    }

    public override void Initialize()
    {
        base.Initialize();
        _game = (Game)Services.GetService<IGame>();
        _sb = new SpriteBatch(_game.GraphicsDevice);
        _pixel = CreateWhitePixel(_game.GraphicsDevice);
    }

    protected override void Destroy()
    {
        _sb?.Dispose();
        _pixel?.Dispose();
        base.Destroy();
    }

    public override void Draw(GameTime gameTime)
    {
        if (_game == null || _sb == null || _pixel == null) return;

        var gd = _game.GraphicsDevice;
        var ctx = _game.GraphicsContext;
        var cmd = ctx.CommandList;
        var back = gd.Presenter?.BackBuffer;
        if (back == null) return;

        int sw = back.Width;
        int sh = back.Height;

        float speedCx = sw - 255f;
        float rpmCx = sw - 110f;
        float cy = sh - 132f;
        float midCx = (speedCx + rpmCx) * 0.5f;

        float panelLeft = speedCx - BezelRadius - 28f;
        float panelTop = cy - BezelRadius - 20f;
        float panelRight = rpmCx + BezelRadius + 16f;
        float panelBottom = cy + BezelRadius + 36f;

        cmd.SetRenderTargetAndViewport(null, back);
        _sb.Begin(ctx, SpriteSortMode.Deferred, BlendStates.AlphaBlend);

        DrawBackdrop(panelLeft, panelTop, panelRight - panelLeft, panelBottom - panelTop);
        DrawSpeedDial(speedCx, cy);
        DrawRpmDial(rpmCx, cy);
        DrawPedalBars(midCx - 72f, cy + 88f, 144f, 8f);

        _sb.End();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Drawing routines
    // ─────────────────────────────────────────────────────────────────────────

    private void DrawBackdrop(float x, float y, float w, float h)
    {
        DrawRect(new RectangleF(x + 4f, y + 5f, w, h), new Color(0, 0, 0, 60));
        DrawRect(new RectangleF(x, y, w, h), new Color(12, 14, 18, 182));
        DrawRect(new RectangleF(x + 2f, y + 2f, w - 4f, h - 4f), new Color(22, 26, 32, 118));
        DrawRect(new RectangleF(x, y, w, 2f), new Color(132, 138, 148, 52));
        DrawRect(new RectangleF(x, y + h - 2f, w, 2f), new Color(6, 7, 10, 180));
        DrawRect(new RectangleF(x, y, 2f, h), new Color(72, 78, 88, 40));
        DrawRect(new RectangleF(x + w - 2f, y, 2f, h), new Color(6, 7, 10, 150));
    }

    private void DrawSpeedDial(float cx, float cy)
    {
        float speedFrac = Math.Clamp(SpeedKmh / MaxSpeedKmh, 0f, 1f);

        DrawDialBase(cx, cy);
        DrawArcBandFraction(cx, cy, TrackRadius, TrackThickness, SpeedWarningFrac, 1f, new Color(118, 44, 30, 78), true);
        DrawValueBand(cx, cy, speedFrac, GetSpeedColor);
        DrawTicks(cx, cy, SpeedTicks, 1);
        DrawNeedle(cx, cy, speedFrac, new Color(255, 148, 78, 255));
        DrawCentreCap(cx, cy, new Color(255, 148, 78, 255));
    }

    private void DrawRpmDial(float cx, float cy)
    {
        float rpmFrac = Math.Clamp(EngineRpm / MaxRpm, 0f, 1f);

        DrawDialBase(cx, cy);
        DrawArcBandFraction(cx, cy, TrackRadius, TrackThickness, RpmAmberFrac, RpmRedlineFrac, new Color(126, 92, 24, 84), true);
        DrawArcBandFraction(cx, cy, TrackRadius, TrackThickness, RpmRedlineFrac, 1f, new Color(142, 42, 34, 96), true);
        DrawValueBand(cx, cy, rpmFrac, GetRpmColor);
        DrawTicks(cx, cy, RpmTicks, 1);
        DrawNeedle(cx, cy, rpmFrac, new Color(118, 214, 255, 255));
        DrawCentreCap(cx, cy, new Color(118, 214, 255, 255));

        var markerStart = PointOnCircle(cx, cy, TickOuterRadius - 16f, StartDeg + SweepDeg * RpmRedlineFrac);
        var markerEnd = PointOnCircle(cx, cy, TickOuterRadius + 1f, StartDeg + SweepDeg * RpmRedlineFrac);
        DrawLine(markerStart + new Vector2(1f, 1f), markerEnd + new Vector2(1f, 1f), new Color(0, 0, 0, 90), 5f);
        DrawLine(markerStart, markerEnd, new Color(255, 88, 74, 255), 3f);
    }

    private void DrawDialBase(float cx, float cy)
    {
        DrawFilledCircle(cx + 2f, cy + 3f, BezelRadius, new Color(0, 0, 0, 70));
        DrawFilledCircle(cx, cy, BezelRadius, new Color(10, 12, 15, 235));
        DrawCircleStroke(cx, cy, BezelRadius - 1f, 2f, new Color(128, 134, 144, 64));
        DrawFilledCircle(cx, cy, FaceRadius, new Color(24, 28, 34, 248));
        DrawCircleStroke(cx, cy, FaceRadius - 1.5f, 1.5f, new Color(255, 255, 255, 22));
        DrawArcBand(cx, cy, TrackRadius, TrackThickness + 4f, StartDeg, SweepDeg, new Color(0, 0, 0, 32), true);
        DrawArcBand(cx, cy, TrackRadius, TrackThickness, StartDeg, SweepDeg, new Color(60, 65, 74, 210), true);
        DrawArcBand(cx, cy, TrackRadius - (TrackThickness * 0.33f), 1.5f, StartDeg, SweepDeg, new Color(255, 255, 255, 20), true);
    }

    private void DrawValueBand(float cx, float cy, float fraction, Func<float, Color> colorAtFraction)
    {
        fraction = Math.Clamp(fraction, 0f, 1f);
        if (fraction < 0.002f)
            return;

        DrawGradientArcBand(cx, cy, TrackRadius, TrackThickness + 4f, 0f, fraction, f => WithAlpha(colorAtFraction(f), 42), true);
        DrawGradientArcBand(cx, cy, TrackRadius, TrackThickness, 0f, fraction, colorAtFraction, true);
        DrawArcBand(cx, cy, TrackRadius - (TrackThickness * 0.28f), 1.6f, StartDeg, SweepDeg * fraction, new Color(255, 255, 255, 24), true);
    }

    private void DrawTicks(float cx, float cy, int majorTickCount, int minorTicksPerGap)
    {
        minorTicksPerGap = Math.Max(minorTicksPerGap, 0);
        int sectionCount = Math.Max(majorTickCount - 1, 1);

        for (int i = 0; i < majorTickCount; i++)
        {
            float frac = majorTickCount <= 1 ? 0f : (float)i / (majorTickCount - 1);
            DrawTick(cx, cy, frac, MajorTickInnerRadius, TickOuterRadius, new Color(232, 236, 240, 230), 2.5f);

            if (i >= sectionCount)
                continue;

            for (int j = 1; j <= minorTicksPerGap; j++)
            {
                float minorFrac = (i + (float)j / (minorTicksPerGap + 1)) / sectionCount;
                DrawTick(cx, cy, minorFrac, MinorTickInnerRadius, TickOuterRadius - 1f, new Color(170, 176, 186, 140), 1.2f);
            }
        }
    }

    private void DrawTick(float cx, float cy, float fraction, float innerRadius, float outerRadius, Color color, float thickness)
    {
        float angleDeg = StartDeg + SweepDeg * Math.Clamp(fraction, 0f, 1f);
        var start = PointOnCircle(cx, cy, innerRadius, angleDeg);
        var end = PointOnCircle(cx, cy, outerRadius, angleDeg);
        DrawLine(start, end, color, thickness);
    }

    private void DrawNeedle(float cx, float cy, float fraction, Color color)
    {
        float angleDeg = StartDeg + SweepDeg * Math.Clamp(fraction, 0f, 1f);
        float angleRad = angleDeg * MathF.PI / 180f;
        var direction = new Vector2(MathF.Cos(angleRad), -MathF.Sin(angleRad));
        var tail = new Vector2(cx, cy) - direction * NeedleTailLength;
        var tip = new Vector2(cx, cy) + direction * NeedleLength;
        var shadowOffset = new Vector2(1.5f, 2f);

        DrawLine(tail + shadowOffset, tip + shadowOffset, new Color(0, 0, 0, 100), 5f);
        DrawLine(tail, tip, new Color(18, 20, 24, 235), 4f);
        DrawLine(tail + direction * 2f, tip, color, 2.25f);
    }

    private void DrawCentreCap(float cx, float cy, Color color)
    {
        DrawFilledCircle(cx, cy, 10f, new Color(18, 20, 24, 245));
        DrawCircleStroke(cx, cy, 10f, 1.5f, new Color(200, 204, 212, 70));
        DrawFilledCircle(cx, cy, 6f, new Color(225, 227, 232, 255));
        DrawFilledCircle(cx, cy, 3f, color);
    }

    private void DrawPedalBars(float x, float y, float w, float h)
    {
        DrawPedalBar(x, y, w, h, ThrottleInput, new Color(68, 218, 102, 220));
        DrawPedalBar(x, y + 12f, w, h, BrakeInput, new Color(255, 92, 70, 220));
    }

    private void DrawPedalBar(float x, float y, float w, float h, float value, Color fillColor)
    {
        DrawRect(new RectangleF(x, y, w, h), new Color(16, 18, 22, 190));
        DrawRect(new RectangleF(x + 1f, y + 1f, w - 2f, h - 2f), new Color(44, 48, 56, 205));

        if (value > 0.001f)
        {
            float innerWidth = (w - 2f) * Math.Clamp(value, 0f, 1f);
            DrawRect(new RectangleF(x + 1f, y + 1f, innerWidth, h - 2f), fillColor);
        }

        DrawRect(new RectangleF(x + 1f, y + 1f, w - 2f, 1f), new Color(255, 255, 255, 28));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Primitive helpers (SpriteBatch + 1×1 pixel texture)
    // ─────────────────────────────────────────────────────────────────────────

    private void DrawLine(Vector2 start, Vector2 end, Color color, float thickness = 1f)
    {
        var delta = end - start;
        var length = delta.Length();
        if (length < 0.5f) return;

        float angle = MathF.Atan2(delta.Y, delta.X);
        var center = (start + end) * 0.5f;
        _sb!.Draw(_pixel!, center, null, color, angle, new Vector2(0.5f, 0.5f), new Vector2(length, thickness));
    }

    private void DrawRect(RectangleF rect, Color color)
    {
        _sb!.Draw(_pixel!, new Vector2(rect.X, rect.Y), null, color, 0f, Vector2.Zero, new Vector2(rect.Width, rect.Height));
    }

    private void DrawFilledCircle(float cx, float cy, float radius, Color color)
    {
        int radiusCeiling = (int)MathF.Ceiling(radius);
        float radiusSquared = radius * radius;

        for (int y = -radiusCeiling; y <= radiusCeiling; y++)
        {
            float yPos = y;
            float xSquared = radiusSquared - (yPos * yPos);
            if (xSquared < 0f)
                continue;

            float xExtent = MathF.Sqrt(xSquared);
            DrawRect(new RectangleF(cx - xExtent, cy + yPos - 0.5f, xExtent * 2f, 1f), color);
        }
    }

    private void DrawCircleStroke(float cx, float cy, float radius, float thickness, Color color)
    {
        DrawArcBand(cx, cy, radius, thickness, 0f, 360f, color, false);
    }

    private void DrawArcBandFraction(float cx, float cy, float radius, float thickness, float startFrac, float endFrac, Color color, bool roundCaps)
    {
        startFrac = Math.Clamp(startFrac, 0f, 1f);
        endFrac = Math.Clamp(endFrac, 0f, 1f);
        if (endFrac <= startFrac)
            return;

        DrawArcBand(cx, cy, radius, thickness, StartDeg + SweepDeg * startFrac, SweepDeg * (endFrac - startFrac), color, roundCaps);
    }

    private void DrawArcBand(float cx, float cy, float radius, float thickness, float startDeg, float sweepDeg, Color color, bool roundCaps)
    {
        if (MathF.Abs(sweepDeg) < 0.05f || thickness <= 0.1f)
            return;

        int segmentCount = Math.Max(1, GetArcSegments(radius, sweepDeg));
        Vector2 first = Vector2.Zero;
        Vector2 last = Vector2.Zero;

        for (int i = 0; i < segmentCount; i++)
        {
            float segmentStart = startDeg + (sweepDeg * i / segmentCount);
            float segmentEnd = startDeg + (sweepDeg * (i + 1) / segmentCount);
            var p0 = PointOnCircle(cx, cy, radius, segmentStart);
            var p1 = PointOnCircle(cx, cy, radius, segmentEnd);

            if (i == 0)
                first = p0;
            last = p1;

            DrawLine(p0, p1, color, thickness);
        }

        if (roundCaps)
        {
            float capRadius = thickness * 0.5f;
            DrawFilledCircle(first.X, first.Y, capRadius, color);
            DrawFilledCircle(last.X, last.Y, capRadius, color);
        }
    }

    private void DrawGradientArcBand(float cx, float cy, float radius, float thickness, float startFrac, float endFrac, Func<float, Color> colorAtFraction, bool roundCaps)
    {
        startFrac = Math.Clamp(startFrac, 0f, 1f);
        endFrac = Math.Clamp(endFrac, 0f, 1f);
        if (endFrac <= startFrac || thickness <= 0.1f)
            return;

        float sweepDeg = SweepDeg * (endFrac - startFrac);
        int segmentCount = Math.Max(1, GetArcSegments(radius, sweepDeg));
        Vector2 first = Vector2.Zero;
        Vector2 last = Vector2.Zero;
        Color firstColor = colorAtFraction(startFrac);
        Color lastColor = colorAtFraction(endFrac);

        for (int i = 0; i < segmentCount; i++)
        {
            float segmentStartFrac = Lerp(startFrac, endFrac, (float)i / segmentCount);
            float segmentEndFrac = Lerp(startFrac, endFrac, (float)(i + 1) / segmentCount);
            float segmentMidFrac = (segmentStartFrac + segmentEndFrac) * 0.5f;

            var p0 = PointOnCircle(cx, cy, radius, StartDeg + SweepDeg * segmentStartFrac);
            var p1 = PointOnCircle(cx, cy, radius, StartDeg + SweepDeg * segmentEndFrac);

            if (i == 0)
                first = p0;
            last = p1;

            DrawLine(p0, p1, colorAtFraction(segmentMidFrac), thickness);
        }

        if (roundCaps)
        {
            float capRadius = thickness * 0.5f;
            DrawFilledCircle(first.X, first.Y, capRadius, firstColor);
            DrawFilledCircle(last.X, last.Y, capRadius, lastColor);
        }
    }

    private static int GetArcSegments(float radius, float sweepDeg)
    {
        return Math.Max(16, (int)MathF.Ceiling(MathF.Abs(sweepDeg) * MathF.Max(radius, 1f) / 20f));
    }

    private static Vector2 PointOnCircle(float cx, float cy, float radius, float angleDeg)
    {
        float angleRad = angleDeg * MathF.PI / 180f;
        return new Vector2(cx + MathF.Cos(angleRad) * radius, cy - MathF.Sin(angleRad) * radius);
    }

    private static Color GetSpeedColor(float fraction)
    {
        var green = new Color(72, 220, 112, 255);
        var amber = new Color(244, 196, 68, 255);
        var red = new Color(255, 92, 68, 255);

        fraction = Math.Clamp(fraction, 0f, 1f);
        if (fraction <= SpeedCautionFrac)
            return green;
        if (fraction <= SpeedWarningFrac)
            return LerpColor(green, amber, (fraction - SpeedCautionFrac) / (SpeedWarningFrac - SpeedCautionFrac));

        return LerpColor(amber, red, (fraction - SpeedWarningFrac) / (1f - SpeedWarningFrac));
    }

    private static Color GetRpmColor(float fraction)
    {
        var blue = new Color(98, 188, 255, 255);
        var brightBlue = new Color(126, 225, 255, 255);
        var amber = new Color(244, 196, 74, 255);
        var red = new Color(255, 78, 60, 255);

        fraction = Math.Clamp(fraction, 0f, 1f);
        if (fraction <= RpmAmberFrac)
            return LerpColor(blue, brightBlue, fraction / RpmAmberFrac);
        if (fraction <= RpmRedlineFrac)
            return LerpColor(brightBlue, amber, (fraction - RpmAmberFrac) / (RpmRedlineFrac - RpmAmberFrac));

        return LerpColor(amber, red, (fraction - RpmRedlineFrac) / (1f - RpmRedlineFrac));
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

    private static Color WithAlpha(Color color, byte alpha)
    {
        return new Color(color.R, color.G, color.B, alpha);
    }

    private static byte LerpByte(byte a, byte b, float t)
    {
        return (byte)MathF.Round(a + ((b - a) * Math.Clamp(t, 0f, 1f)));
    }

    private static float Lerp(float a, float b, float t)
    {
        return a + ((b - a) * Math.Clamp(t, 0f, 1f));
    }

    // ─────────────────────────────────────────────────────────────────────────

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
