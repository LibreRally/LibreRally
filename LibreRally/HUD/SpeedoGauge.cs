using System;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Graphics;

namespace LibreRally.HUD;

/// <summary>
/// Draws a vector-art circular speedometer + RPM bar to the back-buffer each frame.
/// Register with <c>((Game)Game).GameSystems.Add(gauge)</c> from a SyncScript.
/// </summary>
public class SpeedoGauge : GameSystemBase
{
    // ── Data sources (set from SyncScript each frame) ────────────────────────
    public float SpeedKmh   { get; set; }
    public float MaxSpeedKmh { get; set; } = 220f;
    public float EngineRpm  { get; set; }
    public float MaxRpm     { get; set; } = 7500f;
    public int   CurrentGear { get; set; } = 1;
    public float ThrottleInput { get; set; }
    public float BrakeInput    { get; set; }

    // ── Gauge geometry (screen coordinates, updated in Draw from backbuffer size) ──
    private const float StartDeg   = 225f;
    private const float SweepDeg   = -270f;
    private const int   SpeedTicks = 11;
    private const int   RpmTicks   = 8;
    private const float ArcOuter   = 66f;
    private const float ArcInner   = 48f;
    private const float TickOuter  = 64f;

    private SpriteBatch? _sb;
    private Texture?     _pixel;
    private Game?        _game;

    public SpeedoGauge(IServiceRegistry services) : base(services)
    {
        Enabled    = true;
        Visible    = true;
        DrawOrder  = 9990; // after scene, before DebugText (which is 10000)
        UpdateOrder = 9990;
    }

    public override void Initialize()
    {
        base.Initialize();
        _game  = (Game)Services.GetService<IGame>();
        _sb    = new SpriteBatch(_game.GraphicsDevice);
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
        var gd   = _game.GraphicsDevice;
        var ctx  = _game.GraphicsContext;
        var cmd  = ctx.CommandList;
        var back = gd.Presenter?.BackBuffer;
        if (back == null) return;

        int sw = back.Width;
        int sh = back.Height;

        float speedCx = sw - 255f;
        float rpmCx   = sw - 110f;
        float cy      = sh - 132f;

        cmd.SetRenderTargetAndViewport(null, back);
        _sb.Begin(ctx, SpriteSortMode.Deferred, BlendStates.AlphaBlend);

        DrawBackdrop(speedCx - 90f, cy - 82f, 225f, 162f);
        DrawSpeedDial(speedCx, cy);
        DrawRpmDial(rpmCx, cy);
        DrawPedalBars(speedCx - 12f, cy + 86f, 132f, 8f);

        _sb.End();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Drawing routines
    // ─────────────────────────────────────────────────────────────────────────

    private void DrawBackdrop(float x, float y, float w, float h)
    {
        DrawRect(new RectangleF(x, y, w, h), new Color(16, 18, 22, 210));
        DrawRect(new RectangleF(x, y, w, 2f), new Color(120, 125, 135, 120));
        DrawRect(new RectangleF(x, y + h - 2f, w, 2f), new Color(26, 28, 32, 200));
    }

    private void DrawSpeedDial(float cx, float cy)
    {
        DrawDialBase(cx, cy);

        float speedFrac  = Math.Clamp(SpeedKmh / MaxSpeedKmh, 0f, 1f);
        float fillSweep  = SweepDeg * speedFrac;
        Color fillColor  = speedFrac < 0.55f
                             ? new Color(50, 220, 80, 255)
                             : speedFrac < 0.80f
                                  ? new Color(230, 200, 40, 255)
                                  : new Color(230, 60, 40, 255);
        if (speedFrac > 0.005f)
            DrawArc(cx, cy, ArcInner, ArcOuter, StartDeg, fillSweep, fillColor, 96);

        DrawTicks(cx, cy, SpeedTicks);
        DrawNeedle(cx, cy, speedFrac, new Color(255, 110, 60, 255));
        DrawCentreCap(cx, cy, new Color(255, 110, 60, 255));
    }

    private void DrawRpmDial(float cx, float cy)
    {
        float rpmFrac = Math.Clamp(EngineRpm / MaxRpm, 0f, 1f);
        const float redlineFrac = 0.86f;

        DrawDialBase(cx, cy);

        float greenSweep = SweepDeg * MathF.Min(rpmFrac, 0.72f);
        if (greenSweep < -0.5f)
            DrawArc(cx, cy, ArcInner, ArcOuter, StartDeg, greenSweep, new Color(80, 180, 255, 255), 96);

        if (rpmFrac > 0.72f)
        {
            float amberFrac = MathF.Min(rpmFrac, redlineFrac) - 0.72f;
            if (amberFrac > 0f)
            {
                DrawArc(cx, cy, ArcInner, ArcOuter,
                    StartDeg + SweepDeg * 0.72f,
                    SweepDeg * amberFrac,
                    new Color(245, 190, 55, 255), 48);
            }
        }

        if (rpmFrac > redlineFrac)
        {
            DrawArc(cx, cy, ArcInner, ArcOuter,
                StartDeg + SweepDeg * redlineFrac,
                SweepDeg * (rpmFrac - redlineFrac),
                new Color(255, 70, 45, 255), 36);
        }

        DrawTicks(cx, cy, RpmTicks);
        DrawNeedle(cx, cy, rpmFrac, new Color(110, 210, 255, 255));
        DrawCentreCap(cx, cy, new Color(110, 210, 255, 255));

        // Redline marker
        float redlineAngle = (StartDeg + SweepDeg * redlineFrac) * MathF.PI / 180f;
        var p0 = new Vector2(cx + MathF.Cos(redlineAngle) * (TickOuter - 18f),
                             cy - MathF.Sin(redlineAngle) * (TickOuter - 18f));
        var p1 = new Vector2(cx + MathF.Cos(redlineAngle) * TickOuter,
                             cy - MathF.Sin(redlineAngle) * TickOuter);
        DrawLine(p0, p1, new Color(255, 50, 50, 255), 4f);
    }

    private void DrawDialBase(float cx, float cy)
    {
        DrawFilledCircle(cx, cy, ArcInner - 7f, new Color(22, 24, 28, 235));
        DrawArc(cx, cy, ArcOuter + 3f, ArcOuter + 9f, 0f, 360f, new Color(18, 18, 20, 245), 180);
        DrawArc(cx, cy, ArcInner, ArcOuter, StartDeg, SweepDeg, new Color(62, 62, 70, 240), 180);
    }

    private void DrawTicks(float cx, float cy, int tickCount)
    {
        for (int i = 0; i < tickCount; i++)
        {
            float frac      = tickCount <= 1 ? 0f : (float)i / (tickCount - 1);
            float angleDeg  = StartDeg + SweepDeg * frac;
            float angleRad  = angleDeg * MathF.PI / 180f;
            float inner     = (i % 2 == 0) ? TickOuter - 14f : TickOuter - 8f;
            float thickness = (i % 2 == 0) ? 2.5f : 1.5f;
            Color tickColor = (i % 2 == 0) ? Color.White : new Color(185, 185, 195, 180);

            var p0 = new Vector2(cx + MathF.Cos(angleRad) * inner, cy - MathF.Sin(angleRad) * inner);
            var p1 = new Vector2(cx + MathF.Cos(angleRad) * TickOuter, cy - MathF.Sin(angleRad) * TickOuter);
            DrawLine(p0, p1, tickColor, thickness);
        }
    }

    private void DrawNeedle(float cx, float cy, float fraction, Color color)
    {
        float angle = (StartDeg + SweepDeg * Math.Clamp(fraction, 0f, 1f)) * MathF.PI / 180f;
        var tip  = new Vector2(cx + MathF.Cos(angle) * (ArcOuter - 5f), cy - MathF.Sin(angle) * (ArcOuter - 5f));
        var basePoint = new Vector2(cx - MathF.Cos(angle) * 16f, cy + MathF.Sin(angle) * 16f);
        DrawLine(basePoint, tip, color, 3.0f);
    }

    private void DrawCentreCap(float cx, float cy, Color color)
    {
        DrawFilledCircle(cx, cy, 8f, new Color(210, 210, 215, 255));
        DrawFilledCircle(cx, cy, 4f, color);
    }

    private void DrawPedalBars(float x, float y, float w, float h)
    {
        DrawRect(new RectangleF(x, y, w, h), new Color(42, 42, 46, 180));
        if (ThrottleInput > 0.001f)
            DrawRect(new RectangleF(x, y, w * Math.Clamp(ThrottleInput, 0f, 1f), h),
                new Color(60, 220, 90, 210));

        DrawRect(new RectangleF(x, y + 12f, w, h), new Color(42, 42, 46, 180));
        if (BrakeInput > 0.001f)
            DrawRect(new RectangleF(x, y + 12f, w * Math.Clamp(BrakeInput, 0f, 1f), h),
                new Color(255, 85, 65, 210));
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Primitive helpers (SpriteBatch + 1×1 pixel texture)
    // ─────────────────────────────────────────────────────────────────────────

    private void DrawLine(Vector2 start, Vector2 end, Color color, float thickness = 1f)
    {
        var delta  = end - start;
        var length = delta.Length();
        if (length < 0.5f) return;
        float angle  = MathF.Atan2(delta.Y, delta.X);
        var   center = (start + end) * 0.5f;
        _sb!.Draw(_pixel!, center, null, color, angle,
                  new Vector2(0.5f, 0.5f), new Vector2(length, thickness));
    }

    private void DrawRect(RectangleF rect, Color color)
    {
        _sb!.Draw(_pixel!, new Vector2(rect.X, rect.Y), null, color, 0f,
                  Vector2.Zero, new Vector2(rect.Width, rect.Height));
    }

    private void DrawFilledCircle(float cx, float cy, float radius, Color color)
    {
        // Approximate circle as 32-segment arc (thin segments that overlap)
        const int Segments = 96;
        float step = 2f * MathF.PI / Segments;
        for (int i = 0; i < Segments; i++)
        {
            float a0 = step * i;
            float a1 = step * (i + 1);
            // Draw a thin pie wedge from center to arc
            var mid = new Vector2(cx + MathF.Cos((a0 + a1) * 0.5f) * radius * 0.5f,
                                   cy + MathF.Sin((a0 + a1) * 0.5f) * radius * 0.5f);
            float segLen   = radius;
            float segWidth = radius * step + 1f;
            float angle    = (a0 + a1) * 0.5f;
            _sb!.Draw(_pixel!, mid, null, color, angle,
                      new Vector2(0.5f, 0.5f), new Vector2(segLen, segWidth));
        }
    }

    private void DrawArc(float cx, float cy, float innerR, float outerR,
                         float startDeg, float sweepDeg, Color color, int segments)
    {
        float midR     = (innerR + outerR) * 0.5f;
        float arcWidth = outerR - innerR;
        float step     = sweepDeg / segments;

        for (int i = 0; i < segments; i++)
        {
            float aDeg = startDeg + step * (i + 0.5f);
            float aRad = aDeg * MathF.PI / 180f;
            var   pos  = new Vector2(cx + MathF.Cos(aRad) * midR,
                                      cy - MathF.Sin(aRad) * midR);

            // Segment length ≈ arc length for one step + 1 px overlap to prevent gaps
            float segLen = MathF.Abs(step) * MathF.PI / 180f * midR + 1.5f;
            // Orient segment tangent to the arc (perpendicular to radius = angle + 90°)
            float tangentAngle = aRad + MathF.PI * 0.5f;

            _sb!.Draw(_pixel!, pos, null, color, tangentAngle,
                      new Vector2(0.5f, 0.5f), new Vector2(segLen, arcWidth));
        }
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
