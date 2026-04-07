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
    private const float StartDeg  = 225f;   // 7:30 = 0 km/h
    private const float SweepDeg  = -270f;  // 270° clockwise sweep
    private const int   SpeedTicks = 11;    // 0,20,40,…,200 km/h
    private const float ArcOuter  = 88f;
    private const float ArcInner  = 62f;
    private const float TickOuter = 86f;

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

        float cx = sw - 155f;
        float cy = sh - 155f;

        cmd.SetRenderTargetAndViewport(null, back);
        _sb.Begin(ctx, SpriteSortMode.Deferred, BlendStates.AlphaBlend);

        DrawDial(cx, cy);

        _sb.End();
    }

    // ─────────────────────────────────────────────────────────────────────────
    // Drawing routines
    // ─────────────────────────────────────────────────────────────────────────

    private void DrawDial(float cx, float cy)
    {
        // 1. Background circle
        DrawFilledCircle(cx, cy, ArcOuter + 12f, new Color(30, 30, 30, 210));

        // 2. Speed arc background (dark grey track)
        DrawArc(cx, cy, ArcInner, ArcOuter, StartDeg, SweepDeg, new Color(70, 70, 70, 255), 72);

        // 3. Speed fill arc (green → yellow → red)
        float speedFrac  = Math.Clamp(SpeedKmh / MaxSpeedKmh, 0f, 1f);
        float fillSweep  = SweepDeg * speedFrac;
        Color fillColor  = speedFrac < 0.55f
                             ? new Color(50, 220, 80, 255)
                             : speedFrac < 0.80f
                                 ? new Color(230, 200, 40, 255)
                                 : new Color(230, 60, 40, 255);
        if (speedFrac > 0.005f)
            DrawArc(cx, cy, ArcInner, ArcOuter, StartDeg, fillSweep, fillColor, 72);

        // 4. Tick marks
        for (int i = 0; i < SpeedTicks; i++)
        {
            float frac    = (float)i / (SpeedTicks - 1);
            float angleDeg = StartDeg + SweepDeg * frac;
            float angleRad = angleDeg * MathF.PI / 180f;
            bool  major    = (i % 2 == 0);
            float inner    = major ? TickOuter - 16f : TickOuter - 8f;
            float thickness = major ? 3f : 1.5f;
            Color tickColor = major ? Color.White : new Color(180, 180, 180, 200);

            var p0 = new Vector2(cx + MathF.Cos(angleRad) * inner,    cy - MathF.Sin(angleRad) * inner);
            var p1 = new Vector2(cx + MathF.Cos(angleRad) * TickOuter, cy - MathF.Sin(angleRad) * TickOuter);
            DrawLine(p0, p1, tickColor, thickness);
        }

        // 5. Needle
        float needleAngle = (StartDeg + SweepDeg * speedFrac) * MathF.PI / 180f;
        var   needleTip   = new Vector2(cx + MathF.Cos(needleAngle) * (ArcOuter - 4f),
                                         cy - MathF.Sin(needleAngle) * (ArcOuter - 4f));
        var   needleBase  = new Vector2(cx - MathF.Cos(needleAngle) * 18f,
                                         cy + MathF.Sin(needleAngle) * 18f);
        DrawLine(needleBase, needleTip, new Color(255, 80, 40, 255), 3.0f);

        // 6. Centre cap
        DrawFilledCircle(cx, cy, 10f, new Color(200, 200, 200, 255));
        DrawFilledCircle(cx, cy,  5f, new Color(255, 80, 40, 255));

        // 7. RPM bar — thin horizontal bar below gauge
        DrawRpmBar(cx - 75f, cy + 95f, 150f, 10f);
    }

    private void DrawRpmBar(float x, float y, float w, float h)
    {
        // Background
        DrawRect(new RectangleF(x, y, w, h), new Color(50, 50, 50, 200));

        float rpmFrac = Math.Clamp(EngineRpm / MaxRpm, 0f, 1f);
        // Redline starts at 85% of max RPM
        float redFrac = 0.85f;

        if (rpmFrac > 0f)
        {
            float fillW    = w * rpmFrac;
            Color rpmColor = rpmFrac < redFrac
                               ? new Color(80, 160, 255, 255)
                               : new Color(255, 60, 40, 255);
            DrawRect(new RectangleF(x, y, fillW, h), rpmColor);
        }

        // Redline marker line
        DrawRect(new RectangleF(x + w * redFrac - 1f, y, 2f, h), new Color(255, 40, 40, 255));
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
        const int Segments = 32;
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
