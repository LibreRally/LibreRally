using System;
using System.Collections.Generic;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Graphics;

namespace LibreRally.HUD;

public readonly record struct DriverAssistWheelItem(string Title, string Description, bool Enabled, bool Available);

public sealed class DriverAssistWheelOverlay : GameSystemBase
{
    private const float NodeWidth = 176f;
    private const float NodeHeight = 84f;
    private const float NodeRadius = 176f;

    public IReadOnlyList<DriverAssistWheelItem> Items { get; set; } = Array.Empty<DriverAssistWheelItem>();
    public int SelectedIndex { get; set; }
    public bool OverlayVisible { get; set; }
    public string VehicleName { get; set; } = string.Empty;
    public string StatusText { get; set; } = string.Empty;

    private Game? _game;
    private SpriteBatch? _spriteBatch;
    private Texture? _pixel;
    private SpriteFont? _font;

    public DriverAssistWheelOverlay(IServiceRegistry services) : base(services)
    {
        Enabled = true;
        Visible = true;
        DrawOrder = 9996;
        UpdateOrder = 9996;
    }

    public override void Initialize()
    {
        base.Initialize();

        _game = (Game?)Services.GetService<IGame>();
        if (_game == null)
        {
            return;
        }

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
        DrawWheel(backBuffer.Width, backBuffer.Height);
        _spriteBatch.End();
    }

    private void DrawWheel(int viewportWidth, int viewportHeight)
    {
        var centre = new Vector2(viewportWidth * 0.5f, viewportHeight * 0.5f - 12f);
        var clampedIndex = Math.Clamp(SelectedIndex, 0, Math.Max(0, Items.Count - 1));

        DrawRect(new RectangleF(0f, 0f, viewportWidth, viewportHeight), new Color(4, 6, 10, 118));
        DrawFilledCircle(centre.X, centre.Y, 116f, new Color(14, 20, 28, 244));
        DrawCircleStroke(centre.X, centre.Y, 116f, 2f, new Color(210, 214, 220, 44));
        DrawCircleStroke(centre.X, centre.Y, 118f, 14f, new Color(214, 148, 78, 26));
        DrawTextCentered("Driver Assists", centre.X, centre.Y - 28f, new Color(240, 243, 247, 255));
        DrawTextCentered(string.IsNullOrWhiteSpace(VehicleName) ? "LibreRally" : VehicleName, centre.X, centre.Y - 4f, new Color(255, 230, 204, 255));
        DrawTextCentered("X / Tab open  •  D-Pad select  •  A / Enter toggle  •  B / Esc close", centre.X, centre.Y + 28f, new Color(183, 193, 205, 235));

        for (var i = 0; i < Items.Count; i++)
        {
            var item = Items[i];
            var angle = MathF.PI * (-0.5f + (2f * i / Math.Max(Items.Count, 1)));
            var nodeCentre = centre + new Vector2(MathF.Cos(angle), MathF.Sin(angle)) * NodeRadius;
            var nodeRect = new RectangleF(
                nodeCentre.X - NodeWidth * 0.5f,
                nodeCentre.Y - NodeHeight * 0.5f,
                NodeWidth,
                NodeHeight);

            var isSelected = i == clampedIndex;
            DrawNode(nodeRect, item, isSelected);
            DrawLine(centre, nodeCentre, isSelected ? new Color(214, 148, 78, 140) : new Color(120, 130, 142, 56), isSelected ? 3.5f : 2f);
        }

        if (Items.Count > 0)
        {
            var selected = Items[clampedIndex];
            DrawTextCentered(selected.Description, centre.X, centre.Y + 154f, selected.Available ? new Color(214, 219, 227, 255) : new Color(156, 164, 174, 255));
        }

        DrawTextCentered(StatusText, centre.X, centre.Y + 182f, new Color(183, 193, 205, 255));
    }

    private void DrawNode(RectangleF rect, DriverAssistWheelItem item, bool isSelected)
    {
        var panelColor = !item.Available
            ? new Color(24, 30, 40, 166)
            : item.Enabled
                ? new Color(80, 50, 20, isSelected ? 246 : 228)
                : isSelected
                    ? new Color(56, 66, 82, 244)
                    : new Color(30, 38, 48, 214);
        var borderColor = !item.Available
            ? new Color(100, 108, 120, 48)
            : isSelected
                ? new Color(255, 196, 126, 210)
                : new Color(170, 176, 186, 72);
        var statusColor = !item.Available
            ? new Color(128, 136, 148, 216)
            : item.Enabled
                ? new Color(255, 230, 184, 255)
                : new Color(180, 190, 204, 255);

        DrawRect(new RectangleF(rect.X + 3f, rect.Y + 4f, rect.Width, rect.Height), new Color(0, 0, 0, 52));
        DrawRect(rect, panelColor);
        DrawRectOutline(rect, borderColor, isSelected ? 3f : 2f);

        var centreX = rect.X + rect.Width * 0.5f;
        DrawTextCentered(item.Title, centreX, rect.Y + 16f, item.Available ? new Color(240, 243, 247, 255) : new Color(150, 160, 170, 255));
        DrawTextCentered(item.Available ? (item.Enabled ? "ON" : "OFF") : "N/A", centreX, rect.Y + 42f, statusColor);
    }

    private void DrawTextCentered(string text, float x, float y, Color color)
    {
        _spriteBatch!.DrawString(_font!, text, new Vector2(x, y), color, TextAlignment.Center);
    }

    private void DrawRect(RectangleF rect, Color color)
    {
        _spriteBatch!.Draw(_pixel!, new Vector2(rect.X, rect.Y), null, color, 0f, Vector2.Zero, new Vector2(rect.Width, rect.Height));
    }

    private void DrawRectOutline(RectangleF rect, Color color, float thickness)
    {
        DrawRect(new RectangleF(rect.X, rect.Y, rect.Width, thickness), color);
        DrawRect(new RectangleF(rect.X, rect.Bottom - thickness, rect.Width, thickness), color);
        DrawRect(new RectangleF(rect.X, rect.Y, thickness, rect.Height), color);
        DrawRect(new RectangleF(rect.Right - thickness, rect.Y, thickness, rect.Height), color);
    }

    private void DrawFilledCircle(float cx, float cy, float radius, Color color)
    {
        for (var fillRadius = radius; fillRadius > 0f; fillRadius -= 2f)
        {
            DrawCircleStroke(cx, cy, fillRadius, 2f, color);
        }
    }

    private void DrawCircleStroke(float cx, float cy, float radius, float thickness, Color color)
    {
        DrawArc(cx, cy, radius, thickness, 0f, MathF.PI * 2f, color);
    }

    private void DrawArc(float cx, float cy, float radius, float thickness, float startAngle, float endAngle, Color color)
    {
        const int segments = 64;
        var angleSpan = endAngle - startAngle;
        var previousInner = new Vector2(cx + MathF.Cos(startAngle) * (radius - thickness * 0.5f), cy + MathF.Sin(startAngle) * (radius - thickness * 0.5f));
        var previousOuter = new Vector2(cx + MathF.Cos(startAngle) * (radius + thickness * 0.5f), cy + MathF.Sin(startAngle) * (radius + thickness * 0.5f));

        for (var i = 1; i <= segments; i++)
        {
            var angle = startAngle + angleSpan * i / segments;
            var inner = new Vector2(cx + MathF.Cos(angle) * (radius - thickness * 0.5f), cy + MathF.Sin(angle) * (radius - thickness * 0.5f));
            var outer = new Vector2(cx + MathF.Cos(angle) * (radius + thickness * 0.5f), cy + MathF.Sin(angle) * (radius + thickness * 0.5f));
            DrawTriangle(previousInner, previousOuter, inner, color);
            DrawTriangle(inner, previousOuter, outer, color);
            previousInner = inner;
            previousOuter = outer;
        }
    }

    private void DrawTriangle(Vector2 a, Vector2 b, Vector2 c, Color color)
    {
        DrawLine(a, b, color, 1f);
        DrawLine(b, c, color, 1f);
        DrawLine(c, a, color, 1f);
    }

    private void DrawLine(Vector2 start, Vector2 end, Color color, float thickness)
    {
        var delta = end - start;
        var length = delta.Length();
        if (length <= 0.01f)
        {
            return;
        }

        var rotation = MathF.Atan2(delta.Y, delta.X);
        _spriteBatch!.Draw(
            _pixel!,
            start,
            null,
            color,
            rotation,
            new Vector2(0f, 0.5f),
            new Vector2(length, thickness));
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
