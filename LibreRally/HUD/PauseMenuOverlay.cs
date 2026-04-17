using System;
using System.Collections.Generic;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Graphics;

namespace LibreRally.HUD;

public readonly record struct PauseMenuItem(string Title, string Description);

public sealed class PauseMenuOverlay : GameSystemBase
{
    private const float PanelWidth = 620f;
    private const float HeaderHeight = 92f;
    private const float RowHeight = 70f;
    private const float FooterHeight = 38f;

    public IReadOnlyList<PauseMenuItem> Items { get; set; } = Array.Empty<PauseMenuItem>();
    public int SelectedIndex { get; set; }
    public bool OverlayVisible { get; set; }
    public string VehicleName { get; set; } = string.Empty;
    public string StatusText { get; set; } = string.Empty;

    private Game? _game;
    private SpriteBatch? _spriteBatch;
    private Texture? _pixel;
    private SpriteFont? _font;

    public PauseMenuOverlay(IServiceRegistry services) : base(services)
    {
        Enabled = true;
        Visible = true;
        DrawOrder = 9995;
        UpdateOrder = 9995;
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
        DrawPanel(backBuffer.Width, backBuffer.Height);
        _spriteBatch.End();
    }

    private void DrawPanel(int viewportWidth, int viewportHeight)
    {
        var itemCount = Math.Max(Items.Count, 1);
        var panelHeight = HeaderHeight + FooterHeight + itemCount * RowHeight;
        var panelX = (viewportWidth - PanelWidth) * 0.5f;
        var panelY = (viewportHeight - panelHeight) * 0.5f;

        DrawRect(new RectangleF(0, 0, viewportWidth, viewportHeight), new Color(5, 8, 14, 180));
        DrawRect(new RectangleF(panelX + 6f, panelY + 6f, PanelWidth, panelHeight), new Color(0, 0, 0, 60));
        DrawRect(new RectangleF(panelX, panelY, PanelWidth, panelHeight), new Color(16, 22, 30, 242));
        DrawRect(new RectangleF(panelX + 2f, panelY + 2f, PanelWidth - 4f, panelHeight - 4f), new Color(22, 29, 39, 220));

        DrawText("Paused", panelX + 22f, panelY + 18f, new Color(240, 243, 247, 255));
        DrawText(string.IsNullOrWhiteSpace(VehicleName) ? "LibreRally" : VehicleName, panelX + 22f, panelY + 40f, new Color(255, 230, 204, 255));
        DrawText("D-Pad Up/Down select  •  A activate  •  B/Start resume", panelX + 22f, panelY + 62f, new Color(183, 193, 205, 255));

        var clampedIndex = Math.Clamp(SelectedIndex, 0, Math.Max(0, Items.Count - 1));
        for (var i = 0; i < Items.Count; i++)
        {
            var item = Items[i];
            var rowY = panelY + HeaderHeight + i * RowHeight;
            var selected = i == clampedIndex;
            DrawRect(new RectangleF(panelX + 14f, rowY, PanelWidth - 28f, RowHeight - 6f), selected
                ? new Color(214, 148, 78, 196)
                : new Color(37, 44, 56, 156));
            DrawText(item.Title, panelX + 24f, rowY + 10f, selected ? new Color(255, 246, 236, 255) : new Color(236, 240, 247, 255));
            DrawText(item.Description, panelX + 24f, rowY + 34f, selected ? new Color(255, 228, 198, 236) : new Color(168, 177, 188, 235));
        }

        DrawText(StatusText, panelX + 22f, panelY + panelHeight - 24f, new Color(183, 193, 205, 255));
    }

    private void DrawText(string text, float x, float y, Color color)
    {
        _spriteBatch!.DrawString(_font!, text, new Vector2(x, y), color, TextAlignment.Left);
    }

    private void DrawRect(RectangleF rect, Color color)
    {
        _spriteBatch!.Draw(_pixel!, new Vector2(rect.X, rect.Y), null, color, 0f, Vector2.Zero, new Vector2(rect.Width, rect.Height));
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
