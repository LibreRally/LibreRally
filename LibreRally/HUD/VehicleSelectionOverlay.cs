using System;
using System.Collections.Generic;
using LibreRally.Vehicle.Content;
using Stride.Core;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Games;
using Stride.Graphics;

namespace LibreRally.HUD;

public sealed class VehicleSelectionOverlay : GameSystemBase
{
    private const float PanelX = 680f;
    private const float PanelY = 54f;
    private const float PanelWidth = 420f;
    private const float HeaderHeight = 54f;
    private const float RowHeight = 18f;
    private const float FooterHeight = 34f;

    public IReadOnlyList<BeamNgVehicleDescriptor> Vehicles { get; set; } = Array.Empty<BeamNgVehicleDescriptor>();
    public int SelectedIndex { get; set; }
    public bool OverlayVisible { get; set; }
    public string StatusText { get; set; } = string.Empty;

    private Game? _game;
    private SpriteBatch? _spriteBatch;
    private Texture? _pixel;
    private SpriteFont? _font;

    public VehicleSelectionOverlay(IServiceRegistry services) : base(services)
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
        DrawPanel();
        _spriteBatch.End();
    }

    private void DrawPanel()
    {
        var visibleCount = Math.Max(Vehicles.Count, 1);
        var panelHeight = HeaderHeight + FooterHeight + RowHeight * visibleCount;

        DrawRect(new RectangleF(PanelX + 4f, PanelY + 5f, PanelWidth, panelHeight), new Color(0, 0, 0, 56));
        DrawRect(new RectangleF(PanelX, PanelY, PanelWidth, panelHeight), new Color(10, 14, 20, 216));
        DrawRect(new RectangleF(PanelX + 2f, PanelY + 2f, PanelWidth - 4f, panelHeight - 4f), new Color(20, 26, 34, 170));

        DrawText("Vehicle Select (F2 / Start)", PanelX + 14f, PanelY + 12f, new Color(236, 240, 247, 255));
        DrawText("Up/Down or D-pad, Enter/A load, Esc/B close", PanelX + 14f, PanelY + 28f, new Color(184, 192, 203, 255));

        var listY = PanelY + HeaderHeight;
        if (Vehicles.Count == 0)
        {
            DrawText("No vehicles found in Resources\\BeamNG Vehicles.", PanelX + 14f, listY + 2f, Color.White);
        }
        else
        {
            var clampedIndex = Math.Clamp(SelectedIndex, 0, Vehicles.Count - 1);
            for (var i = 0; i < Vehicles.Count; i++)
            {
                var rowY = listY + i * RowHeight;
                var isSelected = i == clampedIndex;
                var rowColor = isSelected
                    ? new Color(214, 148, 78, 196)
                    : new Color(38, 44, 54, 138);
                DrawRect(new RectangleF(PanelX + 10f, rowY, PanelWidth - 20f, RowHeight - 2f), rowColor);

                var label = Vehicles[i].DisplayName;
                var detail = Vehicles[i].SourceLabel;
                var textColor = isSelected ? new Color(255, 246, 236, 255) : new Color(224, 230, 238, 255);
                var detailColor = isSelected ? new Color(255, 228, 198, 235) : new Color(162, 172, 184, 230);

                DrawText(label, PanelX + 16f, rowY + 2f, textColor);
                DrawText(detail, PanelX + PanelWidth - 72f, rowY + 2f, detailColor);
            }
        }

        DrawText(StatusText, PanelX + 14f, PanelY + panelHeight - 22f, new Color(184, 192, 203, 255));
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
