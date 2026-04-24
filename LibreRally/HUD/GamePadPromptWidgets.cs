using System;
using System.Collections.Generic;
using System.IO;
using Myra.Graphics2D.TextureAtlases;
using Myra.Graphics2D.UI;
using Stride.Core.Mathematics;
using Stride.Engine;
using Stride.Graphics;
using StrideTextureImage = Stride.Graphics.Image;

namespace LibreRally.HUD
{
	/// <summary>
	/// Enumerates the Xbox controller prompt icons used by the Myra overlays.
	/// </summary>
	internal enum GamePadPromptIcon
	{
		A,
		B,
		Menu,
		LeftBumper,
		RightBumper,
		DPad,
		DPadUp,
		DPadDown,
		DPadLeft,
		DPadRight,
	}

	/// <summary>
	/// Represents one controller prompt consisting of one or more icons and a text label.
	/// </summary>
	/// <param name="Text">The prompt label.</param>
	/// <param name="Icons">The icon sequence shown ahead of the label.</param>
	internal readonly record struct GamePadPrompt(string Text, IReadOnlyList<GamePadPromptIcon> Icons);

	/// <summary>
	/// Builds Myra widgets that combine Xbox controller prompt icons with short labels.
	/// </summary>
	internal static class GamePadPromptWidgets
	{
		private const int IconSize = 20;

		private static readonly string IconRoot = Path.Combine(
			AppContext.BaseDirectory,
			"Resources",
			"Xbox Series Button Icons and Controls",
			"Buttons Full Solid",
			"Black",
			"128w");

		private static readonly Dictionary<string, Texture> TexturesByPath = new(StringComparer.OrdinalIgnoreCase);
		private static readonly Dictionary<string, TextureRegion?> RegionsByPath = new(StringComparer.OrdinalIgnoreCase);

		/// <summary>
		/// Creates a controller prompt widget row for the supplied prompt list.
		/// </summary>
		/// <param name="game">The active Stride game instance.</param>
		/// <param name="textColor">The text color for prompt labels.</param>
		/// <param name="fallbackText">Fallback text shown when icon rendering is unavailable.</param>
		/// <param name="prompts">The prompts to render in the row.</param>
		/// <returns>A Myra widget representing the prompt row.</returns>
		internal static Widget CreatePromptStrip(Game? game, Color textColor, string fallbackText, params GamePadPrompt[] prompts)
		{
			if (game == null || prompts.Length == 0)
			{
				return CreateFallbackLabel(fallbackText, textColor);
			}

			var row = new HorizontalStackPanel { Spacing = 12, };

			foreach (var prompt in prompts)
			{
				row.Widgets.Add(CreatePromptWidget(game, prompt, textColor));
			}

			return row;
		}

		/// <summary>
		/// Creates a prompt descriptor from a label and one or more icons.
		/// </summary>
		/// <param name="text">The prompt label.</param>
		/// <param name="icons">The icons shown before the label.</param>
		/// <returns>A prompt descriptor.</returns>
		internal static GamePadPrompt Prompt(string text, params GamePadPromptIcon[] icons) => new(text, icons);

		private static Widget CreatePromptWidget(Game game, GamePadPrompt prompt, Color textColor)
		{
			var stack = new HorizontalStackPanel { Spacing = 6, };

			foreach (var icon in prompt.Icons)
			{
				var region = TryGetRegion(game, icon);
				if (region == null)
				{
					continue;
				}

				stack.Widgets.Add(new Myra.Graphics2D.UI.Image
				{
					Width = IconSize, Height = IconSize, Renderable = region, ResizeMode = ImageResizeMode.Stretch,
				});
			}

			stack.Widgets.Add(new Label { Text = prompt.Text, TextColor = textColor, });
			return stack;
		}

		private static Label CreateFallbackLabel(string text, Color textColor) => new() { Text = text, TextColor = textColor, Wrap = true, };

		private static TextureRegion? TryGetRegion(Game game, GamePadPromptIcon icon)
		{
			var iconPath = ResolveIconPath(icon);
			if (RegionsByPath.TryGetValue(iconPath, out var cached))
			{
				return cached;
			}

			if (!File.Exists(iconPath))
			{
				RegionsByPath[iconPath] = null;
				return null;
			}

			try
			{
				using var stream = File.OpenRead(iconPath);
				using var image = StrideTextureImage.Load(stream);
				var texture = Texture.New(game.GraphicsDevice, image);
				var region = new TextureRegion(texture);
				TexturesByPath[iconPath] = texture;
				RegionsByPath[iconPath] = region;
				return region;
			}
			catch
			{
				RegionsByPath[iconPath] = null;
				return null;
			}
		}

		private static string ResolveIconPath(GamePadPromptIcon icon)
		{
			var fileName = icon switch
			{
				GamePadPromptIcon.A => "A.png",
				GamePadPromptIcon.B => "B.png",
				GamePadPromptIcon.Menu => "Menu.png",
				GamePadPromptIcon.LeftBumper => "Left Bumper.png",
				GamePadPromptIcon.RightBumper => "Right Bumper.png",
				GamePadPromptIcon.DPad => "D-Pad.png",
				GamePadPromptIcon.DPadUp => "D-Pad Up.png",
				GamePadPromptIcon.DPadDown => "D-Pad Down.png",
				GamePadPromptIcon.DPadLeft => "D-Pad Left.png",
				GamePadPromptIcon.DPadRight => "D-Pad Right.png",
				_ => "D-Pad.png",
			};

			return Path.Combine(IconRoot, fileName);
		}
	}
}
