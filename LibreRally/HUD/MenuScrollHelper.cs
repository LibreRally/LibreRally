using System;

namespace LibreRally.HUD
{
	/// <summary>
	/// Calculates vertical scroll offsets that keep the selected menu item inside the visible viewport.
	/// </summary>
	internal static class MenuScrollHelper
	{
		/// <summary>
		/// Computes the vertical scroll offset needed to fully reveal a selected item.
		/// </summary>
		/// <param name="currentScroll">The current vertical scroll offset.</param>
		/// <param name="viewportHeight">The visible viewport height.</param>
		/// <param name="itemTop">The selected item's top edge within the scroll content.</param>
		/// <param name="itemHeight">The selected item's height.</param>
		/// <param name="maximumScroll">The maximum vertical scroll offset allowed by the container.</param>
		/// <returns>The clamped vertical scroll offset that keeps the selected item visible.</returns>
		internal static int ComputeVisibleVerticalScroll(
			int currentScroll,
			int viewportHeight,
			int itemTop,
			int itemHeight,
			int maximumScroll)
		{
			var clampedMaximum = Math.Max(0, maximumScroll);
			var clampedCurrent = Math.Clamp(currentScroll, 0, clampedMaximum);
			if (viewportHeight <= 0 || itemHeight <= 0)
			{
				return clampedCurrent;
			}

			var itemBottom = itemTop + itemHeight;
			if (itemTop < clampedCurrent)
			{
				return Math.Clamp(itemTop, 0, clampedMaximum);
			}

			if (itemBottom > clampedCurrent + viewportHeight)
			{
				return Math.Clamp(itemBottom - viewportHeight, 0, clampedMaximum);
			}

			return clampedCurrent;
		}

		/// <summary>
		/// Computes the vertical scroll offset needed to reveal a fixed-height item in an indexed list.
		/// </summary>
		/// <param name="currentScroll">The current vertical scroll offset.</param>
		/// <param name="viewportHeight">The visible viewport height.</param>
		/// <param name="selectedIndex">The selected item index.</param>
		/// <param name="itemHeight">The item height.</param>
		/// <param name="itemSpacing">The spacing between adjacent items.</param>
		/// <param name="itemCount">The total number of items in the list.</param>
		/// <returns>The clamped vertical scroll offset that keeps the indexed item visible.</returns>
		internal static int ComputeVisibleVerticalScrollForFixedList(
			int currentScroll,
			int viewportHeight,
			int selectedIndex,
			int itemHeight,
			int itemSpacing,
			int itemCount)
		{
			if (itemCount <= 0)
			{
				return 0;
			}

			var clampedIndex = Math.Clamp(selectedIndex, 0, itemCount - 1);
			var itemPitch = Math.Max(0, itemHeight + itemSpacing);
			var itemTop = clampedIndex * itemPitch;
			var contentHeight = Math.Max(0, itemCount * itemPitch - itemSpacing);
			var maximumScroll = Math.Max(0, contentHeight - Math.Max(0, viewportHeight));
			return ComputeVisibleVerticalScroll(currentScroll, viewportHeight, itemTop, itemHeight, maximumScroll);
		}
	}
}
