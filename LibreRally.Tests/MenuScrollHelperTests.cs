using LibreRally.HUD;

namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies the shared menu scroll helper keeps the selected item inside the viewport.
	/// </summary>
	public sealed class MenuScrollHelperTests
	{
		/// <summary>
		/// Verifies that compute visible vertical scroll leaves visible item unchanged.
		/// </summary>
		[Fact]
		public void ComputeVisibleVerticalScroll_LeavesVisibleItemUnchanged()
		{
			var scroll = MenuScrollHelper.ComputeVisibleVerticalScroll(
				currentScroll: 100,
				viewportHeight: 300,
				itemTop: 180,
				itemHeight: 70,
				maximumScroll: 500);

			Assert.Equal(100, scroll);
		}

		/// <summary>
		/// Verifies that compute visible vertical scroll scrolls up when item is above viewport.
		/// </summary>
		[Fact]
		public void ComputeVisibleVerticalScroll_ScrollsUpWhenItemIsAboveViewport()
		{
			var scroll = MenuScrollHelper.ComputeVisibleVerticalScroll(
				currentScroll: 160,
				viewportHeight: 300,
				itemTop: 120,
				itemHeight: 70,
				maximumScroll: 500);

			Assert.Equal(120, scroll);
		}

		/// <summary>
		/// Verifies that compute visible vertical scroll scrolls down when item is below viewport.
		/// </summary>
		[Fact]
		public void ComputeVisibleVerticalScroll_ScrollsDownWhenItemIsBelowViewport()
		{
			var scroll = MenuScrollHelper.ComputeVisibleVerticalScroll(
				currentScroll: 100,
				viewportHeight: 300,
				itemTop: 360,
				itemHeight: 90,
				maximumScroll: 500);

			Assert.Equal(150, scroll);
		}

		/// <summary>
		/// Verifies that compute visible vertical scroll clamps to maximum scroll.
		/// </summary>
		[Fact]
		public void ComputeVisibleVerticalScroll_ClampsToMaximumScroll()
		{
			var scroll = MenuScrollHelper.ComputeVisibleVerticalScroll(
				currentScroll: 100,
				viewportHeight: 300,
				itemTop: 700,
				itemHeight: 120,
				maximumScroll: 400);

			Assert.Equal(400, scroll);
		}

		/// <summary>
		/// Verifies that compute visible vertical scroll for fixed list scrolls to reveal indexed item.
		/// </summary>
		[Fact]
		public void ComputeVisibleVerticalScrollForFixedList_ScrollsToRevealIndexedItem()
		{
			var scroll = MenuScrollHelper.ComputeVisibleVerticalScrollForFixedList(
				currentScroll: 0,
				viewportHeight: 320,
				selectedIndex: 5,
				itemHeight: 78,
				itemSpacing: 8,
				itemCount: 6);

			Assert.Equal(188, scroll);
		}

		/// <summary>
		/// Verifies that compute visible vertical scroll for fixed list clamps at top for first item.
		/// </summary>
		[Fact]
		public void ComputeVisibleVerticalScrollForFixedList_ClampsAtTopForFirstItem()
		{
			var scroll = MenuScrollHelper.ComputeVisibleVerticalScrollForFixedList(
				currentScroll: 150,
				viewportHeight: 442,
				selectedIndex: 0,
				itemHeight: 116,
				itemSpacing: 8,
				itemCount: 12);

			Assert.Equal(0, scroll);
		}
	}
}
