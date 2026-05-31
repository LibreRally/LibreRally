namespace LibreRally.Tests
{
	/// <summary>
	/// Verifies Cesium texture decoding helpers used by the runtime tileset importer.
	/// </summary>
	public class CesiumTextureDecoderTests
	{
		/// <summary>
		/// Verifies that RGB pixel data is expanded to RGBA with opaque alpha.
		/// </summary>
		[Fact]
		public void ConvertToRgba8_RgbSource_AppendsOpaqueAlpha()
		{
			var source = new byte[]
			{
				10, 20, 30,
				40, 50, 60,
			};

			var converted = CesiumTextureDecoder.ConvertToRgba8(source, channels: 3, bytesPerChannel: 1);

			Assert.Equal(
				new byte[]
				{
					10, 20, 30, 255,
					40, 50, 60, 255,
				},
				converted);
		}

		/// <summary>
		/// Verifies that grayscale plus alpha sources are expanded into RGBA.
		/// </summary>
		[Fact]
		public void ConvertToRgba8_GrayAlphaSource_ExpandsChannels()
		{
			var source = new byte[]
			{
				80, 200,
				25, 255,
			};

			var converted = CesiumTextureDecoder.ConvertToRgba8(source, channels: 2, bytesPerChannel: 1);

			Assert.Equal(
				new byte[]
				{
					80, 80, 80, 200,
					25, 25, 25, 255,
				},
				converted);
		}

		/// <summary>
		/// Verifies that 16-bit channels are reduced to 8-bit output.
		/// </summary>
		[Fact]
		public void ConvertToRgba8_Rgba16Source_DownsamplesTo8Bit()
		{
			var source = new byte[]
			{
				0x00, 0x00,
				0xFF, 0x7F,
				0xFF, 0xFF,
				0x00, 0x80,
			};

			var converted = CesiumTextureDecoder.ConvertToRgba8(source, channels: 4, bytesPerChannel: 2);

			Assert.Equal(new byte[] { 0, 127, 255, 127 }, converted);
		}
	}
}
