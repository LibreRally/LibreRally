using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using LibreRally.Vehicle.Rendering;
using Stride.Core.Diagnostics;

namespace LibreRally.Vehicle
{
	/// <summary>
	/// Discovers and loads DAE and DTS model sources for vehicle visual assembly.
	/// </summary>
	internal sealed class VehicleModelSourceResolver
	{
		private static readonly Logger Log = GlobalLogger.GetLogger("VehicleModelSourceResolver");

		/// <summary>
		/// Loads every supported model source under the specified vehicle folder.
		/// </summary>
		/// <param name="folder">The vehicle folder to scan.</param>
		/// <returns>The loaded model sources.</returns>
		public List<VehicleModelSource> LoadFromFolder(string folder)
		{
			var sw = System.Diagnostics.Stopwatch.StartNew();
			var enumerateSw = System.Diagnostics.Stopwatch.StartNew();
			var modelFiles = Directory.EnumerateFiles(folder, "*.dae", SearchOption.AllDirectories)
				.Concat(Directory.EnumerateFiles(folder, "*.dts", SearchOption.AllDirectories))
				.OrderBy(path => Path.GetDirectoryName(path)?.Equals(folder, StringComparison.OrdinalIgnoreCase) == true ? 0 : 1)
				.ThenBy(path => path, StringComparer.OrdinalIgnoreCase)
				.ToList();
			enumerateSw.Stop();
			Log.Info($"[VehicleModelSourceResolver] File enumeration: {enumerateSw.ElapsedMilliseconds}ms ({modelFiles.Count} files found)");

			var resultSw = System.Diagnostics.Stopwatch.StartNew();
			var result = LoadFromFiles(modelFiles);
			resultSw.Stop();
			Log.Info($"[VehicleModelSourceResolver] File loading: {resultSw.ElapsedMilliseconds}ms ({result.Count} sources loaded)");
			
			sw.Stop();
			Log.Info($"[VehicleModelSourceResolver] Total LoadFromFolder: {sw.ElapsedMilliseconds}ms");
			return result;
		}

		/// <summary>
		/// Loads supported model sources from the supplied file list.
		/// </summary>
		/// <param name="modelFiles">The candidate model files.</param>
		/// <returns>The successfully loaded model sources.</returns>
		public List<VehicleModelSource> LoadFromFiles(IEnumerable<string> modelFiles)
		{
			var result = new List<VehicleModelSource>();
			int successCount = 0, failureCount = 0;
			foreach (var modelFile in modelFiles
				         .Where(path => !string.IsNullOrWhiteSpace(path) && File.Exists(path))
				         .Distinct(StringComparer.OrdinalIgnoreCase))
			{
				try
				{
					var fileSw = System.Diagnostics.Stopwatch.StartNew();
					Dictionary<string, string> textureMap;
					List<ColladaMesh> meshes;
					if (modelFile.EndsWith(".dae", StringComparison.OrdinalIgnoreCase))
					{
						textureMap = ColladaLoader.LoadTextureMap(modelFile);
						meshes = ColladaLoader.Load(modelFile);
						fileSw.Stop();
						Log.Info($"DAE ({fileSw.ElapsedMilliseconds}ms): {Path.GetFileName(modelFile)} | {meshes.Count} sub-meshes | Collada textures: {textureMap.Count}");
					}
					else
					{
						textureMap = new Dictionary<string, string>(StringComparer.OrdinalIgnoreCase);
						meshes = DtsLoader.Load(modelFile);
						fileSw.Stop();
						Log.Info($"DTS ({fileSw.ElapsedMilliseconds}ms): {Path.GetFileName(modelFile)} | {meshes.Count} sub-meshes");
					}

					result.Add(new VehicleModelSource(modelFile, meshes, textureMap));
					successCount++;
				}
				catch (Exception ex)
				{
					Log.Warning($"Could not load model '{Path.GetFileName(modelFile)}': {ex.Message}");
					failureCount++;
				}
			}

			Log.Info($"[VehicleModelSourceResolver] LoadFromFiles: {successCount} succeeded, {failureCount} failed");
			return result;
		}
	}

	/// <summary>
	/// Represents one loaded source file together with its parsed meshes and texture map.
	/// </summary>
	/// <param name="SourcePath">The original source path.</param>
	/// <param name="Meshes">The parsed meshes from the source file.</param>
	/// <param name="TextureMap">The collada material-to-texture map.</param>
	internal sealed record VehicleModelSource(string SourcePath, List<ColladaMesh> Meshes, Dictionary<string, string> TextureMap);
}
