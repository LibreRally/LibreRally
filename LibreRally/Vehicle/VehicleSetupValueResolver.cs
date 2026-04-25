using System;
using System.Collections.Generic;

namespace LibreRally.Vehicle
{
	internal static class VehicleSetupValueResolver
	{
		private static readonly string[] EffectiveValueSuffixPriority =
		[
			"_asphalt",
			"_tarmac",
			"_gravel",
			"_dirt",
			"_mud",
			"_snow",
			"_rally",
			"_race",
		];

		public static float GetPositiveValue(IReadOnlyDictionary<string, float> vars, float fallback, params string[] baseNames)
		{
			return TryResolvePositiveValue(vars, out var value, out _, baseNames) ? value : fallback;
		}

		public static float GetPositiveValue(
			IReadOnlyDictionary<string, float>? preferredVars,
			IReadOnlyDictionary<string, float> vars,
			float fallback,
			params string[] baseNames)
		{
			return TryResolvePositiveValue(preferredVars, vars, out var value, out _, baseNames) ? value : fallback;
		}

		public static float GetFiniteValue(IReadOnlyDictionary<string, float> vars, float fallback, params string[] baseNames)
		{
			return TryResolveFiniteValue(vars, out var value, out _, baseNames) ? value : fallback;
		}

		public static float GetFiniteValue(
			IReadOnlyDictionary<string, float>? preferredVars,
			IReadOnlyDictionary<string, float> vars,
			float fallback,
			params string[] baseNames)
		{
			return TryResolveFiniteValue(preferredVars, vars, out var value, out _, baseNames) ? value : fallback;
		}

		public static float? TryGetFiniteValue(IReadOnlyDictionary<string, float> vars, params string[] baseNames)
		{
			return TryResolveFiniteValue(vars, out var value, out _, baseNames) ? value : null;
		}

		public static float? TryGetFiniteValue(
			IReadOnlyDictionary<string, float>? preferredVars,
			IReadOnlyDictionary<string, float> vars,
			params string[] baseNames)
		{
			return TryResolveFiniteValue(preferredVars, vars, out var value, out _, baseNames) ? value : null;
		}

		public static bool TryResolvePositiveValue(
			IReadOnlyDictionary<string, float> vars,
			out float value,
			out string resolvedName,
			params string[] baseNames)
		{
			return TryResolveValue(null, vars, requirePositive: true, out value, out resolvedName, baseNames);
		}

		public static bool TryResolvePositiveValue(
			IReadOnlyDictionary<string, float>? preferredVars,
			IReadOnlyDictionary<string, float> vars,
			out float value,
			out string resolvedName,
			params string[] baseNames)
		{
			return TryResolveValue(preferredVars, vars, requirePositive: true, out value, out resolvedName, baseNames);
		}

		public static bool TryResolveFiniteValue(
			IReadOnlyDictionary<string, float> vars,
			out float value,
			out string resolvedName,
			params string[] baseNames)
		{
			return TryResolveValue(null, vars, requirePositive: false, out value, out resolvedName, baseNames);
		}

		public static bool TryResolveFiniteValue(
			IReadOnlyDictionary<string, float>? preferredVars,
			IReadOnlyDictionary<string, float> vars,
			out float value,
			out string resolvedName,
			params string[] baseNames)
		{
			return TryResolveValue(preferredVars, vars, requirePositive: false, out value, out resolvedName, baseNames);
		}

		private static bool TryResolveValue(
			IReadOnlyDictionary<string, float>? preferredVars,
			IReadOnlyDictionary<string, float> vars,
			bool requirePositive,
			out float value,
			out string resolvedName,
			params string[] baseNames)
		{
			if (TryResolveValueFromDictionary(preferredVars, requirePositive, out value, out resolvedName, baseNames))
			{
				return true;
			}

			return TryResolveValueFromDictionary(vars, requirePositive, out value, out resolvedName, baseNames);
		}

		private static bool TryResolveValueFromDictionary(
			IReadOnlyDictionary<string, float>? vars,
			bool requirePositive,
			out float value,
			out string resolvedName,
			params string[] baseNames)
		{
			if (vars == null)
			{
				value = default;
				resolvedName = string.Empty;
				return false;
			}

			foreach (var candidate in EnumerateCandidates(baseNames))
			{
				if (!vars.TryGetValue(candidate, out var candidateValue))
				{
					continue;
				}

				if ((!requirePositive || candidateValue > 0f) && float.IsFinite(candidateValue))
				{
					value = candidateValue;
					resolvedName = candidate;
					return true;
				}
			}

			value = default;
			resolvedName = string.Empty;
			return false;
		}

		private static IEnumerable<string> EnumerateCandidates(IEnumerable<string> baseNames)
		{
			var yielded = new HashSet<string>(StringComparer.OrdinalIgnoreCase);
			foreach (var baseName in baseNames)
			{
				if (string.IsNullOrWhiteSpace(baseName))
				{
					continue;
				}

				if (HasKnownSuffix(baseName))
				{
					if (yielded.Add(baseName))
					{
						yield return baseName;
					}

					continue;
				}

				foreach (var suffix in EffectiveValueSuffixPriority)
				{
					var suffixedName = baseName + suffix;
					if (yielded.Add(suffixedName))
					{
						yield return suffixedName;
					}
				}

				if (yielded.Add(baseName))
				{
					yield return baseName;
				}
			}
		}

		private static bool HasKnownSuffix(string name)
		{
			foreach (var suffix in EffectiveValueSuffixPriority)
			{
				if (name.EndsWith(suffix, StringComparison.OrdinalIgnoreCase))
				{
					return true;
				}
			}

			return false;
		}
	}
}
