using System;
using System.Collections.Generic;
using System.Linq;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally.Vehicle.Rendering;

internal sealed record SuspensionVisualLinkSpec(
    string Name,
    Entity StartEntity,
    Vector3 StartLocalPosition,
    Entity EndEntity,
    Vector3 EndLocalPosition,
    bool EndUsesNonSpinTransform,
    float Radius,
    Color4 Color);

internal sealed class SuspensionVisualKinematicsRig
{
    private readonly SuspensionVisualLinkRuntime[] _links;

    public SuspensionVisualKinematicsRig(IEnumerable<SuspensionVisualLinkRuntime> links)
    {
        _links = links.ToArray();
    }

    public void UpdateVisuals()
    {
        foreach (var link in _links)
        {
            link.StartEntity.Transform.UpdateWorldMatrix();
            link.EndEntity.Transform.UpdateWorldMatrix();

            var start = Vector3.TransformCoordinate(link.StartLocalPosition, link.StartEntity.Transform.WorldMatrix);
            var end = link.EndUsesNonSpinTransform
                ? TransformWheelLocalPositionWithoutSpin(
                    link.StartEntity.Transform.WorldMatrix,
                    link.EndEntity.Transform.WorldMatrix,
                    link.EndLocalPosition)
                : Vector3.TransformCoordinate(link.EndLocalPosition, link.EndEntity.Transform.WorldMatrix);
            var delta = end - start;
            var length = delta.Length();
            if (length < 1e-4f)
            {
                continue;
            }

            var forward = delta / length;
            var upReference = MathF.Abs(Vector3.Dot(forward, Vector3.UnitY)) > 0.97f
                ? Vector3.UnitZ
                : Vector3.UnitY;
            var rotation = CreateRotationFromXAxis(forward, upReference);

            link.VisualEntity.Transform.Position = (start + end) * 0.5f;
            link.VisualEntity.Transform.Rotation = rotation;
            link.VisualEntity.Transform.Scale = new Vector3(length, link.Radius, link.Radius);
        }
    }

    internal sealed record SuspensionVisualLinkRuntime(
        Entity VisualEntity,
        Entity StartEntity,
        Vector3 StartLocalPosition,
        Entity EndEntity,
        Vector3 EndLocalPosition,
        bool EndUsesNonSpinTransform,
        float Radius);

    internal static Vector3 TransformWheelLocalPositionWithoutSpin(
        Matrix chassisWorld,
        Matrix wheelWorld,
        Vector3 wheelLocalPosition)
    {
        var fallbackUp = SafeNormalize(chassisWorld.Up, Vector3.UnitY);
        var fallbackForward = SafeNormalize(chassisWorld.Backward, Vector3.UnitZ);
        var wheelRight = SafeNormalize(wheelWorld.Right, Vector3.UnitX);
        var wheelUp = SafeNormalize(ProjectOnPlane(fallbackUp, wheelRight), fallbackUp);
        var wheelForward = SafeNormalize(Vector3.Cross(wheelRight, wheelUp), fallbackForward);

        return wheelWorld.TranslationVector +
               wheelRight * wheelLocalPosition.X +
               wheelUp * wheelLocalPosition.Y +
               wheelForward * wheelLocalPosition.Z;
    }

    private static Quaternion CreateRotationFromXAxis(Vector3 forward, Vector3 upReference)
    {
        var alignAxis = Vector3.Cross(Vector3.UnitX, forward);
        Quaternion primaryRotation;
        if (alignAxis.LengthSquared() < 1e-6f)
        {
            primaryRotation = Vector3.Dot(Vector3.UnitX, forward) >= 0f
                ? Quaternion.Identity
                : Quaternion.RotationAxis(upReference, MathF.PI);
        }
        else
        {
            var dot = (float)Math.Clamp(Vector3.Dot(Vector3.UnitX, forward), -1f, 1f);
            primaryRotation = Quaternion.RotationAxis(Vector3.Normalize(alignAxis), MathF.Acos(dot));
        }

        var currentUp = Vector3.Transform(Vector3.UnitY, primaryRotation);
        var projectedUp = Vector3.Normalize(currentUp - forward * Vector3.Dot(currentUp, forward));
        var targetUp = Vector3.Normalize(upReference - forward * Vector3.Dot(upReference, forward));
        var signedAngle = MathF.Atan2(
            Vector3.Dot(Vector3.Cross(projectedUp, targetUp), forward),
            (float)Math.Clamp(Vector3.Dot(projectedUp, targetUp), -1f, 1f));
        var twistRotation = Quaternion.RotationAxis(forward, signedAngle);
        return Quaternion.Normalize(twistRotation * primaryRotation);
    }

    private static Vector3 ProjectOnPlane(Vector3 vector, Vector3 planeNormal)
    {
        return vector - planeNormal * Vector3.Dot(vector, planeNormal);
    }

    private static Vector3 SafeNormalize(Vector3 value, Vector3 fallback)
    {
        var lengthSquared = value.LengthSquared();
        return lengthSquared < 1e-6f
            ? fallback
            : value / MathF.Sqrt(lengthSquared);
    }
}

internal static class SuspensionVisualKinematicsRigBuilder
{
    private static readonly LinkTemplate[] Templates =
    [
        new("front_upperarm", "wheel_FL", "uppermounts_F", "upperarm_F", 0.038f, new Color4(0.28f, 0.30f, 0.32f, 1f)),
        new("front_upperarm", "wheel_FR", "uppermounts_F", "upperarm_F", 0.038f, new Color4(0.28f, 0.30f, 0.32f, 1f)),
        new("front_lowerarm", "wheel_FL", "lowermounts_F", "lowerarm_F", 0.045f, new Color4(0.24f, 0.25f, 0.27f, 1f)),
        new("front_lowerarm", "wheel_FR", "lowermounts_F", "lowerarm_F", 0.045f, new Color4(0.24f, 0.25f, 0.27f, 1f)),
        new("front_coilover", "wheel_FL", "shocktop_F", "lowerarm_F", 0.030f, new Color4(0.86f, 0.73f, 0.20f, 1f)),
        new("front_coilover", "wheel_FR", "shocktop_F", "lowerarm_F", 0.030f, new Color4(0.86f, 0.73f, 0.20f, 1f)),
        new("front_tierod", "wheel_FL", "tierod_M", "tierod_F", 0.022f, new Color4(0.64f, 0.66f, 0.70f, 1f)),
        new("front_tierod", "wheel_FR", "tierod_M", "tierod_F", 0.022f, new Color4(0.64f, 0.66f, 0.70f, 1f)),
        new("rear_trailingarm", "wheel_RL", "lowermounts_R", "lowerarm_R", 0.046f, new Color4(0.24f, 0.25f, 0.27f, 1f)),
        new("rear_trailingarm", "wheel_RR", "lowermounts_R", "lowerarm_R", 0.046f, new Color4(0.24f, 0.25f, 0.27f, 1f)),
        new("rear_coilover", "wheel_RL", "shocktop_R", "shockbottom_R", 0.030f, new Color4(0.86f, 0.73f, 0.20f, 1f)),
        new("rear_coilover", "wheel_RR", "shocktop_R", "shockbottom_R", 0.030f, new Color4(0.86f, 0.73f, 0.20f, 1f)),
        new("rear_halfshaft", "wheel_RL", "transaxle", "hub_R", 0.024f, new Color4(0.36f, 0.38f, 0.42f, 1f)),
        new("rear_halfshaft", "wheel_RR", "transaxle", "hub_R", 0.024f, new Color4(0.36f, 0.38f, 0.42f, 1f)),
    ];

    public static IReadOnlyList<SuspensionVisualLinkSpec> BuildLinkSpecs(VehicleDefinition definition, Physics.VehicleBuilderResult result)
    {
        var wheelEntities = new Dictionary<string, Entity>(StringComparer.OrdinalIgnoreCase)
        {
            ["wheel_FL"] = result.WheelFL,
            ["wheel_FR"] = result.WheelFR,
            ["wheel_RL"] = result.WheelRL,
            ["wheel_RR"] = result.WheelRR,
        };
        var specs = new List<SuspensionVisualLinkSpec>();

        foreach (var template in Templates)
        {
            if (!wheelEntities.TryGetValue(template.WheelKey, out var wheelEntity))
            {
                continue;
            }

            if (!TryResolveAnchor(definition, result.ChassisEntity, template.StartGroup, template.WheelKey, out var startLocalPosition))
            {
                continue;
            }

            var endOnChassis = template.EndGroup.Equals("transaxle", StringComparison.OrdinalIgnoreCase);
            var endEntity = endOnChassis ? result.ChassisEntity : wheelEntity;
            if (!TryResolveAnchor(definition, endEntity, template.EndGroup, template.WheelKey, out var endLocalPosition))
            {
                continue;
            }

            specs.Add(new SuspensionVisualLinkSpec(
                $"{template.Name}_{template.WheelKey}",
                result.ChassisEntity,
                startLocalPosition,
                endEntity,
                endLocalPosition,
                endEntity != result.ChassisEntity,
                template.Radius,
                template.Color));
        }

        return specs;
    }

    public static bool IsSuspensionKinematicFlexBody(AssembledFlexBody flexBody)
    {
        return ContainsKinematicVisualToken(flexBody.MeshName) ||
               ContainsKinematicVisualToken(flexBody.SourcePartName) ||
               ContainsKinematicVisualToken(flexBody.SourceSlotType);
    }

    private static bool TryResolveAnchor(
        VehicleDefinition definition,
        Entity entity,
        string groupName,
        string wheelKey,
        out Vector3 localPosition)
    {
        localPosition = default;
        var sideFilter = wheelKey.EndsWith("L", StringComparison.OrdinalIgnoreCase) ? 1f : -1f;
        var matchingNodes = definition.Nodes.Values
            .Where(node => node.Groups.Contains(groupName, StringComparer.OrdinalIgnoreCase))
            .ToList();
        if (matchingNodes.Count == 0)
        {
            return false;
        }

        if (!groupName.Equals("transaxle", StringComparison.OrdinalIgnoreCase))
        {
            var sideNodes = matchingNodes
                .Where(node => MathF.Abs(node.Position.X) < 0.05f || MathF.Sign(node.Position.X) == MathF.Sign(sideFilter))
                .ToList();
            if (sideNodes.Count > 0)
            {
                matchingNodes = sideNodes;
            }
        }

        var worldPosition = Vehicle.Physics.VehiclePhysicsBuilder.BeamNGToStride(ComputeCentroid(matchingNodes));
        localPosition = worldPosition - entity.Transform.Position;
        return true;
    }

    private static System.Numerics.Vector3 ComputeCentroid(IEnumerable<AssembledNode> nodes)
    {
        var total = System.Numerics.Vector3.Zero;
        var count = 0;
        foreach (var node in nodes)
        {
            total += node.Position;
            count++;
        }

        return count == 0 ? System.Numerics.Vector3.Zero : total / count;
    }

    private static bool ContainsKinematicVisualToken(string value)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return false;
        }

        return value.Contains("upperarm", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("lowerarm", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("coilover", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("tierod", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("halfshaft", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("driveshaft", StringComparison.OrdinalIgnoreCase) ||
               value.Contains("propshaft", StringComparison.OrdinalIgnoreCase) ||
               ContainsDelimitedToken(value, "arm_r") ||
               ContainsDelimitedToken(value, "arm_l");
    }

    private static bool ContainsDelimitedToken(string value, string token)
    {
        var normalized = value
            .Replace("-", "_")
            .Replace(".", "_")
            .Replace(' ', '_');
        return normalized.Contains($"_{token}_", StringComparison.OrdinalIgnoreCase) ||
               normalized.StartsWith($"{token}_", StringComparison.OrdinalIgnoreCase) ||
               normalized.EndsWith($"_{token}", StringComparison.OrdinalIgnoreCase) ||
               normalized.Equals(token, StringComparison.OrdinalIgnoreCase);
    }

    private readonly record struct LinkTemplate(
        string Name,
        string WheelKey,
        string StartGroup,
        string EndGroup,
        float Radius,
        Color4 Color);
}
