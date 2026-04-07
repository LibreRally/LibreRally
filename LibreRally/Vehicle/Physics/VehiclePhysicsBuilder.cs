using System;
using System.Collections.Generic;
using System.Linq;
using Stride.BepuPhysics;
using Stride.BepuPhysics.Constraints;
using Stride.BepuPhysics.Definitions.Colliders;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally.Vehicle.Physics;

/// <summary>
/// Result of <see cref="VehiclePhysicsBuilder.Build"/>.
/// </summary>
public record VehicleBuilderResult(
    Entity RootEntity,
    Entity ChassisEntity,
    BodyComponent ChassisBody,
    Entity WheelFL,
    Entity WheelFR,
    Entity WheelRL,
    Entity WheelRR);

/// <summary>
/// Builds the Stride entity hierarchy and BEPU physics bodies for a vehicle.
///
/// Coordinate mapping from BeamNG (X=right, Y=forward, Z=up)
/// to Stride (X=right, Y=up, Z=backward):
///   strideX = jbeamX
///   strideY = jbeamZ
///   strideZ = -jbeamY
/// </summary>
public static class VehiclePhysicsBuilder
{
    /// <summary>
    /// Creates a root entity containing:
    ///   - A chassis entity with a <see cref="BodyComponent"/> and compound box colliders.
    ///   - One entity per detachable part, each with a <see cref="BodyComponent"/> and
    ///     a <see cref="WeldConstraintComponent"/> connecting it to the chassis.
    ///   - Four wheel entities with suspension constraints.
    /// </summary>
    public static VehicleBuilderResult Build(VehicleDefinition def)
    {
        var root = new Entity(def.VehicleName);

        var chassisPart = def.Parts.FirstOrDefault(p => !p.Detachable)
            ?? throw new InvalidOperationException("Vehicle has no chassis part.");

        var (chassisEntity, chassisBody) = BuildChassisEntity(def, chassisPart);
        root.AddChild(chassisEntity);

        // Detachable parts disabled: each part adds a weld-constrained BodyComponent which
        // destabilises the solver when there are 10-20 of them. Re-enable once physics is stable.
        // foreach (var part in def.Parts.Where(p => p.Detachable)) { ... }

        var (fl, fr, rl, rr) = BuildWheels(def, chassisBody, chassisEntity.Transform.Position);
        root.AddChild(fl);
        root.AddChild(fr);
        root.AddChild(rl);
        root.AddChild(rr);

        return new VehicleBuilderResult(root, chassisEntity, chassisBody, fl, fr, rl, rr);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Chassis
    // ──────────────────────────────────────────────────────────────────────────

    private static (Entity entity, BodyComponent body) BuildChassisEntity(
        VehicleDefinition def,
        VehiclePart chassisPart)
    {
        var allChassisNodes = chassisPart.ExclusiveNodeIds
            .Where(id => def.Nodes.ContainsKey(id))
            .Select(id => def.Nodes[id])
            .ToList();

        // Steelwheel slot nodes (fw1r/fw1rr etc.) sit at BeamNG Z=0 in the jbeam file —
        // their real positions are resolved by simulation beams in BeamNG, not stored as
        // absolute coordinates.  Excluding nodes with Z < 0.10 m removes these "floating
        // origin" nodes so they don't pull the chassis centroid down to ground level.
        //
        // We also limit to Z < 0.40 m (below the body sills) for the collision box so that
        // the box never extends down to ground level and blocks the wheels from landing.
        const float MinNodeZ  = 0.10f;  // excludes Z=0 steelwheel reference nodes
        const float BodyMinZ  = 0.55f;  // above sills: cabin-only box doesn't drag on the ground

        var structuralNodes = allChassisNodes.Where(n => n.Position.Z >= MinNodeZ).ToList();
        if (structuralNodes.Count < 5) structuralNodes = allChassisNodes;

        var bodyNodes = structuralNodes.Where(n => n.Position.Z >= BodyMinZ).ToList();
        if (bodyNodes.Count < 5) bodyNodes = structuralNodes;

        // Chassis entity position = centroid of body nodes (places origin up in the cabin,
        // not down in the suspension)
        Vector3 centroid = ComputeCentroid(bodyNodes.Select(n => n.Position));
        BoundingBox aabb  = ComputeAABB(bodyNodes.Select(n => n.Position));

        var entity = new Entity("chassis") { Transform = { Position = centroid } };

        var extents   = aabb.Maximum - aabb.Minimum;
        float totalMass = Math.Max(allChassisNodes.Sum(n => n.Weight), 100f);

        var body = new BodyComponent
        {
            Collider = new CompoundCollider
            {
                Colliders =
                {
                    new BoxCollider
                    {
                        Size = new Vector3(
                            Math.Max(extents.X, 0.3f),
                            Math.Max(extents.Y, 0.3f),
                            Math.Max(extents.Z, 0.3f)),
                        Mass = totalMass,
                    }
                }
            },
            Gravity = true,
        };

        Console.Error.WriteLine($"[VehiclePhysicsBuilder] Chassis allNodes={allChassisNodes.Count} " +
                          $"structural={structuralNodes.Count} body={bodyNodes.Count} centroid={centroid:F3} " +
                          $"boxExtents={extents:F3} mass={totalMass:F0}kg");
        entity.Add(body);
        return (entity, body);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Detachable parts
    // ──────────────────────────────────────────────────────────────────────────

    private static (Entity entity, BodyComponent body) BuildDetachablePartEntity(
        VehicleDefinition def,
        VehiclePart part,
        BodyComponent chassisBody,
        Entity chassisEntity)
    {
        var partNodes = part.ExclusiveNodeIds
            .Where(id => def.Nodes.ContainsKey(id))
            .Select(id => def.Nodes[id])
            .ToList();

        if (partNodes.Count == 0)
            return (new Entity(part.Name), new BodyComponent { Collider = new CompoundCollider() });

        Vector3 centroid = ComputeCentroid(partNodes.Select(n => n.Position));
        BoundingBox aabb = ComputeAABB(partNodes.Select(n => n.Position));
        float totalMass = partNodes.Sum(n => n.Weight);

        var extents = aabb.Maximum - aabb.Minimum;

        var entity = new Entity(part.Name)
        {
            Transform = { Position = centroid }
        };

        var body = new BodyComponent
        {
            Collider = new CompoundCollider
            {
                Colliders =
                {
                    new BoxCollider
                    {
                        Size = new Vector3(
                            Math.Max(extents.X, 0.05f),
                            Math.Max(extents.Y, 0.05f),
                            Math.Max(extents.Z, 0.05f)),
                        Mass = Math.Max(totalMass, 2f),
                    }
                }
            },
            Gravity = true,
        };

        entity.Add(body);

        // Weld this part to the chassis; the BreakablePartComponent will remove it on impact
        var weld = new WeldConstraintComponent
        {
            A = chassisBody,
            B = body,
        };
        entity.Add(weld);

        // Add the break monitor
        entity.Add(new BreakablePartComponent
        {
            BreakStrength = part.BreakStrength,
            WeldConstraint = weld,
        });

        return (entity, body);
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Wheels
    // ──────────────────────────────────────────────────────────────────────────

    private static readonly (string label, string suffix, bool isFront)[] WheelSlots =
    {
        ("wheel_FL", "_FL", true),
        ("wheel_FR", "_FR", true),
        ("wheel_RL", "_RL", false),
        ("wheel_RR", "_RR", false),
    };

    private static (Entity fl, Entity fr, Entity rl, Entity rr) BuildWheels(
        VehicleDefinition def,
        BodyComponent chassisBody,
        Vector3 chassisWorldPos)
    {
        // ── Step 1: Try to get wheel centres from flexbody positions ────────────
        // Brake disc flexbodies have an explicit absolute BeamNG pos (e.g. {x:-0.73, y:-1.31, z:0.285})
        // which is exactly the wheel centre.  This is far more reliable than averaging node positions
        // because steelwheel nodes start at the origin (Z=0) and are positioned by beams at runtime.
        var wheelCentresFromFlexbodies = new Dictionary<string, System.Numerics.Vector3>(StringComparer.OrdinalIgnoreCase);
        foreach (var (label, suffix, _) in WheelSlots)
        {
            foreach (var fb in def.FlexBodies)
            {
                if (fb.Position == null) continue;
                // Skip positions that carry no height info (z == 0 in BeamNG space).
                // Steelwheel mesh flexbodies store only a lateral offset (x ≠ 0, y=0, z=0)
                // because their real position is supplied by the slot's nodeOffset at runtime.
                // Brake disc flexbodies store the absolute wheel centre including z ≈ 0.285 m.
                if (Math.Abs(fb.Position.Value.Z) < 0.05f) continue;
                // Match flexbodies whose groups include the exact wheel group (e.g. "wheel_FR")
                bool matchesGroup = fb.NodeGroups.Any(g =>
                    g.Equals("wheel" + suffix, StringComparison.OrdinalIgnoreCase)
                    || g.Equals("wheelhub" + suffix, StringComparison.OrdinalIgnoreCase));
                if (!matchesGroup) continue;

                Console.Error.WriteLine($"[VehiclePhysicsBuilder] {label} flexbody candidate: mesh={fb.MeshName} " +
                                  $"pos={fb.Position.Value.X:F3},{fb.Position.Value.Y:F3},{fb.Position.Value.Z:F3}");
                if (!wheelCentresFromFlexbodies.ContainsKey(label))
                    wheelCentresFromFlexbodies[label] = fb.Position.Value;
            }
        }

        // ── Step 2: Fall back to node-based detection ──────────────────────────
        // Only used when flexbody positions are unavailable.
        // Filter: only groups that start with "wheel" (matches "wheel_FR", "wheelhub_FR")
        // and skip nodes with Z < 0.10 m (the Z=0 steelwheel reference nodes).
        var wheelGroupNodes = new Dictionary<string, List<AssembledNode>>(StringComparer.OrdinalIgnoreCase);

        foreach (var node in def.Nodes.Values)
        {
            if (node.Position.Z < 0.10f) continue;  // skip steelwheel origin nodes

            foreach (var group in node.Groups)
            {
                foreach (var (label, suffix, _) in WheelSlots)
                {
                    if (!wheelCentresFromFlexbodies.ContainsKey(label)
                        && group.StartsWith("wheel", StringComparison.OrdinalIgnoreCase)
                        && group.EndsWith(suffix, StringComparison.OrdinalIgnoreCase))
                    {
                        if (!wheelGroupNodes.ContainsKey(label))
                            wheelGroupNodes[label] = new List<AssembledNode>();
                        if (!wheelGroupNodes[label].Any(n => n.Id == node.Id))
                            wheelGroupNodes[label].Add(node);
                    }
                }
            }
        }

        // Pattern 2: sunburst-style shared "_hub_R" group split by X sign for rear.
        // Only applies when flexbody positions are unavailable for that corner.
        foreach (var (label, _, _) in WheelSlots.Where(s => !s.isFront))
        {
            if (wheelCentresFromFlexbodies.ContainsKey(label)) continue;
            if (wheelGroupNodes.ContainsKey(label)) continue;

            foreach (var node in def.Nodes.Values)
            {
                if (node.Position.Z < 0.10f) continue;
                bool isRearHub = node.Groups.Any(g =>
                    g.Contains("hub_R", StringComparison.OrdinalIgnoreCase)
                    && !g.Contains("hub_FR", StringComparison.OrdinalIgnoreCase)
                    && !g.Contains("hub_FL", StringComparison.OrdinalIgnoreCase));
                if (!isRearHub) continue;
                string l = node.Position.X < 0 ? "wheel_RR" : "wheel_RL";
                if (!wheelGroupNodes.ContainsKey(l))
                    wheelGroupNodes[l] = new List<AssembledNode>();
                if (!wheelGroupNodes[l].Any(n => n.Id == node.Id))
                    wheelGroupNodes[l].Add(node);
            }
        }

        // Only fill missing wheel slots from estimation; never overwrite already-found positions.
        var allFoundWheels = new HashSet<string>(wheelCentresFromFlexbodies.Keys, StringComparer.OrdinalIgnoreCase);
        allFoundWheels.UnionWith(wheelGroupNodes.Keys);
        if (allFoundWheels.Count < 4)
        {
            foreach (var (k, v) in EstimateWheelPositions(def))
                if (!allFoundWheels.Contains(k))
                    wheelGroupNodes[k] = v;
        }

        // ── Step 3: Spring parameters from .pc vars, with beam-derived fallback ──
        // rally_pro_asphalt.pc provides:
        //   spring_F_asphalt=60000 N/m,  damp_bump_F_asphalt=4400,  damp_rebound_F_asphalt=11500
        //   spring_R_asphalt=50000 N/m,  damp_bump_R_asphalt=4400,  damp_rebound_R_asphalt=9000
        float quarterMass = (float)(def.Nodes.Values.Sum(n => n.Weight) / 4.0);
        quarterMass = Math.Max(quarterMass, 50f);

        float GetVar(string name, float fallback) =>
            def.Vars.TryGetValue(name, out float v) && v > 0 ? v : fallback;

        float springF   = GetVar("spring_F_asphalt",   GetVar("spring_F",  60000f));
        float springR   = GetVar("spring_R_asphalt",   GetVar("spring_R",  50000f));
        float dampBumpF = GetVar("damp_bump_F_asphalt", GetVar("damp_bump_F", 4400f));
        float dampRebF  = GetVar("damp_rebound_F_asphalt", GetVar("damp_rebound_F", 11500f));
        float dampBumpR = GetVar("damp_bump_R_asphalt", GetVar("damp_bump_R", 4400f));
        float dampRebR  = GetVar("damp_rebound_R_asphalt", GetVar("damp_rebound_R", 9000f));

        // Average bump+rebound for single BEPU damping value; BEPU uses SpringDampingRatio
        float dampF = (dampBumpF + dampRebF) * 0.5f;
        float dampR = (dampBumpR + dampRebR) * 0.5f;

        float ComputeFreq(float k) =>
            Math.Clamp((float)(Math.Sqrt(k / quarterMass) / (2 * Math.PI)), 1.0f, 6.0f);
        float ComputeRatio(float c, float k) =>
            Math.Clamp(c / (2f * (float)Math.Sqrt(k * quarterMass)), 0.4f, 2.0f);

        float springFreqF  = ComputeFreq(springF);
        float dampRatioF   = ComputeRatio(dampF, springF);
        float springFreqR  = ComputeFreq(springR);
        float dampRatioR   = ComputeRatio(dampR, springR);

        Console.Error.WriteLine($"[VehiclePhysicsBuilder] quarterMass={quarterMass:F0}kg " +
                          $"F: spring={springF:F0}N/m freq={springFreqF:F2}Hz ratio={dampRatioF:F2} " +
                          $"R: spring={springR:F0}N/m freq={springFreqR:F2}Hz ratio={dampRatioR:F2}");
        Console.Error.WriteLine($"[VehiclePhysicsBuilder] Wheel centres from flexbodies: {string.Join(", ", wheelCentresFromFlexbodies.Keys)}");

        // Wheel radius from vars (default 0.305m ≈ 195/65 R15)
        float wheelRadius = GetVar("tireWidth", 0f);  // not typically a var; use fixed default
        wheelRadius = 0.305f;

        Entity GetWheel(string label, bool isFront)
        {
            Vector3 pos;
            if (wheelCentresFromFlexbodies.TryGetValue(label, out var jbeamPos))
            {
                pos = BeamNGToStride(jbeamPos);
                Console.Error.WriteLine($"[VehiclePhysicsBuilder] {label} centre (flexbody)={pos:F3}");
            }
            else if (wheelGroupNodes.TryGetValue(label, out var nodes) && nodes.Count > 0)
            {
                pos = ComputeWheelCenter(nodes);
                Console.Error.WriteLine($"[VehiclePhysicsBuilder] {label} centre (nodes)={pos:F3}");
            }
            else
            {
                Console.Error.WriteLine($"[VehiclePhysicsBuilder] {label}: no position data — using fallback");
                pos = Vector3.Zero;
            }
            float freq  = isFront ? springFreqF  : springFreqR;
            float ratio = isFront ? dampRatioF   : dampRatioR;
            return BuildWheelEntity(label, pos, chassisBody, chassisWorldPos, isFront, freq, ratio, wheelRadius);
        }

        return (GetWheel("wheel_FL", true), GetWheel("wheel_FR", true),
                GetWheel("wheel_RL", false), GetWheel("wheel_RR", false));
    }

    /// <summary>
    /// Computes wheel centre from jbeam hub nodes (fallback when no flexbody pos is available).
    /// Nodes with Z &lt; 0.10 must already be filtered out before calling this.
    /// Uses the most laterally-outer node for X, the mean of all filtered nodes for Y,
    /// and the minimum node Z (hub lower ball joint) + a fixed spindle offset for Z.
    /// </summary>
    private static Vector3 ComputeWheelCenter(List<AssembledNode> nodes)
    {
        const float WheelRadius    = 0.305f;
        const float SpindleOffset  = 0.08f;  // hub lower ball joint → wheel centre height

        var positions = nodes.Select(n => n.Position).ToList();

        float meanY  = positions.Average(p => p.Y);  // BeamNG Y = longitudinal
        float minZ   = positions.Min(p => p.Z);       // lowest valid node ≈ hub lower ball joint
        float centreZ = Math.Max(minZ + SpindleOffset, WheelRadius);

        // The outermost lateral node gives the wheel contact-patch lateral position.
        float outerX = positions.OrderByDescending(p => Math.Abs(p.X)).First().X;

        return new Vector3(outerX, centreZ, -meanY);  // BeamNG → Stride
    }

    private static Entity BuildWheelEntity(
        string name,
        Vector3 position,
        BodyComponent chassisBody,
        Vector3 chassisWorldPos,
        bool isFront,
        float springFreq = 2.5f,
        float dampingRatio = 0.9f,
        float wheelRadius = 0.305f)
    {
        const float WheelWidth  = 0.175f;
        const float WheelMass   = 18f;

        var entity = new Entity(name) { Transform = { Position = position } };
        var body = new BodyComponent
        {
            FrictionCoefficient = 1.5f,
            Collider = new CompoundCollider
            {
                Colliders =
                {
                    new CylinderCollider
                    {
                        Radius = wheelRadius,
                        Length = WheelWidth,
                        Mass   = WheelMass,
                        // Default cylinder axis is Y; rotate 90° around Z to orient axis along X (wheel rolling axis)
                        RotationLocal = Quaternion.RotationZ(MathF.PI / 2),
                    }
                }
            },
            Gravity = true,
        };
        entity.Add(body);

        var localOffsetA = position - chassisWorldPos;

        // ── Suspension ────────────────────────────────────────────────────────
        // PointOnLine constrains the wheel centre to a vertical rail fixed in chassis space.
        entity.Add(new PointOnLineServoConstraintComponent
        {
            A = chassisBody, B = body,
            LocalOffsetA  = localOffsetA,
            LocalOffsetB  = Vector3.Zero,
            LocalDirection = Vector3.UnitY,
            ServoMaximumForce = 80000f,
        });

        // LinearAxisServo provides the spring/damper along that rail.
        entity.Add(new LinearAxisServoConstraintComponent
        {
            A = chassisBody, B = body,
            LocalOffsetA     = localOffsetA,
            LocalOffsetB     = Vector3.Zero,
            LocalPlaneNormal = Vector3.UnitY,
            TargetOffset     = 0f,
            SpringFrequency    = springFreq,
            SpringDampingRatio = dampingRatio,
            ServoMaximumForce  = 80000f,
        });

        // ── Angular constraints ───────────────────────────────────────────────
        if (isFront)
        {
            // Front: allow steering (swivel around chassis Y) AND spin (hinge around wheel X).
            entity.Add(new AngularSwivelHingeConstraintComponent
            {
                A = chassisBody, B = body,
                LocalSwivelAxisA = Vector3.UnitY,
                LocalHingeAxisB  = Vector3.UnitX,
                SpringFrequency    = 30f,
                SpringDampingRatio = 2f,
            });
        }
        else
        {
            // Rear: spin only; hinge axes aligned → no steer, no wobble.
            entity.Add(new AngularHingeConstraintComponent
            {
                A = chassisBody, B = body,
                LocalHingeAxisA = Vector3.UnitX,
                LocalHingeAxisB = Vector3.UnitX,
                SpringFrequency    = 30f,
                SpringDampingRatio = 2f,
            });
        }

        // ── Drive motor ────────────────────────────────────────────────────────
        // Positive TargetVelocity → wheel bottom moves in −Z → car pushed in +Z (forward).
        // MotorDamping acts as the proportional gain: motor_torque = Damping × velocityError,
        // clamped to MotorMaximumForce.  We set Damping high (500) so the torque cap
        // (set per-frame by RallyCarComponent based on the active gear ratio) is always
        // the binding constraint, not the proportional gain.
        var driveMotor = new AngularAxisMotorConstraintComponent
        {
            A = chassisBody, B = body,
            LocalAxisA        = Vector3.UnitX,
            TargetVelocity    = 0f,
            MotorMaximumForce = 8000f,
            MotorDamping      = 500f,
        };
        entity.Add(driveMotor);

        // ── Steer motor (front wheels only) ───────────────────────────────────
        AngularAxisMotorConstraintComponent? steerMotor = null;
        if (isFront)
        {
            steerMotor = new AngularAxisMotorConstraintComponent
            {
                A = chassisBody, B = body,
                LocalAxisA        = Vector3.UnitY,
                TargetVelocity    = 0f,
                MotorMaximumForce = 4000f,
                MotorDamping      = 80f,
            };
            entity.Add(steerMotor);
        }

        entity.Add(new WheelSettings { DriveMotor = driveMotor, SteerMotor = steerMotor });
        return entity;
    }

    private static Dictionary<string, List<AssembledNode>> EstimateWheelPositions(VehicleDefinition def)
    {
        if (def.Nodes.Count == 0)
            return new Dictionary<string, List<AssembledNode>>();

        var allPositions = def.Nodes.Values.Select(n => n.Position).ToList();
        float minX = allPositions.Min(p => p.X);
        float maxX = allPositions.Max(p => p.X);
        float minY = allPositions.Min(p => p.Y);  // jbeam Y = forward
        float maxY = allPositions.Max(p => p.Y);
        float groundZ = allPositions.Min(p => p.Z); // jbeam Z = up → wheel bottom

        float wheelZ = groundZ + 0.34f;
        float wheelY_front = minY;
        float wheelY_rear = maxY;

        return new Dictionary<string, List<AssembledNode>>
        {
            ["wheel_FL"] = new() { new AssembledNode("est_fl", new System.Numerics.Vector3(minX, wheelY_front, wheelZ), 20, new(), true) },
            ["wheel_FR"] = new() { new AssembledNode("est_fr", new System.Numerics.Vector3(maxX, wheelY_front, wheelZ), 20, new(), true) },
            ["wheel_RL"] = new() { new AssembledNode("est_rl", new System.Numerics.Vector3(minX, wheelY_rear, wheelZ), 20, new(), true) },
            ["wheel_RR"] = new() { new AssembledNode("est_rr", new System.Numerics.Vector3(maxX, wheelY_rear, wheelZ), 20, new(), true) },
        };
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Coordinate conversion  (BeamNG → Stride)
    // BeamNG: X=right, Y=forward, Z=up
    // Stride: X=right, Y=up, Z=backward
    // ──────────────────────────────────────────────────────────────────────────

    public static Vector3 BeamNGToStride(System.Numerics.Vector3 v)
        => new Vector3(v.X, v.Z, -v.Y);

    // ──────────────────────────────────────────────────────────────────────────
    // Math helpers
    // ──────────────────────────────────────────────────────────────────────────

    private static Vector3 ComputeCentroid(IEnumerable<System.Numerics.Vector3> beamngPositions)
    {
        var list = beamngPositions.ToList();
        if (list.Count == 0) return Vector3.Zero;
        var sum = System.Numerics.Vector3.Zero;
        foreach (var p in list) sum += p;
        sum /= list.Count;
        return BeamNGToStride(sum);
    }

    private static BoundingBox ComputeAABB(
        IEnumerable<System.Numerics.Vector3> beamngPositions)
    {
        var list = beamngPositions.Select(BeamNGToStride).ToList();
        if (list.Count == 0)
            return new BoundingBox(Vector3.Zero, Vector3.Zero);

        var min = list[0];
        var max = list[0];
        foreach (var p in list)
        {
            min = new Vector3(MathF.Min(min.X, p.X), MathF.Min(min.Y, p.Y), MathF.Min(min.Z, p.Z));
            max = new Vector3(MathF.Max(max.X, p.X), MathF.Max(max.Y, p.Y), MathF.Max(max.Z, p.Z));
        }
        return new BoundingBox(min, max);
    }
}
