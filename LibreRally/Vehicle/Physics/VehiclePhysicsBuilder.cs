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
        // Detachable parts are currently not spawned as separate rigid bodies, so the single
        // chassis body must carry the mass of every attached node in the assembled vehicle.
        //
        // Keep mass derived from the full sprung node cloud, but build the *collision* proxy from
        // body-shell nodes only. Suspension/pickup/skid nodes that sit very low in the jbeam should
        // influence mass and COM, but should not make the rigid chassis hull drag on the ground.
        const float MinNodeZ = 0.10f;        // excludes unresolved steelwheel origin nodes
        const float CollisionMinZ = 0.35f;   // floor pan / sill region, safely above the wheels' contact patch
        const float BodySplitZ = 0.55f;      // split lower body / upper cabin

        var chassisNodes = chassisPart.ExclusiveNodeIds
            .Where(id => def.Nodes.ContainsKey(id))
            .Select(id => def.Nodes[id])
            .ToList();

        var rigidNodes = chassisNodes
            .Where(n => n.Position.Z >= MinNodeZ)
            .Where(n => !IsWheelLikeNode(n))
            .ToList();
        if (rigidNodes.Count < 5)
        {
            rigidNodes = def.Nodes.Values
                .Where(n => n.Position.Z >= MinNodeZ)
                .Where(n => !IsWheelLikeNode(n))
                .ToList();
        }

        var sprungNodes = def.Nodes.Values
            .Where(n => n.Position.Z >= MinNodeZ)
            .Where(n => !IsWheelLikeNode(n))
            .ToList();
        if (sprungNodes.Count < 5)
            sprungNodes = rigidNodes;

        var upperMassNodes = sprungNodes.Where(n => n.Position.Z >= BodySplitZ).ToList();
        var lowerMassNodes = sprungNodes.Where(n => n.Position.Z < BodySplitZ).ToList();

        var collisionNodes = sprungNodes.Where(n => n.Position.Z >= CollisionMinZ).ToList();
        if (collisionNodes.Count < 5)
            collisionNodes = sprungNodes;

        var upperCollisionNodes = collisionNodes.Where(n => n.Position.Z >= BodySplitZ).ToList();
        var lowerCollisionNodes = collisionNodes.Where(n => n.Position.Z < BodySplitZ).ToList();

        float totalMass = sprungNodes.Sum(n => n.Weight);
        float upperMass = upperMassNodes.Sum(n => n.Weight);
        float lowerMass = lowerMassNodes.Sum(n => n.Weight);
        if (upperCollisionNodes.Count < 5)
        {
            upperCollisionNodes = collisionNodes;
            upperMass = totalMass;
            lowerCollisionNodes = new List<AssembledNode>();
            lowerMass = 0f;
        }
        else if (lowerCollisionNodes.Count < 5)
        {
            upperMass = totalMass;
            lowerMass = 0f;
        }

        var boxSpecs = new List<(Vector3 Center, Vector3 Size, float Mass)>();

        void AddBoxSpec(List<AssembledNode> nodes, float mass, bool centralFloorProxy = false)
        {
            if (nodes.Count == 0 || mass <= 0f)
                return;

            BoundingBox aabb = ComputeAABB(nodes.Select(n => n.Position));
            Vector3 extents = aabb.Maximum - aabb.Minimum;
            Vector3 center = (aabb.Minimum + aabb.Maximum) * 0.5f;
            if (centralFloorProxy)
            {
                // Keep the low mass in the middle of the car instead of spanning into the
                // wheel wells. This approximates the engine / tunnel / floor pan without
                // letting the rigid lower hull occupy the tyres' sweep volume.
                extents.X *= 0.65f;
                extents.Z *= 0.50f;
            }
            boxSpecs.Add((
                center,
                new Vector3(
                    Math.Max(extents.X, 0.3f),
                    Math.Max(extents.Y, 0.12f),
                    Math.Max(extents.Z, 0.3f)),
                mass));
        }

        if (lowerMass > 1f)
            AddBoxSpec(lowerCollisionNodes, lowerMass, centralFloorProxy: true);
        AddBoxSpec(upperCollisionNodes, Math.Max(upperMass, 1f));
        if (boxSpecs.Count == 0)
            AddBoxSpec(collisionNodes, Math.Max(totalMass, 1f));

        Vector3 centerOfMass = Vector3.Zero;
        float boxMassSum = 0f;
        foreach (var spec in boxSpecs)
        {
            centerOfMass += spec.Center * spec.Mass;
            boxMassSum += spec.Mass;
        }
        centerOfMass /= Math.Max(boxMassSum, 1f);

        var entity = new Entity("chassis") { Transform = { Position = centerOfMass } };

        var collider = new CompoundCollider();
        foreach (var spec in boxSpecs)
        {
            collider.Colliders.Add(new BoxCollider
            {
                Size = spec.Size,
                PositionLocal = spec.Center - centerOfMass,
                Mass = spec.Mass,
            });
        }

        var body = new BodyComponent
        {
            Collider = collider,
            Gravity = true,
        };

        var nodeCom = ComputeWeightedCentroid(sprungNodes);
        var upperAabb = upperCollisionNodes.Count > 0 ? ComputeAABB(upperCollisionNodes.Select(n => n.Position)) : new BoundingBox(Vector3.Zero, Vector3.Zero);
        var lowerAabb = lowerCollisionNodes.Count > 0 ? ComputeAABB(lowerCollisionNodes.Select(n => n.Position)) : new BoundingBox(Vector3.Zero, Vector3.Zero);
        Console.Error.WriteLine($"[VehiclePhysicsBuilder] Chassis chassisNodes={chassisNodes.Count} rigid={rigidNodes.Count} sprung={sprungNodes.Count} " +
                          $"upper={upperCollisionNodes.Count} lower={lowerCollisionNodes.Count} nodeCom={nodeCom:F3} boxCom={centerOfMass:F3} " +
                          $"upperExtents={(upperAabb.Maximum - upperAabb.Minimum):F3} " +
                          $"lowerExtents={(lowerAabb.Maximum - lowerAabb.Minimum):F3} mass={totalMass:F0}kg");
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
        float sprungMass = (float)def.Nodes.Values
            .Where(n => n.Position.Z >= 0.10f)
            .Where(n => !IsWheelLikeNode(n))
            .Sum(n => n.Weight);
        if (sprungMass <= 0f)
            sprungMass = (float)def.Nodes.Values.Sum(n => n.Weight);

        float quarterMass = sprungMass / 4.0f;
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

        Console.Error.WriteLine($"[VehiclePhysicsBuilder] sprungMass={sprungMass:F0}kg quarterMass={quarterMass:F0}kg " +
                          $"F: spring={springF:F0}N/m freq={springFreqF:F2}Hz ratio={dampRatioF:F2} " +
                          $"R: spring={springR:F0}N/m freq={springFreqR:F2}Hz ratio={dampRatioR:F2}");
        Console.Error.WriteLine($"[VehiclePhysicsBuilder] Wheel centres from flexbodies: {string.Join(", ", wheelCentresFromFlexbodies.Keys)}");

        // Wheel radius from vars (default 0.305m ≈ 195/65 R15)
        float wheelRadius = GetVar("tireWidth", 0f);  // not typically a var; use fixed default
        wheelRadius = 0.305f;

        float staticNormalLoad = Math.Max(quarterMass * 9.81f, 0f);
        float ComputeStaticSag(float springRate) => staticNormalLoad / Math.Max(springRate, 1f);
        float ComputeBumpTravel(float springRate) => Math.Clamp(ComputeStaticSag(springRate) * 6f, 0.08f, 0.18f);
        float ComputeReboundTravel(float springRate) => Math.Clamp(ComputeStaticSag(springRate) * 4f, 0.06f, 0.16f);

        float bumpTravelF = ComputeBumpTravel(springF);
        float reboundTravelF = ComputeReboundTravel(springF);
        float bumpTravelR = ComputeBumpTravel(springR);
        float reboundTravelR = ComputeReboundTravel(springR);

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
            float bumpTravel = isFront ? bumpTravelF : bumpTravelR;
            float reboundTravel = isFront ? reboundTravelF : reboundTravelR;
            return BuildWheelEntity(label, pos, chassisBody, chassisWorldPos, isFront, freq, ratio, wheelRadius, staticNormalLoad, bumpTravel, reboundTravel);
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
        float wheelRadius = 0.305f,
        float staticNormalLoad = 0f,
        float bumpTravel = 0.1f,
        float reboundTravel = 0.08f)
    {
        const float WheelWidth  = 0.175f;
        const float WheelMass   = 18f;

        var entity = new Entity(name) { Transform = { Position = position } };
        var body = new BodyComponent
        {
            // The slip-based tyre model is authoritative for contact-patch grip. Keep the
            // passive wheel collider nearly frictionless so BEPU support contacts don't
            // double-count longitudinal/lateral grip and pin the car in place.
            FrictionCoefficient = 0.05f,
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
        var localOffsetB = Vector3.Zero;
        var suspensionAxis = Vector3.UnitY;
        const float TargetOffset = 0f;
        float minimumOffset = -Math.Max(reboundTravel, 0f);
        float maximumOffset = Math.Max(bumpTravel, 0f);

        // ── Suspension ────────────────────────────────────────────────────────
        // PointOnLine constrains the wheel centre to a vertical rail fixed in chassis space.
        entity.Add(new PointOnLineServoConstraintComponent
        {
            A = chassisBody, B = body,
            LocalOffsetA  = localOffsetA,
            LocalOffsetB  = localOffsetB,
            LocalDirection = suspensionAxis,
            ServoMaximumForce = 80000f,
        });

        // LinearAxisServo provides the spring/damper along that rail.
        entity.Add(new LinearAxisServoConstraintComponent
        {
            A = chassisBody, B = body,
            LocalOffsetA     = localOffsetA,
            LocalOffsetB     = localOffsetB,
            LocalPlaneNormal = suspensionAxis,
            TargetOffset     = TargetOffset,
            SpringFrequency    = springFreq,
            SpringDampingRatio = dampingRatio,
            ServoMaximumForce  = 80000f,
        });

        entity.Add(new LinearAxisLimitConstraintComponent
        {
            A = chassisBody, B = body,
            LocalOffsetA = localOffsetA,
            LocalOffsetB = localOffsetB,
            LocalAxis = suspensionAxis,
            MinimumOffset = minimumOffset,
            MaximumOffset = maximumOffset,
            SpringFrequency = Math.Clamp(springFreq * 4f, 10f, 20f),
            SpringDampingRatio = 1.1f,
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
        // Use a full angular-velocity motor instead of a single chassis-fixed X-axis motor.
        // Front wheels steer around Y, so a chassis-fixed drive axis injects self-aligning
        // torque under throttle once the wheel axle is no longer parallel to chassis X.
        // RallyCarComponent updates the target vector every frame from the wheel's actual axle
        // direction, which keeps drive torque aligned with the current hinge axis.
        var driveMotor = new AngularMotorConstraintComponent
        {
            A = chassisBody, B = body,
            TargetVelocityLocalA = Vector3.Zero,
            MotorMaximumForce    = 8000f,
            MotorDamping         = 500f,
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
                // Steering is a position servo in RallyCarComponent, so this motor needs
                // enough authority to hold lock against tyre-aligning load at speed.
                MotorMaximumForce = 12000f,
                MotorDamping      = 4000f,
            };
            entity.Add(steerMotor);
        }

        entity.Add(new WheelSettings
        {
            DriveMotor = driveMotor,
            SteerMotor = steerMotor,
            StaticNormalLoad = staticNormalLoad,
            SuspensionLocalOffsetA = localOffsetA,
            SuspensionLocalOffsetB = localOffsetB,
            SuspensionLocalAxis = suspensionAxis,
            SuspensionTargetOffset = TargetOffset,
            SuspensionMinimumOffset = minimumOffset,
            SuspensionMaximumOffset = maximumOffset,
            TyreModel = new TyreModel(wheelRadius),
        });
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

    private static bool IsWheelLikeNode(AssembledNode node)
    {
        foreach (string group in node.Groups)
        {
            if (group.StartsWith("wheel", StringComparison.OrdinalIgnoreCase) ||
                group.Contains("wheelhub", StringComparison.OrdinalIgnoreCase) ||
                group.StartsWith("hub_", StringComparison.OrdinalIgnoreCase) ||
                group.Contains("_hub_", StringComparison.OrdinalIgnoreCase))
                return true;
        }

        return false;
    }

    // ──────────────────────────────────────────────────────────────────────────
    // Math helpers
    // ──────────────────────────────────────────────────────────────────────────

    private static Vector3 ComputeWeightedCentroid(IEnumerable<AssembledNode> nodes)
    {
        var list = nodes.ToList();
        if (list.Count == 0)
            return Vector3.Zero;

        System.Numerics.Vector3 weightedSum = System.Numerics.Vector3.Zero;
        float totalWeight = 0f;
        foreach (var node in list)
        {
            float weight = MathF.Max(node.Weight, 0f);
            weightedSum += node.Position * weight;
            totalWeight += weight;
        }

        if (totalWeight <= 0f)
            return ComputeCentroid(list.Select(n => n.Position));

        return BeamNGToStride(weightedSum / totalWeight);
    }

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
