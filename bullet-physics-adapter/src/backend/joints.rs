use super::*;
use rsbullet_core::{ConstraintCreateOptions, ForceFrame, JointType};

#[derive(Clone, Debug)]
pub(super) struct JointRecord {
    native_id: i32,
    definition: PhysicsJointSnapshot,
    effective_angular_inertia: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    fn body(id: u64, position: [f32; 3], kind: PhysicsBodyKind) -> PhysicsBodySnapshot {
        PhysicsBodySnapshot {
            entity: id,
            kind,
            shape: CollisionShape::Sphere { radius: 0.15 },
            flags: PhysicsBodyFlags {
                casts_contacts: true,
                participates_in_queries: true,
                ..Default::default()
            },
            material: PhysicsMaterial {
                friction: 0.7,
                restitution: 0.0,
                density: 100.0,
            },
            position,
            rotation: [0.0, 0.0, 0.0, 1.0],
            linear_velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
            linear_damping: Some(0.02),
            angular_damping: Some(0.2),
            mass_properties: None,
            convex_hulls: vec![],
            bounds_min: [-0.15; 3],
            bounds_max: [0.15; 3],
        }
    }
    #[test]
    fn small_limb_cone_stop_dissipates_fast_spin_without_exploding() {
        let mut backend = BulletPacketPhysicsBackend::new(32, 32).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
        let mut limb = body(2, [0.0, 3.0, 0.0], PhysicsBodyKind::Dynamic);
        limb.shape = CollisionShape::Sphere { radius: 0.035 };
        limb.material.density = 650.0;
        limb.bounds_min = [-0.035; 3];
        limb.bounds_max = [0.035; 3];
        limb.rotation = [0.0, 0.0, 0.43496552, 0.90044713];
        input.bodies = vec![body(1, [0.0, 3.0, 0.0], PhysicsBodyKind::Static), limb];
        input.joints.push(PhysicsJointSnapshot {
            id: 9,
            parent: 1,
            child: 2,
            parent_anchor: [0.0; 3],
            child_anchor: [0.0; 3],
            parent_frame_rotation: [0.0, 0.0, 0.0, 1.0],
            child_frame_rotation: [0.0, 0.0, 0.0, 1.0],
            max_force: 30000.0,
            cone_limit: 0.2,
            angular_stiffness: 12.0,
            angular_damping: 0.04,
            collision_family: 7,
        });
        input.commands.push(PhysicsCommand {
            seq: 1,
            kind: PhysicsCommandKind::SetAngularVelocity {
                entity: 2,
                velocity: [0.0, 0.0, 80.0],
            },
        });
        let mut max_spin = 0.0f32;
        let mut angle = 10.0f32;
        for frame in 1..=180 {
            input.fixed_tick = frame;
            input.frame_index = frame;
            let output = backend.step_frame(input.clone()).unwrap();
            input.commands.clear();
            let pose = output.pose_updates.iter().find(|p| p.entity == 2).unwrap();
            max_spin = max_spin.max(
                output
                    .velocity_updates
                    .iter()
                    .find(|v| v.entity == 2)
                    .unwrap()
                    .angular_velocity
                    .iter()
                    .map(|v| v * v)
                    .sum::<f32>()
                    .sqrt(),
            );
            angle = 2.0 * pose.rotation[3].abs().clamp(0.0, 1.0).acos();
            assert!((pose.position[1] - 3.0).abs() < 0.03);
        }
        assert!(max_spin < 85.0, "unstable small-limb torque: {max_spin}");
        assert!(angle < 0.25, "cone failed to settle: {angle}");
    }
    #[test]
    fn native_socket_holds_under_gravity_and_is_removed_with_its_body() {
        let mut backend = BulletPacketPhysicsBackend::new(32, 32).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
        input.bodies = vec![
            body(1, [0.0, 3.0, 0.0], PhysicsBodyKind::Static),
            body(2, [0.0, 2.0, 0.0], PhysicsBodyKind::Dynamic),
        ];
        input.joints.push(PhysicsJointSnapshot {
            id: 9,
            parent: 1,
            child: 2,
            parent_anchor: [0.0, -0.5, 0.0],
            child_anchor: [0.0, 0.5, 0.0],
            parent_frame_rotation: [0.0, 0.0, 0.0, 1.0],
            child_frame_rotation: [0.0, 0.0, 0.0, 1.0],
            max_force: 30000.0,
            cone_limit: 1.5,
            angular_stiffness: 8.0,
            angular_damping: 0.03,
            collision_family: 7,
        });
        input.commands.push(PhysicsCommand {
            seq: 1,
            kind: PhysicsCommandKind::SetLinearVelocity {
                entity: 2,
                velocity: [2.0, 0.0, 0.0],
            },
        });
        let mut maximum_error = 0.0f32;
        for frame in 1..=240 {
            input.frame_index = frame;
            input.fixed_tick = frame;
            let output = backend.step_frame(input.clone()).unwrap();
            input.commands.clear();
            let pose = output.pose_updates.iter().find(|p| p.entity == 2).unwrap();
            let pivot =
                physics_pose(pose.position, pose.rotation) * nalgebra::Point3::new(0.0, 0.5, 0.0);
            maximum_error =
                maximum_error.max((pivot - nalgebra::Point3::new(0.0, 2.5, 0.0)).norm() as f32);
        }
        assert!(maximum_error < 0.03, "native joint drift={maximum_error}");
        assert_eq!(backend.joints.len(), 1);
        input.joints.clear();
        input.bodies.retain(|b| b.entity == 2);
        let output = backend.step_frame(input).unwrap();
        assert!(backend.joints.is_empty());
        assert!(backend.joint_filtered_pairs.is_empty());
        assert!(output
            .events
            .iter()
            .any(|e| matches!(e, PhysicsEvent::BodyDestroyed { entity: 1 })));
    }
}

impl BulletPacketPhysicsBackend {
    pub(super) fn configure_joint_solver(
        &mut self,
        active: bool,
        continuous: bool,
    ) -> Result<(), String> {
        if self.solver_quality == Some((active, continuous)) {
            return Ok(());
        }
        self.client
            .set_physics_engine_parameter(&rsbullet_core::PhysicsEngineParametersUpdate {
                // Limb/chassis mass ratios need extra native constraint convergence.
                num_solver_iterations: Some(if active { 120 } else { 50 }),
                // Extra discrete samples keep thin fragments from crossing a
                // support slab in one step. Compound CCD is not advertised.
                num_sub_steps: Some(if active || continuous { 4 } else { 0 }),
                ..Default::default()
            })
            .map_err(bullet_error)?;
        self.solver_quality = Some((active, continuous));
        Ok(())
    }

    pub(super) fn sync_joints(&mut self, requested: &[PhysicsJointSnapshot]) -> Result<(), String> {
        let desired: BTreeMap<_, _> = requested.iter().map(|j| (j.id, *j)).collect();
        let stale: Vec<_> = self
            .joints
            .iter()
            .filter_map(|(id, record)| (desired.get(id) != Some(&record.definition)).then_some(*id))
            .collect();
        let mut changed = !stale.is_empty();
        for id in stale {
            self.remove_joint(id)?;
        }
        for definition in requested {
            if self.joints.contains_key(&definition.id) {
                continue;
            }
            let parent = self
                .records
                .get(&definition.parent)
                .ok_or_else(|| format!("missing joint parent {}", definition.parent))?;
            let child = self
                .records
                .get(&definition.child)
                .ok_or_else(|| format!("missing joint child {}", definition.child))?;
            // Native Bullet owns positional constraint solving at every fixed step.
            let native_id = self
                .client
                .create_constraint(&ConstraintCreateOptions {
                    parent_body: parent.body_id,
                    child_body: child.body_id,
                    joint_type: JointType::Point2Point,
                    parent_frame: physics_pose(
                        std::array::from_fn(|i| {
                            definition.parent_anchor[i]
                                - parent.mass_properties.map_or(0.0, |p| p.center_of_mass[i])
                        }),
                        definition.parent_frame_rotation,
                    ),
                    child_frame: physics_pose(
                        std::array::from_fn(|i| {
                            definition.child_anchor[i]
                                - child.mass_properties.map_or(0.0, |p| p.center_of_mass[i])
                        }),
                        definition.child_frame_rotation,
                    ),
                    max_applied_force: definition.max_force as f64,
                    erp: Some(0.65),
                    ..Default::default()
                })
                .map_err(bullet_error)?;
            let parent_dynamics = self
                .client
                .get_dynamics_info(parent.body_id, -1)
                .map_err(bullet_error)?;
            let child_dynamics = self
                .client
                .get_dynamics_info(child.body_id, -1)
                .map_err(bullet_error)?;
            let inverse_inertia = |mass: f64, diagonal: [f64; 3]| {
                if mass <= 0.0 {
                    0.0
                } else {
                    1.0 / diagonal.into_iter().fold(f64::INFINITY, f64::min).max(1e-8)
                }
            };
            let sum = inverse_inertia(parent_dynamics.mass, parent_dynamics.local_inertia_diagonal)
                + inverse_inertia(child_dynamics.mass, child_dynamics.local_inertia_diagonal);
            let effective_angular_inertia = if sum > 0.0 { 1.0 / sum } else { 0.0 };
            self.joints.insert(
                definition.id,
                JointRecord {
                    native_id,
                    definition: *definition,
                    effective_angular_inertia,
                },
            );
            changed = true;
        }
        if changed {
            let mut families = BTreeMap::<u64, BTreeSet<u64>>::new();
            for joint in requested {
                if joint.collision_family == 0 {
                    continue;
                }
                families
                    .entry(joint.collision_family)
                    .or_default()
                    .extend([joint.parent, joint.child]);
            }
            let mut desired_pairs = BTreeSet::new();
            for family in families.values() {
                let members: Vec<_> = family.iter().copied().collect();
                for (i, a) in members.iter().enumerate() {
                    for b in &members[i + 1..] {
                        desired_pairs.insert((*a, *b));
                    }
                }
            }
            for (a, b) in self.joint_filtered_pairs.difference(&desired_pairs) {
                if self.explicit_filtered_pairs.contains(&(*a, *b)) {
                    continue;
                }
                if let (Some(a), Some(b)) = (self.records.get(a), self.records.get(b)) {
                    self.client
                        .set_collision_filter_pair(a.body_id, b.body_id, -1, -1, true)
                        .map_err(bullet_error)?;
                    self.metrics.filter_pair_updates += 1;
                }
            }
            for (a, b) in desired_pairs.difference(&self.joint_filtered_pairs) {
                self.client
                    .set_collision_filter_pair(
                        self.records[a].body_id,
                        self.records[b].body_id,
                        -1,
                        -1,
                        false,
                    )
                    .map_err(bullet_error)?;
                self.metrics.filter_pair_updates += 1;
            }
            self.joint_filtered_pairs = desired_pairs;
        }
        Ok(())
    }

    fn remove_joint(&mut self, id: u64) -> Result<(), String> {
        if let Some(joint) = self.joints.remove(&id) {
            self.client
                .remove_constraint(joint.native_id)
                .map_err(bullet_error)?;
        }
        Ok(())
    }

    pub(super) fn remove_body_joints(&mut self, entity: u64) -> Result<(), String> {
        let ids: Vec<_> = self
            .joints
            .iter()
            .filter_map(|(id, j)| {
                (j.definition.parent == entity || j.definition.child == entity).then_some(*id)
            })
            .collect();
        for id in ids {
            self.remove_joint(id)?;
        }
        let pairs: Vec<_> = self
            .joint_filtered_pairs
            .iter()
            .copied()
            .filter(|(a, b)| *a == entity || *b == entity)
            .collect();
        for (a, b) in pairs {
            if let (Some(a), Some(b)) = (self.records.get(&a), self.records.get(&b)) {
                self.client
                    .set_collision_filter_pair(a.body_id, b.body_id, -1, -1, true)
                    .map_err(bullet_error)?;
                self.metrics.filter_pair_updates += 1;
            }
            self.joint_filtered_pairs.remove(&(a, b));
        }
        Ok(())
    }

    pub(super) fn apply_joint_angular_limits(&mut self, dt: f64) -> Result<(), String> {
        // The C API exposes native point constraints. A bounded equal/opposite
        // torque supplies a compliant cone stop and relative angular damping.
        // No pose reset or animation drives a simulated limb.
        let definitions: Vec<_> = self
            .joints
            .values()
            .map(|record| (record.definition, record.effective_angular_inertia))
            .collect();
        for (j, inertia) in definitions {
            let parent = self.records[&j.parent].body_id;
            let child = self.records[&j.child].body_id;
            let (a, av) = self.pre_step_base_state(parent)?;
            let (b, bv) = self.pre_step_base_state(child)?;
            let frame_a = a.rotation * physics_pose([0.0; 3], j.parent_frame_rotation).rotation;
            let frame_b = b.rotation * physics_pose([0.0; 3], j.child_frame_rotation).rotation;
            let relative = frame_a.inverse() * frame_b;
            let displacement = relative.scaled_axis();
            let angle = displacement.norm();
            let excess = if angle > j.cone_limit as f64 {
                displacement * ((angle - j.cone_limit as f64) / angle)
            } else {
                Vector3::zeros()
            };
            let relative_spin = Vector3::new(bv[3] - av[3], bv[4] - av[4], bv[5] - av[5]);
            if inertia <= 0.0 {
                continue;
            }
            // Implicit spring/damping prevents a small hand's inertia from
            // turning a cone-stop torque into hundreds of radians per second.
            let stiffness = if excess.norm_squared() > 1e-12 {
                j.angular_stiffness as f64
            } else {
                0.0
            };
            let damping = (j.angular_damping as f64).max(2.0 * (stiffness * inertia).sqrt());
            let implicit = 1.0 + (damping * dt + stiffness * dt * dt) / inertia;
            let mut torque = (-(frame_a * excess) * stiffness
                - relative_spin * (damping + stiffness * dt))
                / implicit;
            let length = torque.norm();
            if length > 40.0 {
                torque *= 40.0 / length;
            }
            if torque.norm_squared() < 1e-10 {
                continue;
            }
            self.client
                .apply_external_torque(child, -1, <[f64; 3]>::from(torque), ForceFrame::World)
                .map_err(bullet_error)?;
            self.client
                .apply_external_torque(parent, -1, <[f64; 3]>::from(-torque), ForceFrame::World)
                .map_err(bullet_error)?;
        }
        Ok(())
    }
}
