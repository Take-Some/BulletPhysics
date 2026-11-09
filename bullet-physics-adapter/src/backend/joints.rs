use super::*;
use rsbullet_core::{ConstraintCreateOptions, ForceFrame, JointType};

#[derive(Clone, Debug)]
pub(super) struct JointRecord {
    native_id: i32,
    definition: PhysicsJointSnapshot,
    effective_angular_inertia: f64,
    parent_inverse_mass: f64,
    child_inverse_mass: f64,
    parent_inverse_inertia: [f64; 3],
    child_inverse_inertia: [f64; 3],
}

#[cfg(test)]
mod tests;

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
            .filter_map(|(id, record)| {
                (!desired
                    .get(id)
                    .is_some_and(|next| same_socket(*next, record.definition)))
                .then_some(*id)
            })
            .collect();
        let mut changed = !stale.is_empty();
        for id in stale {
            self.remove_joint(id)?;
        }
        for definition in requested {
            if let Some(record) = self.joints.get_mut(&definition.id) {
                // Drive targets change continuously; native anchors stay resident.
                record.definition = *definition;
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
                    parent_inverse_mass: if parent_dynamics.mass > 0.0 {
                        1.0 / parent_dynamics.mass
                    } else {
                        0.0
                    },
                    child_inverse_mass: if child_dynamics.mass > 0.0 {
                        1.0 / child_dynamics.mass
                    } else {
                        0.0
                    },
                    parent_inverse_inertia: parent_dynamics.local_inertia_diagonal.map(|v| {
                        if parent_dynamics.mass > 0.0 {
                            1.0 / v.max(1e-8)
                        } else {
                            0.0
                        }
                    }),
                    child_inverse_inertia: child_dynamics.local_inertia_diagonal.map(|v| {
                        if child_dynamics.mass > 0.0 {
                            1.0 / v.max(1e-8)
                        } else {
                            0.0
                        }
                    }),
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

    pub(super) fn apply_joint_angular_limits(
        &mut self,
        dt: f64,
        gravity: f64,
    ) -> Result<(), String> {
        use super::joint_drives::{DriveBody, DriveRow};
        let definitions: Vec<_> = self
            .joints
            .values()
            .map(|r| {
                (
                    r.definition,
                    r.effective_angular_inertia,
                    r.parent_inverse_inertia,
                    r.child_inverse_inertia,
                    r.parent_inverse_mass,
                    r.child_inverse_mass,
                )
            })
            .collect();
        let mut indices = BTreeMap::<u64, usize>::new();
        let mut native_ids = Vec::new();
        let mut bodies = Vec::<DriveBody>::new();
        let mut positions = Vec::new();
        let mut inverse_masses = Vec::new();
        for (j, _, parent, child, pa, ca) in &definitions {
            for (entity, inertia, mass) in [(j.parent, *parent, *pa), (j.child, *child, *ca)] {
                if indices.contains_key(&entity) {
                    continue;
                }
                let native = self.records[&entity].body_id;
                let (pose, v) = self.pre_step_base_state(native)?;
                let rotation = pose.rotation.to_rotation_matrix();
                let inverse = rotation.matrix()
                    * nalgebra::Matrix3::from_diagonal(&Vector3::from(inertia))
                    * rotation.matrix().transpose();
                indices.insert(entity, bodies.len());
                native_ids.push(native);
                positions.push(pose.translation.vector);
                inverse_masses.push(mass);
                bodies.push(DriveBody {
                    linear_velocity: Vector3::new(v[0], v[1], v[2]),
                    rotation: pose.rotation,
                    spin: Vector3::new(v[3], v[4], v[5]),
                    inverse_inertia: inverse,
                });
            }
        }
        let mut rows = Vec::with_capacity(definitions.len() * 2);
        let mut constraints = Vec::new();
        for (j, inertia, _, _, _, _) in definitions {
            let parent = indices[&j.parent];
            let child = indices[&j.child];
            let parent_com = self.records[&j.parent]
                .mass_properties
                .map_or([0.0; 3], |p| p.center_of_mass);
            let child_com = self.records[&j.child]
                .mass_properties
                .map_or([0.0; 3], |p| p.center_of_mass);
            let ra = bodies[parent].rotation
                * Vector3::from(std::array::from_fn::<_, 3, _>(|i| {
                    (j.parent_anchor[i] - parent_com[i]) as f64
                }));
            let rb = bodies[child].rotation
                * Vector3::from(std::array::from_fn::<_, 3, _>(|i| {
                    (j.child_anchor[i] - child_com[i]) as f64
                }));
            constraints.extend(super::joint_mobility::point_rows(
                Some(parent),
                Some(child),
                ra,
                rb,
            ));
            if inertia <= 0.0 {
                continue;
            }
            let frame_a =
                bodies[parent].rotation * physics_pose([0.0; 3], j.parent_frame_rotation).rotation;
            let frame_b =
                bodies[child].rotation * physics_pose([0.0; 3], j.child_frame_rotation).rotation;
            for (term_index, (error, stiffness, damping, max_torque)) in
                super::joint_drives::terms(j, frame_a.inverse() * frame_b)
                    .into_iter()
                    .enumerate()
            {
                let feed_forward = if term_index == 1 {
                    j.motor.map_or(Vector3::zeros(), |m| {
                        frame_a * Vector3::from(m.feed_forward_torque.map(|v| v as f64))
                    })
                } else {
                    Vector3::zeros()
                };
                if max_torque <= 0.0
                    || (stiffness == 0.0
                        && damping == 0.0
                        && feed_forward.norm_squared() == 0.0
                        && (term_index != 1 || j.motor.is_none_or(|m| m.world_drive.is_none())))
                {
                    continue;
                }
                let world_drive = if term_index == 1 {
                    j.motor.and_then(|m| m.world_drive).map(|g| {
                        let is_child = g.body == newviso_physics_api::PhysicsJointBody::Child;
                        let controlled = if is_child { child } else { parent };
                        super::joint_drives::WorldDrive {
                            child: is_child,
                            error: (bodies[controlled].rotation
                                * physics_pose([0.0; 3], g.target_rotation).rotation.inverse())
                            .scaled_axis()
                                * (if is_child { 1.0 } else { -1.0 }),
                            stiffness: g.stiffness as f64,
                            damping: g.damping as f64,
                        }
                    })
                } else {
                    None
                };
                rows.push(DriveRow {
                    world_drive,
                    parent,
                    child,
                    error: frame_a * error,
                    stiffness,
                    damping: damping.max(2.0 * (stiffness * inertia).sqrt()),
                    max_torque,
                    feed_forward,
                });
            }
        }
        // Loaded normal rows determine the supported inertia. Tangential
        // friction stays finite in Bullet; it is never a three-axis socket.
        for contact in self
            .client
            .get_contact_points(None, None, None, None)
            .map_err(bullet_error)?
        {
            if !contact.contact_distance.is_finite() {
                continue;
            }
            let a = self
                .body_to_entity
                .get(&contact.body_a)
                .and_then(|e| indices.get(e))
                .copied();
            let b = self
                .body_to_entity
                .get(&contact.body_b)
                .and_then(|e| indices.get(e))
                .copied();
            if a.is_none() && b.is_none() {
                continue;
            }
            let external_dynamic = |native, own: Option<usize>| {
                own.is_none()
                    && self
                        .body_to_entity
                        .get(&native)
                        .and_then(|e| self.records.get(e))
                        .is_some_and(|r| r.kind == PhysicsBodyKindDto::Dynamic)
            };
            if external_dynamic(contact.body_a, a) || external_dynamic(contact.body_b, b) {
                continue;
            }
            let ra = a.map_or(Vector3::zeros(), |i| {
                Vector3::from(contact.position_on_a) - positions[i]
            });
            let rb = b.map_or(Vector3::zeros(), |i| {
                Vector3::from(contact.position_on_b) - positions[i]
            });
            let material_friction = |native| {
                self.body_to_entity
                    .get(&native)
                    .and_then(|e| self.records.get(e))
                    .map_or(0.0, |r| r.material.friction as f64)
            };
            let mut row = super::joint_mobility::contact_row(
                a,
                b,
                ra,
                rb,
                Vector3::from(contact.contact_normal_on_b),
                (material_friction(contact.body_a) * material_friction(contact.body_b))
                    .clamp(0.0, 10.0),
            );
            // Zero-load contacts may become load bearing. A positive gap only
            // permits a reaction if this tick's predicted motion closes it.
            row.velocity_bias = contact.contact_distance.max(0.0) / dt;
            constraints.push(row);
        }
        let impulses = super::joint_mobility::solve_with_gravity(
            &bodies,
            &inverse_masses,
            &rows,
            &constraints,
            dt,
            gravity,
        );
        for (native, impulse) in native_ids.into_iter().zip(impulses) {
            if impulse.norm_squared() < 1e-20 {
                continue;
            }
            self.client
                .apply_external_torque(
                    native,
                    -1,
                    <[f64; 3]>::from(impulse / dt),
                    ForceFrame::World,
                )
                .map_err(bullet_error)?;
        }
        Ok(())
    }
}

fn same_socket(mut a: PhysicsJointSnapshot, b: PhysicsJointSnapshot) -> bool {
    a.cone_limit = b.cone_limit;
    a.angular_stiffness = b.angular_stiffness;
    a.angular_damping = b.angular_damping;
    a.angular_limits = b.angular_limits;
    a.motor = b.motor;
    a == b
}
