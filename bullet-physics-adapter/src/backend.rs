use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::f64::consts::PI;
use std::time::Duration;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use newengine_physics_api::*;
use rsbullet_core::{
    CollisionGeometry, CollisionId, CollisionShapeOptions, DynamicsUpdate, Mode,
    MultiBodyCreateOptions, PhysicsClient,
};

#[derive(Debug, Clone, PartialEq)]
enum ShapeSource {
    Primitive(CollisionShapeDto),
    Authored(PhysicsColliderDto),
}

#[derive(Debug, Clone)]
struct BodyRecord {
    body_id: i32,
    collision_shape_id: i32,
    kind: PhysicsBodyKindDto,
    shape: ShapeSource,
    persistent: bool,
    produces_output: bool,
    casts_contacts: bool,
    material: PhysicsMaterialDto,
    linear_damping: Option<f32>,
    angular_damping: Option<f32>,
}

#[derive(Debug, Clone, Copy)]
struct ContactSample {
    point: PhysicsVec3,
    normal: PhysicsVec3,
    impulse: f32,
}

pub struct BulletPacketPhysicsBackend {
    client: PhysicsClient,
    records: BTreeMap<PhysicsEntityKey, BodyRecord>,
    body_to_entity: HashMap<i32, PhysicsEntityKey>,
    active_contacts: BTreeSet<(PhysicsEntityKey, PhysicsEntityKey)>,
    max_bodies: u32,
    max_queries_per_frame: u32,
    last_dt: Option<f32>,
    last_gravity: Option<f32>,
}

impl BulletPacketPhysicsBackend {
    pub fn new(max_bodies: u32, max_queries_per_frame: u32) -> Result<Self, String> {
        let client = PhysicsClient::connect(Mode::Direct).map_err(bullet_error)?;
        Ok(Self {
            client,
            records: BTreeMap::new(),
            body_to_entity: HashMap::new(),
            active_contacts: BTreeSet::new(),
            max_bodies: max_bodies.max(1),
            max_queries_per_frame: max_queries_per_frame.max(1),
            last_dt: None,
            last_gravity: None,
        })
    }

    pub fn step_frame(&mut self, input: PhysicsFrameInput) -> Result<PhysicsFrameOutput, String> {
        self.configure_world(input.dt, input.gravity)?;

        let mut output = PhysicsFrameOutput {
            fixed_tick: input.fixed_tick,
            ..PhysicsFrameOutput::default()
        };

        for collider in &input.colliders {
            if self.sync_authored_collider(collider)? {
                output.events.push(PhysicsEventDto::BodyCreated {
                    entity: collider.entity,
                });
            }
        }

        let desired_entities = input
            .bodies
            .iter()
            .map(|body| body.entity)
            .collect::<HashSet<_>>();
        for body in &input.bodies {
            if self.sync_frame_body(body)? {
                output.events.push(PhysicsEventDto::BodyCreated {
                    entity: body.entity,
                });
            }
        }

        let stale_entities = self
            .records
            .iter()
            .filter_map(|(&entity, record)| {
                (!record.persistent && !desired_entities.contains(&entity)).then_some(entity)
            })
            .collect::<Vec<_>>();
        for entity in stale_entities {
            self.destroy_body(entity)?;
            output
                .events
                .push(PhysicsEventDto::BodyDestroyed { entity });
        }

        output.report.commands_applied =
            self.apply_commands(&input.commands, &mut output.events)?;

        self.client.step_simulation().map_err(bullet_error)?;
        self.collect_body_outputs(&mut output)?;
        self.collect_contact_events(input.dt, &mut output)?;
        output.query_hits = self.execute_queries(&input)?;

        let dynamic_bodies = self
            .records
            .values()
            .filter(|record| record.kind == PhysicsBodyKindDto::Dynamic)
            .count();
        output.report = PhysicsStepReportDto {
            fixed_tick: input.fixed_tick,
            dt: input.dt,
            substeps: u32::from(input.dt.is_finite() && input.dt > 0.0),
            active_bodies: self.records.len(),
            static_bodies: self.records.len().saturating_sub(dynamic_bodies),
            dynamic_bodies,
            contacts: self.active_contacts.len(),
            commands_applied: output.report.commands_applied,
        };
        Ok(output)
    }

    pub fn shutdown(&mut self) {
        let _ = self.client.reset_simulation();
        self.records.clear();
        self.body_to_entity.clear();
        self.active_contacts.clear();
        self.last_dt = None;
        self.last_gravity = None;
    }

    fn configure_world(&mut self, dt: f32, gravity: f32) -> Result<(), String> {
        let dt = if dt.is_finite() && dt > 0.0 {
            dt.clamp(1.0 / 1000.0, 0.25)
        } else {
            1.0 / 60.0
        };
        if self.last_dt != Some(dt) {
            self.client
                .set_time_step(Duration::from_secs_f64(dt as f64))
                .map_err(bullet_error)?;
            self.last_dt = Some(dt);
        }

        let gravity = if gravity.is_finite() {
            gravity.abs().clamp(0.0, 1000.0)
        } else {
            9.81
        };
        if self.last_gravity != Some(gravity) {
            self.client
                .set_gravity([0.0, -(gravity as f64), 0.0])
                .map_err(bullet_error)?;
            self.last_gravity = Some(gravity);
        }
        Ok(())
    }

    fn sync_frame_body(&mut self, snapshot: &PhysicsFrameBodySnapshot) -> Result<bool, String> {
        if snapshot.flags.is_trigger {
            return Err(format!(
                "Bullet provider does not advertise TriggerBodies; entity {} requested a trigger",
                snapshot.entity
            ));
        }

        let shape = ShapeSource::Primitive(snapshot.shape);
        let recreate = self.records.get(&snapshot.entity).is_some_and(|record| {
            record.persistent
                || record.kind != snapshot.kind
                || record.shape != shape
                || record.material != snapshot.material
                || record.linear_damping != snapshot.linear_damping
                || record.angular_damping != snapshot.angular_damping
        });
        if recreate {
            self.destroy_body(snapshot.entity)?;
        }

        if let Some(record) = self.records.get_mut(&snapshot.entity) {
            record.casts_contacts = snapshot.flags.casts_contacts;
            let body_id = record.body_id;
            if snapshot.kind != PhysicsBodyKindDto::Dynamic {
                self.client
                    .reset_base_position_and_orientation(
                        body_id,
                        vec3_f64(snapshot.position),
                        quat_f64(snapshot.rotation),
                    )
                    .map_err(bullet_error)?;
            }
            return Ok(false);
        }

        self.ensure_body_capacity()?;
        let collision_shape_id = self.create_collision_shape(&shape)?;
        let mass = body_mass(snapshot.kind, snapshot.shape, snapshot.material.density);
        let body_id = match self.create_body(
            collision_shape_id,
            mass,
            snapshot.position,
            snapshot.rotation,
        ) {
            Ok(body_id) => body_id,
            Err(error) => {
                let _ = self.client.remove_collision_shape(collision_shape_id);
                return Err(error);
            }
        };

        let cylinder_motion = matches!(snapshot.shape, CollisionShapeDto::Cylinder { .. });
        let dynamics = DynamicsUpdate {
            lateral_friction: Some(snapshot.material.friction.clamp(0.0, 10.0) as f64),
            rolling_friction: cylinder_motion.then_some(0.0025),
            spinning_friction: cylinder_motion.then_some(0.0015),
            linear_damping: snapshot.linear_damping
                .filter(|value| value.is_finite())
                .map(|value| value.clamp(0.0, 20.0) as f64)
                .or_else(|| cylinder_motion.then_some(0.015)),
            angular_damping: snapshot.angular_damping
                .filter(|value| value.is_finite())
                .map(|value| value.clamp(0.0, 20.0) as f64)
                .or_else(|| cylinder_motion.then_some(0.025)),
            ..DynamicsUpdate::default()
        };
        if let Err(error) = self.client.change_dynamics(body_id, -1, &dynamics) {
            let _ = self.client.remove_body(body_id);
            let _ = self.client.remove_collision_shape(collision_shape_id);
            return Err(bullet_error(error));
        }

        // Keep restitution in its own command. Bullet Direct has historically accepted the
        // mixed friction/damping update while silently retaining zero restitution on maximal-
        // coordinate bodies; a dedicated update makes the contact coefficient observable.
        let restitution = DynamicsUpdate {
            restitution: Some(snapshot.material.restitution.clamp(0.0, 1.0) as f64),
            ..DynamicsUpdate::default()
        };
        if let Err(error) = self.client.change_dynamics(body_id, -1, &restitution) {
            let _ = self.client.remove_body(body_id);
            let _ = self.client.remove_collision_shape(collision_shape_id);
            return Err(bullet_error(error));
        }

        if snapshot.kind == PhysicsBodyKindDto::Dynamic {
            if let Err(error) = self.client.reset_base_velocity(
                body_id,
                Some(vec3_f64(snapshot.linear_velocity)),
                Some(vec3_f64(snapshot.angular_velocity)),
            ) {
                let _ = self.client.remove_body(body_id);
                let _ = self.client.remove_collision_shape(collision_shape_id);
                return Err(bullet_error(error));
            }
        }

        self.body_to_entity.insert(body_id, snapshot.entity);
        self.records.insert(
            snapshot.entity,
            BodyRecord {
                body_id,
                collision_shape_id,
                kind: snapshot.kind,
                shape,
                persistent: false,
                produces_output: snapshot.kind != PhysicsBodyKindDto::Static,
                casts_contacts: snapshot.flags.casts_contacts,
                material: snapshot.material,
                linear_damping: snapshot.linear_damping,
                angular_damping: snapshot.angular_damping,
            },
        );
        Ok(true)
    }

    fn sync_authored_collider(
        &mut self,
        snapshot: &PhysicsFrameColliderSnapshot,
    ) -> Result<bool, String> {
        if snapshot.flags.is_trigger {
            return Err(format!(
                "Bullet provider does not advertise TriggerBodies; collider {} requested a trigger",
                snapshot.entity
            ));
        }

        let shape = ShapeSource::Authored(snapshot.collider.clone());
        let recreate = self.records.get(&snapshot.entity).is_some_and(|record| {
            !record.persistent || record.shape != shape || record.material != snapshot.material
        });
        if recreate {
            self.destroy_body(snapshot.entity)?;
        }
        if let Some(record) = self.records.get_mut(&snapshot.entity) {
            record.casts_contacts = snapshot.flags.casts_contacts;
            self.client
                .reset_base_position_and_orientation(
                    record.body_id,
                    vec3_f64(snapshot.position),
                    quat_f64(snapshot.rotation),
                )
                .map_err(bullet_error)?;
            return Ok(false);
        }

        self.ensure_body_capacity()?;
        let collision_shape_id = self.create_collision_shape(&shape)?;
        let body_id = match self.create_body(
            collision_shape_id,
            0.0,
            snapshot.position,
            snapshot.rotation,
        ) {
            Ok(body_id) => body_id,
            Err(error) => {
                let _ = self.client.remove_collision_shape(collision_shape_id);
                return Err(error);
            }
        };
        self.body_to_entity.insert(body_id, snapshot.entity);
        self.records.insert(
            snapshot.entity,
            BodyRecord {
                body_id,
                collision_shape_id,
                kind: PhysicsBodyKindDto::Static,
                shape,
                persistent: true,
                produces_output: false,
                casts_contacts: snapshot.flags.casts_contacts,
                material: snapshot.material,
                linear_damping: None,
                angular_damping: None,
            },
        );
        Ok(true)
    }

    fn create_body(
        &mut self,
        collision_shape_id: i32,
        mass: f64,
        position: PhysicsVec3,
        rotation: PhysicsQuat,
    ) -> Result<i32, String> {
        let mut options = MultiBodyCreateOptions::default();
        options.base.mass = mass;
        options.base.pose = physics_pose(position, rotation);
        options.base.collision_shape = CollisionId(collision_shape_id);
        options.use_maximal_coordinates = true;
        self.client
            .create_multi_body(&options)
            .map_err(bullet_error)
    }

    fn create_collision_shape(&mut self, source: &ShapeSource) -> Result<i32, String> {
        match source {
            ShapeSource::Primitive(CollisionShapeDto::Box { half_extents }) => {
                let half_extents = positive_vec3(*half_extents, "box half extents")?;
                let geometry = CollisionGeometry::Box {
                    half_extents: vec3_f64(half_extents),
                };
                self.client
                    .create_collision_shape(&geometry, None::<CollisionShapeOptions>)
                    .map_err(bullet_error)
            }
            ShapeSource::Primitive(CollisionShapeDto::Sphere { radius }) => {
                let radius = positive(*radius, "sphere radius")? as f64;
                let geometry = CollisionGeometry::Sphere { radius };
                self.client
                    .create_collision_shape(&geometry, None::<CollisionShapeOptions>)
                    .map_err(bullet_error)
            }
            ShapeSource::Primitive(CollisionShapeDto::Capsule {
                radius,
                half_height,
            }) => {
                let radius = positive(*radius, "capsule radius")? as f64;
                let height = positive(*half_height, "capsule half height")? as f64 * 2.0;
                let geometry = CollisionGeometry::Capsule { radius, height };
                self.client
                    .create_collision_shape(&geometry, None::<CollisionShapeOptions>)
                    .map_err(bullet_error)
            }
            ShapeSource::Primitive(CollisionShapeDto::Cylinder {
                radius,
                half_height,
            }) => {
                let radius = positive(*radius, "cylinder radius")? as f64;
                let height = positive(*half_height, "cylinder half height")? as f64 * 2.0;
                let geometry = CollisionGeometry::Cylinder { radius, height };
                let transform = Isometry3::from_parts(
                    Translation3::new(0.0, 0.0, 0.0),
                    UnitQuaternion::from_euler_angles(-std::f64::consts::FRAC_PI_2, 0.0, 0.0),
                );
                self.client
                    .create_collision_shape(&geometry, Some(CollisionShapeOptions::from(transform)))
                    .map_err(bullet_error)
            }
            ShapeSource::Authored(PhysicsColliderDto::Mesh(mesh)) => {
                self.create_mesh_collision_shape(mesh_vertices(mesh)?, mesh_indices(mesh)?)
            }
            ShapeSource::Authored(PhysicsColliderDto::Heightfield(heightfield)) => {
                let (vertices, indices) = heightfield_mesh(heightfield)?;
                self.create_mesh_collision_shape(vertices, indices)
            }
        }
    }

    fn create_mesh_collision_shape(
        &mut self,
        vertices: Vec<[f64; 3]>,
        indices: Vec<i32>,
    ) -> Result<i32, String> {
        let geometry = CollisionGeometry::Mesh {
            vertices: &vertices,
            indices: &indices,
            scale: [1.0, 1.0, 1.0],
        };
        self.client
            .create_collision_shape(&geometry, None::<CollisionShapeOptions>)
            .map_err(bullet_error)
    }

    fn ensure_body_capacity(&self) -> Result<(), String> {
        if self.records.len() >= self.max_bodies as usize {
            Err(format!(
                "Bullet provider body limit exceeded: {}",
                self.max_bodies
            ))
        } else {
            Ok(())
        }
    }

    fn apply_commands(
        &mut self,
        commands: &[PhysicsCommandDto],
        events: &mut Vec<PhysicsEventDto>,
    ) -> Result<usize, String> {
        let mut applied = 0;
        for command in commands {
            match command.kind {
                PhysicsCommandKindDto::SetBodyPose {
                    entity,
                    position,
                    rotation,
                } => {
                    if let Some(record) = self.records.get(&entity) {
                        self.client
                            .reset_base_position_and_orientation(
                                record.body_id,
                                vec3_f64(position),
                                quat_f64(rotation),
                            )
                            .map_err(bullet_error)?;
                    }
                    applied += 1;
                }
                PhysicsCommandKindDto::SetLinearVelocity { entity, velocity } => {
                    if let Some(record) = self.records.get(&entity) {
                        self.client
                            .reset_base_velocity(record.body_id, Some(vec3_f64(velocity)), None)
                            .map_err(bullet_error)?;
                    }
                    applied += 1;
                }
                PhysicsCommandKindDto::ApplyImpulse { entity, impulse, point: _ } => {
                    if let Some(record) = self.records.get(&entity) {
                        let dynamics = self.client.get_dynamics_info(record.body_id, -1).map_err(bullet_error)?;
                        let mass = dynamics.mass as f32;
                        if mass.is_finite() && mass > 1.0e-6 && impulse.iter().all(|v| v.is_finite()) {
                            let velocity = self.client.get_base_velocity(record.body_id).map_err(bullet_error)?;
                            let next = [
                                velocity[0] + (impulse[0] / mass) as f64,
                                velocity[1] + (impulse[1] / mass) as f64,
                                velocity[2] + (impulse[2] / mass) as f64,
                            ];
                            self.client.reset_base_velocity(record.body_id, Some(next), None).map_err(bullet_error)?;
                        }
                    }
                    applied += 1;
                }
                PhysicsCommandKindDto::DestroyBody { entity } => {
                    if self.records.contains_key(&entity) {
                        self.destroy_body(entity)?;
                        events.push(PhysicsEventDto::BodyDestroyed { entity });
                    }
                    applied += 1;
                }
            }
        }
        Ok(applied)
    }

    fn destroy_body(&mut self, entity: PhysicsEntityKey) -> Result<(), String> {
        let Some(record) = self.records.remove(&entity) else {
            return Ok(());
        };
        self.body_to_entity.remove(&record.body_id);
        self.client
            .remove_body(record.body_id)
            .map_err(bullet_error)?;
        self.client
            .remove_collision_shape(record.collision_shape_id)
            .map_err(bullet_error)?;
        Ok(())
    }

    fn collect_body_outputs(&mut self, output: &mut PhysicsFrameOutput) -> Result<(), String> {
        for (&entity, record) in &self.records {
            if !record.produces_output {
                continue;
            }
            let pose = self
                .client
                .get_base_position_and_orientation(record.body_id)
                .map_err(bullet_error)?;
            let velocity = self
                .client
                .get_base_velocity(record.body_id)
                .map_err(bullet_error)?;
            let rotation = pose.rotation.quaternion();
            output.pose_updates.push(PhysicsBodyPoseUpdate {
                entity,
                position: [
                    pose.translation.vector.x as f32,
                    pose.translation.vector.y as f32,
                    pose.translation.vector.z as f32,
                ],
                rotation: [
                    rotation.i as f32,
                    rotation.j as f32,
                    rotation.k as f32,
                    rotation.w as f32,
                ],
            });
            output.velocity_updates.push(PhysicsBodyVelocityUpdate {
                entity,
                linear_velocity: [velocity[0] as f32, velocity[1] as f32, velocity[2] as f32],
                angular_velocity: [velocity[3] as f32, velocity[4] as f32, velocity[5] as f32],
            });
        }
        Ok(())
    }

    fn collect_contact_events(
        &mut self,
        dt: f32,
        output: &mut PhysicsFrameOutput,
    ) -> Result<(), String> {
        let body_metadata_source = self
            .records
            .iter()
            .map(|(&entity, record)| (entity, record.body_id, record.material))
            .collect::<Vec<_>>();
        let mut body_metadata =
            HashMap::<PhysicsEntityKey, (PhysicsVec3, PhysicsMaterialDto)>::new();
        for (entity, body_id, material) in body_metadata_source {
            let velocity = self
                .client
                .get_base_velocity(body_id)
                .map_err(bullet_error)?;
            body_metadata.insert(
                entity,
                (
                    sanitize_contact_velocity([
                        velocity[0] as f32,
                        velocity[1] as f32,
                        velocity[2] as f32,
                    ]),
                    material,
                ),
            );
        }

        let points = self
            .client
            .get_contact_points(None, None, None, None)
            .map_err(bullet_error)?;
        let mut current = BTreeMap::<(u64, u64), ContactSample>::new();
        for point in points {
            let Some(&entity_a) = self.body_to_entity.get(&point.body_a) else {
                continue;
            };
            let Some(&entity_b) = self.body_to_entity.get(&point.body_b) else {
                continue;
            };
            if entity_a == entity_b
                || !self
                    .records
                    .get(&entity_a)
                    .is_some_and(|record| record.casts_contacts)
                || !self
                    .records
                    .get(&entity_b)
                    .is_some_and(|record| record.casts_contacts)
            {
                continue;
            }

            let (pair, normal) = if entity_a <= entity_b {
                ((entity_a, entity_b), vec3_f32(point.contact_normal_on_b))
            } else {
                (
                    (entity_b, entity_a),
                    negate(vec3_f32(point.contact_normal_on_b)),
                )
            };
            let sample = ContactSample {
                point: vec3_f32(point.position_on_b),
                normal,
                impulse: (point.normal_force * dt.max(0.0) as f64) as f32,
            };
            current
                .entry(pair)
                .and_modify(|existing| {
                    let replace_geometry = sample.impulse > existing.impulse;
                    existing.impulse += sample.impulse;
                    if replace_geometry {
                        existing.point = sample.point;
                        existing.normal = sample.normal;
                    }
                })
                .or_insert(sample);
        }

        for (&(a, b), sample) in &current {
            let (relative_velocity, materials) =
                match (body_metadata.get(&a), body_metadata.get(&b)) {
                    (Some((velocity_a, material_a)), Some((velocity_b, material_b))) => (
                        Some([
                            velocity_b[0] - velocity_a[0],
                            velocity_b[1] - velocity_a[1],
                            velocity_b[2] - velocity_a[2],
                        ]),
                        PhysicsContactMaterialPairDto {
                            a: Some(*material_a),
                            b: Some(*material_b),
                        },
                    ),
                    _ => (None, PhysicsContactMaterialPairDto::default()),
                };
            let contact = PhysicsContactEventDto {
                a,
                b,
                point: sample.point,
                normal: sample.normal,
                impulse: sample.impulse,
                relative_velocity,
                materials,
            };
            output
                .events
                .push(if self.active_contacts.contains(&(a, b)) {
                    PhysicsEventDto::ContactPersist(contact)
                } else {
                    PhysicsEventDto::ContactBegin(contact)
                });
        }
        let current_pairs = current.keys().copied().collect::<BTreeSet<_>>();
        for &(a, b) in self.active_contacts.difference(&current_pairs) {
            output.events.push(PhysicsEventDto::ContactEnd { a, b });
        }
        self.active_contacts = current_pairs;
        Ok(())
    }

    fn execute_queries(
        &mut self,
        input: &PhysicsFrameInput,
    ) -> Result<Vec<PhysicsQueryHitDto>, String> {
        let mut hits = Vec::new();
        for query in input
            .queries
            .iter()
            .take(self.max_queries_per_frame as usize)
        {
            match query.kind {
                PhysicsQueryKindDto::Ray { origin, dir, max_t } => {
                    if let Some(hit) = self.cast_ray(query.seq, query.ignore_entity, origin, dir, max_t)? {
                        hits.push(hit);
                    }
                }
                // Bullet's current adapter exposes closest-hit rayTest only. Preserve functional
                // firearm fallback, while Jolt/Gravitas remains the authoritative all-hit provider.
                PhysicsQueryKindDto::BallisticRay { origin, dir, max_t, .. } => {
                    if let Some(hit) = self.cast_ray(query.seq, query.ignore_entity, origin, dir, max_t)? {
                        hits.push(hit);
                    }
                }
                PhysicsQueryKindDto::Sphere { .. } | PhysicsQueryKindDto::Aabb { .. } => {}
            }
        }
        Ok(hits)
    }

    fn cast_ray(
        &mut self,
        seq: u64,
        ignore_entity: Option<u64>,
        origin: PhysicsVec3,
        dir: PhysicsVec3,
        max_t: f32,
    ) -> Result<Option<PhysicsQueryHitDto>, String> {
        if !max_t.is_finite() || max_t <= 0.0 {
            return Ok(None);
        }
        let length_sq = dir.iter().map(|v| v * v).sum::<f32>();
        if !length_sq.is_finite() || length_sq <= 1.0e-12 {
            return Ok(None);
        }
        let inv_len = length_sq.sqrt().recip();
        let unit = [dir[0] * inv_len, dir[1] * inv_len, dir[2] * inv_len];
        let ray_from = vec3_f64(origin);
        let ray_to = [
            (origin[0] + unit[0] * max_t) as f64,
            (origin[1] + unit[1] * max_t) as f64,
            (origin[2] + unit[2] * max_t) as f64,
        ];

        let ignored_entity = ignore_entity.unwrap_or(seq);
        let ignored_body = self
            .records
            .get(&ignored_entity)
            .map(|record| record.body_id);
        if let Some(body_id) = ignored_body {
            self.client
                .set_collision_filter_group_mask(body_id, -1, 0, 0)
                .map_err(bullet_error)?;
        }
        let ray_result = self.client.ray_test(ray_from, ray_to, None, Some(1));
        if let Some(body_id) = ignored_body {
            self.client
                .set_collision_filter_group_mask(body_id, -1, 1, -1)
                .map_err(bullet_error)?;
        }
        let ray_hits = ray_result.map_err(bullet_error)?;
        let Some(hit) = ray_hits.into_iter().find(|hit| hit.object_unique_id >= 0) else {
            return Ok(None);
        };
        let Some(&entity) = self.body_to_entity.get(&hit.object_unique_id) else {
            return Ok(None);
        };
        let normal = vec3_f32(hit.hit_normal_world);
        Ok(Some(PhysicsQueryHitDto {
            seq,
            entity,
            position: vec3_f32(hit.hit_position_world),
            normal,
            distance: (hit.hit_fraction as f32).clamp(0.0, 1.0) * max_t,
            subshape_id: 0,
            hit_index: 0,
            back_face: unit[0] * normal[0] + unit[1] * normal[1] + unit[2] * normal[2] > 0.0,
        }))
    }
}

impl Drop for BulletPacketPhysicsBackend {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn bullet_error(error: impl std::fmt::Display) -> String {
    error.to_string()
}

fn positive(value: f32, label: &str) -> Result<f32, String> {
    if value.is_finite() && value > 1.0e-5 {
        Ok(value)
    } else {
        Err(format!("{label} must be finite and positive, got {value}"))
    }
}

fn positive_vec3(value: PhysicsVec3, label: &str) -> Result<PhysicsVec3, String> {
    Ok([
        positive(value[0], label)?,
        positive(value[1], label)?,
        positive(value[2], label)?,
    ])
}

fn vec3_f64(value: PhysicsVec3) -> [f64; 3] {
    [value[0] as f64, value[1] as f64, value[2] as f64]
}

fn vec3_f32(value: [f64; 3]) -> PhysicsVec3 {
    [value[0] as f32, value[1] as f32, value[2] as f32]
}

fn quat_f64(value: PhysicsQuat) -> [f64; 4] {
    [
        value[0] as f64,
        value[1] as f64,
        value[2] as f64,
        value[3] as f64,
    ]
}

fn physics_pose(position: PhysicsVec3, rotation: PhysicsQuat) -> Isometry3<f64> {
    let q = Quaternion::new(
        rotation[3] as f64,
        rotation[0] as f64,
        rotation[1] as f64,
        rotation[2] as f64,
    );
    let rotation = if q.norm_squared().is_finite() && q.norm_squared() > 1.0e-12 {
        UnitQuaternion::new_normalize(q)
    } else {
        UnitQuaternion::identity()
    };
    Isometry3::from_parts(
        Translation3::new(position[0] as f64, position[1] as f64, position[2] as f64),
        rotation,
    )
}

fn negate(value: PhysicsVec3) -> PhysicsVec3 {
    [-value[0], -value[1], -value[2]]
}

#[inline]
fn sanitize_contact_velocity(value: PhysicsVec3) -> PhysicsVec3 {
    value.map(|component| {
        if component.is_finite() {
            component
        } else {
            0.0
        }
    })
}

fn body_mass(kind: PhysicsBodyKindDto, shape: CollisionShapeDto, density: f32) -> f64 {
    if kind != PhysicsBodyKindDto::Dynamic {
        return 0.0;
    }
    let density = if density.is_finite() && density > 0.0 {
        density as f64
    } else {
        1.0
    };
    let volume = match shape {
        CollisionShapeDto::Box { half_extents } => {
            8.0 * half_extents[0].abs() as f64
                * half_extents[1].abs() as f64
                * half_extents[2].abs() as f64
        }
        CollisionShapeDto::Sphere { radius } => 4.0 / 3.0 * PI * (radius.abs() as f64).powi(3),
        CollisionShapeDto::Capsule {
            radius,
            half_height,
        } => {
            let r = radius.abs() as f64;
            PI * r * r * (2.0 * half_height.abs() as f64) + 4.0 / 3.0 * PI * r.powi(3)
        }
        CollisionShapeDto::Cylinder {
            radius,
            half_height,
        } => {
            let r = radius.abs() as f64;
            PI * r * r * (2.0 * half_height.abs() as f64)
        }
    };
    (density * volume).max(1.0e-4)
}
fn mesh_vertices(mesh: &MeshColliderDto) -> Result<Vec<[f64; 3]>, String> {
    if mesh.vertices.is_empty() {
        return Err("mesh collider has no vertices".to_owned());
    }
    if mesh.vertices.len() > i32::MAX as usize {
        return Err("mesh collider has too many vertices for Bullet".to_owned());
    }
    Ok(mesh.vertices.iter().copied().map(vec3_f64).collect())
}

fn mesh_indices(mesh: &MeshColliderDto) -> Result<Vec<i32>, String> {
    if mesh.triangles.is_empty() {
        return Err("mesh collider has no triangles".to_owned());
    }
    let vertex_count = mesh.vertices.len() as u32;
    let mut out = Vec::with_capacity(mesh.triangles.len() * 3);
    for triangle in &mesh.triangles {
        if triangle
            .iter()
            .any(|index| *index >= vertex_count || *index > i32::MAX as u32)
        {
            return Err(format!(
                "mesh collider triangle index out of range: {triangle:?}"
            ));
        }
        out.extend(triangle.iter().map(|index| *index as i32));
    }
    Ok(out)
}

fn heightfield_mesh(
    heightfield: &HeightfieldColliderDto,
) -> Result<(Vec<[f64; 3]>, Vec<i32>), String> {
    let nx = heightfield.sample_count_x as usize;
    let nz = heightfield.sample_count_z as usize;
    let sample_count = nx
        .checked_mul(nz)
        .ok_or_else(|| "heightfield sample dimensions overflow".to_owned())?;
    if nx < 2
        || nz < 2
        || sample_count > i32::MAX as usize
        || heightfield.heights.len() != sample_count
    {
        return Err(format!(
            "heightfield dimensions/data mismatch: {}x{} with {} heights",
            nx,
            nz,
            heightfield.heights.len()
        ));
    }
    let sx = positive(heightfield.spacing[0], "heightfield spacing x")? as f64;
    let sz = positive(heightfield.spacing[1], "heightfield spacing z")? as f64;
    let origin = vec3_f64(heightfield.local_origin);
    let mut vertices = Vec::with_capacity(nx * nz);
    for z in 0..nz {
        for x in 0..nx {
            let height = heightfield.heights[z * nx + x];
            if !height.is_finite() {
                return Err(format!("heightfield sample ({x}, {z}) is not finite"));
            }
            vertices.push([
                origin[0] + x as f64 * sx,
                origin[1] + height as f64,
                origin[2] + z as f64 * sz,
            ]);
        }
    }
    let mut indices = Vec::with_capacity((nx - 1) * (nz - 1) * 6);
    for z in 0..(nz - 1) {
        for x in 0..(nx - 1) {
            let a = (z * nx + x) as i32;
            let b = a + 1;
            let c = a + nx as i32;
            let d = c + 1;
            indices.extend_from_slice(&[a, c, b, b, c, d]);
        }
    }
    Ok((vertices, indices))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn body(
        entity: u64,
        kind: PhysicsBodyKindDto,
        shape: CollisionShapeDto,
        position: PhysicsVec3,
    ) -> PhysicsFrameBodySnapshot {
        PhysicsFrameBodySnapshot {
            entity,
            kind,
            shape,
            flags: PhysicsBodyFlagsDto {
                is_trigger: false,
                participates_in_queries: true,
                casts_contacts: true,
                continuous_collision: false,
            },
            material: PhysicsMaterialDto {
                friction: 0.8,
                restitution: 0.0,
                density: 1.0,
            },
            position,
            rotation: [0.0, 0.0, 0.0, 1.0],
            linear_velocity: [0.0, 0.0, 0.0],
            angular_velocity: [0.0, 0.0, 0.0],
            linear_damping: None,
            angular_damping: None,
            bounds_min: [-10.0, -10.0, -10.0],
            bounds_max: [10.0, 10.0, 10.0],
        }
    }

    #[test]
    fn bullet_direct_world_steps_and_returns_dynamic_pose() {
        let mut backend = BulletPacketPhysicsBackend::new(128, 32).expect("Bullet direct world");
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
        input.bodies.push(body(
            1,
            PhysicsBodyKindDto::Static,
            CollisionShapeDto::Box {
                half_extents: [5.0, 0.5, 5.0],
            },
            [0.0, -0.5, 0.0],
        ));
        input.bodies.push(body(
            2,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Sphere { radius: 0.5 },
            [0.0, 2.0, 0.0],
        ));
        let output = backend.step_frame(input).expect("Bullet step");
        assert!(output.pose_updates.iter().any(|pose| pose.entity == 2));
        assert_eq!(output.report.dynamic_bodies, 1);
        assert_eq!(output.report.static_bodies, 1);
    }

    #[test]
    fn bullet_cylinder_preserves_brass_contact_dynamics() {
        let mut backend = BulletPacketPhysicsBackend::new(128, 32).expect("Bullet direct world");
        let mut shell = body(
            7,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Cylinder {
                radius: 0.00635,
                half_height: 0.02940,
            },
            [0.0, 0.2, 0.0],
        );
        shell.material.friction = 0.28;
        shell.material.restitution = 0.34;
        shell.material.density = 1610.0;
        shell.linear_velocity = [1.8, -1.4, 0.4];
        shell.angular_velocity = [18.0, 11.0, 23.0];
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
        input.bodies.push(shell);
        backend.step_frame(input).expect("Bullet shell step");
        let record = backend.records.get(&7).expect("shell body record");
        let info = backend
            .client
            .get_dynamics_info(record.body_id, -1)
            .expect("shell dynamics info");
        assert!((info.mass - 0.011992).abs() < 0.0002, "mass={}", info.mass);
        assert!((info.lateral_friction - 0.28).abs() < 1.0e-6);
        // Bullet Direct omits rigid-body restitution from CMD_GET_DYNAMICS_INFO; verify it
        // behaviorally in the rebound test below instead of trusting the zero-filled field.
        assert!((info.rolling_friction - 0.0025).abs() < 1.0e-6);
        assert!((info.spinning_friction - 0.0015).abs() < 1.0e-6);
        assert!((info.linear_damping - 0.015).abs() < 1.0e-6);
        assert!((info.angular_damping - 0.025).abs() < 1.0e-6);
    }

    #[test]
    fn bullet_cylinder_rebounds_and_retains_angular_motion() {
        let mut backend = BulletPacketPhysicsBackend::new(128, 32).expect("Bullet direct world");
        let mut ground = body(
            1,
            PhysicsBodyKindDto::Static,
            CollisionShapeDto::Box {
                half_extents: [2.0, 0.05, 2.0],
            },
            [0.0, -0.05, 0.0],
        );
        ground.material.friction = 0.6;
        ground.material.restitution = 1.0;

        let mut shell = body(
            7,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Cylinder {
                radius: 0.00635,
                half_height: 0.02940,
            },
            [0.0, 0.12, 0.0],
        );
        shell.material.friction = 0.28;
        shell.material.restitution = 0.34;
        shell.material.density = 1610.0;
        shell.linear_velocity = [0.9, -1.8, 0.25];
        shell.angular_velocity = [18.0, 11.0, 23.0];

        let mut saw_contact = false;
        let mut saw_enriched_contact = false;
        let mut max_upward = f32::NEG_INFINITY;
        let mut max_angular = 0.0_f32;
        for tick in 1..=80_u64 {
            let mut input = PhysicsFrameInput::empty(tick, tick, 1.0 / 240.0);
            input.bodies.push(ground.clone());
            input.bodies.push(shell.clone());
            let output = backend
                .step_frame(input)
                .expect("Bullet shell simulation step");
            for event in &output.events {
                if let PhysicsEventDto::ContactBegin(contact) = event {
                    if contact.a == 1 && contact.b == 7 {
                        saw_contact = true;
                        saw_enriched_contact |= contact.relative_velocity.is_some()
                            && contact.materials.a.is_some()
                            && contact.materials.b.is_some();
                        assert_eq!(contact.materials.a.unwrap().friction, 0.6);
                        assert_eq!(contact.materials.b.unwrap().restitution, 0.34);
                    }
                }
            }
            if let Some(velocity) = output.velocity_updates.iter().find(|v| v.entity == 7) {
                if saw_contact {
                    max_upward = max_upward.max(velocity.linear_velocity[1]);
                    let a = velocity.angular_velocity;
                    max_angular = max_angular.max((a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt());
                }
            }
        }
        assert!(saw_enriched_contact);
        assert!(saw_contact, "shell never contacted the floor");
        assert!(
            max_upward > 0.05,
            "shell never rebounded: max_upward={max_upward}"
        );
        assert!(
            max_angular > 1.0,
            "shell lost angular motion too aggressively: {max_angular}"
        );
    }

    #[test]
    fn heightfield_adapter_builds_y_up_triangle_mesh() {
        let (vertices, indices) = heightfield_mesh(&HeightfieldColliderDto {
            sample_count_x: 2,
            sample_count_z: 2,
            spacing: [1.0, 1.0],
            local_origin: [0.0, 0.0, 0.0],
            heights: vec![0.0, 0.0, 1.0, 1.0],
            min_height: 0.0,
            max_height: 1.0,
        })
        .expect("heightfield mesh");
        assert_eq!(vertices.len(), 4);
        assert_eq!(indices, vec![0, 2, 1, 1, 2, 3]);
    }
}
