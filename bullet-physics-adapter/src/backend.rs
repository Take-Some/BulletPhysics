use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};
use std::f64::consts::PI;
use std::time::Duration;
mod box_surfaces;
mod collision_filters;
mod joints;
#[cfg(test)]
mod performance_tests;
mod queries;
#[cfg(test)]
mod scalability_tests;
#[cfg(test)]
mod settled_fragment_tests;
mod validation;
use joints::JointRecord;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
use newviso_physics_api::*;
use rsbullet_core::{
    CollisionGeometry, CollisionId, CollisionShapeOptions, DynamicsUpdate, Mode,
    MultiBodyCreateOptions, PhysicsClient,
};

#[derive(Debug, Clone, PartialEq)]
enum ShapeSource {
    Primitive(CollisionShapeDto),
    Convex(Vec<Vec<PhysicsVec3>>),
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
    participates_in_queries: bool,
    material: PhysicsMaterialDto,
    linear_damping: Option<f32>,
    angular_damping: Option<f32>,
    mass_properties: Option<PhysicsMassProperties>,
    authored_position: PhysicsVec3,
    authored_rotation: PhysicsQuat,
}

impl BodyRecord {
    fn is_exact_box(&self) -> bool {
        self.kind == PhysicsBodyKindDto::Static
            && matches!(
                self.shape,
                ShapeSource::Primitive(CollisionShapeDto::Box { .. })
            )
    }

    fn query_group(&self, participates: bool) -> i32 {
        // All bodies keep a physical mask of -1. Bit 1 is reserved for
        // native ray participation; exact boxes use our authored BVH.
        if participates && !self.is_exact_box() {
            1
        } else {
            2
        }
    }
}

#[derive(Debug, Default, Clone, serde::Serialize)]
pub(crate) struct FrameMetrics {
    pub fixed_tick: u64,
    pub active_bodies: usize,
    pub native_pose_updates: usize,
    pub body_state_reads: usize,
    pub filter_pair_updates: usize,
    pub query_batches: usize,
    pub rays: usize,
    pub exact_box_tests: usize,
    pub query_index_rebuilds: usize,
    pub step_ms: Option<f64>,
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
    joints: BTreeMap<u64, JointRecord>,
    joint_filtered_pairs: BTreeSet<(u64, u64)>,
    explicit_filtered_pairs: BTreeSet<(u64, u64)>,
    max_bodies: u32,
    max_queries_per_frame: u32,
    last_dt: Option<f32>,
    last_gravity: Option<f32>,
    solver_quality: Option<(bool, bool)>,
    compound_files: HashMap<i32, std::path::PathBuf>,
    query_index: queries::BoxQueryIndex,
    query_index_dirty: bool,
    query_batch_size: usize,
    query_threads: i32,
    profile_steps: bool,
    desired_entities: HashSet<PhysicsEntityKey>,
    pre_step_states: HashMap<i32, (Isometry3<f64>, [f64; 6])>,
    contact_velocities: HashMap<PhysicsEntityKey, PhysicsVec3>,
    pub(crate) metrics: FrameMetrics,
}

impl BulletPacketPhysicsBackend {
    #[cfg(test)]
    pub fn new(max_bodies: u32, max_queries_per_frame: u32) -> Result<Self, String> {
        Self::with_settings(max_bodies, max_queries_per_frame, 256, 1, false)
    }

    pub(crate) fn with_settings(
        max_bodies: u32,
        max_queries_per_frame: u32,
        query_batch_size: usize,
        query_threads: i32,
        profile_steps: bool,
    ) -> Result<Self, String> {
        let client = PhysicsClient::connect(Mode::Direct).map_err(bullet_error)?;
        Ok(Self {
            client,
            records: BTreeMap::new(),
            body_to_entity: HashMap::new(),
            active_contacts: BTreeSet::new(),
            joints: BTreeMap::new(),
            joint_filtered_pairs: BTreeSet::new(),
            explicit_filtered_pairs: BTreeSet::new(),
            max_bodies: max_bodies.max(1),
            max_queries_per_frame: max_queries_per_frame.max(1),
            last_dt: None,
            last_gravity: None,
            solver_quality: None,
            compound_files: HashMap::new(),
            query_index: queries::BoxQueryIndex::default(),
            query_index_dirty: true,
            query_batch_size: query_batch_size.clamp(1, 256),
            query_threads: query_threads.clamp(1, 64),
            profile_steps,
            desired_entities: HashSet::new(),
            pre_step_states: HashMap::new(),
            contact_velocities: HashMap::new(),
            metrics: FrameMetrics::default(),
        })
    }

    pub fn step_frame(&mut self, input: PhysicsFrameInput) -> Result<PhysicsFrameOutput, String> {
        self.validate_provider_frame(&input)?;
        self.metrics = FrameMetrics {
            fixed_tick: input.fixed_tick,
            ..Default::default()
        };
        let started = self.profile_steps.then(std::time::Instant::now);
        self.pre_step_states.clear();
        self.configure_world(input.dt, input.gravity)?;

        let mut output = PhysicsFrameOutput {
            fixed_tick: input.fixed_tick,
            ..PhysicsFrameOutput::default()
        };

        // Retire transient bodies first so replacement frames can use the
        // full configured capacity without exceeding it during synchronization.
        let stale_entities = self
            .records
            .iter()
            .filter_map(|(&entity, record)| {
                (!record.persistent && !self.desired_entities.contains(&entity)).then_some(entity)
            })
            .collect::<Vec<_>>();
        for entity in stale_entities {
            self.destroy_body(entity)?;
            output
                .events
                .push(PhysicsEventDto::BodyDestroyed { entity });
        }

        for collider in &input.colliders {
            if self.sync_authored_collider(collider)? {
                output.events.push(PhysicsEventDto::BodyCreated {
                    entity: collider.entity,
                });
            }
        }

        for body in &input.bodies {
            if self.sync_frame_body(body)? {
                output.events.push(PhysicsEventDto::BodyCreated {
                    entity: body.entity,
                });
            }
        }

        output.report.commands_applied =
            self.apply_commands(&input.commands, &mut output.events)?;

        self.sync_joints(&input.joints)?;
        self.sync_explicit_collision_pairs(&input.disabled_collision_pairs)?;
        self.configure_joint_solver(
            !input.joints.is_empty(),
            input.bodies.iter().any(|body| {
                body.kind == PhysicsBodyKindDto::Dynamic && body.flags.continuous_collision
            }),
        )?;
        self.apply_joint_angular_limits(input.dt as f64)?;
        // Measure approach velocity before the solver removes it at impact.
        self.capture_contact_velocities()?;
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
            substeps: u32::from(input.dt.is_finite() && input.dt > 0.0)
                * if self
                    .solver_quality
                    .is_some_and(|(joints, continuous)| joints || continuous)
                {
                    4
                } else {
                    1
                },
            active_bodies: self.records.len(),
            static_bodies: self.records.len().saturating_sub(dynamic_bodies),
            dynamic_bodies,
            contacts: self.active_contacts.len(),
            commands_applied: output.report.commands_applied,
        };
        self.metrics.active_bodies = self.records.len();
        self.metrics.step_ms = started.map(|start| start.elapsed().as_secs_f64() * 1000.0);
        Ok(output)
    }

    pub fn shutdown(&mut self) {
        let _ = self.client.reset_simulation();
        for (_, path) in self.compound_files.drain() {
            let _ = std::fs::remove_file(path);
        }
        self.joints.clear();
        self.joint_filtered_pairs.clear();
        self.explicit_filtered_pairs.clear();
        self.records.clear();
        self.body_to_entity.clear();
        self.active_contacts.clear();
        self.last_dt = None;
        self.solver_quality = None;
        self.last_gravity = None;
        self.query_index = queries::BoxQueryIndex::default();
        self.query_index_dirty = true;
        self.desired_entities.clear();
        self.pre_step_states.clear();
        self.contact_velocities.clear();
        self.metrics = FrameMetrics::default();
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

        let recreate = self.records.get(&snapshot.entity).is_some_and(|record| {
            let same_shape = match &record.shape {
                ShapeSource::Primitive(shape) => {
                    snapshot.convex_hulls.is_empty() && *shape == snapshot.shape
                }
                ShapeSource::Convex(hulls) => *hulls == snapshot.convex_hulls,
                ShapeSource::Authored(_) => false,
            };
            record.persistent
                || record.kind != snapshot.kind
                || !same_shape
                || record.material != snapshot.material
                || record.linear_damping != snapshot.linear_damping
                || record.angular_damping != snapshot.angular_damping
                || record.mass_properties != snapshot.mass_properties
        });
        if recreate {
            self.destroy_body(snapshot.entity)?;
        }

        if let Some(record) = self.records.get_mut(&snapshot.entity) {
            let pose_changed = record.authored_position != snapshot.position
                || record.authored_rotation != snapshot.rotation;
            if record.query_group(record.participates_in_queries)
                != record.query_group(snapshot.flags.participates_in_queries)
            {
                self.client
                    .set_collision_filter_group_mask(
                        record.body_id,
                        -1,
                        record.query_group(snapshot.flags.participates_in_queries),
                        -1,
                    )
                    .map_err(bullet_error)?;
            }
            if record.is_exact_box()
                && (pose_changed
                    || record.participates_in_queries != snapshot.flags.participates_in_queries)
            {
                self.query_index_dirty = true;
            }
            record.casts_contacts = snapshot.flags.casts_contacts;
            record.participates_in_queries = snapshot.flags.participates_in_queries;
            let body_id = record.body_id;
            record.authored_position = snapshot.position;
            record.authored_rotation = snapshot.rotation;
            if snapshot.kind != PhysicsBodyKindDto::Dynamic && pose_changed {
                self.client
                    .reset_base_position_and_orientation(
                        body_id,
                        {
                            let offset = physics_pose([0.0; 3], snapshot.rotation).rotation
                                * Vector3::from(vec3_f64(
                                    record
                                        .mass_properties
                                        .map_or([0.0; 3], |p| p.center_of_mass),
                                ));
                            let origin = Vector3::from(vec3_f64(snapshot.position));
                            (origin + offset).into()
                        },
                        quat_f64(snapshot.rotation),
                    )
                    .map_err(bullet_error)?;
                self.metrics.native_pose_updates += 1;
            }
            return Ok(false);
        }

        self.ensure_body_capacity()?;
        // Geometry is owned only on creation/rebuild, never on an unchanged frame.
        let shape = if snapshot.convex_hulls.is_empty() {
            ShapeSource::Primitive(snapshot.shape)
        } else {
            ShapeSource::Convex(snapshot.convex_hulls.clone())
        };
        let collision_shape_id = self.create_frame_collision_shape(&shape, snapshot.kind)?;
        let mass = if snapshot.kind == PhysicsBodyKindDto::Dynamic {
            snapshot
                .mass_properties
                .map(|p| p.mass as f64)
                .unwrap_or_else(|| {
                    body_mass(snapshot.kind, snapshot.shape, snapshot.material.density)
                })
        } else {
            0.0
        };
        let body_id = match self.create_body(
            collision_shape_id,
            mass,
            snapshot.position,
            snapshot.rotation,
            snapshot.mass_properties,
        ) {
            Ok(body_id) => body_id,
            Err(error) => {
                let _ = self.release_collision_shape(collision_shape_id);
                return Err(error);
            }
        };

        let cylinder_motion = matches!(snapshot.shape, CollisionShapeDto::Cylinder { .. });
        let dynamics = DynamicsUpdate {
            mass: snapshot.mass_properties.map(|_| mass),
            activation_state: snapshot.mass_properties.map(|_| 2),
            local_inertia_diagonal: snapshot
                .mass_properties
                .filter(|_| snapshot.kind == PhysicsBodyKindDto::Dynamic)
                .map(|p| vec3_f64(p.inertia_diagonal)),
            lateral_friction: Some(snapshot.material.friction.clamp(0.0, 10.0) as f64),
            rolling_friction: cylinder_motion.then_some(0.0025),
            spinning_friction: cylinder_motion.then_some(0.0015),
            linear_damping: snapshot
                .linear_damping
                .filter(|value| value.is_finite())
                .map(|value| value.clamp(0.0, 20.0) as f64)
                .or_else(|| cylinder_motion.then_some(0.015)),
            angular_damping: snapshot
                .angular_damping
                .filter(|value| value.is_finite())
                .map(|value| value.clamp(0.0, 20.0) as f64)
                .or_else(|| cylinder_motion.then_some(0.025)),
            ..DynamicsUpdate::default()
        };
        if let Err(error) = self.client.change_dynamics(body_id, -1, &dynamics) {
            self.discard_untracked_body(body_id, collision_shape_id);
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
            self.discard_untracked_body(body_id, collision_shape_id);
            return Err(bullet_error(error));
        }

        if let Some(properties) = snapshot
            .mass_properties
            .filter(|_| snapshot.kind == PhysicsBodyKindDto::Dynamic)
        {
            if let Err(error) = self.client.change_dynamics(
                body_id,
                -1,
                &DynamicsUpdate {
                    mass: Some(properties.mass as f64),
                    local_inertia_diagonal: Some(vec3_f64(properties.inertia_diagonal)),
                    ..DynamicsUpdate::default()
                },
            ) {
                self.discard_untracked_body(body_id, collision_shape_id);
                return Err(bullet_error(error));
            }
        }
        if snapshot.kind == PhysicsBodyKindDto::Dynamic {
            if let Err(error) = self.client.reset_base_velocity(
                body_id,
                Some({
                    let offset = physics_pose([0.0; 3], snapshot.rotation).rotation
                        * Vector3::from(vec3_f64(
                            snapshot
                                .mass_properties
                                .map_or([0.0; 3], |p| p.center_of_mass),
                        ));
                    let spin = Vector3::from(vec3_f64(snapshot.angular_velocity));
                    (Vector3::from(vec3_f64(snapshot.linear_velocity)) + spin.cross(&offset)).into()
                }),
                Some(vec3_f64(snapshot.angular_velocity)),
            ) {
                self.discard_untracked_body(body_id, collision_shape_id);
                return Err(bullet_error(error));
            }
        }

        let group = if snapshot.flags.participates_in_queries
            && !(snapshot.kind == PhysicsBodyKindDto::Static
                && matches!(shape, ShapeSource::Primitive(CollisionShapeDto::Box { .. })))
        {
            1
        } else {
            2
        };
        if let Err(error) = self
            .client
            .set_collision_filter_group_mask(body_id, -1, group, -1)
        {
            self.discard_untracked_body(body_id, collision_shape_id);
            return Err(bullet_error(error));
        }
        self.query_index_dirty |= snapshot.kind == PhysicsBodyKindDto::Static
            && matches!(shape, ShapeSource::Primitive(CollisionShapeDto::Box { .. }));
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
                participates_in_queries: snapshot.flags.participates_in_queries,
                material: snapshot.material,
                linear_damping: snapshot.linear_damping,
                angular_damping: snapshot.angular_damping,
                mass_properties: snapshot.mass_properties,
                authored_position: snapshot.position,
                authored_rotation: snapshot.rotation,
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

        let recreate = self.records.get(&snapshot.entity).is_some_and(|record| {
            !record.persistent || !matches!(&record.shape, ShapeSource::Authored(collider) if *collider == snapshot.collider)
                || record.material != snapshot.material
        });
        if recreate {
            self.destroy_body(snapshot.entity)?;
        }
        if let Some(record) = self.records.get_mut(&snapshot.entity) {
            let group = record.query_group(snapshot.flags.participates_in_queries);
            if group != record.query_group(record.participates_in_queries) {
                self.client
                    .set_collision_filter_group_mask(record.body_id, -1, group, -1)
                    .map_err(bullet_error)?;
            }
            record.casts_contacts = snapshot.flags.casts_contacts;
            record.participates_in_queries = snapshot.flags.participates_in_queries;
            if record.authored_position != snapshot.position
                || record.authored_rotation != snapshot.rotation
            {
                self.client
                    .reset_base_position_and_orientation(
                        record.body_id,
                        vec3_f64(snapshot.position),
                        quat_f64(snapshot.rotation),
                    )
                    .map_err(bullet_error)?;
                record.authored_position = snapshot.position;
                record.authored_rotation = snapshot.rotation;
                self.metrics.native_pose_updates += 1;
            }
            return Ok(false);
        }

        self.ensure_body_capacity()?;
        let shape = ShapeSource::Authored(snapshot.collider.clone());
        let collision_shape_id = self.create_collision_shape(&shape)?;
        let body_id = match self.create_body(
            collision_shape_id,
            0.0,
            snapshot.position,
            snapshot.rotation,
            None,
        ) {
            Ok(body_id) => body_id,
            Err(error) => {
                let _ = self.release_collision_shape(collision_shape_id);
                return Err(error);
            }
        };
        let group = if snapshot.flags.participates_in_queries {
            1
        } else {
            2
        };
        if let Err(error) = self
            .client
            .set_collision_filter_group_mask(body_id, -1, group, -1)
        {
            self.discard_untracked_body(body_id, collision_shape_id);
            return Err(bullet_error(error));
        }
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
                participates_in_queries: snapshot.flags.participates_in_queries,
                material: snapshot.material,
                linear_damping: None,
                angular_damping: None,
                mass_properties: None,
                authored_position: snapshot.position,
                authored_rotation: snapshot.rotation,
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
        mass_properties: Option<PhysicsMassProperties>,
    ) -> Result<i32, String> {
        let mut options = MultiBodyCreateOptions::default();
        options.base.mass = mass;
        options.base.pose = physics_pose(position, rotation);
        options.base.collision_shape = CollisionId(collision_shape_id);
        if let Some(properties) = mass_properties {
            options.base.inertial_pose = Isometry3::translation(
                properties.center_of_mass[0] as f64,
                properties.center_of_mass[1] as f64,
                properties.center_of_mass[2] as f64,
            );
        }
        options.use_maximal_coordinates = true;
        self.client
            .create_multi_body(&options)
            .map_err(bullet_error)
    }

    fn create_collision_shape(&mut self, source: &ShapeSource) -> Result<i32, String> {
        match source {
            ShapeSource::Convex(hulls) => {
                if let [hull] = hulls.as_slice() {
                    // One authored child is a convex point cloud, without OBJ
                    // face reconstruction or convex decomposition.
                    let vertices = hull
                        .iter()
                        .map(|point| vec3_f64(*point))
                        .collect::<Vec<_>>();
                    return self
                        .client
                        .create_collision_shape(
                            &CollisionGeometry::ConvexMesh {
                                vertices: &vertices,
                                scale: [1.0; 3],
                            },
                            None::<CollisionShapeOptions>,
                        )
                        .map_err(bullet_error);
                }
                // The shared-memory array supports 16 children and one inline mesh upload.
                // OBJ groups are imported as distinct convex hulls by Bullet, preserving every
                // authored component without merging them or reusing the last upload.
                use std::fmt::Write as _;
                static NEXT_MESH: std::sync::atomic::AtomicU64 =
                    std::sync::atomic::AtomicU64::new(1);
                let serial = NEXT_MESH.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                let path = std::env::temp_dir().join(format!(
                    "northstar-bullet-{}-{serial}.obj",
                    std::process::id()
                ));
                let mut obj = String::new();
                let mut base = 1;
                for (index, hull) in hulls.iter().enumerate() {
                    writeln!(obj, "o component_{index}").unwrap();
                    for p in hull {
                        writeln!(obj, "v {} {} {}", p[0], p[1], p[2]).unwrap();
                    }
                    for vertex in 1..hull.len() - 1 {
                        writeln!(obj, "f {} {} {}", base, base + vertex, base + vertex + 1)
                            .unwrap();
                    }
                    base += hull.len();
                }
                std::fs::write(&path, obj)
                    .map_err(|e| format!("cannot stage compound collision: {e}"))?;
                let filename = path.to_string_lossy().replace('\\', "/");
                let result = self
                    .client
                    .create_collision_shape(
                        &CollisionGeometry::MeshFile {
                            file: filename.as_str(),
                            scale: [1.0; 3],
                        },
                        None::<CollisionShapeOptions>,
                    )
                    .map_err(bullet_error);
                if let Ok(id) = &result {
                    self.compound_files.insert(*id, path);
                } else {
                    let _ = std::fs::remove_file(&path);
                }
                result
            }
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
                // Bullet's imported capsule uses local Z; engine characters use Y.
                let transform = Isometry3::from_parts(
                    Translation3::new(0.0, 0.0, 0.0),
                    UnitQuaternion::from_euler_angles(-std::f64::consts::FRAC_PI_2, 0.0, 0.0),
                );
                self.client
                    .create_collision_shape(&geometry, Some(CollisionShapeOptions::from(transform)))
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
                    if let Some(record) = self.records.get_mut(&entity) {
                        self.query_index_dirty |= record.is_exact_box();
                        record.authored_position = position;
                        record.authored_rotation = rotation;
                        self.client
                            .reset_base_position_and_orientation(
                                record.body_id,
                                {
                                    let offset = physics_pose([0.0; 3], rotation).rotation
                                        * Vector3::from(vec3_f64(
                                            record
                                                .mass_properties
                                                .map_or([0.0; 3], |p| p.center_of_mass),
                                        ));
                                    [
                                        position[0] as f64 + offset.x,
                                        position[1] as f64 + offset.y,
                                        position[2] as f64 + offset.z,
                                    ]
                                },
                                quat_f64(rotation),
                            )
                            .map_err(bullet_error)?;
                        self.metrics.native_pose_updates += 1;
                    }
                    applied += 1;
                }
                PhysicsCommandKindDto::SetLinearVelocity { entity, velocity } => {
                    if let Some(record) = self.records.get(&entity) {
                        let (pose, current) = self
                            .client
                            .get_base_state(record.body_id)
                            .map_err(bullet_error)?;
                        self.metrics.body_state_reads += 1;
                        let offset = pose.rotation
                            * Vector3::from(vec3_f64(
                                record
                                    .mass_properties
                                    .map_or([0.0; 3], |p| p.center_of_mass),
                            ));
                        let spin = Vector3::new(current[3], current[4], current[5]);
                        let com_velocity = Vector3::from(vec3_f64(velocity)) + spin.cross(&offset);
                        self.client
                            .reset_base_velocity(record.body_id, Some(com_velocity.into()), None)
                            .map_err(bullet_error)?;
                    }
                    applied += 1;
                }
                PhysicsCommandKindDto::SetAngularVelocity { entity, velocity } => {
                    if let Some(record) = self.records.get(&entity) {
                        self.client
                            .reset_base_velocity(record.body_id, None, Some(vec3_f64(velocity)))
                            .map_err(bullet_error)?;
                    }
                    applied += 1;
                }
                PhysicsCommandKindDto::ApplyImpulse {
                    entity,
                    impulse,
                    point,
                } => {
                    if let Some(record) = self
                        .records
                        .get(&entity)
                        .filter(|record| record.kind == PhysicsBodyKindDto::Dynamic)
                    {
                        // Native impulse updates momentum and wakes the island.
                        // Preserve command order; do not defer it as a force.
                        self.client
                            .apply_rigid_body_impulse(
                                record.body_id,
                                vec3_f64(impulse),
                                vec3_f64(point),
                            )
                            .map_err(bullet_error)?;
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
        let Some(record) = self.records.get(&entity) else {
            return Ok(());
        };
        let body_id = record.body_id;
        let shape_id = record.collision_shape_id;
        let exact_box = record.is_exact_box();
        self.remove_body_joints(entity)?;
        self.remove_body_collision_filters(entity)?;
        // Keep the registry intact if the native body removal fails.
        self.client.remove_body(body_id).map_err(bullet_error)?;
        self.records.remove(&entity);
        self.body_to_entity.remove(&body_id);
        self.pre_step_states.remove(&body_id);
        self.query_index_dirty |= exact_box;
        self.release_collision_shape(shape_id)?;
        Ok(())
    }

    fn release_collision_shape(&mut self, shape_id: i32) -> Result<(), String> {
        let result = self
            .client
            .remove_collision_shape(shape_id)
            .map_err(bullet_error);
        if let Some(path) = self.compound_files.remove(&shape_id) {
            let _ = std::fs::remove_file(path);
        }
        result
    }

    fn discard_untracked_body(&mut self, body_id: i32, shape_id: i32) {
        let _ = self.client.remove_body(body_id);
        let _ = self.release_collision_shape(shape_id);
    }

    fn collect_body_outputs(&mut self, output: &mut PhysicsFrameOutput) -> Result<(), String> {
        let count = self
            .records
            .values()
            .filter(|record| record.produces_output)
            .count();
        output.pose_updates.reserve(count);
        output.velocity_updates.reserve(count);
        for (&entity, record) in &self.records {
            if !record.produces_output {
                continue;
            }
            let (pose, velocity) = self
                .client
                .get_base_state(record.body_id)
                .map_err(bullet_error)?;
            self.metrics.body_state_reads += 1;
            let centre_offset = pose.rotation
                * Vector3::from(vec3_f64(
                    record
                        .mass_properties
                        .map_or([0.0; 3], |p| p.center_of_mass),
                ));
            let visual_position = pose.translation.vector - centre_offset;
            let spin = Vector3::new(velocity[3], velocity[4], velocity[5]);
            let visual_velocity =
                Vector3::new(velocity[0], velocity[1], velocity[2]) - spin.cross(&centre_offset);
            let rotation = pose.rotation.quaternion();
            output.pose_updates.push(PhysicsBodyPoseUpdate {
                entity,
                position: [
                    visual_position.x as f32,
                    visual_position.y as f32,
                    visual_position.z as f32,
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
                linear_velocity: [
                    visual_velocity.x as f32,
                    visual_velocity.y as f32,
                    visual_velocity.z as f32,
                ],
                angular_velocity: [velocity[3] as f32, velocity[4] as f32, velocity[5] as f32],
            });
        }
        Ok(())
    }

    fn pre_step_base_state(&mut self, body_id: i32) -> Result<(Isometry3<f64>, [f64; 6]), String> {
        if let Some(state) = self.pre_step_states.get(&body_id) {
            return Ok(*state);
        }
        let state = self.client.get_base_state(body_id).map_err(bullet_error)?;
        self.metrics.body_state_reads += 1;
        self.pre_step_states.insert(body_id, state);
        Ok(state)
    }

    fn capture_contact_velocities(&mut self) -> Result<(), String> {
        self.contact_velocities.clear();
        // Materials already live in BodyRecord. Static bodies cannot approach
        // a contact; querying their native state adds no information.
        let sources: Vec<_> = self
            .records
            .iter()
            .filter(|(_, record)| record.casts_contacts)
            .map(|(&entity, record)| (entity, record.body_id, record.kind))
            .collect();
        self.contact_velocities.reserve(sources.len());
        for (entity, body_id, kind) in sources {
            let velocity = if kind == PhysicsBodyKindDto::Static {
                [0.0; 3]
            } else {
                let (_, state) = self.pre_step_base_state(body_id)?;
                sanitize_contact_velocity([state[0] as f32, state[1] as f32, state[2] as f32])
            };
            self.contact_velocities.insert(entity, velocity);
        }
        Ok(())
    }

    fn collect_contact_events(
        &mut self,
        dt: f32,
        output: &mut PhysicsFrameOutput,
    ) -> Result<(), String> {
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
            let (relative_velocity, materials) = match (
                self.contact_velocities.get(&a),
                self.contact_velocities.get(&b),
            ) {
                (Some(velocity_a), Some(velocity_b)) => (
                    Some([
                        velocity_b[0] - velocity_a[0],
                        velocity_b[1] - velocity_a[1],
                        velocity_b[2] - velocity_a[2],
                    ]),
                    PhysicsContactMaterialPairDto {
                        a: Some(self.records[&a].material),
                        b: Some(self.records[&b].material),
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
                surface_id: None,
                surface_entity: None,
                surface_triangle: None,
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
}

impl Drop for BulletPacketPhysicsBackend {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn ray_authored_box(
    origin: PhysicsVec3,
    dir: PhysicsVec3,
    max_t: f32,
    position: PhysicsVec3,
    rotation: PhysicsQuat,
    half: PhysicsVec3,
) -> Option<(f32, PhysicsVec3)> {
    let pose = physics_pose(position, rotation);
    let start =
        pose.rotation.inverse() * (Vector3::from(vec3_f64(origin)) - pose.translation.vector);
    let direction = pose.rotation.inverse() * Vector3::from(vec3_f64(dir));
    let mut near = 0.0_f64;
    let mut far = max_t as f64;
    let mut near_face = Vector3::zeros();
    let mut far_face = Vector3::zeros();
    for axis in 0..3 {
        if direction[axis].abs() < 1.0e-12 {
            if start[axis].abs() > half[axis] as f64 {
                return None;
            }
            continue;
        }
        let mut a = (-half[axis] as f64 - start[axis]) / direction[axis];
        let mut b = (half[axis] as f64 - start[axis]) / direction[axis];
        let mut sign = -1.0;
        if a > b {
            std::mem::swap(&mut a, &mut b);
            sign = 1.0;
        }
        if a > near {
            near = a;
            near_face = Vector3::zeros();
            near_face[axis] = sign;
        }
        if b < far {
            far = b;
            far_face = Vector3::zeros();
            far_face[axis] = -sign;
        }
        if near > far {
            return None;
        }
    }
    let (distance, face) = if near_face.norm_squared() > 0.0 {
        (near, near_face)
    } else {
        (far, far_face)
    };
    if distance < 0.0 || distance > max_t as f64 || face.norm_squared() == 0.0 {
        return None;
    }
    let normal = pose.rotation * face;
    Some((
        distance as f32,
        [normal.x as f32, normal.y as f32, normal.z as f32],
    ))
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

    pub(super) fn body(
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
            mass_properties: None,
            convex_hulls: Vec::new(),
            bounds_min: [-10.0, -10.0, -10.0],
            bounds_max: [10.0, 10.0, 10.0],
        }
    }

    #[test]
    fn character_capsule_uses_engine_y_up_axis() {
        let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
        input.gravity = 0.0;
        input.bodies.push(body(
            7,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Capsule {
                radius: 0.3,
                half_height: 0.6,
            },
            [0.0, 5.0, 0.0],
        ));
        backend.step_frame(input).unwrap();
        let hit = backend
            .cast_ray(1, None, [0.0, 7.0, 0.0], [0.0, -1.0, 0.0], 4.0)
            .unwrap()
            .unwrap();
        assert!((hit.position[1] - 5.9).abs() < 0.002, "{hit:?}");
    }

    #[test]
    fn thin_apron_rays_always_select_top_support_above_overlapping_ground() {
        let mut backend = BulletPacketPhysicsBackend::new(16, 16).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
        input.bodies.push(body(
            1,
            PhysicsBodyKindDto::Static,
            CollisionShapeDto::Box {
                half_extents: [200.0, 0.25, 200.0],
            },
            [0.0, -0.25, 0.0],
        ));
        input.bodies.push(body(
            2,
            PhysicsBodyKindDto::Static,
            CollisionShapeDto::Box {
                half_extents: [95.0, 0.025, 150.0],
            },
            [0.0, 0.025, 0.0],
        ));
        backend.step_frame(input).unwrap();
        for tick in 0..100 {
            let x = -40.0 + tick as f32 * 0.003;
            let hit = backend
                .cast_ray(tick, None, [x, 0.6, 60.0], [0.0, -1.0, 0.0], 1.0)
                .unwrap()
                .unwrap();
            assert_eq!(hit.entity, 2);
            assert_eq!(hit.normal, [0.0, 1.0, 0.0]);
            assert!((hit.position[1] - 0.05).abs() < 0.00001);
        }
    }

    #[test]
    fn deforming_vehicle_collision_recreates_body_and_preserves_motion() {
        let mut backend = BulletPacketPhysicsBackend::new(16, 8).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
        input.gravity = 0.0;
        let mut car = body(
            7,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Box {
                half_extents: [1.0; 3],
            },
            [0.0, 5.0, 0.0],
        );
        car.mass_properties = Some(PhysicsMassProperties {
            mass: 1000.0,
            center_of_mass: [0.3, 0.2, 0.1],
            inertia_diagonal: [200.0, 300.0, 400.0],
        });
        car.convex_hulls = vec![vec![
            [-1.0, -1.0, -1.0],
            [1.0, -1.0, -1.0],
            [-1.0, 1.0, -1.0],
            [1.0, 1.0, -1.0],
            [-1.0, -1.0, 1.0],
            [1.0, -1.0, 1.0],
            [-1.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
        ]];
        car.angular_velocity = [0.0, 0.0, 2.0];
        car.linear_velocity = [3.0, 1.0, 0.0];
        car.angular_damping = Some(0.0);
        car.linear_damping = Some(0.0);
        input.bodies.push(car);
        for tick in 0..8 {
            input.fixed_tick = tick + 1;
            let out = backend.step_frame(input.clone()).unwrap();
            let v = out.velocity_updates.iter().find(|v| v.entity == 7).unwrap();
            let pose = out.pose_updates.iter().find(|p| p.entity == 7).unwrap();
            assert!((v.angular_velocity[2] - 2.0).abs() < 0.01);
            input.bodies[0].position = pose.position;
            input.bodies[0].rotation = pose.rotation;
            input.bodies[0].linear_velocity = v.linear_velocity;
            input.bodies[0].angular_velocity = v.angular_velocity;
            input.bodies[0].convex_hulls[0][0][0] += 0.01;
        }
        assert_eq!(backend.compound_files.len(), 0);
    }

    #[test]
    fn compound_vehicle_preserves_more_than_sixteen_distinct_hulls() {
        let mut backend = BulletPacketPhysicsBackend::new(16, 8).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
        input.gravity = 0.0;
        let mut car = body(
            7,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Box {
                half_extents: [1.0; 3],
            },
            [0.0, 5.0, 0.0],
        );
        car.convex_hulls = (0..22)
            .map(|i| {
                let x = i as f32 * 3.0;
                vec![
                    [x - 0.5, -0.5, -0.5],
                    [x + 0.5, -0.5, -0.5],
                    [x - 0.5, 0.5, -0.5],
                    [x + 0.5, 0.5, -0.5],
                    [x - 0.5, -0.5, 0.5],
                    [x + 0.5, -0.5, 0.5],
                    [x - 0.5, 0.5, 0.5],
                    [x + 0.5, 0.5, 0.5],
                ]
            })
            .collect();
        input.bodies.push(car);
        backend.step_frame(input).unwrap();
        for x in [0.0, 30.0, 63.0] {
            let hits = backend
                .client
                .ray_test([x, 5.0, 2.0], [x, 5.0, -2.0], None, None)
                .unwrap();
            assert!(
                hits.iter()
                    .any(|h| h.object_unique_id == backend.records[&7].body_id),
                "missing hull at {x}"
            );
        }
    }

    #[test]
    fn off_centre_impulse_uses_authored_inertia_and_rotates_body() {
        let mut backend = BulletPacketPhysicsBackend::new(16, 8).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
        input.gravity = 0.0;
        let mut car = body(
            7,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Box {
                half_extents: [1.0; 3],
            },
            [0.0, 5.0, 0.0],
        );
        car.mass_properties = Some(PhysicsMassProperties {
            mass: 1000.0,
            center_of_mass: [0.0; 3],
            inertia_diagonal: [200.0, 300.0, 400.0],
        });
        car.linear_damping = Some(0.0);
        car.angular_damping = Some(0.0);
        input.bodies.push(car);
        input.commands.push(PhysicsCommandDto {
            seq: 1,
            kind: PhysicsCommandKindDto::ApplyImpulse {
                entity: 7,
                impulse: [0.0, 1000.0, 0.0],
                point: [1.0, 5.0, 0.0],
            },
        });
        let output = backend.step_frame(input).unwrap();
        let v = output
            .velocity_updates
            .iter()
            .find(|v| v.entity == 7)
            .unwrap();
        assert!((v.linear_velocity[1] - 1.0).abs() < 0.01, "{v:?}");
        assert!((v.angular_velocity[2] - 2.5).abs() < 0.05, "{v:?}");
        let info = backend
            .client
            .get_dynamics_info(backend.records[&7].body_id, -1)
            .unwrap();
        assert!(
            (info.local_inertia_diagonal[2] - 400.0).abs() < 0.1,
            "{info:?}"
        );
    }

    #[test]
    fn centre_of_mass_does_not_shift_visual_origin() {
        let mut backend = BulletPacketPhysicsBackend::new(16, 8).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
        input.gravity = 0.0;
        let mut car = body(
            7,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Box {
                half_extents: [1.0; 3],
            },
            [10.0, 5.0, 3.0],
        );
        car.rotation = [
            0.0,
            std::f32::consts::FRAC_1_SQRT_2,
            0.0,
            std::f32::consts::FRAC_1_SQRT_2,
        ];
        car.mass_properties = Some(PhysicsMassProperties {
            mass: 1000.0,
            center_of_mass: [0.4, 0.2, -0.1],
            inertia_diagonal: [200.0, 300.0, 400.0],
        });
        input.bodies.push(car);
        let output = backend.step_frame(input).unwrap();
        let pose = output.pose_updates.iter().find(|p| p.entity == 7).unwrap();
        for (actual, expected) in pose.position.into_iter().zip([10.0, 5.0, 3.0]) {
            assert!((actual - expected).abs() < 0.001, "{pose:?}");
        }
    }

    #[test]
    fn rollover_contact_does_not_cancel_angular_momentum_at_roof_orientation() {
        let mut backend = BulletPacketPhysicsBackend::new(16, 8).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
        input.gravity = 9.81;
        input.bodies.push(body(
            1,
            PhysicsBodyKindDto::Static,
            CollisionShapeDto::Box {
                half_extents: [30.0, 0.5, 30.0],
            },
            [0.0, -0.5, 0.0],
        ));
        let mut car = body(
            7,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Box {
                half_extents: [1.0, 0.7, 2.0],
            },
            [0.0, 1.35, 0.0],
        );
        car.mass_properties = Some(PhysicsMassProperties {
            mass: 1800.0,
            center_of_mass: [0.0; 3],
            inertia_diagonal: [3200.0, 5400.0, 1200.0],
        });
        car.angular_velocity = [0.0, 0.0, 12.0];
        car.linear_velocity = [4.0, 0.0, 0.0];
        car.angular_damping = Some(0.05);
        car.material.friction = 0.12;
        car.material.restitution = 0.05;
        input.bodies.push(car);
        let mut saw_roof = false;
        let mut roof_spin = 0.0_f32;
        let mut passed_roof = false;
        for tick in 0..360 {
            input.fixed_tick = tick + 1;
            input.frame_index = tick + 1;
            let out = backend.step_frame(input.clone()).unwrap();
            let pose = out.pose_updates.iter().find(|p| p.entity == 7).unwrap();
            let velocity = out.velocity_updates.iter().find(|v| v.entity == 7).unwrap();
            let q = pose.rotation;
            let up = 1.0 - 2.0 * (q[0] * q[0] + q[2] * q[2]);
            if up < -0.9 {
                saw_roof = true;
                roof_spin = roof_spin.max(velocity.angular_velocity[2].abs());
            }
            if saw_roof && up > 0.0 {
                passed_roof = true;
            }
        }
        assert!(
            saw_roof && roof_spin > 1.0 && passed_roof,
            "roof={saw_roof} roof_spin={roof_spin} continued={passed_roof}"
        );
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
    fn ray_query_without_ignore_does_not_suppress_entity_matching_sequence() {
        let mut backend = BulletPacketPhysicsBackend::new(128, 32).expect("Bullet direct world");
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
        input.bodies.push(body(
            1,
            PhysicsBodyKindDto::Static,
            CollisionShapeDto::Box {
                half_extents: [0.5, 0.5, 0.5],
            },
            [0.0, 0.0, 0.0],
        ));
        input.queries.push(PhysicsQuery {
            seq: 1,
            ignore_entity: None,
            kind: PhysicsQueryKindDto::Ray {
                origin: [0.0, 0.0, 2.0],
                dir: [0.0, 0.0, -1.0],
                max_t: 5.0,
            },
        });

        let output = backend.step_frame(input).expect("Bullet ray query");
        assert_eq!(output.query_hits.len(), 1);
        assert_eq!(output.query_hits[0].entity, 1);
    }

    #[test]
    fn ray_query_respects_participates_in_queries_flag() {
        let mut backend = BulletPacketPhysicsBackend::new(128, 32).expect("Bullet direct world");
        let mut hidden = body(
            7,
            PhysicsBodyKindDto::Static,
            CollisionShapeDto::Box {
                half_extents: [0.5, 0.5, 0.5],
            },
            [0.0, 0.0, 0.0],
        );
        hidden.flags.participates_in_queries = false;

        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
        input.bodies.push(hidden);
        input.queries.push(PhysicsQuery {
            seq: 99,
            ignore_entity: None,
            kind: PhysicsQueryKindDto::Ray {
                origin: [0.0, 0.0, 2.0],
                dir: [0.0, 0.0, -1.0],
                max_t: 5.0,
            },
        });

        let output = backend
            .step_frame(input)
            .expect("Bullet filtered ray query");
        assert!(output.query_hits.is_empty());
    }

    #[test]
    fn set_angular_velocity_command_updates_native_body() {
        let mut backend = BulletPacketPhysicsBackend::new(128, 32).expect("Bullet direct world");
        let mut initial = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
        initial.bodies.push(body(
            5,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Sphere { radius: 0.5 },
            [0.0, 2.0, 0.0],
        ));
        backend.step_frame(initial).expect("initial Bullet step");

        let mut next = PhysicsFrameInput::empty(2, 2, 1.0 / 120.0);
        next.bodies.push(body(
            5,
            PhysicsBodyKindDto::Dynamic,
            CollisionShapeDto::Sphere { radius: 0.5 },
            [0.0, 2.0, 0.0],
        ));
        next.commands.push(PhysicsCommandDto {
            seq: 1,
            kind: PhysicsCommandKindDto::SetAngularVelocity {
                entity: 5,
                velocity: [3.0, 4.0, 5.0],
            },
        });

        let output = backend.step_frame(next).expect("angular velocity command");
        let velocity = output
            .velocity_updates
            .iter()
            .find(|update| update.entity == 5)
            .expect("dynamic body velocity update");
        assert!((velocity.angular_velocity[0] - 3.0).abs() < 0.1);
        assert!((velocity.angular_velocity[1] - 4.0).abs() < 0.1);
        assert!((velocity.angular_velocity[2] - 5.0).abs() < 0.1);
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
