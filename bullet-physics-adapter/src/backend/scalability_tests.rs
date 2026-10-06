use super::*;

fn body(id: u64, kind: PhysicsBodyKindDto, position: PhysicsVec3) -> PhysicsFrameBodySnapshot {
    tests::body(
        id,
        kind,
        CollisionShapeDto::Sphere { radius: 0.5 },
        position,
    )
}

fn ray(seq: u64, ignore_entity: Option<u64>, origin: PhysicsVec3) -> PhysicsQuery {
    PhysicsQuery {
        seq,
        ignore_entity,
        kind: PhysicsQueryKindDto::Ray {
            origin,
            dir: [0.0, -1.0, 0.0],
            max_t: 10.0,
        },
    }
}

#[test]
fn native_batches_keep_all_rays_and_input_order_across_chunk_boundaries() {
    let mut backend = BulletPacketPhysicsBackend::with_settings(8, 1024, 64, 1, false).unwrap();
    let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    frame.gravity = 0.0;
    frame.bodies = vec![body(1, PhysicsBodyKindDto::Static, [0.0; 3])];
    frame.queries = (0..600)
        .map(|i| {
            ray(
                (600 - i) / 2,
                None,
                [if i % 2 == 0 { 0.0 } else { 8.0 }, 4.0, 0.0],
            )
        })
        .collect();
    let out = backend.step_frame(frame.clone()).unwrap();
    assert_eq!(out.query_hits.len(), 300);
    let expected: Vec<_> = frame.queries.iter().step_by(2).map(|q| q.seq).collect();
    assert_eq!(
        out.query_hits.iter().map(|h| h.seq).collect::<Vec<_>>(),
        expected
    );
    assert!(out
        .query_hits
        .iter()
        .all(|h| h.entity == 1 && (h.distance - 3.5).abs() < 1e-4));
    assert_eq!(backend.metrics.query_batches, 10);
}

#[test]
fn mixed_ignore_groups_restore_filters_and_preserve_order() {
    let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
    let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    frame.gravity = 0.0;
    frame.bodies = vec![
        body(1, PhysicsBodyKindDto::Static, [0.0, 2.0, 0.0]),
        body(2, PhysicsBodyKindDto::Static, [0.0; 3]),
    ];
    frame.queries = vec![
        ray(9, Some(1), [0.0, 5.0, 0.0]),
        ray(3, None, [0.0, 5.0, 0.0]),
        ray(9, Some(2), [0.0, 5.0, 0.0]),
        ray(1, Some(999), [0.0, 5.0, 0.0]),
    ];
    let out = backend.step_frame(frame).unwrap();
    assert_eq!(
        out.query_hits
            .iter()
            .map(|h| (h.seq, h.entity))
            .collect::<Vec<_>>(),
        vec![(9, 2), (3, 1), (9, 1), (1, 1)]
    );
    assert_eq!(
        backend
            .cast_ray(1, None, [0.0, 5.0, 0.0], [0.0, -1.0, 0.0], 10.0)
            .unwrap()
            .unwrap()
            .entity,
        1
    );
}

#[test]
fn query_participation_changes_keep_physical_contacts() {
    let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
    let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    frame.gravity = 0.0;
    frame.bodies = vec![
        body(1, PhysicsBodyKindDto::Static, [0.0; 3]),
        body(2, PhysicsBodyKindDto::Dynamic, [0.0, 0.8, 0.0]),
    ];
    frame.bodies[0].flags.participates_in_queries = false;
    frame.queries = vec![ray(1, Some(2), [0.0, 5.0, 0.0])];
    let out = backend.step_frame(frame.clone()).unwrap();
    assert!(out.query_hits.is_empty());
    assert!(out
        .events
        .iter()
        .any(|e| matches!(e, PhysicsEventDto::ContactBegin(_))));
    frame.bodies[0].flags.participates_in_queries = true;
    assert_eq!(backend.step_frame(frame).unwrap().query_hits[0].entity, 1);
}

#[test]
fn exact_box_bvh_matches_brute_force_for_rotations_inside_rays_and_ignores() {
    let mut backend = BulletPacketPhysicsBackend::new(512, 512).unwrap();
    let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    frame.gravity = 0.0;
    for i in 0..256 {
        let mut b = tests::body(
            i + 1,
            PhysicsBodyKindDto::Static,
            CollisionShapeDto::Box {
                half_extents: [0.7, 0.4, 0.6],
            },
            [(i % 16) as f32 * 3.0, 0.0, (i / 16) as f32 * 3.0],
        );
        let q = UnitQuaternion::from_euler_angles(0.2, i as f64 * 0.17, 0.1);
        let q = q.quaternion();
        b.rotation = [q.i as f32, q.j as f32, q.k as f32, q.w as f32];
        frame.bodies.push(b);
    }
    for i in 0..128 {
        let p = frame.bodies[i].position;
        frame.queries.push(ray(
            i as u64,
            (i % 3 == 0).then_some(i as u64 + 1),
            [p[0], if i % 2 == 0 { 4.0 } else { 0.0 }, p[2]],
        ));
    }
    let out = backend.step_frame(frame.clone()).unwrap();
    for query in &frame.queries {
        let PhysicsQueryKindDto::Ray { origin, dir, max_t } = query.kind else {
            unreachable!()
        };
        let expected = frame
            .bodies
            .iter()
            .filter(|b| query.ignore_entity != Some(b.entity))
            .filter_map(|b| {
                let CollisionShapeDto::Box { half_extents } = b.shape else {
                    unreachable!()
                };
                ray_authored_box(origin, dir, max_t, b.position, b.rotation, half_extents)
                    .map(|(distance, normal)| (b.entity, distance, normal))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1).then(a.0.cmp(&b.0)));
        let actual = out.query_hits.iter().find(|h| h.seq == query.seq);
        match (expected, actual) {
            (Some((id, distance, normal)), Some(hit)) => {
                assert_eq!(hit.entity, id);
                assert!((hit.distance - distance).abs() < 1e-5);
                assert_eq!(hit.normal, normal);
            }
            (None, None) => {}
            _ => panic!("BVH mismatch for query {}", query.seq),
        }
    }
    assert!(backend.metrics.exact_box_tests < 128 * 16);
    backend.step_frame(frame).unwrap();
    assert_eq!(backend.metrics.query_index_rebuilds, 0);
    assert_eq!(backend.metrics.native_pose_updates, 0);
    assert_eq!(backend.metrics.body_state_reads, 0);
}

#[test]
fn exact_box_index_tracks_pose_commands_removal_and_participation() {
    let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
    let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    frame.bodies.push(tests::body(
        1,
        PhysicsBodyKindDto::Static,
        CollisionShapeDto::Box {
            half_extents: [0.5; 3],
        },
        [0.0; 3],
    ));
    frame.queries.push(ray(1, None, [0.0, 5.0, 0.0]));
    assert_eq!(
        backend.step_frame(frame.clone()).unwrap().query_hits.len(),
        1
    );
    frame.commands.push(PhysicsCommand {
        seq: 1,
        kind: PhysicsCommandKindDto::SetBodyPose {
            entity: 1,
            position: [8.0, 0.0, 0.0],
            rotation: [0.0, 0.0, 0.0, 1.0],
        },
    });
    assert!(backend
        .step_frame(frame.clone())
        .unwrap()
        .query_hits
        .is_empty());
    frame.commands.clear();
    frame.bodies[0].flags.participates_in_queries = false;
    assert!(backend
        .step_frame(frame.clone())
        .unwrap()
        .query_hits
        .is_empty());
    frame.bodies[0].flags.participates_in_queries = true;
    assert_eq!(
        backend.step_frame(frame.clone()).unwrap().query_hits.len(),
        1
    );
    frame.bodies.clear();
    assert!(backend.step_frame(frame).unwrap().query_hits.is_empty());
}

#[test]
fn full_capacity_replacement_retires_old_bodies_first() {
    let mut backend = BulletPacketPhysicsBackend::new(2, 8).unwrap();
    let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    frame.bodies = vec![
        body(1, PhysicsBodyKindDto::Static, [0.0; 3]),
        body(2, PhysicsBodyKindDto::Static, [2.0, 0.0, 0.0]),
    ];
    backend.step_frame(frame.clone()).unwrap();
    frame.bodies[0].entity = 3;
    frame.bodies[1].entity = 4;
    let out = backend.step_frame(frame).unwrap();
    assert_eq!(out.report.active_bodies, 2);
    assert!(!backend.records.contains_key(&1));
    assert!(backend.records.contains_key(&4));
}

#[test]
fn invalid_limits_and_duplicate_entities_fail_before_world_changes() {
    let mut backend = BulletPacketPhysicsBackend::new(1, 1).unwrap();
    let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    frame
        .bodies
        .push(body(1, PhysicsBodyKindDto::Static, [0.0; 3]));
    backend.step_frame(frame.clone()).unwrap();
    frame
        .bodies
        .push(body(2, PhysicsBodyKindDto::Static, [2.0, 0.0, 0.0]));
    assert!(backend.step_frame(frame.clone()).is_err());
    assert!(backend.records.contains_key(&1));
    assert!(!backend.records.contains_key(&2));
    frame.bodies[1].entity = 1;
    assert!(backend
        .step_frame(frame.clone())
        .unwrap_err()
        .contains("duplicate"));
    frame.bodies.clear();
    frame.queries = vec![ray(1, None, [0.0; 3]), ray(2, None, [0.0; 3])];
    assert!(backend
        .step_frame(frame)
        .unwrap_err()
        .contains("query limit"));
    assert!(backend.records.contains_key(&1));
}

#[test]
fn dynamic_feedback_uses_one_native_read_per_phase() {
    let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
    let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    frame
        .bodies
        .push(body(1, PhysicsBodyKindDto::Dynamic, [0.0, 10.0, 0.0]));
    frame.bodies[0].flags.casts_contacts = false;
    backend.step_frame(frame.clone()).unwrap();
    assert_eq!(backend.metrics.body_state_reads, 1);
    frame.bodies[0].flags.casts_contacts = true;
    backend.step_frame(frame).unwrap();
    assert_eq!(backend.metrics.body_state_reads, 2);
}

#[test]
fn native_wrapper_rejects_batches_that_would_be_silently_truncated() {
    let mut backend = BulletPacketPhysicsBackend::new(8, 512).unwrap();
    let from = vec![[0.0; 3]; 257];
    let to = vec![[1.0; 3]; 257];
    assert!(backend
        .client
        .ray_test_batch(&rsbullet_core::RayTestBatchOptions {
            ray_from_positions: &from,
            ray_to_positions: &to,
            ..Default::default()
        })
        .is_err());
}

#[test]
fn explicit_filter_survives_joint_removal_and_body_recreation() {
    let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
    let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    frame.gravity = 0.0;
    frame.bodies = vec![
        body(1, PhysicsBodyKindDto::Static, [0.0; 3]),
        body(2, PhysicsBodyKindDto::Dynamic, [0.2, 0.0, 0.0]),
    ];
    frame.disabled_collision_pairs = vec![[1, 2]];
    frame.joints.push(PhysicsJointSnapshot {
        id: 1,
        parent: 1,
        child: 2,
        parent_anchor: [0.2, 0.0, 0.0],
        child_anchor: [0.0; 3],
        parent_frame_rotation: [0.0, 0.0, 0.0, 1.0],
        child_frame_rotation: [0.0, 0.0, 0.0, 1.0],
        max_force: 1000.0,
        cone_limit: 1.5,
        angular_stiffness: 0.0,
        angular_damping: 0.0,
        collision_family: 1,
    });
    backend.step_frame(frame.clone()).unwrap();
    frame.joints.clear();
    let out = backend.step_frame(frame.clone()).unwrap();
    assert!(out.events.iter().all(|e| !matches!(
        e,
        PhysicsEventDto::ContactBegin(_) | PhysicsEventDto::ContactPersist(_)
    )));
    assert_eq!(backend.metrics.filter_pair_updates, 0);
    frame.bodies[1].shape = CollisionShapeDto::Sphere { radius: 0.6 };
    let out = backend.step_frame(frame).unwrap();
    assert!(out.events.iter().all(|e| !matches!(
        e,
        PhysicsEventDto::ContactBegin(_) | PhysicsEventDto::ContactPersist(_)
    )));
    assert_eq!(backend.metrics.filter_pair_updates, 2);
}

#[test]
fn persistent_mesh_reuses_geometry_and_only_updates_changed_transform() {
    let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
    let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    frame.colliders.push(PhysicsFrameColliderSnapshot {
        entity: 10,
        collider: PhysicsColliderDto::Mesh(MeshColliderDto {
            vertices: vec![[-2.0, 0.0, -2.0], [2.0, 0.0, -2.0], [0.0, 0.0, 2.0]],
            triangles: vec![[0, 2, 1]],
            material_indices: vec![],
        }),
        flags: PhysicsBodyFlags {
            casts_contacts: true,
            participates_in_queries: true,
            ..Default::default()
        },
        material: PhysicsMaterialDto {
            friction: 0.6,
            restitution: 0.0,
            density: 100.0,
        },
        position: [0.0; 3],
        rotation: [0.0, 0.0, 0.0, 1.0],
        bounds_min: [-2.0, 0.0, -2.0],
        bounds_max: [2.0, 0.0, 2.0],
    });
    frame.queries.push(ray(1, None, [0.0, 4.0, 0.0]));
    assert_eq!(
        backend.step_frame(frame.clone()).unwrap().query_hits[0].entity,
        10
    );
    let shape_id = backend.records[&10].collision_shape_id;
    let vertex_ptr = match &backend.records[&10].shape {
        ShapeSource::Authored(PhysicsColliderDto::Mesh(mesh)) => mesh.vertices.as_ptr(),
        _ => unreachable!(),
    };
    backend.step_frame(frame.clone()).unwrap();
    assert_eq!(backend.metrics.native_pose_updates, 0);
    assert_eq!(backend.metrics.body_state_reads, 0);
    assert_eq!(backend.records[&10].collision_shape_id, shape_id);
    match &backend.records[&10].shape {
        ShapeSource::Authored(PhysicsColliderDto::Mesh(mesh)) => {
            assert_eq!(mesh.vertices.as_ptr(), vertex_ptr)
        }
        _ => unreachable!(),
    }
    frame.colliders[0].position = [8.0, 0.0, 0.0];
    assert!(backend
        .step_frame(frame.clone())
        .unwrap()
        .query_hits
        .is_empty());
    assert_eq!(backend.metrics.native_pose_updates, 1);
    frame.colliders.clear();
    frame.queries = vec![ray(2, None, [8.0, 4.0, 0.0])];
    assert_eq!(backend.step_frame(frame).unwrap().query_hits[0].entity, 10);
}
