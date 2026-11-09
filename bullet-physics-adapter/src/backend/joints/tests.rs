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
        angular_limits: None,
        motor: None,
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
        angular_limits: None,
        motor: None,
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

#[test]
fn physical_motor_tracks_targets_without_recreating_the_native_socket() {
    let mut backend = BulletPacketPhysicsBackend::new(32, 32).unwrap();
    let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
    input.gravity = 0.0;
    let mut limb = body(2, [0.0, 3.0, 0.0], PhysicsBodyKind::Dynamic);
    limb.angular_damping = Some(0.0);
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
        cone_limit: 1.5,
        angular_stiffness: 8.0,
        angular_damping: 0.0,
        collision_family: 7,
        angular_limits: Some(PhysicsAngularLimits {
            swing_x: [0.0, 1.0],
            swing_z: [-0.03, 0.03],
            twist: [-0.03, 0.03],
            stiffness: 400.0,
            damping: 0.0,
            max_torque: 100.0,
        }),
        motor: None,
    });
    let mut socket = None;
    let mut angle = 0.0;
    for frame in 1..=360 {
        input.fixed_tick = frame;
        input.frame_index = frame;
        let target = (frame as f32 / 120.0).min(1.0) * 0.6;
        input.joints[0].motor = Some(PhysicsJointMotor {
            world_drive: None,
            target_rotation: [(target * 0.5).sin(), 0.0, 0.0, (target * 0.5).cos()],
            stiffness: 8.0,
            damping: 0.0,
            max_torque: 3.0,
            feed_forward_torque: [0.0; 3],
        });
        let output = backend.step_frame(input.clone()).unwrap();
        let id = backend.joints[&9].native_id;
        if let Some(previous) = socket {
            assert_eq!(id, previous);
        } else {
            socket = Some(id);
        }
        let p = output.pose_updates.iter().find(|p| p.entity == 2).unwrap();
        angle = physics_pose(p.position, p.rotation)
            .rotation
            .scaled_axis()
            .x;
        assert!(angle.is_finite());
    }
    assert!(
        (angle - 0.6).abs() < 0.03,
        "physical drive failed target: {angle}"
    );
    input.joints[0].motor = None;
    backend.step_frame(input).unwrap();
    assert_eq!(backend.joints[&9].native_id, socket.unwrap());
    assert!(backend.joints[&9].definition.motor.is_none());
}

#[test]
fn dynamic_joint_motor_conserves_total_angular_momentum() {
    let mut backend = BulletPacketPhysicsBackend::new(32, 32).unwrap();
    let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
    input.gravity = 0.0;
    let mut a = body(1, [0.0, 3.0, 0.0], PhysicsBodyKind::Dynamic);
    a.angular_damping = Some(0.0);
    let mut b = a.clone();
    b.entity = 2;
    input.bodies = vec![a, b];
    input.joints.push(PhysicsJointSnapshot {
        id: 9,
        parent: 1,
        child: 2,
        parent_anchor: [0.0; 3],
        child_anchor: [0.0; 3],
        parent_frame_rotation: [0.0, 0.0, 0.0, 1.0],
        child_frame_rotation: [0.0, 0.0, 0.0, 1.0],
        max_force: 30000.0,
        cone_limit: 1.5,
        angular_stiffness: 0.0,
        angular_damping: 0.0,
        collision_family: 7,
        angular_limits: None,
        motor: Some(PhysicsJointMotor {
            world_drive: None,
            target_rotation: [0.0, 0.0, 0.2955202, 0.9553365],
            stiffness: 8.0,
            damping: 0.0,
            max_torque: 3.0,
            feed_forward_torque: [0.0; 3],
        }),
    });
    let mut relative = 0.0;
    for frame in 1..=240 {
        input.fixed_tick = frame;
        input.frame_index = frame;
        let output = backend.step_frame(input.clone()).unwrap();
        let a = output
            .velocity_updates
            .iter()
            .find(|p| p.entity == 1)
            .unwrap();
        let b = output
            .velocity_updates
            .iter()
            .find(|p| p.entity == 2)
            .unwrap();
        for i in 0..3 {
            assert!((a.angular_velocity[i] + b.angular_velocity[i]).abs() < 1e-4);
        }
        let pa = output.pose_updates.iter().find(|p| p.entity == 1).unwrap();
        let pb = output.pose_updates.iter().find(|p| p.entity == 2).unwrap();
        relative = (physics_pose(pa.position, pa.rotation).rotation.inverse()
            * physics_pose(pb.position, pb.rotation).rotation)
            .angle();
    }
    assert!(
        (relative - 0.6).abs() < 0.03,
        "internal drive did not articulate: {relative}"
    );
}

#[test]
fn offset_loaded_joint_holds_weight_in_rotated_native_frames() {
    for hz in [60.0, 120.0, 240.0] {
        let mut backend = BulletPacketPhysicsBackend::new(16, 16).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / hz);
        input.gravity = 9.81;
        let root = body(1, [0.0, 3.0, 0.0], PhysicsBodyKind::Static);
        let rotation = UnitQuaternion::from_euler_angles(0.2, 1.3, -0.15);
        let mut limb = body(2, [0.2, 3.0, 0.0], PhysicsBodyKind::Dynamic);
        limb.rotation = std::array::from_fn(|i| rotation.quaternion().coords[i] as f32);
        limb.mass_properties = Some(PhysicsMassProperties {
            mass: 2.0,
            center_of_mass: [0.0; 3],
            inertia_diagonal: [0.02, 0.03, 0.04],
        });
        limb.angular_damping = Some(0.0);
        limb.linear_damping = Some(0.0);
        input.bodies = vec![root, limb];
        input.joints.push(PhysicsJointSnapshot {
            id: 1,
            parent: 1,
            child: 2,
            parent_anchor: [0.0; 3],
            child_anchor: <[f64; 3]>::from(rotation.inverse() * Vector3::new(-0.2, 0.0, 0.0))
                .map(|v| v as f32),
            parent_frame_rotation: [0.0, 0.0, 0.0, 1.0],
            child_frame_rotation: std::array::from_fn(|i| {
                rotation.inverse().quaternion().coords[i] as f32
            }),
            max_force: 30000.0,
            cone_limit: std::f32::consts::PI,
            angular_stiffness: 0.0,
            angular_damping: 0.0,
            collision_family: 1,
            angular_limits: None,
            motor: Some(PhysicsJointMotor {
                world_drive: None,
                target_rotation: [0.0, 0.0, 0.0, 1.0],
                feed_forward_torque: [0.0, 0.0, 2.0 * 9.81 * 0.2],
                stiffness: 1000.0,
                damping: 0.1,
                max_torque: 200.0,
            }),
        });
        let mut drift = 0.0f64;
        let mut angle = 0.0f64;
        for frame in 1..=(hz as u64 * 2) {
            input.fixed_tick = frame;
            input.frame_index = frame;
            let out = backend.step_frame(input.clone()).unwrap();
            let p = out.pose_updates.iter().find(|p| p.entity == 2).unwrap();
            drift = drift.max(
                (Vector3::from(p.position.map(|v| v as f64)) - Vector3::new(0.2, 3.0, 0.0)).norm(),
            );
            angle = angle
                .max((physics_pose([0.0; 3], p.rotation).rotation * rotation.inverse()).angle());
        }
        assert!(
            drift < 0.003 && angle < 0.015,
            "native weight hold hz={hz} drift={drift} angle={angle}"
        );
    }
}

#[test]
fn parent_world_drive_tracks_without_external_angular_momentum() {
    for hz in [60.0, 120.0, 240.0] {
        let mut backend = BulletPacketPhysicsBackend::new(32, 32).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / hz);
        input.gravity = 0.0;
        let mut a = body(1, [0.0, 3.0, 0.0], PhysicsBodyKind::Dynamic);
        a.angular_damping = Some(0.0);
        let mut b = a.clone();
        b.entity = 2;
        input.bodies = vec![a, b];
        let angle = 0.4f32;
        input.joints.push(PhysicsJointSnapshot {
            id: 1,
            parent: 1,
            child: 2,
            parent_anchor: [0.0; 3],
            child_anchor: [0.0; 3],
            parent_frame_rotation: [0.0, 0.0, 0.0, 1.0],
            child_frame_rotation: [0.0, 0.0, 0.0, 1.0],
            max_force: 30000.0,
            cone_limit: std::f32::consts::PI,
            angular_stiffness: 0.0,
            angular_damping: 0.0,
            collision_family: 1,
            angular_limits: None,
            motor: Some(PhysicsJointMotor {
                world_drive: Some(newviso_physics_api::PhysicsWorldAngularDrive {
                    body: newviso_physics_api::PhysicsJointBody::Parent,
                    target_rotation: [(angle * 0.5).sin(), 0.0, 0.0, (angle * 0.5).cos()],
                    stiffness: 8.0,
                    damping: 0.0,
                }),
                target_rotation: [0.0, 0.0, 0.0, 1.0],
                feed_forward_torque: [0.0; 3],
                stiffness: 0.0,
                damping: 0.0,
                max_torque: 2.0,
            }),
        });
        let mut parent_angle = 0.0;
        for frame in 1..=(hz as u64 * 2) {
            input.frame_index = frame;
            input.fixed_tick = frame;
            let out = backend.step_frame(input.clone()).unwrap();
            let va = out.velocity_updates.iter().find(|v| v.entity == 1).unwrap();
            let vb = out.velocity_updates.iter().find(|v| v.entity == 2).unwrap();
            for axis in 0..3 {
                assert!((va.angular_velocity[axis] + vb.angular_velocity[axis]).abs() < 0.0002);
            }
            let p = out.pose_updates.iter().find(|p| p.entity == 1).unwrap();
            parent_angle = physics_pose(p.position, p.rotation)
                .rotation
                .scaled_axis()
                .x;
        }
        assert!(
            (parent_angle - angle as f64).abs() < 0.015,
            "parent drive hz={hz} angle={parent_angle}"
        );
    }
}
#[test]
fn coupled_motor_chain_dissipates_an_impulse_without_self_excitation() {
    for hz in [60.0, 120.0] {
        let mut backend = BulletPacketPhysicsBackend::new(32, 32).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / hz);
        input.gravity = 0.0;
        for id in 1..=5 {
            let mut b = body(
                id,
                [0.0, 4.0 - id as f32 * 0.5, 0.0],
                PhysicsBodyKind::Dynamic,
            );
            b.angular_damping = Some(0.0);
            b.linear_damping = Some(0.0);
            input.bodies.push(b);
            if id > 1 {
                input.joints.push(PhysicsJointSnapshot {
                    id,
                    parent: id - 1,
                    child: id,
                    parent_anchor: [0.0, -0.25, 0.0],
                    child_anchor: [0.0, 0.25, 0.0],
                    parent_frame_rotation: [0.0, 0.0, 0.0, 1.0],
                    child_frame_rotation: [0.0, 0.0, 0.0, 1.0],
                    max_force: 30000.0,
                    cone_limit: 0.3,
                    angular_stiffness: 400.0,
                    angular_damping: 0.08,
                    collision_family: 7,
                    angular_limits: None,
                    motor: Some(PhysicsJointMotor {
                        world_drive: None,
                        target_rotation: [0.0, 0.0, 0.0, 1.0],
                        stiffness: 80.0,
                        damping: 0.1,
                        max_torque: 10.0,
                        feed_forward_torque: [0.0; 3],
                    }),
                });
            }
        }
        input.commands.push(PhysicsCommand {
            seq: 1,
            kind: PhysicsCommandKind::SetAngularVelocity {
                entity: 5,
                velocity: [0.0, 0.0, 15.0],
            },
        });
        let mass = 4.0 / 3.0 * std::f32::consts::PI * 0.15f32.powi(3) * 100.0;
        let inertia = 0.4 * mass * 0.15f32.powi(2);
        let budget = 0.5 * inertia * 15.0f32.powi(2);
        let mut maximum = 0.0f32;
        for frame in 1..=360 {
            input.fixed_tick = frame;
            input.frame_index = frame;
            let output = backend.step_frame(input.clone()).unwrap();
            input.commands.clear();
            let energy = output
                .velocity_updates
                .iter()
                .map(|v| {
                    0.5 * (mass * v.linear_velocity.iter().map(|x| x * x).sum::<f32>()
                        + inertia * v.angular_velocity.iter().map(|x| x * x).sum::<f32>())
                })
                .sum::<f32>();
            maximum = maximum.max(energy);
            assert!(
                energy.is_finite() && energy < budget * 2.5,
                "coupled chain gained energy hz={hz} maximum={maximum} budget={budget}"
            );
        }
    }
}

#[test]
fn flat_sole_reports_the_full_native_support_patch() {
    let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
    let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
    let mut ground = body(1, [0.0, -0.1, 0.0], PhysicsBodyKind::Static);
    ground.shape = CollisionShape::Box {
        half_extents: [2.0, 0.1, 2.0],
    };
    let mut sole = body(2, [0.0, 0.031, 0.0], PhysicsBodyKind::Dynamic);
    sole.shape = CollisionShape::Box {
        half_extents: [0.15, 0.03, 0.2],
    };
    input.bodies = vec![ground, sole];
    let mut last = PhysicsFrameOutput::default();
    for frame in 1..=60 {
        input.fixed_tick = frame;
        input.frame_index = frame;
        last = backend.step_frame(input.clone()).unwrap();
    }
    let patch = last
        .contact_manifolds
        .iter()
        .find(|m| m.a == 1 && m.b == 2)
        .expect("native support manifold");
    assert!(
        patch.points.len() >= 3,
        "one representative point cannot describe a sole"
    );
    let extent = |i| {
        patch
            .points
            .iter()
            .map(|p| p.position[i])
            .fold(f32::NEG_INFINITY, f32::max)
            - patch
                .points
                .iter()
                .map(|p| p.position[i])
                .fold(f32::INFINITY, f32::min)
    };
    assert!(extent(0) > 0.25 && extent(2) > 0.3);
    assert!(patch.points.iter().all(|p| p.normal[1] < -0.9));
    assert!(patch.points.iter().map(|p| p.impulse).sum::<f32>() > 0.0);
}
#[test]
fn finite_friction_sole_does_not_turn_a_loaded_drive_into_a_launch_impulse() {
    let mut backend = BulletPacketPhysicsBackend::new(16, 16).unwrap();
    let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
    let mut floor = body(1, [0.0, -0.1, 0.0], PhysicsBodyKind::Static);
    floor.shape = CollisionShape::Box {
        half_extents: [3.0, 0.1, 3.0],
    };
    let mut torso = body(2, [0.0, 1.13, 0.0], PhysicsBodyKind::Dynamic);
    torso.shape = CollisionShape::Box {
        half_extents: [0.16, 0.35, 0.1],
    };
    torso.material.density = 900.0;
    let mut shin = body(3, [0.0, 0.48, 0.0], PhysicsBodyKind::Dynamic);
    shin.shape = CollisionShape::Capsule {
        radius: 0.065,
        half_height: 0.28,
    };
    shin.material.density = 500.0;
    let mut sole = body(4, [0.0, 0.03, 0.0], PhysicsBodyKind::Dynamic);
    sole.shape = CollisionShape::Box {
        half_extents: [0.1, 0.03, 0.19],
    };
    sole.material.density = 300.0;
    input.bodies = vec![floor, torso, shin, sole];
    for (id, parent, child, pa, ca, k, max) in [
        (10, 2, 3, [0.0, -0.3, 0.0], [0.0, 0.35, 0.0], 600.0, 200.0),
        (11, 3, 4, [0.0, -0.35, 0.0], [0.0, 0.1, 0.0], 900.0, 160.0),
    ] {
        input.joints.push(PhysicsJointSnapshot {
            id,
            parent,
            child,
            parent_anchor: pa,
            child_anchor: ca,
            parent_frame_rotation: [0.0, 0.0, 0.0, 1.0],
            child_frame_rotation: [0.0, 0.0, 0.0, 1.0],
            max_force: 30000.0,
            cone_limit: 1.5,
            angular_stiffness: 400.0,
            angular_damping: 0.08,
            collision_family: 7,
            angular_limits: Some(PhysicsAngularLimits {
                swing_x: [-0.6, 0.6],
                swing_z: [-0.4, 0.4],
                twist: [-0.2, 0.2],
                stiffness: 400.0,
                damping: 0.08,
                max_torque: 300.0,
            }),
            motor: Some(PhysicsJointMotor {
                world_drive: None,
                target_rotation: [0.0, 0.0, 0.0, 1.0],
                stiffness: k,
                damping: 0.1,
                max_torque: max,
                feed_forward_torque: [0.0; 3],
            }),
        });
    }
    input.commands.push(PhysicsCommand {
        seq: 1,
        kind: PhysicsCommandKind::SetLinearVelocity {
            entity: 2,
            velocity: [0.3, 0.0, 0.0],
        },
    });
    let mut budget = 0.0;
    let mut peak = 0.0f64;
    let mut spin = 0.0f32;
    for frame in 0..360 {
        input.fixed_tick = frame + 1;
        input.frame_index = frame + 1;
        let output = backend.step_frame(input.clone()).unwrap();
        input.commands.clear();
        let mut energy = 0.0;
        for velocity in output.velocity_updates.iter().filter(|v| v.entity != 1) {
            let p = output
                .pose_updates
                .iter()
                .find(|p| p.entity == velocity.entity)
                .unwrap();
            let native = backend.records[&velocity.entity].body_id;
            let dynamics = backend.client.get_dynamics_info(native, -1).unwrap();
            let w = physics_pose(p.position, p.rotation).rotation.inverse()
                * Vector3::from(velocity.angular_velocity.map(|v| v as f64));
            energy += 0.5
                * dynamics.mass
                * velocity
                    .linear_velocity
                    .iter()
                    .map(|v| (*v as f64).powi(2))
                    .sum::<f64>()
                + 0.5
                    * (0..3)
                        .map(|i| dynamics.local_inertia_diagonal[i] * w[i] * w[i])
                        .sum::<f64>();
            if frame == 0 {
                budget += dynamics.mass * 9.81 * p.position[1] as f64;
            }
            spin = spin.max(
                velocity
                    .angular_velocity
                    .iter()
                    .map(|v| v * v)
                    .sum::<f32>()
                    .sqrt(),
            );
        }
        peak = peak.max(energy);
    }
    assert!(
        peak < budget * 2.0,
        "articulation created energy: peak={peak} gravity_budget={budget} spin={spin}"
    );
    assert!(
        spin < 120.0,
        "a bounded sole drive launched a limb at {spin} rad/s"
    );
}
#[test]
fn native_substeps_preserve_authored_torque_on_rotated_principal_axes() {
    let rotation = UnitQuaternion::from_euler_angles(0.4, 1.1, -0.3);
    let inertia = [1.0, 2.0, 4.0];
    for hz in [60.0, 120.0, 240.0] {
        for axis in 0..3 {
            let mut backend = BulletPacketPhysicsBackend::new(16, 16).unwrap();
            let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / hz);
            input.gravity = 0.0;
            let mut root = body(1, [0.0, 3.0, 0.0], PhysicsBodyKind::Static);
            root.rotation = std::array::from_fn(|i| rotation.quaternion().coords[i] as f32);
            let mut limb = root.clone();
            limb.entity = 2;
            limb.kind = PhysicsBodyKind::Dynamic;
            limb.angular_damping = Some(0.0);
            limb.linear_damping = Some(0.0);
            limb.mass_properties = Some(PhysicsMassProperties {
                mass: 1.0,
                center_of_mass: [0.0; 3],
                inertia_diagonal: inertia.map(|v| v as f32),
            });
            input.bodies = vec![root, limb];
            input.joints.push(PhysicsJointSnapshot {
                id: 1,
                parent: 1,
                child: 2,
                parent_anchor: [0.0; 3],
                child_anchor: [0.0; 3],
                parent_frame_rotation: [0.0, 0.0, 0.0, 1.0],
                child_frame_rotation: [0.0, 0.0, 0.0, 1.0],
                max_force: 30000.0,
                cone_limit: std::f32::consts::PI,
                angular_stiffness: 0.0,
                angular_damping: 0.0,
                angular_limits: None,
                collision_family: 1,
                motor: Some(PhysicsJointMotor {
                    world_drive: None,
                    target_rotation: [0.0, 0.0, 0.0, 1.0],
                    feed_forward_torque: std::array::from_fn(
                        |i| if i == axis { 12.0 } else { 0.0 },
                    ),
                    stiffness: 0.0,
                    damping: 0.0,
                    max_torque: 100.0,
                }),
            });
            let output = backend.step_frame(input).unwrap();
            let actual = output
                .velocity_updates
                .iter()
                .find(|v| v.entity == 2)
                .unwrap();
            let expected = rotation * Vector3::ith(axis, 12.0 / hz as f64 / inertia[axis]);
            let actual = Vector3::from(actual.angular_velocity.map(|v| v as f64));
            assert!((actual - expected).norm() < 1e-5,
                    "native torque integral hz={hz} axis={axis}: actual={actual:?}, expected={expected:?}");
            assert_eq!(output.report.substeps, 4);
        }
    }
}
