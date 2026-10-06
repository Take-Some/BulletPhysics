use super::tests::body;
use super::*;

#[test]
fn settled_authored_panel_responds_to_off_centre_impulse() {
    let mut backend = BulletPacketPhysicsBackend::new(16, 8).unwrap();
    let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
    input.gravity = -9.81;
    input.bodies.push(body(
        1,
        PhysicsBodyKindDto::Static,
        CollisionShapeDto::Box {
            half_extents: [200.0, 0.25, 200.0],
        },
        [0.0, -0.25, 0.0],
    ));
    // The authored test apron is a thin slab above the larger ground box.
    input.bodies.push(body(
        8,
        PhysicsBodyKindDto::Static,
        CollisionShapeDto::Box {
            half_extents: [95.0, 0.025, 150.0],
        },
        [0.0, 0.025, 0.0],
    ));
    let mut panel = body(
        7,
        PhysicsBodyKindDto::Dynamic,
        CollisionShapeDto::Box {
            half_extents: [0.91, 0.11, 0.78],
        },
        [-39.58525, 3.77152, 58.39797],
    );
    let center = [0.0, 0.5756836, -1.6000977];
    panel.convex_hulls = vec![vec![
        [0.0, 0.69150668, -0.94623584],
        [0.0, 0.55293924, -2.30460405],
        [-0.8545866, 0.55293924, -2.25150108],
        [-0.90990072, 0.67684430, -0.82715935],
        [0.85461557, 0.55293924, -2.25150108],
        [0.90992975, 0.67684430, -0.82715935],
        [0.0, 0.62201858, -0.94146895],
        [0.0, 0.48349950, -2.29938412],
        [-0.85386181, 0.48347753, -2.24654341],
        [-0.90850919, 0.60735184, -0.82265466],
        [0.85389078, 0.48347753, -2.24654341],
        [0.90853816, 0.60735184, -0.82265466],
    ]
    .into_iter()
    .map(|p: [f32; 3]| std::array::from_fn(|a| p[a] - center[a]))
    .collect()];
    panel.convex_hulls[0].sort_unstable_by(|a, b| {
        a[0].total_cmp(&b[0])
            .then(a[1].total_cmp(&b[1]))
            .then(a[2].total_cmp(&b[2]))
    });
    let mass = 51.377193;
    panel.mass_properties = Some(PhysicsMassProperties {
        mass,
        center_of_mass: [-0.0000019868, 0.0102303, 0.029529335],
        inertia_diagonal: [8.414464, 21.79056, 13.565268],
    });
    panel.linear_velocity = [2.0, -1.5, 0.0];
    panel.flags.continuous_collision = true;
    panel.linear_damping = Some(0.035);
    panel.angular_damping = Some(0.075);
    panel.material.friction = 0.65;
    panel.material.restitution = 0.08;
    input.bodies.push(panel);
    let mut before = [0.0; 3];
    for tick in 0..720 {
        input.fixed_tick = tick + 1;
        input.frame_index = tick + 1;
        let output = backend.step_frame(input.clone()).unwrap();
        let p = output.pose_updates.iter().find(|p| p.entity == 7).unwrap();
        before = p.position;
    }
    input.commands.push(PhysicsCommandDto {
        seq: 1,
        kind: PhysicsCommandKindDto::ApplyImpulse {
            entity: 7,
            impulse: [mass * 3.0, mass * 7.0, 0.0],
            point: [before[0], before[1], before[2] + 0.35],
        },
    });
    input.fixed_tick += 1;
    input.frame_index += 1;
    let immediate = backend.step_frame(input.clone()).unwrap();
    let v = immediate
        .velocity_updates
        .iter()
        .find(|v| v.entity == 7)
        .unwrap();
    assert!(
        v.linear_velocity[1] > 2.0,
        "before={before:?} immediate={v:?} events={:?}",
        immediate.events
    );
    input.commands.clear();
    let mut last = immediate;
    for _ in 0..30 {
        input.fixed_tick += 1;
        input.frame_index += 1;
        last = backend.step_frame(input.clone()).unwrap();
    }
    let p = last.pose_updates.iter().find(|p| p.entity == 7).unwrap();
    let v = last
        .velocity_updates
        .iter()
        .find(|v| v.entity == 7)
        .unwrap();
    assert!(
        p.position[1] > before[1] + 0.03,
        "before={before:?} after={p:?} velocity={v:?}"
    );
    assert!(
        v.linear_velocity.iter().map(|v| v * v).sum::<f32>().sqrt() > 0.5,
        "{v:?}"
    );
}

#[test]
fn explicit_velocity_wakes_a_sleeping_body() {
    let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
    let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 120.0);
    input.gravity = 0.0;
    input.bodies.push(body(
        7,
        PhysicsBodyKindDto::Dynamic,
        CollisionShapeDto::Box {
            half_extents: [0.5; 3],
        },
        [0.0; 3],
    ));
    for _ in 0..600 {
        input.fixed_tick += 1;
        input.frame_index += 1;
        backend.step_frame(input.clone()).unwrap();
    }
    input.commands.push(PhysicsCommandDto {
        seq: 1,
        kind: PhysicsCommandKindDto::SetLinearVelocity {
            entity: 7,
            velocity: [2.0, 0.0, 0.0],
        },
    });
    backend.step_frame(input.clone()).unwrap();
    input.commands.clear();
    let mut last = None;
    for _ in 0..30 {
        input.fixed_tick += 1;
        input.frame_index += 1;
        last = Some(backend.step_frame(input.clone()).unwrap());
    }
    let last = last.unwrap();
    let pose = last.pose_updates.iter().find(|p| p.entity == 7).unwrap();
    assert!(
        pose.position[0] > 0.2,
        "explicit velocity left a settled body asleep: {pose:?}"
    );
}

#[test]
fn contact_reports_approach_velocity_before_the_solver_stops_the_body() {
    let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
    let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
    input.gravity = 0.0;
    input.bodies.push(body(
        1,
        PhysicsBodyKindDto::Static,
        CollisionShapeDto::Box {
            half_extents: [0.5, 2.0, 2.0],
        },
        [0.0; 3],
    ));
    let mut mover = body(
        7,
        PhysicsBodyKindDto::Dynamic,
        CollisionShapeDto::Box {
            half_extents: [0.5; 3],
        },
        [1.005, 0.0, 0.0],
    );
    mover.linear_velocity = [-6.0, 0.0, 0.0];
    mover.flags.continuous_collision = true;
    input.bodies.push(mover);
    let output = backend.step_frame(input).unwrap();
    let velocity = output
        .velocity_updates
        .iter()
        .find(|v| v.entity == 7)
        .unwrap();
    assert!(velocity.linear_velocity[0].abs() < 1.0, "{velocity:?}");
    let contact = output
        .events
        .iter()
        .find_map(|e| match e {
            PhysicsEventDto::ContactBegin(c) if c.a == 1 && c.b == 7 => Some(c),
            _ => None,
        })
        .expect("moving box must actually touch the wall");
    assert!(contact.relative_velocity.unwrap()[0] < -5.0, "{contact:?}");
    assert_eq!(output.report.substeps, 4);
}
