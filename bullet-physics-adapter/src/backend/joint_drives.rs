//! Generic physical angular stops and bounded implicit PD drives.
use super::*;

pub(super) fn project(q: UnitQuaternion<f64>, limits: PhysicsAngularLimits) -> UnitQuaternion<f64> {
    let mut raw = *q.quaternion();
    if raw.w < 0.0 {
        raw = -raw;
    }
    let n = raw.w.hypot(raw.j);
    let twist = if n > 1e-12 {
        UnitQuaternion::new_normalize(Quaternion::new(raw.w / n, 0.0, raw.j / n, 0.0))
    } else {
        UnitQuaternion::identity()
    };
    let swing = (q * twist.inverse()).scaled_axis();
    let angle = 2.0 * twist.quaternion().j.atan2(twist.quaternion().w);
    UnitQuaternion::from_scaled_axis(Vector3::new(
        swing
            .x
            .clamp(limits.swing_x[0] as f64, limits.swing_x[1] as f64),
        0.0,
        swing
            .z
            .clamp(limits.swing_z[0] as f64, limits.swing_z[1] as f64),
    )) * UnitQuaternion::from_axis_angle(
        &Vector3::y_axis(),
        angle.clamp(limits.twist[0] as f64, limits.twist[1] as f64),
    )
}

#[cfg(test)]
fn pd(
    error: Vector3<f64>,
    spin: Vector3<f64>,
    inertia: f64,
    stiffness: f64,
    damping: f64,
    max_torque: f64,
    dt: f64,
) -> Vector3<f64> {
    if inertia <= 0.0 || dt <= 0.0 {
        return Vector3::zeros();
    }
    let damping = damping.max(2.0 * (stiffness * inertia).sqrt());
    let implicit = 1.0 + (damping * dt + stiffness * dt * dt) / inertia;
    let mut torque = (-error * stiffness - spin * (damping + stiffness * dt)) / implicit;
    let magnitude = torque.norm();
    if magnitude > max_torque && magnitude > 0.0 {
        torque *= max_torque / magnitude;
    }
    torque
}

pub(super) struct DriveBody {
    pub linear_velocity: Vector3<f64>,
    pub rotation: UnitQuaternion<f64>,
    pub spin: Vector3<f64>,
    pub inverse_inertia: nalgebra::Matrix3<f64>,
}
pub(super) struct WorldDrive {
    pub child: bool,
    pub error: Vector3<f64>,
    pub stiffness: f64,
    pub damping: f64,
}
pub(super) struct DriveRow {
    pub world_drive: Option<WorldDrive>,
    pub parent: usize,
    pub child: usize,
    pub error: Vector3<f64>,
    pub stiffness: f64,
    pub damping: f64,
    pub max_torque: f64,
    pub feed_forward: Vector3<f64>,
}
pub(super) fn terms(
    j: PhysicsJointSnapshot,
    relative: UnitQuaternion<f64>,
) -> [(Vector3<f64>, f64, f64, f64); 2] {
    let stop = if let Some(l) = j.angular_limits {
        let error = (relative * project(relative, l).inverse()).scaled_axis();
        (
            error,
            if error.norm_squared() > 1e-12 {
                l.stiffness as f64
            } else {
                0.0
            },
            l.damping as f64,
            l.max_torque as f64,
        )
    } else {
        let d = relative.scaled_axis();
        let angle = d.norm();
        let error = if angle > j.cone_limit as f64 {
            d * ((angle - j.cone_limit as f64) / angle)
        } else {
            Vector3::zeros()
        };
        (
            error,
            if error.norm_squared() > 1e-12 {
                j.angular_stiffness as f64
            } else {
                0.0
            },
            j.angular_damping as f64,
            40.0,
        )
    };
    let motor = j.motor.map_or((Vector3::zeros(), 0.0, 0.0, 0.0), |m| {
        let target = physics_pose([0.0; 3], m.target_rotation).rotation;
        let target = j.angular_limits.map_or(target, |l| project(target, l));
        (
            (relative * target.inverse()).scaled_axis(),
            m.stiffness as f64,
            m.damping as f64,
            m.max_torque as f64,
        )
    });
    [stop, motor]
}

#[cfg(test)]
mod tests {
    use super::*;
    fn limits() -> PhysicsAngularLimits {
        PhysicsAngularLimits {
            swing_x: [0.0, 2.0],
            swing_z: [-0.03, 0.03],
            twist: [-0.05, 0.05],
            stiffness: 200.0,
            damping: 0.1,
            max_torque: 100.0,
        }
    }
    #[test]
    fn asymmetric_hinge_rejects_hyperextension_but_permits_flexion() {
        for a in [-1.0, 0.5, 2.5] {
            let q = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), a);
            let actual = project(q, limits()).scaled_axis();
            assert!((actual.x - a.clamp(0.0, 2.0)).abs() < 1e-8);
        }
        let q = UnitQuaternion::from_scaled_axis(Vector3::new(0.4, 0.8, 0.6));
        let p = project(q, limits());
        assert!(p.angle().is_finite());
        assert!((project(p, limits()).inverse() * p).angle() < 1e-8);
        assert!(
            (project(UnitQuaternion::new_unchecked(-*q.quaternion()), limits()).inverse() * p)
                .angle()
                < 1e-8
        );
    }
    #[test]
    fn implicit_drive_stays_bounded_for_tiny_inertia_and_long_steps() {
        for inertia in [0.00001, 0.1, 5.0] {
            for dt in [1.0 / 30.0, 1.0 / 60.0, 1.0 / 120.0] {
                let t = pd(
                    Vector3::new(1.0, 0.3, -0.5),
                    Vector3::new(80.0, 2.0, 0.0),
                    inertia,
                    200.0,
                    0.1,
                    20.0,
                    dt,
                );
                assert!(t.norm() <= 20.0 + 1e-9 && t.iter().all(|v| v.is_finite()));
            }
        }
    }
}

#[cfg(test)]
mod imported_frame_tests {
    use super::*;
    #[test]
    fn installed_neutral_ranges_do_not_apply_stop_torques_inside_limits() {
        let cases:Vec<([f32;4],PhysicsAngularLimits)>=serde_json::from_str(r#"[[[0.043181952089071274, 0.0099294139072299, 0.000724283920135349, 0.999017596244812], {"swing_x": [-0.5235987755982988, 0.7853981633974483], "swing_z": [-0.4363323129985824, 0.4363323129985824], "twist": [-0.6108652381980153, 0.6108652381980153], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[-0.10809265822172164, 0.019372722133994102, 0.02842680551111698, 0.9935455322265624], {"swing_x": [-0.7853981633974483, 0.9599310885968813], "swing_z": [-0.6108652381980153, 0.6108652381980153], "twist": [-1.2217304763960306, 1.2217304763960306], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[0.02916627377271652, 0.17768405377864838, -0.12581293284893036, 0.9755762219429016], {"swing_x": [-1.1344640137963142, 2.443460952792061], "swing_z": [-1.9198621771937625, 1.9198621771937625], "twist": [-1.5707963267948966, 1.5707963267948966], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[0.156436488032341, 7.003545761108398e-07, -0.004582412540912628, 0.9876774549484252], {"swing_x": [0, 2.5307274153917776], "swing_z": [-0.08726646259971647, 0.08726646259971647], "twist": [-1.3962634015954636, 1.3962634015954636], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[0.049636803567409515, 0.12300214171409608, -0.05783764272928238, 0.989475429058075], {"swing_x": [-1.0471975511965976, 1.2217304763960306], "swing_z": [-0.5235987755982988, 0.5235987755982988], "twist": [-0.2617993877991494, 0.2617993877991494], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[-0.08332154154777527, 0.2695567011833191, -0.16402567923069, -0.945247232913971], {"swing_x": [-1.1344640137963142, 2.443460952792061], "swing_z": [-1.9198621771937625, 1.9198621771937625], "twist": [-1.5707963267948966, 1.5707963267948966], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[0.12446225434541702, 1.4603135696233949e-06, -0.004500265698879957, 0.9922141432762146], {"swing_x": [0, 2.5307274153917776], "swing_z": [-0.08726646259971647, 0.08726646259971647], "twist": [-1.3962634015954636, 1.3962634015954636], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[-0.019538894295692444, -0.09594753384590148, 0.010425623506307602, -0.9951400756835938], {"swing_x": [-1.0471975511965976, 1.2217304763960306], "swing_z": [-0.5235987755982988, 0.5235987755982988], "twist": [-0.2617993877991494, 0.2617993877991494], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[0.04558534920215607, -0.14477944374084473, 0.05602128431200981, 0.9868245124816896], {"swing_x": [-0.6108652381980153, 2.1816615649929116], "swing_z": [-0.8726646259971648, 0.8726646259971648], "twist": [-0.7853981633974483, 0.7853981633974483], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[0.10417554527521132, 1.341104507446289e-07, 5.953758955001831e-05, -0.9945589303970336], {"swing_x": [-2.5307274153917776, 0], "swing_z": [-0.06981317007977318, 0.06981317007977318], "twist": [-0.12217304763960307, 0.12217304763960307], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[-0.015326411463320255, 0.016441425308585167, 0.07200216501951218, -0.9971511960029602], {"swing_x": [-0.6108652381980153, 0.8726646259971648], "swing_z": [-0.3490658503988659, 0.3490658503988659], "twist": [-0.2617993877991494, 0.2617993877991494], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[0.03174090012907982, -0.1157282218337059, -0.027119237929582596, -0.9924031496047974], {"swing_x": [-0.6108652381980153, 2.1816615649929116], "swing_z": [-0.8726646259971648, 0.8726646259971648], "twist": [-0.7853981633974483, 0.7853981633974483], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[-0.06958083808422089, 1.862645149230957e-07, 0.005492019932717085, 0.99756121635437], {"swing_x": [-2.5307274153917776, 0], "swing_z": [-0.06981317007977318, 0.06981317007977318], "twist": [-0.12217304763960307, 0.12217304763960307], "stiffness": 400, "damping": 0.08, "max_torque": 300}], [[-0.0550176203250885, -0.004374772310256958, 0.07783307135105133, -0.9954376220703124], {"swing_x": [-0.6108652381980153, 0.8726646259971648], "swing_z": [-0.3490658503988659, 0.3490658503988659], "twist": [-0.2617993877991494, 0.2617993877991494], "stiffness": 400, "damping": 0.08, "max_torque": 300}]]"#).unwrap();
        for (index, (q, l)) in cases.into_iter().enumerate() {
            for sign in [1.0, -1.0] {
                let j = PhysicsJointSnapshot {
                    id: 1,
                    parent: 1,
                    child: 2,
                    parent_anchor: [0.0; 3],
                    child_anchor: [0.0; 3],
                    parent_frame_rotation: [0.0, 0.0, 0.0, 1.0],
                    child_frame_rotation: [0.0, 0.0, 0.0, 1.0],
                    max_force: 100.0,
                    cone_limit: 3.14,
                    angular_stiffness: 0.0,
                    angular_damping: 0.0,
                    collision_family: 1,
                    angular_limits: Some(l),
                    motor: None,
                };
                let row = terms(j, physics_pose([0.0; 3], q.map(|v| v * sign)).rotation)[0];
                assert!(
                    row.0.norm() < 1e-6,
                    "false anatomical stop index={index} sign={sign} error={:?}",
                    row.0
                );
            }
        }
    }
}

#[cfg(test)]
mod feed_forward_tests {
    use super::*;
    #[test]
    fn bias_is_bounded_and_remains_an_internal_couple() {
        let bodies = (0..2)
            .map(|_| DriveBody {
                linear_velocity: Vector3::zeros(),
                rotation: UnitQuaternion::identity(),
                spin: Vector3::zeros(),
                inverse_inertia: nalgebra::Matrix3::identity(),
            })
            .collect::<Vec<_>>();
        let rows = [DriveRow {
            world_drive: None,
            parent: 0,
            child: 1,
            error: Vector3::zeros(),
            stiffness: 0.0,
            damping: 0.0,
            max_torque: 3.0,
            feed_forward: Vector3::new(0.0, 100.0, 0.0),
        }];
        let impulse = super::super::joint_mobility::solve(&bodies, &[1.0, 1.0], &rows, &[], 0.01);
        assert!((impulse[1] - Vector3::new(0.0, 0.03, 0.0)).norm() < 1e-10);
        assert!((impulse[0] + impulse[1]).norm() < 1e-10);
    }
}
