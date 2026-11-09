use super::*;

mod socket_tests {
    use super::*;
    #[test]
    fn socket_mobility_includes_the_parallel_axis_mass_of_an_offset_body() {
        let fixed = DriveBody {
            linear_velocity: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
            spin: Vector3::zeros(),
            inverse_inertia: Matrix3::zeros(),
        };
        let moving = DriveBody {
            linear_velocity: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
            spin: Vector3::zeros(),
            inverse_inertia: Matrix3::identity() * 10.0,
        };
        let constraints = point_rows(
            Some(0),
            Some(1),
            Vector3::zeros(),
            Vector3::new(0.0, 1.0, 0.0),
        );
        let m = mobility(&[&fixed, &moving], &[0.0, 0.5], &constraints);
        assert!(
            (m[(9, 9)] - 1.0 / 2.1).abs() < 1e-6,
            "offset inertia={}",
            m[(9, 9)]
        );
        assert!((m[(10, 10)] - 10.0).abs() < 1e-6);
    }
    #[test]
    fn loaded_contact_patch_changes_angular_response_without_pinning_native_state() {
        let body = DriveBody {
            linear_velocity: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
            spin: Vector3::new(1.0, 2.0, 3.0),
            inverse_inertia: Matrix3::identity() * 10.0,
        };
        let mut c = point_rows(
            None,
            Some(0),
            Vector3::zeros(),
            Vector3::new(-0.1, -0.2, 0.0),
        );
        c.extend(point_rows(
            None,
            Some(0),
            Vector3::zeros(),
            Vector3::new(0.1, -0.2, 0.0),
        ));
        c.extend(point_rows(
            None,
            Some(0),
            Vector3::zeros(),
            Vector3::new(0.0, -0.2, 0.1),
        ));
        let m = mobility(&[&body], &[0.5], &c);
        assert!(m.view((3, 3), (3, 3)).norm() < 1e-5);
        assert_eq!(body.spin, Vector3::new(1.0, 2.0, 3.0));
    }
}

mod damping_tests {
    use super::*;
    #[test]
    fn production_impedance_damps_different_articulated_inertias() {
        for inertia in [0.001, 0.1, 10.0] {
            for dt in [1.0 / 30.0, 1.0 / 120.0, 1.0 / 480.0] {
                let mut angle = 0.2f64;
                let mut speed = 0.0f64;
                let mut crossed = false;
                for _ in 0..(2.0 / dt) as usize {
                    let bodies = [
                        DriveBody {
                            linear_velocity: Vector3::zeros(),
                            rotation: UnitQuaternion::identity(),
                            spin: Vector3::zeros(),
                            inverse_inertia: Matrix3::zeros(),
                        },
                        DriveBody {
                            linear_velocity: Vector3::zeros(),
                            rotation: UnitQuaternion::identity(),
                            spin: Vector3::new(speed, 0.0, 0.0),
                            inverse_inertia: Matrix3::identity() / inertia,
                        },
                    ];
                    let rows = [DriveRow {
                        world_drive: None,
                        parent: 0,
                        child: 1,
                        error: Vector3::new(angle, 0.0, 0.0),
                        stiffness: 40.0,
                        damping: 0.0,
                        max_torque: 1000.0,
                        feed_forward: Vector3::zeros(),
                    }];
                    let p = solve(&bodies, &[0.0, 1.0], &rows, &[], dt);
                    speed += p[1].x / inertia;
                    angle += speed * dt;
                    crossed |= angle < -0.0001;
                }
                assert!(
                    !crossed,
                    "underdamped inertia={inertia} dt={dt} angle={angle}"
                );
                assert!(
                    angle.abs() < 0.03,
                    "did not converge inertia={inertia} dt={dt} angle={angle}"
                );
            }
        }
    }
}

mod constrained_damping_tests {
    use super::*;
    #[test]
    fn contact_rank_roundoff_does_not_turn_into_infinite_critical_damping() {
        let bodies = (0..2)
            .map(|i| DriveBody {
                linear_velocity: Vector3::zeros(),
                rotation: UnitQuaternion::identity(),
                spin: Vector3::new(if i == 0 { 0.0 } else { 1e-5 }, 0.0, 0.0),
                inverse_inertia: Matrix3::identity() * 10.0,
            })
            .collect::<Vec<_>>();
        let mut constraints = Vec::new();
        for i in 0..2 {
            for point in [
                Vector3::new(-0.1, -0.2, 0.0),
                Vector3::new(0.1, -0.2, 0.0),
                Vector3::new(0.0, -0.2, 0.1),
            ] {
                constraints.extend(point_rows(None, Some(i), Vector3::zeros(), point));
            }
        }
        let rows = [DriveRow {
            world_drive: None,
            parent: 0,
            child: 1,
            error: Vector3::zeros(),
            stiffness: 40.0,
            damping: 0.1,
            max_torque: 100.0,
            feed_forward: Vector3::zeros(),
        }];
        let impulse = solve(&bodies, &[0.5, 0.5], &rows, &constraints, 0.01);
        assert!(
            impulse[1].norm() < 1e-5,
            "locked contact roundoff saturated drive {:?}",
            impulse[1]
        );
    }
}

mod normal_support_tests {
    use super::*;
    #[test]
    fn a_loaded_normal_patch_keeps_tangential_translation_and_yaw_free() {
        let body = DriveBody {
            linear_velocity: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
            spin: Vector3::zeros(),
            inverse_inertia: Matrix3::identity() * 10.0,
        };
        let rows = [[-0.1, -0.2, -0.1], [0.1, -0.2, -0.1], [0.0, -0.2, 0.1]]
            .into_iter()
            .map(|p| {
                normal_row(
                    None,
                    Some(0),
                    Vector3::zeros(),
                    Vector3::from(p),
                    Vector3::y(),
                )
            })
            .collect::<Vec<_>>();
        let m = mobility(&[&body], &[0.5], &rows);
        assert!((m[(0, 0)] - 0.5).abs() < 1e-6 && (m[(2, 2)] - 0.5).abs() < 1e-6);
        assert!(
            (m[(4, 4)] - 10.0).abs() < 1e-6,
            "normal contact must not weld yaw"
        );
        assert!(m[(1, 1)] < 1e-6 && m[(3, 3)] < 1e-6 && m[(5, 5)] < 1e-6);
    }
}

mod gravity_prediction_tests {
    use super::*;
    #[test]
    fn implicit_damping_preserves_the_torque_that_exactly_compensates_weight() {
        let fixed = DriveBody {
            linear_velocity: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
            spin: Vector3::zeros(),
            inverse_inertia: Matrix3::zeros(),
        };
        let moving = DriveBody {
            linear_velocity: Vector3::zeros(),
            rotation: UnitQuaternion::identity(),
            spin: Vector3::zeros(),
            inverse_inertia: Matrix3::identity() * 10.0,
        };
        let constraints = point_rows(
            Some(0),
            Some(1),
            Vector3::zeros(),
            Vector3::new(-0.2, 1.0, 0.0),
        );
        let g = 9.81;
        let rows = [DriveRow {
            world_drive: None,
            parent: 0,
            child: 1,
            error: Vector3::zeros(),
            stiffness: 900.0,
            damping: 1.0,
            max_torque: 100.0,
            feed_forward: Vector3::new(0.0, 0.0, 0.4 * g),
        }];
        let bodies = [fixed, moving];
        for dt in [1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0] {
            let impulses = solve_with_gravity(&bodies, &[0.0, 0.5], &rows, &constraints, dt, g);
            assert!(
                (impulses[1].z / dt - 0.4 * g).abs() < 0.0001,
                "weight compensation was filtered: {:?}",
                impulses[1] / dt
            );
        }
    }
}

mod release_prediction_tests {
    use super::*;
    #[test]
    fn lifting_drive_does_not_receive_a_fictitious_tensile_support_force() {
        for dt in [1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0] {
            let bodies = (0..2)
                .map(|_| DriveBody {
                    linear_velocity: Vector3::zeros(),
                    rotation: UnitQuaternion::identity(),
                    spin: Vector3::zeros(),
                    inverse_inertia: Matrix3::identity() * 10.0,
                })
                .collect::<Vec<_>>();
            let rows = [DriveRow {
                parent: 0,
                child: 1,
                error: Vector3::zeros(),
                stiffness: 0.0,
                damping: 0.0,
                feed_forward: Vector3::zeros(),
                max_torque: 3.0,
                world_drive: Some(super::super::joint_drives::WorldDrive {
                    child: false,
                    error: Vector3::new(-0.4, 0.0, 0.0),
                    stiffness: 10.0,
                    damping: 0.0,
                }),
            }];
            let mut sockets = point_rows(Some(0), Some(1), Vector3::zeros(), Vector3::zeros());
            let free = solve(&bodies, &[1.0, 1.0], &rows, &sockets, dt);
            sockets.push(normal_row(
                Some(0),
                None,
                Vector3::z(),
                Vector3::zeros(),
                Vector3::y(),
            ));
            let lifting = solve(&bodies, &[1.0, 1.0], &rows, &sockets, dt);
            assert!(lifting[0].x < 0.0 && (lifting[0] - free[0]).norm() < 1e-9);
            assert!((lifting[0] + lifting[1]).norm() < 1e-12);
            assert!(lifting[0].norm() <= 3.0 * dt + 1e-12);
        }
    }
}

mod contact_gap_tests {
    use super::*;
    #[test]
    fn touching_support_predicts_load_but_a_separated_contact_cannot_push_early() {
        for dt in [1.0 / 60.0, 1.0 / 120.0, 1.0 / 240.0] {
            let bodies: Vec<_> = (0..2)
                .map(|_| DriveBody {
                    linear_velocity: Vector3::zeros(),
                    rotation: UnitQuaternion::identity(),
                    spin: Vector3::zeros(),
                    inverse_inertia: Matrix3::identity() * 10.0,
                })
                .collect();
            let rows = [DriveRow {
                parent: 0,
                child: 1,
                error: Vector3::zeros(),
                stiffness: 0.0,
                damping: 0.0,
                max_torque: 30.0,
                feed_forward: Vector3::zeros(),
                world_drive: Some(super::super::joint_drives::WorldDrive {
                    child: false,
                    error: Vector3::new(0.4, 0.0, 0.0),
                    stiffness: 10.0,
                    damping: 0.0,
                }),
            }];
            let sockets = point_rows(Some(0), Some(1), Vector3::zeros(), Vector3::zeros());
            let free = solve(&bodies, &[1.0, 1.0], &rows, &sockets, dt);
            let mut support =
                normal_row(Some(0), None, Vector3::z(), Vector3::zeros(), Vector3::y());
            let mut contacts = sockets.clone();
            contacts.push(support.clone());
            let touching = solve(&bodies, &[1.0, 1.0], &rows, &contacts, dt);
            support.velocity_bias = 0.1 / dt;
            contacts.pop();
            contacts.push(support);
            let separated = solve(&bodies, &[1.0, 1.0], &rows, &contacts, dt);
            assert!((separated[0] - free[0]).norm() < 1e-9);
            assert!((touching[0] - free[0]).norm() > 1e-6);
            assert!((touching[0] + touching[1]).norm() < 1e-12);
        }
    }
}
