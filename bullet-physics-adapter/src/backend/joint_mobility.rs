//! Constrained mobility for implicit angular drives. Native sockets/contact
//! patches change the inertia seen at a joint; free-body inertia is insufficient.
use super::joint_drives::{DriveBody, DriveRow};
use super::*;
use nalgebra::{DMatrix, Matrix3};
type ConstraintTerms = Vec<(usize, Vector3<f64>, Vector3<f64>)>;
#[derive(Clone)]
pub(super) struct ConstraintRow {
    pub velocity_bias: f64,
    pub friction: Option<(f64, [ConstraintTerms; 2])>,
    pub unilateral: bool,
    pub terms: Vec<(usize, Vector3<f64>, Vector3<f64>)>,
}
pub(super) fn normal_row(
    a: Option<usize>,
    b: Option<usize>,
    ra: Vector3<f64>,
    rb: Vector3<f64>,
    normal: Vector3<f64>,
) -> ConstraintRow {
    let mut terms = Vec::with_capacity(2);
    if let Some(a) = a {
        terms.push((a, normal, ra.cross(&normal)));
    }
    if let Some(b) = b {
        terms.push((b, -normal, -rb.cross(&normal)));
    }
    ConstraintRow {
        terms,
        velocity_bias: 0.0,
        friction: None,
        unilateral: true,
    }
}
pub(super) fn contact_row(
    a: Option<usize>,
    b: Option<usize>,
    ra: Vector3<f64>,
    rb: Vector3<f64>,
    normal: Vector3<f64>,
    friction: f64,
) -> ConstraintRow {
    let mut row = normal_row(a, b, ra, rb, normal);
    if friction > 0.0 {
        let axis = (0..3)
            .min_by(|a, b| normal[*a].abs().total_cmp(&normal[*b].abs()))
            .unwrap();
        let first = normal.cross(&Vector3::ith(axis, 1.0)).normalize();
        let second = normal.cross(&first);
        row.friction = Some((
            friction,
            [
                normal_row(a, b, ra, rb, first).terms,
                normal_row(a, b, ra, rb, second).terms,
            ],
        ));
    }
    row
}
pub(super) fn point_rows(
    a: Option<usize>,
    b: Option<usize>,
    ra: Vector3<f64>,
    rb: Vector3<f64>,
) -> Vec<ConstraintRow> {
    (0..3)
        .map(|i| {
            let mut r = normal_row(a, b, ra, rb, Vector3::ith(i, 1.0));
            r.unilateral = false;
            r
        })
        .collect()
}
fn mobility(
    bodies: &[&DriveBody],
    inverse_masses: &[f64],
    constraints: &[ConstraintRow],
) -> DMatrix<f64> {
    let n = bodies.len();
    let mut inverse = DMatrix::<f64>::zeros(n * 6, n * 6);
    for (i, body) in bodies.iter().enumerate() {
        for j in 0..3 {
            inverse[(6 * i + j, 6 * i + j)] = inverse_masses[i];
        }
        inverse
            .view_mut((6 * i + 3, 6 * i + 3), (3, 3))
            .copy_from(&body.inverse_inertia);
    }
    if constraints.is_empty() {
        return inverse;
    }
    // A socket/contact row touches at most its two resident bodies.
    // Form J M^-1 and J M^-1 J^T directly from those sparse blocks.
    let mut jm = DMatrix::<f64>::zeros(constraints.len(), n * 6);
    for (row, constraint) in constraints.iter().enumerate() {
        for (body, linear, angular) in &constraint.terms {
            let spin = bodies[*body].inverse_inertia * angular;
            for axis in 0..3 {
                jm[(row, 6 * body + axis)] += linear[axis] * inverse_masses[*body];
                jm[(row, 6 * body + 3 + axis)] += spin[axis];
            }
        }
    }
    let mut effective = DMatrix::<f64>::zeros(constraints.len(), constraints.len());
    for (column, constraint) in constraints.iter().enumerate() {
        for (body, linear, angular) in &constraint.terms {
            for row in 0..constraints.len() {
                for axis in 0..3 {
                    effective[(row, column)] += jm[(row, 6 * body + axis)] * linear[axis]
                        + jm[(row, 6 * body + 3 + axis)] * angular[axis];
                }
            }
        }
    }
    let scale = effective.diagonal().iter().copied().fold(0.0f64, f64::max);
    if scale <= f64::EPSILON {
        return inverse;
    }
    // Redundant patch points and static DOFs have dependent Jacobian rows.
    for i in 0..effective.nrows() {
        effective[(i, i)] += scale * 1e-9;
    }
    if let Some(factor) = effective.cholesky() {
        let response = &inverse - jm.transpose() * factor.solve(&jm);
        (&response + response.transpose()) * 0.5
    } else {
        inverse
    }
}
#[cfg(test)]
pub(super) fn solve(
    bodies: &[DriveBody],
    inverse_masses: &[f64],
    rows: &[DriveRow],
    constraints: &[ConstraintRow],
    dt: f64,
) -> Vec<Vector3<f64>> {
    solve_with_gravity(bodies, inverse_masses, rows, constraints, dt, 0.0)
}
pub(super) fn solve_with_gravity(
    bodies: &[DriveBody],
    inverse_masses: &[f64],
    rows: &[DriveRow],
    constraints: &[ConstraintRow],
    dt: f64,
    gravity: f64,
) -> Vec<Vector3<f64>> {
    let n = bodies.len();
    let mut parent: Vec<_> = (0..n).collect();
    fn root(parent: &[usize], mut i: usize) -> usize {
        while parent[i] != i {
            i = parent[i];
        }
        i
    }
    for row in rows {
        let a = root(&parent, row.parent);
        let b = root(&parent, row.child);
        parent[b] = a;
    }
    for c in constraints {
        if let Some(first) = c.terms.first() {
            for term in c.terms.iter().skip(1) {
                let a = root(&parent, first.0);
                let b = root(&parent, term.0);
                parent[b] = a;
            }
        }
    }
    let mut groups = BTreeMap::<usize, Vec<usize>>::new();
    for i in 0..n {
        groups.entry(root(&parent, i)).or_default().push(i);
    }
    let mut result = vec![Vector3::zeros(); n];
    for group in groups.values() {
        let lookup: BTreeMap<_, _> = group
            .iter()
            .enumerate()
            .map(|(local, global)| (*global, local))
            .collect();
        let selected: Vec<_> = rows
            .iter()
            .filter(|r| lookup.contains_key(&r.parent))
            .collect();
        if selected.is_empty() {
            continue;
        }
        let local_constraints: Vec<_> = constraints
            .iter()
            .filter(|c| c.terms.first().is_some_and(|t| lookup.contains_key(&t.0)))
            .map(|c| ConstraintRow {
                unilateral: c.unilateral,
                velocity_bias: c.velocity_bias,
                friction: c.friction.as_ref().map(|(mu, rows)| {
                    (
                        *mu,
                        rows.clone().map(|terms| {
                            terms.iter().map(|(i, l, a)| (lookup[i], *l, *a)).collect()
                        }),
                    )
                }),
                terms: c
                    .terms
                    .iter()
                    .map(|(i, l, a)| (lookup[i], *l, *a))
                    .collect(),
            })
            .collect();
        let local_bodies: Vec<_> = group.iter().map(|i| &bodies[*i]).collect();
        let local_mass: Vec<_> = group.iter().map(|i| inverse_masses[*i]).collect();
        let permanent: Vec<_> = local_constraints
            .iter()
            .filter(|r| !r.unilateral)
            .cloned()
            .collect();
        let inverse = mobility(&local_bodies, &local_mass, &permanent);
        let angular = |a: usize, b: usize| -> Matrix3<f64> {
            inverse
                .fixed_view::<3, 3>(6 * a + 3, 6 * b + 3)
                .into_owned()
        };
        // The implicit damper sees the velocity gravity will produce through
        // native sockets/support, so feedforward compensation is not attenuated.
        let mut gravity_force = nalgebra::DVector::<f64>::zeros(group.len() * 6);
        for (i, inverse_mass) in local_mass.iter().enumerate() {
            if *inverse_mass > 0.0 {
                gravity_force[6 * i + 1] = -gravity / inverse_mass;
            }
        }
        let mut momentum = gravity_force * dt;
        for (i, b) in local_bodies.iter().enumerate() {
            if local_mass[i] > 0.0 {
                for j in 0..3 {
                    momentum[6 * i + j] += b.linear_velocity[j] / local_mass[i];
                }
            }
            let angular_momentum = b
                .inverse_inertia
                .try_inverse()
                .map_or(Vector3::zeros(), |inertia| inertia * b.spin);
            for j in 0..3 {
                momentum[6 * i + 3 + j] += angular_momentum[j];
            }
        }
        let mut predicted = &inverse * momentum;
        let spin = |v: &nalgebra::DVector<f64>, i: usize| {
            Vector3::new(v[6 * i + 3], v[6 * i + 4], v[6 * i + 5])
        };
        let jacobian = |terms: &ConstraintTerms| {
            let mut j = nalgebra::DVector::<f64>::zeros(group.len() * 6);
            for (body, linear, angular) in terms {
                for axis in 0..3 {
                    j[body * 6 + axis] += linear[axis];
                    j[body * 6 + 3 + axis] += angular[axis];
                }
            }
            j
        };
        let normals: Vec<_> = local_constraints
            .iter()
            .filter(|r| r.unilateral)
            .map(|row| {
                let j = jacobian(&row.terms);
                let response = &inverse * &j;
                let effective = j.dot(&response);
                let friction = row.friction.as_ref().map(|(mu, terms)| {
                    let t = terms.each_ref().map(jacobian);
                    let r = t.each_ref().map(|t| &inverse * t);
                    let inverse_tangent = nalgebra::Matrix2::new(
                        t[0].dot(&r[0]),
                        t[0].dot(&r[1]),
                        t[1].dot(&r[0]),
                        t[1].dot(&r[1]),
                    )
                    .try_inverse()
                    .unwrap_or_else(nalgebra::Matrix2::zeros);
                    (*mu, t, r, inverse_tangent)
                });
                (j, response, effective, friction, row.velocity_bias)
            })
            .collect();
        let mut normal_lambda = vec![0.0; normals.len()];
        let mut tangent_lambda = vec![nalgebra::Vector2::zeros(); normals.len()];
        let local_rows: Vec<_> = selected
            .iter()
            .map(|r| (lookup[&r.parent], lookup[&r.child]))
            .collect();

        let actuator_response: Vec<Vec<[f64; 3]>> = local_rows
            .iter()
            .map(|(a, b)| {
                (0..group.len() * 6)
                    .map(|i| {
                        std::array::from_fn(|axis| {
                            inverse[(i, 6 * b + 3 + axis)] - inverse[(i, 6 * a + 3 + axis)]
                        })
                    })
                    .collect()
            })
            .collect();
        let mut lambda = vec![Vector3::zeros(); selected.len()];

        let damping: Vec<_> = selected
            .iter()
            .zip(&local_rows)
            .map(|(r, (a, b))| {
                let effective =
                    angular(*a, *a) + angular(*b, *b) - angular(*a, *b) - angular(*b, *a);
                let eigen = effective.symmetric_eigen();
                let free = local_bodies[*a].inverse_inertia + local_bodies[*b].inverse_inertia;
                let rank_tolerance = free.norm() * f64::EPSILON.sqrt();
                let values = eigen.eigenvalues.map(|mobility| {
                    let critical = if mobility > rank_tolerance {
                        2.0 * (r.stiffness / mobility).sqrt()
                    } else {
                        0.0
                    };
                    r.damping.max(critical)
                });
                eigen.eigenvectors
                    * Matrix3::from_diagonal(&values)
                    * eigen.eigenvectors.transpose()
            })
            .collect();
        // Include parent-world impedance in the same implicit actuator solve.
        let world_damping: Vec<_> = selected
            .iter()
            .zip(&local_rows)
            .map(|(r, (a, b))| {
                let Some(goal) = &r.world_drive else {
                    return Matrix3::zeros();
                };
                let effective = if goal.child {
                    angular(*b, *b) - angular(*b, *a)
                } else {
                    angular(*a, *a) - angular(*a, *b)
                };
                let eigen = ((effective + effective.transpose()) * 0.5).symmetric_eigen();
                let tolerance =
                    (local_bodies[*a].inverse_inertia + local_bodies[*b].inverse_inertia).norm()
                        * f64::EPSILON.sqrt();
                let values = eigen.eigenvalues.map(|mobility| {
                    goal.damping.max(if mobility > tolerance {
                        2.0 * (goal.stiffness / mobility).sqrt()
                    } else {
                        0.0
                    })
                });
                eigen.eigenvectors
                    * Matrix3::from_diagonal(&values)
                    * eigen.eigenvectors.transpose()
            })
            .collect();
        // Actuators supporting one parent keep differential limb pose. The
        // common relative mode is replaced by the parent-world orientation goal.
        let groups: Vec<Vec<usize>> = selected
            .iter()
            .map(|r| {
                if r.world_drive.as_ref().is_none_or(|g| g.child) {
                    return vec![];
                }
                selected
                    .iter()
                    .enumerate()
                    .filter_map(|(i, o)| {
                        (o.parent == r.parent && o.world_drive.as_ref().is_some_and(|g| !g.child))
                            .then_some(i)
                    })
                    .collect()
            })
            .collect();
        let weights: Vec<_> = selected
            .iter()
            .enumerate()
            .map(|(i, r)| {
                if groups[i].is_empty() {
                    return 0.0;
                }
                let sum = groups[i]
                    .iter()
                    .map(|j| selected[*j].world_drive.as_ref().unwrap().stiffness)
                    .sum::<f64>();
                r.world_drive.as_ref().map_or(0.0, |g| {
                    if sum > 0.0 {
                        g.stiffness / sum
                    } else {
                        1.0 / groups[i].len() as f64
                    }
                })
            })
            .collect();
        let impedance: Vec<_> = selected
            .iter()
            .zip(&damping)
            .map(|(r, d)| d + Matrix3::identity() * (r.stiffness * dt))
            .collect();
        let relative_response = |measurement: usize, actuator: usize| {
            let (a, b) = local_rows[measurement];
            let (c, d) = local_rows[actuator];
            angular(b, d) - angular(b, c) - angular(a, d) + angular(a, c)
        };
        let diagonal: Vec<_> = selected
            .iter()
            .enumerate()
            .map(|(i, r)| {
                let (a, b) = local_rows[i];
                let mut response = impedance[i] * relative_response(i, i);
                for j in &groups[i] {
                    response -= impedance[*j] * relative_response(*j, i) * weights[i];
                }
                if let Some(g) = &r.world_drive {
                    response += (world_damping[i] + Matrix3::identity() * (g.stiffness * dt))
                        * (if g.child {
                            angular(b, b) - angular(b, a)
                        } else {
                            angular(a, a) - angular(a, b)
                        });
                }
                (Matrix3::identity() + response * dt)
                    .try_inverse()
                    .unwrap_or_else(Matrix3::zeros)
            })
            .collect();
        let iterations = 64;
        for _ in 0..iterations {
            let mut change = 0.0f64;
            let mut scale = 1.0f64;
            for (index, (row, (a, b))) in selected.iter().zip(&local_rows).enumerate() {
                let relative = spin(&predicted, *b) - spin(&predicted, *a);
                let world_residual = row.world_drive.as_ref().map_or(Vector3::zeros(), |g| {
                    g.error * (dt * g.stiffness)
                        + (world_damping[index] + Matrix3::identity() * (g.stiffness * dt))
                            * (if g.child {
                                spin(&predicted, *b)
                            } else {
                                -spin(&predicted, *a)
                            })
                            * dt
                });
                let mut relative_residual =
                    row.error * (dt * row.stiffness) + impedance[index] * relative * dt;
                for j in &groups[index] {
                    let (pa, ch) = local_rows[*j];
                    relative_residual -= (selected[*j].error * (dt * selected[*j].stiffness)
                        + impedance[*j] * (spin(&predicted, ch) - spin(&predicted, pa)) * dt)
                        * weights[index];
                }
                let residual =
                    lambda[index] + world_residual + relative_residual - row.feed_forward * dt;
                let mut next = lambda[index] - diagonal[index] * residual;
                let bound = row.max_torque * dt;
                let magnitude = next.norm();
                if magnitude > bound && magnitude > 0.0 {
                    next *= bound / magnitude;
                }
                let delta = next - lambda[index];
                for (velocity, response) in predicted.iter_mut().zip(&actuator_response[index]) {
                    for axis in 0..3 {
                        *velocity += response[axis] * delta[axis];
                    }
                }
                lambda[index] = next;
                change = change.max(delta.norm());
                scale = scale.max(next.norm());
            }
            // Contacts can push but cannot pull. A drive unloading a sole
            // therefore changes predicted support instead of welding it down.
            for (i, (j, response, effective, friction, velocity_bias)) in normals.iter().enumerate()
            {
                if *effective <= f64::EPSILON {
                    continue;
                }
                let next =
                    (normal_lambda[i] - (j.dot(&predicted) + velocity_bias) / effective).max(0.0);
                let delta = next - normal_lambda[i];
                predicted += response * delta;
                normal_lambda[i] = next;
                change = change.max(delta.abs());
                scale = scale.max(next);
                if let Some((mu, t, r, inverse_tangent)) = friction {
                    let velocity =
                        nalgebra::Vector2::new(t[0].dot(&predicted), t[1].dot(&predicted));
                    let mut next_tangent = tangent_lambda[i] - inverse_tangent * velocity;
                    let limit = mu * next;
                    let magnitude = next_tangent.norm();
                    if magnitude > limit && magnitude > 0.0 {
                        next_tangent *= limit / magnitude;
                    }
                    let delta = next_tangent - tangent_lambda[i];
                    predicted += &r[0] * delta[0] + &r[1] * delta[1];
                    tangent_lambda[i] = next_tangent;
                    change = change.max(delta.norm());
                    scale = scale.max(next_tangent.norm());
                }
            }
            if change <= f64::EPSILON.sqrt() * scale {
                break;
            }
        }
        for (row, impulse) in selected.iter().zip(lambda) {
            result[row.parent] -= impulse;
            result[row.child] += impulse;
        }
    }
    result
}
#[cfg(test)]
mod tests;
