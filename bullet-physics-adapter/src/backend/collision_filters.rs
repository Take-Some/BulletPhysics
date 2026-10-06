use super::*;

impl BulletPacketPhysicsBackend {
    pub(super) fn sync_explicit_collision_pairs(
        &mut self,
        requested: &[[u64; 2]],
    ) -> Result<(), String> {
        let desired = requested
            .iter()
            .filter(|[a, b]| a != b && self.records.contains_key(a) && self.records.contains_key(b))
            .map(|[a, b]| if a < b { (*a, *b) } else { (*b, *a) })
            .collect::<BTreeSet<_>>();
        for (a, b) in self.explicit_filtered_pairs.difference(&desired) {
            if self.joint_filtered_pairs.contains(&(*a, *b)) {
                continue;
            }
            if let (Some(a), Some(b)) = (self.records.get(a), self.records.get(b)) {
                self.client
                    .set_collision_filter_pair(a.body_id, b.body_id, -1, -1, true)
                    .map_err(bullet_error)?;
                self.metrics.filter_pair_updates += 1;
            }
        }
        // Body retirement also retires its cached pairs; rebuilding a body
        // therefore reapplies them through this difference automatically.
        for (a, b) in desired.difference(&self.explicit_filtered_pairs) {
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
        self.explicit_filtered_pairs = desired;
        Ok(())
    }
    pub(super) fn remove_body_collision_filters(&mut self, id: u64) -> Result<(), String> {
        let pairs = self
            .explicit_filtered_pairs
            .iter()
            .copied()
            .filter(|(a, b)| *a == id || *b == id)
            .collect::<Vec<_>>();
        for (a, b) in pairs {
            if let (Some(a), Some(b)) = (self.records.get(&a), self.records.get(&b)) {
                self.client
                    .set_collision_filter_pair(a.body_id, b.body_id, -1, -1, true)
                    .map_err(bullet_error)?;
                self.metrics.filter_pair_updates += 1;
            }
            self.explicit_filtered_pairs.remove(&(a, b));
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn body(id: u64, kind: PhysicsBodyKind, position: [f32; 3]) -> PhysicsBodySnapshot {
        PhysicsBodySnapshot {
            entity: id,
            kind,
            shape: CollisionShape::Box {
                half_extents: [0.5; 3],
            },
            flags: PhysicsBodyFlags {
                casts_contacts: true,
                participates_in_queries: true,
                ..Default::default()
            },
            material: PhysicsMaterial {
                friction: 0.5,
                restitution: 0.0,
                density: 50.0,
            },
            position,
            rotation: [0.0, 0.0, 0.0, 1.0],
            linear_velocity: [0.0; 3],
            angular_velocity: [0.0; 3],
            linear_damping: None,
            angular_damping: None,
            mass_properties: None,
            convex_hulls: vec![],
            bounds_min: [-1.0; 3],
            bounds_max: [1.0; 3],
        }
    }
    #[test]
    fn overlapping_fragment_ignores_source_then_recontacts_when_filter_ends() {
        let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
        input.gravity = 0.0;
        input.bodies = vec![
            body(1, PhysicsBodyKind::Static, [0.0, 0.0, 0.0]),
            body(2, PhysicsBodyKind::Dynamic, [0.2, 0.0, 0.0]),
        ];
        input.disabled_collision_pairs = vec![[1, 2]];
        for tick in 1..12 {
            input.fixed_tick = tick;
            let out = backend.step_frame(input.clone()).unwrap();
            assert!(out.events.iter().all(|e| !matches!(
                e,
                PhysicsEvent::ContactBegin(_) | PhysicsEvent::ContactPersist(_)
            )));
            let p = out.pose_updates.iter().find(|p| p.entity == 2).unwrap();
            assert!((p.position[0] - 0.2).abs() < 1.0e-5);
        }
        input.disabled_collision_pairs.clear();
        let out = backend.step_frame(input.clone()).unwrap();
        assert!(out.events.iter().any(|e| matches!(
            e,
            PhysicsEvent::ContactBegin(_) | PhysicsEvent::ContactPersist(_)
        )));
        assert!(backend.explicit_filtered_pairs.is_empty());
    }
    #[test]
    fn source_filter_keeps_world_contacts_and_cleans_up_destroyed_body() {
        let mut backend = BulletPacketPhysicsBackend::new(8, 8).unwrap();
        let mut input = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
        input.bodies = vec![
            body(1, PhysicsBodyKind::Static, [0.0, 1.0, 0.0]),
            body(2, PhysicsBodyKind::Dynamic, [0.2, 1.0, 0.0]),
            body(3, PhysicsBodyKind::Static, [0.0, -0.5, 0.0]),
        ];
        input.disabled_collision_pairs = vec![[1, 2]];
        let mut touched_ground = false;
        for tick in 1..90 {
            input.fixed_tick = tick;
            let out = backend.step_frame(input.clone()).unwrap();
            touched_ground |= out.events.iter().any(|e| match e {
                PhysicsEvent::ContactBegin(c) | PhysicsEvent::ContactPersist(c) => {
                    (c.a == 2 && c.b == 3) || (c.a == 3 && c.b == 2)
                }
                _ => false,
            });
        }
        assert!(touched_ground);
        input.bodies.retain(|b| b.entity != 1);
        input.disabled_collision_pairs.clear();
        backend.step_frame(input).unwrap();
        assert!(backend.explicit_filtered_pairs.is_empty());
    }
}
