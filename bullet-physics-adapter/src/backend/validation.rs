use super::*;

impl BulletPacketPhysicsBackend {
    pub(super) fn validate_provider_frame(
        &mut self,
        input: &PhysicsFrameInput,
    ) -> Result<(), String> {
        if input.queries.len() > self.max_queries_per_frame as usize {
            return Err(format!(
                "Bullet provider query limit exceeded: {} > {}",
                input.queries.len(),
                self.max_queries_per_frame
            ));
        }
        self.desired_entities.clear();
        for entity in input
            .bodies
            .iter()
            .map(|b| b.entity)
            .chain(input.colliders.iter().map(|c| c.entity))
        {
            if !self.desired_entities.insert(entity) {
                return Err(format!("duplicate physics entity {entity}"));
            }
        }
        let persistent = self
            .records
            .iter()
            .filter(|(id, record)| record.persistent && !self.desired_entities.contains(id))
            .count();
        if persistent + self.desired_entities.len() > self.max_bodies as usize {
            return Err(format!(
                "Bullet provider body limit exceeded: {}",
                self.max_bodies
            ));
        }
        for body in &input.bodies {
            if body.flags.is_trigger {
                return Err(format!("Bullet provider does not advertise TriggerBodies; entity {} requested a trigger", body.entity));
            }
            match body.shape {
                CollisionShapeDto::Box { half_extents } => {
                    positive_vec3(half_extents, "box half extents")?;
                }
                CollisionShapeDto::Sphere { radius } => {
                    positive(radius, "sphere radius")?;
                }
                CollisionShapeDto::Capsule {
                    radius,
                    half_height,
                }
                | CollisionShapeDto::Cylinder {
                    radius,
                    half_height,
                } => {
                    positive(radius, "shape radius")?;
                    positive(half_height, "shape half height")?;
                }
            }
        }
        for collider in &input.colliders {
            if collider.flags.is_trigger {
                return Err(format!("Bullet provider does not advertise TriggerBodies; collider {} requested a trigger", collider.entity));
            }
        }
        Ok(())
    }
}
