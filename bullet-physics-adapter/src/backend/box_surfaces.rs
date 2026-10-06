use super::*;

impl BulletPacketPhysicsBackend {
    pub(super) fn create_frame_collision_shape(
        &mut self,
        source: &ShapeSource,
        kind: PhysicsBodyKindDto,
    ) -> Result<i32, String> {
        if let (
            PhysicsBodyKindDto::Static,
            ShapeSource::Primitive(CollisionShapeDto::Box { half_extents }),
        ) = (kind, source)
        {
            let h = positive_vec3(*half_extents, "box half extents")?;
            let min = h.iter().copied().fold(f32::INFINITY, f32::min);
            let max = h.iter().copied().fold(0.0, f32::max);
            if max / min > 100.0 {
                // Very wide thin convex boxes can select the underside during
                // EPA penetration recovery. Their exact triangle shell gives
                // small dynamic convex bodies a stable support surface.
                let vertices = vec![
                    [-h[0], -h[1], -h[2]],
                    [h[0], -h[1], -h[2]],
                    [-h[0], h[1], -h[2]],
                    [h[0], h[1], -h[2]],
                    [-h[0], -h[1], h[2]],
                    [h[0], -h[1], h[2]],
                    [-h[0], h[1], h[2]],
                    [h[0], h[1], h[2]],
                ]
                .into_iter()
                .map(vec3_f64)
                .collect();
                let indices = vec![
                    2, 6, 7, 2, 7, 3, 0, 1, 5, 0, 5, 4, 0, 2, 3, 0, 3, 1, 4, 5, 7, 4, 7, 6, 0, 4,
                    6, 0, 6, 2, 1, 3, 7, 1, 7, 5,
                ];
                return self.create_mesh_collision_shape(vertices, indices);
            }
        }
        self.create_collision_shape(source)
    }
}
