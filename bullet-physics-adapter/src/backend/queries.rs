use super::*;
use rsbullet_core::{RayHit, RayTestBatchOptions};

#[derive(Clone, Copy)]
struct PreparedRay {
    index: usize,
    seq: u64,
    ignore: Option<u64>,
    origin: PhysicsVec3,
    unit: PhysicsVec3,
    max_t: f32,
}

impl PreparedRay {
    fn from_query(index: usize, query: &PhysicsQuery) -> Option<Self> {
        let (origin, dir, max_t) = match query.kind {
            PhysicsQueryKindDto::Ray { origin, dir, max_t }
            | PhysicsQueryKindDto::BallisticRay {
                origin, dir, max_t, ..
            } => (origin, dir, max_t),
            _ => return None,
        };
        if !max_t.is_finite()
            || max_t <= 0.0
            || origin.iter().chain(dir.iter()).any(|v| !v.is_finite())
        {
            return None;
        }
        let length = dir.iter().map(|&v| (v as f64).powi(2)).sum::<f64>().sqrt();
        if length <= 1.0e-6 {
            return None;
        }
        Some(Self {
            index,
            seq: query.seq,
            ignore: query.ignore_entity,
            origin,
            unit: dir.map(|v| (v as f64 / length) as f32),
            max_t,
        })
    }

    fn to_hit(self, entity: u64, distance: f32, normal: PhysicsVec3) -> PhysicsQueryHitDto {
        PhysicsQueryHitDto {
            seq: self.seq,
            entity,
            position: std::array::from_fn(|i| self.origin[i] + self.unit[i] * distance),
            normal,
            distance,
            subshape_id: 0,
            hit_index: 0,
            back_face: self
                .unit
                .iter()
                .zip(normal)
                .map(|(a, b)| a * b)
                .sum::<f32>()
                > 0.0,
        }
    }
}

#[derive(Clone, Copy)]
struct Bounds {
    min: [f64; 3],
    max: [f64; 3],
}

impl Bounds {
    fn union(self, other: Self) -> Self {
        Self {
            min: std::array::from_fn(|i| self.min[i].min(other.min[i])),
            max: std::array::from_fn(|i| self.max[i].max(other.max[i])),
        }
    }

    fn ray_entry(self, ray: PreparedRay, limit: f32) -> Option<f64> {
        let mut near = 0.0_f64;
        let mut far = limit as f64;
        for axis in 0..3 {
            let direction = ray.unit[axis] as f64;
            let start = ray.origin[axis] as f64;
            if direction.abs() < 1e-12 {
                if start < self.min[axis] || start > self.max[axis] {
                    return None;
                }
            } else {
                let a = (self.min[axis] - start) / direction;
                let b = (self.max[axis] - start) / direction;
                near = near.max(a.min(b));
                far = far.min(a.max(b));
                if near > far {
                    return None;
                }
            }
        }
        Some(near)
    }
}

struct QueryBox {
    entity: u64,
    position: PhysicsVec3,
    rotation: PhysicsQuat,
    half: PhysicsVec3,
    bounds: Bounds,
}

enum NodeKind {
    Leaf { start: usize, end: usize },
    Branch { left: usize, right: usize },
}
struct Node {
    bounds: Bounds,
    kind: NodeKind,
}

/// Balanced BVH of exact authored static boxes. Native meshes, convex hulls
/// and dynamic bodies continue to use Bullet's own broadphase.
#[derive(Default)]
pub(super) struct BoxQueryIndex {
    boxes: Vec<QueryBox>,
    nodes: Vec<Node>,
}

impl BoxQueryIndex {
    fn rebuild(&mut self, records: &BTreeMap<u64, BodyRecord>) {
        self.boxes.clear();
        self.nodes.clear();
        for (&entity, record) in records {
            if !record.is_exact_box() || !record.participates_in_queries {
                continue;
            }
            let ShapeSource::Primitive(CollisionShapeDto::Box { half_extents: half }) =
                record.shape
            else {
                unreachable!()
            };
            let pose = physics_pose(record.authored_position, record.authored_rotation);
            let matrix = pose.rotation.to_rotation_matrix();
            let extents: [f64; 3] = std::array::from_fn(|i| {
                (0..3).map(|j| matrix[(i, j)].abs() * half[j] as f64).sum()
            });
            let center = pose.translation.vector;
            // Conservative padding avoids pruning boundary hits due to rounding.
            let bounds = Bounds {
                min: std::array::from_fn(|i| {
                    center[i] - extents[i] - 1e-7 * (1.0 + center[i].abs() + extents[i])
                }),
                max: std::array::from_fn(|i| {
                    center[i] + extents[i] + 1e-7 * (1.0 + center[i].abs() + extents[i])
                }),
            };
            self.boxes.push(QueryBox {
                entity,
                position: record.authored_position,
                rotation: record.authored_rotation,
                half,
                bounds,
            });
        }
        if !self.boxes.is_empty() {
            self.build_node(0, self.boxes.len());
        }
    }

    fn build_node(&mut self, start: usize, end: usize) -> usize {
        let bounds = self.boxes[start + 1..end]
            .iter()
            .fold(self.boxes[start].bounds, |a, b| a.union(b.bounds));
        let index = self.nodes.len();
        self.nodes.push(Node {
            bounds,
            kind: NodeKind::Leaf { start, end },
        });
        if end - start > 4 {
            let axis = (0..3)
                .max_by(|&a, &b| {
                    (bounds.max[a] - bounds.min[a]).total_cmp(&(bounds.max[b] - bounds.min[b]))
                })
                .unwrap();
            let middle = (start + end) / 2;
            self.boxes[start..end].select_nth_unstable_by(middle - start, |a, b| {
                (a.bounds.min[axis] + a.bounds.max[axis])
                    .total_cmp(&(b.bounds.min[axis] + b.bounds.max[axis]))
                    .then(a.entity.cmp(&b.entity))
            });
            let left = self.build_node(start, middle);
            let right = self.build_node(middle, end);
            self.nodes[index].kind = NodeKind::Branch { left, right };
        }
        index
    }

    fn cast(&self, ray: PreparedRay, tests: &mut usize) -> Option<PhysicsQueryHitDto> {
        let mut best = None;
        if !self.nodes.is_empty() {
            self.visit(0, ray, &mut best, tests);
        }
        best
    }

    fn visit(
        &self,
        node: usize,
        ray: PreparedRay,
        best: &mut Option<PhysicsQueryHitDto>,
        tests: &mut usize,
    ) {
        let limit = best.as_ref().map_or(ray.max_t, |hit| hit.distance);
        if self.nodes[node].bounds.ray_entry(ray, limit).is_none() {
            return;
        }
        match self.nodes[node].kind {
            NodeKind::Leaf { start, end } => {
                for shape in &self.boxes[start..end] {
                    if ray.ignore == Some(shape.entity) {
                        continue;
                    }
                    *tests += 1;
                    if let Some((distance, normal)) = ray_authored_box(
                        ray.origin,
                        ray.unit,
                        ray.max_t,
                        shape.position,
                        shape.rotation,
                        shape.half,
                    ) {
                        if best.as_ref().is_none_or(|old| {
                            distance < old.distance
                                || (distance == old.distance && shape.entity < old.entity)
                        }) {
                            *best = Some(ray.to_hit(shape.entity, distance, normal));
                        }
                    }
                }
            }
            NodeKind::Branch { left, right } => {
                let a = self.nodes[left].bounds.ray_entry(ray, limit);
                let b = self.nodes[right].bounds.ray_entry(ray, limit);
                match (a, b) {
                    (Some(a), Some(b)) => {
                        let (first, second) = if a <= b { (left, right) } else { (right, left) };
                        self.visit(first, ray, best, tests);
                        self.visit(second, ray, best, tests);
                    }
                    (Some(_), None) => self.visit(left, ray, best, tests),
                    (None, Some(_)) => self.visit(right, ray, best, tests),
                    _ => {}
                }
            }
        }
    }
}

impl BulletPacketPhysicsBackend {
    pub(super) fn execute_queries(
        &mut self,
        input: &PhysicsFrameInput,
    ) -> Result<Vec<PhysicsQueryHitDto>, String> {
        if input.queries.is_empty() {
            return Ok(Vec::new());
        }
        if self.query_index_dirty {
            self.query_index.rebuild(&self.records);
            self.query_index_dirty = false;
            self.metrics.query_index_rebuilds += 1;
        }
        let mut groups = BTreeMap::<Option<u64>, Vec<PreparedRay>>::new();
        let mut results = vec![None; input.queries.len()];
        for (index, query) in input.queries.iter().enumerate() {
            if let Some(ray) = PreparedRay::from_query(index, query) {
                self.metrics.rays += 1;
                results[index] = self
                    .query_index
                    .cast(ray, &mut self.metrics.exact_box_tests);
                groups.entry(ray.ignore).or_default().push(ray);
            }
        }
        // Grouping shares the ignore filter across the batch. Results are
        // written by original input index, including repeated sequence values.
        for (ignore, rays) in groups {
            let suppressed = ignore
                .and_then(|entity| self.records.get(&entity))
                .filter(|record| record.query_group(record.participates_in_queries) == 1)
                .map(|record| record.body_id);
            if let Some(id) = suppressed {
                if let Err(error) = self.client.set_collision_filter_group_mask(id, -1, 2, -1) {
                    let _ = self.client.set_collision_filter_group_mask(id, -1, 1, -1);
                    return Err(bullet_error(error));
                }
            }
            let result = self.cast_native_batches(&rays, &mut results);
            // Always restore on both native query errors and success. Group
            // changes never turn off physical contacts or explicit pair filters.
            let restore = suppressed
                .map(|id| {
                    self.client
                        .set_collision_filter_group_mask(id, -1, 1, -1)
                        .map(|_| ())
                        .map_err(bullet_error)
                })
                .unwrap_or(Ok(()));
            restore?;
            result?;
        }
        Ok(results.into_iter().flatten().collect())
    }

    fn cast_native_batches(
        &mut self,
        rays: &[PreparedRay],
        results: &mut [Option<PhysicsQueryHitDto>],
    ) -> Result<(), String> {
        for chunk in rays.chunks(self.query_batch_size) {
            let from: Vec<_> = chunk.iter().map(|ray| vec3_f64(ray.origin)).collect();
            let to: Vec<_> = chunk
                .iter()
                .map(|ray| {
                    std::array::from_fn(|i| {
                        ray.origin[i] as f64 + ray.unit[i] as f64 * ray.max_t as f64
                    })
                })
                .collect();
            let native = self
                .client
                .ray_test_batch(&RayTestBatchOptions {
                    ray_from_positions: &from,
                    ray_to_positions: &to,
                    collision_filter_mask: Some(1),
                    num_threads: Some(self.query_threads),
                    ..Default::default()
                })
                .map_err(bullet_error)?;
            self.metrics.query_batches += 1;
            if native.len() != chunk.len() {
                return Err(format!(
                    "Bullet ray batch returned {} results for {} rays",
                    native.len(),
                    chunk.len()
                ));
            }
            for (&ray, hit) in chunk.iter().zip(native) {
                if let Some(hit) = self.native_query_hit(ray, hit) {
                    if results[ray.index]
                        .as_ref()
                        .is_none_or(|old| hit.distance < old.distance)
                    {
                        results[ray.index] = Some(hit);
                    }
                }
            }
        }
        Ok(())
    }

    fn native_query_hit(&self, ray: PreparedRay, hit: RayHit) -> Option<PhysicsQueryHitDto> {
        let entity = *self.body_to_entity.get(&hit.object_unique_id)?;
        let record = self.records.get(&entity)?;
        if !record.participates_in_queries || ray.ignore == Some(entity) || record.is_exact_box() {
            return None;
        }
        let mut result = ray.to_hit(
            entity,
            (hit.hit_fraction as f32).clamp(0.0, 1.0) * ray.max_t,
            vec3_f32(hit.hit_normal_world),
        );
        result.position = vec3_f32(hit.hit_position_world);
        Some(result)
    }

    #[cfg(test)]
    pub(super) fn cast_ray(
        &mut self,
        seq: u64,
        ignore_entity: Option<u64>,
        origin: PhysicsVec3,
        dir: PhysicsVec3,
        max_t: f32,
    ) -> Result<Option<PhysicsQueryHitDto>, String> {
        let mut input = PhysicsFrameInput::empty(0, 0, 1.0 / 60.0);
        input.queries.push(PhysicsQuery {
            seq,
            ignore_entity,
            kind: PhysicsQueryKindDto::Ray { origin, dir, max_t },
        });
        Ok(self.execute_queries(&input)?.into_iter().next())
    }
}
