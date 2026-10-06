use super::*;
use std::{hint::black_box, time::Instant};

// Deliberately ignored: timings are informational and machine dependent.
// cargo test --release provider_workload_benchmark -- --ignored --nocapture --test-threads=1
#[test]
#[ignore]
fn provider_workload_benchmark() {
    for (name, count, query_count, dynamic, filtered) in [
        ("static_world", 1024, 0, false, false),
        ("static_box_rays", 1024, 256, false, false),
        ("filtered_sphere_rays", 512, 256, false, true),
        ("dynamic_feedback", 256, 0, true, false),
    ] {
        let mut backend = BulletPacketPhysicsBackend::new(4096, 4096).unwrap();
        let mut frame = PhysicsFrameInput::empty(1, 1, 1.0 / 60.0);
        frame.gravity = 0.0;
        for index in 0..count {
            let mut body = tests::body(
                index as u64 + 1,
                if dynamic {
                    PhysicsBodyKindDto::Dynamic
                } else {
                    PhysicsBodyKindDto::Static
                },
                CollisionShapeDto::Box {
                    half_extents: [0.5; 3],
                },
                [(index % 32) as f32 * 3.0, 0.0, (index / 32) as f32 * 3.0],
            );
            if filtered {
                body.shape = CollisionShapeDto::Sphere { radius: 0.5 };
            }
            body.flags.participates_in_queries = !filtered || index % 2 == 0;
            frame.bodies.push(body);
        }
        for index in 0..query_count {
            frame.queries.push(PhysicsQuery {
                seq: index as u64,
                ignore_entity: filtered.then_some(1),
                kind: PhysicsQueryKindDto::Ray {
                    origin: [(index % 32) as f32 * 3.0, 5.0, (index / 32) as f32 * 3.0],
                    dir: [0.0, -1.0, 0.0],
                    max_t: 10.0,
                },
            });
        }
        let mut samples = Vec::new();
        let mut total_hits = 0;
        for tick in 0..70 {
            let mut input = frame.clone();
            input.fixed_tick = tick;
            let start = Instant::now();
            let output = black_box(backend.step_frame(black_box(input)).unwrap());
            let elapsed = start.elapsed().as_secs_f64() * 1000.0;
            if tick >= 10 {
                samples.push(elapsed);
                total_hits += output.query_hits.len();
            }
        }
        samples.sort_by(f64::total_cmp);
        eprintln!("BENCH {name} bodies={count} rays={query_count} median_ms={:.6} p95_ms={:.6} hits={total_hits}",
            samples[samples.len()/2], samples[samples.len()*95/100]);
    }
}
