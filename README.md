# Bullet Physics Provider

Bullet3-backed implementation of the current NewViso `engine.physics` capability.
The provider remains isolated from runtime/gameplay code: NewViso and Bullet share
only the backend-neutral `newviso-physics-api` wire contract.

## Provider identity

- provider id / route: `engine.physics.bullet`
- engine gateway: `engine.physics`
- service: `physics.api`
- provider ABI: `newviso.physics.provider.v1`
- install name: `bullet-physics`
- backend priority: `180`

Projects can explicitly pin Bullet instead of relying on priority:

```json
{
  "capabilities": {
    "required": [
      {
        "id": "physics.backend",
        "min_version": 1,
        "provider": "engine.physics.bullet"
      }
    ]
  }
}
```

## Current contract

Bullet 0.3 uses `newviso-compat-abi` for plugin lifecycle/service registration and
`newviso-physics-api` for all frame DTOs. The old `NewEngine/neocore2`
`newengine-plugin-kit` / `newengine-physics-api` dependencies are removed.

Implemented:

- static, dynamic and kinematic bodies;
- box, sphere, capsule and cylinder primitive shapes;
- persistent triangle-mesh colliders;
- Y-up heightfields converted to persistent Bullet triangle meshes;
- pose, linear-velocity, angular-velocity and impulse commands;
- linear and angular body state feedback;
- optional `JointConstraints`: native point sockets with compliant cone-stop torques;
- family self-collision filtering and constraint cleanup on body retirement;
- optional CollisionPairFiltering for explicit body pairs, independent of joint families;
- contact begin/persist/end events and material pairs;
- ray and ballistic-ray queries;
- `ignore_entity` and `participates_in_queries` filtering;
- authored collider persistence and explicit destruction;
- capability/version negotiation and backend diagnostics;
- provider-side frame validation.

Trigger bodies and continuous collision detection are not advertised yet.

## Toolchain

This workspace is pinned to:

```text
nightly-x86_64-pc-windows-gnu
```

Nightly is required by the current `robot_behavior 0.5.4` dependency because it uses
`generic_const_exprs`. Native Bullet is built with the MSYS2 MinGW64 GCC/G++, CMake
and Ninja toolchain. Cargo uses the short build cache `C:\NSB\BP` to avoid
Windows/MinGW object dependency path limits.

`vendor\rsbullet_sys` patches upstream 0.3.1 so Bullet GUI/demo targets are not built
or linked into the headless engine provider. `BUILD_EXTRAS` remains enabled, preserving
the Direct/SharedMemory/BulletRobotics C API required by `rsbullet-core`.

`vendor\rsbullet-core` removes unconditional GUI/in-process-server FFI references
from `PhysicsClient::connect` for this headless provider build. `Mode::Direct`, normal
SharedMemory client access and enabled network transports remain intact.

## Build

```cmd
build.bat release
```

Canonical runtime artifact:

```text
pluginsRuntime\bullet-physics-0.3.0-release.dll
```

The shared plugin build pipeline accepts Cargo's normal GNU cdylib name and stages
it under the canonical North Star install name before atomic publication.

## Articulated body support

Frame DTOs accept an optional `joints` list. Native Bullet point constraints hold
local anchors together; equal/opposite bounded torques supply cone stops and
relative angular damping. The implicit torque calculation uses native body
inertia and the fixed timestep to keep light limbs stable. Active joints select
four native substeps and 120 solver iterations; the ordinary configuration is
restored after the last joint is removed. External world contacts and ray participation are
preserved. Removing or rebuilding a body removes its native constraints before
recreation; subsequent frame descriptors recreate the surviving joints.

The feature is advertised in diagnostics. Legacy negotiation remains compatible
with providers that predate this optional field. No gameplay or model-specific
joint names live in the provider. These are generic physics primitives for the engine's Endorfin character-response
integration; exact original anatomical hinge behavior remains separate work.

The native test `native_socket_holds_under_gravity_and_is_removed_with_its_body`
steps a swinging body for 240 frames, checks anchor drift and verifies cleanup.
`small_limb_cone_stop_dissipates_fast_spin_without_exploding` checks a low-inertia
body at high angular velocity. Native regression tests cover these behaviors together with explicit source-pair filtering.

## Explicit body-pair filters

Frames optionally carry disabled_collision_pairs. Each pair names two live,
distinct bodies; validation rejects invalid or repeated canonical pairs.
The provider applies requested pairs after refreshing joints, preserving the union
of explicit and family filters across body/constraint recreation. Ending a pair
restores contact unless an active joint family still suppresses it. Destroying a
body retires its explicit filters.

The runtime uses this feature while a released vehicle component overlaps its
source. The provider has no vehicle roles or model names. Native tests verify
zero overlap ejection, resumed source contacts, continued ground contacts and
filter cleanup. Query participation is independent of contact filtering.

## 0.3.0 performance and maintenance changes

The provider ABI, route, priority and engine DTOs stay compatible. The package
version is 0.3.0; the Bullet/rsbullet dependency versions are unchanged.

- Unchanged convex hulls, meshes and heightfields are compared by reference; the
  provider owns a geometry copy only when creating or rebuilding a collider.
- Static and kinematic transforms are sent to Bullet only when their authored
  pose changes. Persistent mesh transforms update the cached pose as well.
- Pose and velocity feedback comes from one native actual-state request.
  Pre-solver state is reused by joint angular limits and contact approach
  velocity reporting. Static contact metadata needs no native state request.
- Explicit collision filters apply only their differences. Removing a joint
  preserves a still-active explicit filter; rebuilding a body reapplies its pairs.
- Native rays are grouped by ignored entity and submitted in bounded batches.
  Original query order and sequence values are preserved. Query group bits
  exclude non-participating bodies while all physical collision masks remain -1.
- Exact authored static-box rays use a balanced BVH. Only creation, removal,
  movement or participation changes invalidate this index. Other shapes use
  Bullet's broadphase; thin-box support behavior is preserved.
- Oversized batches are rejected by the vendored native wrapper instead of
  silently discarding rays after its 256-element inline buffer.
- Duplicate entities, triggers and configured frame limits are checked before
  world synchronization. Transient bodies are retired before replacements, so
  streaming can replace a full-capacity frame.
- Error paths release native shapes and staged compound OBJ files. Native body
  removal completes before the registry forgets the body.

### Settings

Existing project configs can omit all new fields:

```json
{
  "debug_text": "North Star | Bullet Physics",
  "max_bodies": 16384,
  "max_queries_per_frame": 4096,
  "query_batch_size": 256,
  "query_threads": 1,
  "profile_steps": false
}
```

| Setting | Default | Accepted range / behavior |
| --- | ---: | --- |
| max_bodies | 16384 | Clamped to 128–1048576 at config load |
| max_queries_per_frame | 4096 | Clamped to 1–65536; oversized frames return an error |
| query_batch_size | 256 | 1–256; the inline Bullet ray buffer limits each native call |
| query_threads | 1 | 1–64 native ray workers; choose using measurements on the target workload |
| profile_steps | false | Adds elapsed step time when enabled; counters are always available |

Invalid setting types and out-of-range ray tuning return a configuration error.
Settings are applied at plugin initialization; live updates are not advertised.

The optional service method `bullet.metrics.v1` returns
`newviso.bullet.metrics.v1` JSON with initialized state, effective settings and
the current frame counters: native pose updates, state reads, collision-filter
updates, ray batches, rays, exact-box tests, BVH rebuilds, active body count and
fixed tick. `step_ms` is null unless `profile_steps` is enabled. Before the first
frame and after shutdown, `frame` is null. Existing service methods are unchanged.

### Reproducible performance measurements

Measured on 2026-10-06 on the same Windows computer and GNU Release toolchain.
Each workload has 10 warm-up frames and 60 measured frames. Frame cloning is
outside the timed region; native stepping, provider synchronization, feedback,
contact collection and queries are inside it. Profiling is disabled.
The original and optimized workloads return the same aggregate hit counts.

| Workload | Bodies | Rays/frame | Before median, ms | 0.3.0 median, ms | Speedup |
| --- | ---: | ---: | ---: | ---: | ---: |
| Static world | 1024 | 0 | 1.006300 | 0.103200 | 9.75× |
| Static box rays | 1024 | 256 | 4.289700 | 0.293500 | 14.62× |
| Filtered sphere rays | 512 | 256 | 59.751200 | 0.161500 | 369.98× |
| Dynamic feedback | 256 | 0 | 0.503200 | 0.364800 | 1.38× |

The filtered-ray case intentionally contains 256 non-queryable bodies and a
shared ignored entity. Its large improvement removes repeated broadphase filter
refreshes for every ray. These are provider workload timings, not whole-game FPS
or a claim that every physical simulation becomes hundreds of times faster.

Run the normal regressions:

```cmd
cargo test --release --locked --offline -p engine-physics-bullet -- --test-threads=1
```

Run the explicitly ignored informational benchmark:

```cmd
cargo test --release --locked --offline -p engine-physics-bullet provider_workload_benchmark -- --ignored --nocapture --test-threads=1
```

Format/check with the installed stable GNU formatter; runtime compilation still
uses the workspace's nightly GNU toolchain:

```cmd
cargo +stable-x86_64-pc-windows-gnu fmt --all -- --check
```

The release suite includes native regression tests, config tests, service
lifecycle/metrics tests and an optional workload benchmark. Coverage includes
600-ray chunk boundaries, mixed ignore groups and repeated sequences, rotated
and inside-box rays, BVH invalidation, persistent mesh movement, physical contact
preservation, filter unions, body replacement at capacity, joint stability,
inertia/center-of-mass behavior and sleeping-body impulses.

Ballistic rays retain the existing closest-hit fallback. Sphere/AABB overlap
queries, triggers and native swept/compound CCD are still outside the advertised
implementation. Additional discrete substeps for continuous flags do not change
that capability contract.
