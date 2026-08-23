# Bullet Physics Provider

Alternative Bullet3-backed implementation of the North Star `physics.api`
provider contract. It is intentionally a separate plugin and does not link
Gravitas/Jolt or engine runtime internals.

## Provider identity

- plugin id / route: `engine.physics.bullet`
- service: `physics.api`
- provider ABI: `newengine.physics-provider/v1`
- install name: `bullet-physics`
- default priority: `180` (Gravitas remains the default at `200`)

The native Bullet3 C API is compiled by `rsbullet-core` / `rsbullet-sys` into
the plugin artifact. Runtime code sees only backend-neutral DTO packets.

## Implemented test scope

- static, dynamic and kinematic bodies plus authored static colliders;
- box, sphere and capsule shapes;
- static triangle meshes;
- Y-up heightfields converted to persistent triangle meshes;
- linear velocity and pose commands;
- contact begin/persist/end events;
- ray queries, including owner-body exclusion for ground probes;
- persistent authored colliders and explicit destruction;
- diagnostics and capability negotiation.

Trigger bodies are deliberately not advertised yet. Sending one returns a
stable `physics_step_failed` problem instead of silently giving it solid-body
semantics.

## Toolchain

The workspace pins Rust nightly through `rust-toolchain.toml` because the
current `rsbullet-core` dependency graph includes `robot_behavior`, which uses
`generic_const_exprs`.

## Build

```cmd
build.bat release
```

The canonical runtime artifact is:

```text
pluginsRuntime\bullet-physics-0.1.0-release.dll
```

## Select Bullet for a test run

Both providers can remain installed. Gravitas has the higher default priority.
For a Bullet-only smoke run, exclude Gravitas for the current process through
the host composition policy:

```cmd
set NEWENGINE_PLUGIN_EXCLUDE_IDS=engine.physics.gravitas
cargo run --bin NewEngine --profile dev
```

Expected route diagnostic:

```text
engine.physics -> physics.api owner:engine.physics.bullet
```

## Contract test

```cmd
cargo test -p engine-physics-bullet bullet_descriptor_conforms_to_physics_provider_contract
```
