use newviso_physics_api::{
    PhysicsBackendCapabilities, PhysicsBackendClass, PhysicsFeature, PhysicsLimits,
};

pub(crate) fn backend_capabilities(
    max_bodies: u32,
    max_queries_per_frame: u32,
) -> PhysicsBackendCapabilities {
    PhysicsBackendCapabilities {
        backend_class: PhysicsBackendClass::Native,
        features: vec![
            PhysicsFeature::StaticColliders,
            PhysicsFeature::DynamicBodies,
            PhysicsFeature::KinematicBodies,
            PhysicsFeature::Contacts,
            PhysicsFeature::Queries,
            PhysicsFeature::NativeBackend,
            PhysicsFeature::HeightfieldColliders,
            PhysicsFeature::MeshColliders,
            PhysicsFeature::AngularVelocity,
            PhysicsFeature::JointConstraints,
            PhysicsFeature::CollisionPairFiltering,
        ],
        limits: PhysicsLimits {
            max_bodies,
            max_queries_per_frame,
            max_substeps: 4,
        },
    }
}
