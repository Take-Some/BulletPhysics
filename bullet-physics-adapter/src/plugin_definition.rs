#![forbid(unsafe_op_in_unsafe_fn)]

use newviso_compat_abi::provider::{
    CapabilityDesc, CapabilityKind, CapabilityRole, PluginDescriptor, PluginKind,
};
use newviso_physics_api::{
    ENGINE_PHYSICS_SERVICE_ID, PHYSICS_BACKEND_CAPABILITY_ID, PHYSICS_PROVIDER_ABI_ID,
    PHYSICS_SERVICE_ID,
};

use crate::{
    PHYSICS_BACKEND_ID, PHYSICS_BACKEND_NAME, PHYSICS_BACKEND_VERSION, PHYSICS_PROVIDER_GATEWAY_ID,
};

pub(crate) fn descriptor() -> PluginDescriptor {
    PluginDescriptor {
        id: PHYSICS_BACKEND_ID.into(),
        name: PHYSICS_BACKEND_NAME.into(),
        version: PHYSICS_BACKEND_VERSION.into(),
        kind: PluginKind::Runtime,
        capabilities: vec![
            CapabilityDesc {
                id: PHYSICS_SERVICE_ID.into(),
                role: CapabilityRole::Provides,
                kind: CapabilityKind::ServiceV1,
                version: 1,
                describe_json: serde_json::json!({
                    "role": "physics-backend-bridge",
                    "service": PHYSICS_SERVICE_ID,
                    "provider": PHYSICS_BACKEND_ID
                })
                .to_string()
                .into(),
            },
            CapabilityDesc {
                id: PHYSICS_BACKEND_CAPABILITY_ID.into(),
                role: CapabilityRole::Provides,
                kind: CapabilityKind::ServiceV1,
                version: 1,
                describe_json: serde_json::json!({
                    "engine_gateway": ENGINE_PHYSICS_SERVICE_ID,
                    "contract": PHYSICS_SERVICE_ID,
                    "service_id": PHYSICS_SERVICE_ID,
                    "backend_priority": 180,
                    "provider_route": PHYSICS_PROVIDER_GATEWAY_ID,
                    "provider_abi": PHYSICS_PROVIDER_ABI_ID,
                    "backend": "bullet",
                    "features": [
                        "static-colliders",
                        "dynamic-bodies",
                        "kinematic-bodies",
                        "contacts",
                        "queries",
                        "mesh-colliders",
                        "heightfield-colliders",
                        "angular-velocity",
                        "joint-constraints",
                        "collision-pair-filtering",
                        "batched-ray-queries"
                    ]
                })
                .to_string()
                .into(),
            },
        ]
        .into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn descriptor_exposes_current_newviso_physics_contract() {
        let descriptor = descriptor();
        assert_eq!(descriptor.id.as_str(), PHYSICS_BACKEND_ID);
        assert!(descriptor.capabilities.iter().any(|capability| {
            capability.id.as_str() == PHYSICS_SERVICE_ID
                && capability.role == CapabilityRole::Provides
        }));
        assert!(descriptor.capabilities.iter().any(|capability| {
            capability.id.as_str() == PHYSICS_BACKEND_CAPABILITY_ID
                && capability
                    .describe_json
                    .as_str()
                    .contains(PHYSICS_PROVIDER_ABI_ID)
        }));
    }
}
