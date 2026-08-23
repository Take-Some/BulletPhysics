#![forbid(unsafe_op_in_unsafe_fn)]

use newengine_physics_api::{
    PHYSICS_BACKEND_CAPABILITY_ID, PHYSICS_BACKEND_SERVICE_SPEC, PHYSICS_PROVIDER_ABI_ID,
    PHYSICS_SERVICE_ID,
};
use newengine_plugin_api::prelude::*;

use crate::{
    PHYSICS_BACKEND_ID, PHYSICS_BACKEND_NAME, PHYSICS_BACKEND_VERSION,
    PHYSICS_PROVIDER_GATEWAY_ID,
};

const PHYSICS_SERVICES: &[PluginServiceDefinition] = &[plugin_service(
    PHYSICS_SERVICE_ID,
    1,
    r#"{"role":"physics-backend-bridge","contract":"physics.api"}"#,
)];

const PHYSICS_BACKEND_ROUTES: &[PluginBackendRouteDefinition] = &[optional_backend_route_with_abi(
    PHYSICS_BACKEND_CAPABILITY_ID,
    PHYSICS_BACKEND_SERVICE_SPEC,
    PHYSICS_PROVIDER_ABI_ID,
    Some(PHYSICS_PROVIDER_GATEWAY_ID),
    Some("bullet"),
    None,
    180,
    &[],
    &[],
    &[],
)];

const PLUGIN_DEFINITION: PluginDefinition = PluginDefinition {
    id: PHYSICS_BACKEND_ID,
    name: PHYSICS_BACKEND_NAME,
    version: PHYSICS_BACKEND_VERSION,
    kind: PluginKind::Runtime,
    services: PHYSICS_SERVICES,
    backend_routes: PHYSICS_BACKEND_ROUTES,
    capabilities: &[],
};

pub(crate) fn descriptor() -> PluginDescriptor {
    PLUGIN_DEFINITION.descriptor()
}

#[cfg(test)]
mod abi_tests {
    use super::*;

    #[test]
    fn bullet_descriptor_conforms_to_physics_provider_contract() {
        let descriptor = descriptor();
        let report = newengine_contract_conformance::validate_provider_abi(
            &descriptor,
            newengine_physics_api::PHYSICS_BACKEND_SERVICE_SPEC,
            newengine_physics_api::PHYSICS_PROVIDER_ABI_CONTRACT_SPEC,
        )
        .expect("provider descriptor contract conformance");
        assert_eq!(
            report.contract_key,
            newengine_physics_api::PHYSICS_PROVIDER_ABI_CONTRACT_SPEC.key
        );
    }
}

