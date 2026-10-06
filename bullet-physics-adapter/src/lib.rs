#![forbid(unsafe_op_in_unsafe_fn)]

use std::sync::{Arc, Mutex};

mod backend;
mod capabilities;
mod config;
mod plugin_definition;
#[cfg(test)]
mod service_tests;

use abi_stable::erased_types::TD_Opaque;
use abi_stable::prefix_type::PrefixTypeTrait;
use abi_stable::std_types::{RResult, RString, RVec};
use backend::BulletPacketPhysicsBackend;
use config::{parse_backend_config, PhysicsPluginConfig, DEFAULT_SETTINGS_JSON};
use newviso_compat_abi::{
    provider::{
        Blob, CapabilityId, ConfigApplyResultV1, ConfigBlobV1, ConfigDiagV1, ConfigPatchV1,
        HostApiV1, MethodName, PluginDescriptor, PluginModule, PluginModuleDyn, PluginModule_TO,
        PluginRootV1, PluginRootV1Ref, ServiceV1, ServiceV1_TO,
    },
    signature::{BootstrapPhase, ProviderKind, ProviderSignatureV1},
};
use newviso_physics_api::*;

pub const PHYSICS_BACKEND_ID: &str = "engine.physics.bullet";
pub const PHYSICS_PROVIDER_GATEWAY_ID: &str = "engine.physics.bullet";
pub const PHYSICS_BACKEND_NAME: &str = "Bullet Physics";
pub const PHYSICS_BACKEND_VERSION: &str = env!("CARGO_PKG_VERSION");

#[derive(Default)]
struct PhysicsPlugin {
    enabled: bool,
}

impl PhysicsPlugin {
    fn descriptor() -> PluginDescriptor {
        plugin_definition::descriptor()
    }

    fn init_service(
        &mut self,
        host: HostApiV1,
        config: PhysicsPluginConfig,
    ) -> RResult<(), RString> {
        let service = PhysicsBackendService::new(
            PhysicsBackendInfo {
                backend_id: PHYSICS_BACKEND_ID.to_owned(),
                backend_name: PHYSICS_BACKEND_NAME.to_owned(),
                backend_version: PHYSICS_BACKEND_VERSION.to_owned(),
                debug_text: config.debug_text.clone(),
                capabilities: capabilities::backend_capabilities(
                    config.max_bodies,
                    config.max_queries_per_frame,
                ),
                protocol_version: PhysicsApiVersion::default(),
            },
            config,
        );

        let dyn_svc = ServiceV1_TO::from_value(service, TD_Opaque);
        match (host.register_service_v1)(dyn_svc) {
            RResult::ROk(()) => {
                log::info!(
                    "physics plugin: service registered id='{}' backend='{}'",
                    PHYSICS_SERVICE_ID,
                    PHYSICS_BACKEND_ID
                );
                self.enabled = true;
                RResult::ROk(())
            }
            RResult::RErr(error) => RResult::RErr(error),
        }
    }
}

impl PluginModule for PhysicsPlugin {
    fn descriptor(&self) -> PluginDescriptor {
        Self::descriptor()
    }

    fn config_defaults(&self) -> RResult<ConfigBlobV1, RString> {
        RResult::ROk(ConfigBlobV1 {
            content_type: "application/json".into(),
            bytes: DEFAULT_SETTINGS_JSON.as_bytes().to_vec().into(),
            format_version: 1,
        })
    }

    fn config_apply_patches(
        &self,
        base: &ConfigBlobV1,
        patches: RVec<ConfigPatchV1>,
    ) -> RResult<ConfigApplyResultV1, RString> {
        let mut effective = match parse_json_object(base.bytes.as_slice(), "Bullet defaults") {
            Ok(value) => value,
            Err(error) => return RResult::RErr(error.into()),
        };
        for patch in patches.iter() {
            let patch = match parse_json_object(patch.bytes.as_slice(), "Bullet patch") {
                Ok(value) => value,
                Err(error) => return RResult::RErr(error.into()),
            };
            merge_json_replace(&mut effective, &patch);
        }
        match serde_json::to_vec(&effective) {
            Ok(bytes) => RResult::ROk(ConfigApplyResultV1 {
                effective: ConfigBlobV1 {
                    content_type: "application/json".into(),
                    bytes: bytes.into(),
                    format_version: 1,
                },
                diags: RVec::new(),
                changed: true,
            }),
            Err(error) => RResult::RErr(error.to_string().into()),
        }
    }

    fn config_supports_live_update(&self) -> bool {
        false
    }

    fn config_update_live(
        &mut self,
        _effective: &ConfigBlobV1,
    ) -> RResult<RVec<ConfigDiagV1>, RString> {
        RResult::ROk(RVec::new())
    }

    fn init(&mut self, host: HostApiV1, effective: ConfigBlobV1) -> RResult<(), RString> {
        match parse_backend_config(&effective) {
            Ok(config) => self.init_service(host, config),
            Err(error) => RResult::RErr(error.into()),
        }
    }

    fn start(&mut self) -> RResult<(), RString> {
        RResult::ROk(())
    }

    fn fixed_update(&mut self, _dt: f32) -> RResult<(), RString> {
        RResult::ROk(())
    }

    fn update(&mut self, _dt: f32) -> RResult<(), RString> {
        RResult::ROk(())
    }

    fn render(&mut self, _dt: f32) -> RResult<(), RString> {
        RResult::ROk(())
    }

    fn shutdown(&mut self) {
        self.enabled = false;
    }
}

struct PacketPhysicsBackend {
    native: Option<BulletPacketPhysicsBackend>,
    init_error: Option<String>,
    settings: PhysicsPluginConfig,
}

impl PacketPhysicsBackend {
    fn new(settings: PhysicsPluginConfig) -> Self {
        Self {
            native: None,
            init_error: None,
            settings,
        }
    }

    fn step(&mut self, input: PhysicsFrameInput) -> Result<PhysicsFrameOutput, String> {
        if let Some(error) = &self.init_error {
            return Err(error.clone());
        }
        if self.native.is_none() {
            match BulletPacketPhysicsBackend::with_settings(
                self.settings.max_bodies,
                self.settings.max_queries_per_frame,
                self.settings.query_batch_size as usize,
                self.settings.query_threads as i32,
                self.settings.profile_steps,
            ) {
                Ok(native) => self.native = Some(native),
                Err(error) => {
                    self.init_error = Some(error.clone());
                    return Err(error);
                }
            }
        }
        self.native
            .as_mut()
            .expect("Bullet backend initialized above")
            .step_frame(input)
    }

    fn shutdown(&mut self) {
        if let Some(native) = &mut self.native {
            native.shutdown();
        }
        self.native = None;
        self.init_error = None;
    }
}

#[derive(Clone)]
struct PhysicsBackendService {
    backend: Arc<Mutex<PacketPhysicsBackend>>,
    info: PhysicsBackendInfo,
    settings: PhysicsPluginConfig,
}

impl PhysicsBackendService {
    fn new(info: PhysicsBackendInfo, settings: PhysicsPluginConfig) -> Self {
        Self {
            backend: Arc::new(Mutex::new(PacketPhysicsBackend::new(settings.clone()))),
            info,
            settings,
        }
    }

    fn ok_json<T: serde::Serialize>(value: &T) -> RResult<Blob, RString> {
        match encode_json(value) {
            Ok(bytes) => RResult::ROk(Blob::from(bytes)),
            Err(error) => RResult::RErr(error.into()),
        }
    }

    fn with_backend<T>(&self, action: impl FnOnce(&mut PacketPhysicsBackend) -> T) -> T {
        let mut guard = self
            .backend
            .lock()
            .unwrap_or_else(|error| error.into_inner());
        action(&mut guard)
    }

    fn negotiate(
        &self,
        request: PhysicsCapabilityNegotiationRequest,
    ) -> PhysicsCapabilityNegotiationResponse {
        let enabled_features = request
            .optional_features
            .iter()
            .chain(request.required_features.iter())
            .copied()
            .filter(|feature| self.info.capabilities.supports(*feature))
            .collect::<Vec<_>>();
        let missing_required_features = request
            .required_features
            .iter()
            .copied()
            .filter(|feature| !self.info.capabilities.supports(*feature))
            .collect::<Vec<_>>();
        let backend_version = self.info.protocol_version;
        let version_compatible = request.preferred_version.major == backend_version.major;
        let accepted_version = PhysicsApiVersion {
            major: backend_version.major,
            minor: request.preferred_version.minor.min(backend_version.minor),
            patch: if request.preferred_version.minor == backend_version.minor {
                request.preferred_version.patch.min(backend_version.patch)
            } else {
                0
            },
        };
        let mut notices = Vec::new();
        if !version_compatible {
            notices.push(serde_json::json!({
                "code": "physics.protocol_major_mismatch",
                "requested": request.preferred_version,
                "backend": backend_version,
            }));
        }
        PhysicsCapabilityNegotiationResponse {
            accepted_version,
            backend_version,
            ok: version_compatible && missing_required_features.is_empty(),
            enabled_features,
            missing_required_features,
            notices,
        }
    }

    fn invoke_service(&self, request: PhysicsServiceRequest) -> PhysicsServiceResponse {
        match request {
            PhysicsServiceRequest::Negotiate(request) => {
                PhysicsServiceResponse::Negotiation(self.negotiate(request))
            }
            PhysicsServiceRequest::StepFrame(input) => {
                if let Err(error) = validate_frame(&input) {
                    return PhysicsServiceResponse::Problem(
                        PhysicsProblemDetails::new(
                            "physics_invalid_frame",
                            "Physics frame validation failed",
                            error,
                        )
                        .with_backend(PHYSICS_BACKEND_ID)
                        .with_phase("validate")
                        .recoverable(true),
                    );
                }
                self.with_backend(|backend| match backend.step(input) {
                    Ok(output) => PhysicsServiceResponse::FrameOutput(output),
                    Err(error) => PhysicsServiceResponse::Problem(
                        PhysicsProblemDetails::new(
                            "physics_step_failed",
                            "Bullet physics backend step failed",
                            error,
                        )
                        .with_backend(PHYSICS_BACKEND_ID)
                        .with_phase("step"),
                    ),
                })
            }
            PhysicsServiceRequest::DiagnosticsSnapshot => {
                PhysicsServiceResponse::DiagnosticsSnapshot(self.info.clone())
            }
        }
    }
}

impl ServiceV1 for PhysicsBackendService {
    fn id(&self) -> CapabilityId {
        CapabilityId::from(PHYSICS_SERVICE_ID)
    }

    fn describe(&self) -> RString {
        serde_json::json!({
            "id": PHYSICS_SERVICE_ID,
            "version": 1,
            "methods": [
                PHYSICS_SERVICE_METHOD_INFO,
                PHYSICS_SERVICE_METHOD_INVOKE,
                "bullet.metrics.v1",
                PHYSICS_SERVICE_METHOD_SHUTDOWN_V1
            ],
            "backend_id": self.info.backend_id,
            "backend_name": self.info.backend_name,
            "backend_version": self.info.backend_version,
            "capabilities": self.info.capabilities,
            "settings": self.settings,
        })
        .to_string()
        .into()
    }

    fn call(&self, method: MethodName, payload: Blob) -> RResult<Blob, RString> {
        match method.as_str() {
            PHYSICS_SERVICE_METHOD_INFO => Self::ok_json(&self.info),
            "bullet.metrics.v1" => self.with_backend(|backend| {
                Self::ok_json(&serde_json::json!({
                    "schema": "newviso.bullet.metrics.v1",
                    "initialized": backend.native.is_some(),
                    "settings": backend.settings,
                    "frame": backend.native.as_ref().map(|native| &native.metrics),
                    "init_error": backend.init_error,
                }))
            }),
            PHYSICS_SERVICE_METHOD_SHUTDOWN_V1 => {
                self.with_backend(PacketPhysicsBackend::shutdown);
                RResult::ROk(Blob::from(Vec::new()))
            }
            PHYSICS_SERVICE_METHOD_INVOKE => {
                let request = match decode_json(payload.as_slice()) {
                    Ok(request) => request,
                    Err(error) => return RResult::RErr(error.into()),
                };
                Self::ok_json(&self.invoke_service(request))
            }
            unknown => RResult::RErr(format!("physics service: unknown method '{unknown}'").into()),
        }
    }
}

fn parse_json_object(raw: &[u8], what: &str) -> Result<serde_json::Value, String> {
    let parsed: serde_json::Value =
        serde_json::from_slice(raw).map_err(|error| format!("{what} parse failed: {error}"))?;
    parsed
        .is_object()
        .then_some(parsed)
        .ok_or_else(|| format!("{what} must be a JSON object"))
}

fn merge_json_replace(dst: &mut serde_json::Value, src: &serde_json::Value) {
    match (dst, src) {
        (serde_json::Value::Object(dst_map), serde_json::Value::Object(src_map)) => {
            for (key, value) in src_map {
                merge_json_replace(
                    dst_map
                        .entry(key.clone())
                        .or_insert(serde_json::Value::Null),
                    value,
                );
            }
        }
        (dst, src) => *dst = src.clone(),
    }
}

extern "C" fn create_module() -> PluginModuleDyn<'static> {
    PluginModule_TO::from_value(PhysicsPlugin::default(), TD_Opaque)
}

#[no_mangle]
pub extern "C" fn newengine_plugin_signature_v1() -> ProviderSignatureV1 {
    ProviderSignatureV1 {
        id: RString::from(PHYSICS_BACKEND_ID),
        name: RString::from(PHYSICS_BACKEND_NAME),
        version: RString::from(PHYSICS_BACKEND_VERSION),
        kind: ProviderKind::Runtime,
        bootstrap_phase: BootstrapPhase::Engine,
    }
}

#[no_mangle]
pub extern "C" fn newengine_plugin_root_v1() -> PluginRootV1Ref {
    PluginRootV1::leak_into_prefix(PluginRootV1 {
        create: create_module,
    })
}
