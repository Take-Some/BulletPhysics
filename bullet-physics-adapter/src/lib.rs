#![forbid(unsafe_op_in_unsafe_fn)]

use std::sync::{Arc, Mutex};

mod backend;
mod capabilities;
mod plugin_definition;

use abi_stable::erased_types::TD_Opaque;
use abi_stable::std_types::{RResult, RString, RVec};
use backend::BulletPacketPhysicsBackend;
use newengine_physics_api::*;
use newengine_plugin_api::prelude::*;

pub const PHYSICS_BACKEND_ID: &str = "engine.physics.bullet";
pub const PHYSICS_PROVIDER_GATEWAY_ID: &str = "engine.physics.bullet";
pub const PHYSICS_BACKEND_NAME: &str = "Bullet Physics";
pub const PHYSICS_BACKEND_VERSION: &str = env!("CARGO_PKG_VERSION");

const DEFAULT_SETTINGS_JSON: &str = r#"{"debug_text":"North Star | Bullet Physics","max_bodies":16384,"max_queries_per_frame":4096}"#;

#[derive(Debug, Clone)]
struct PhysicsPluginConfig {
    debug_text: String,
    max_bodies: u32,
    max_queries_per_frame: u32,
}

impl Default for PhysicsPluginConfig {
    fn default() -> Self {
        Self {
            debug_text: "North Star | Bullet Physics".to_owned(),
            max_bodies: 16 * 1024,
            max_queries_per_frame: 4096,
        }
    }
}

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
                debug_text: config.debug_text,
                capabilities: capabilities::backend_capabilities(
                    config.max_bodies,
                    config.max_queries_per_frame,
                ),
                protocol_version: PhysicsApiVersion::default(),
            },
            config.max_bodies,
            config.max_queries_per_frame,
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
    max_bodies: u32,
    max_queries_per_frame: u32,
}

impl PacketPhysicsBackend {
    fn new(max_bodies: u32, max_queries_per_frame: u32) -> Self {
        Self {
            native: None,
            init_error: None,
            max_bodies,
            max_queries_per_frame,
        }
    }

    fn step(&mut self, input: PhysicsFrameInput) -> Result<PhysicsFrameOutput, String> {
        if let Some(error) = &self.init_error {
            return Err(error.clone());
        }
        if self.native.is_none() {
            match BulletPacketPhysicsBackend::new(self.max_bodies, self.max_queries_per_frame) {
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
}

impl PhysicsBackendService {
    fn new(info: PhysicsBackendInfo, max_bodies: u32, max_queries_per_frame: u32) -> Self {
        Self {
            backend: Arc::new(Mutex::new(PacketPhysicsBackend::new(
                max_bodies,
                max_queries_per_frame,
            ))),
            info,
        }
    }

    fn ok_json<T: serde::Serialize>(value: &T) -> RResult<Blob, RString> {
        match encode_json(value) {
            Ok(bytes) => RResult::ROk(Blob::from(bytes)),
            Err(error) => RResult::RErr(error.into()),
        }
    }

    fn with_backend<T>(&self, action: impl FnOnce(&mut PacketPhysicsBackend) -> T) -> T {
        let mut guard = self.backend.lock().unwrap_or_else(|error| error.into_inner());
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
        PhysicsCapabilityNegotiationResponse {
            accepted_version: PhysicsApiVersion::default(),
            backend_version: self.info.protocol_version,
            ok: missing_required_features.is_empty(),
            enabled_features,
            missing_required_features,
            notices: Vec::new(),
        }
    }

    fn invoke_service(&self, request: PhysicsServiceRequest) -> PhysicsServiceResponse {
        match request {
            PhysicsServiceRequest::Negotiate(request) => {
                PhysicsServiceResponse::Negotiation(self.negotiate(request))
            }
            PhysicsServiceRequest::StepFrame(input) => {
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
                PHYSICS_SERVICE_METHOD_SHUTDOWN_V1
            ],
            "backend_id": self.info.backend_id,
            "backend_name": self.info.backend_name,
            "backend_version": self.info.backend_version,
            "capabilities": self.info.capabilities,
        })
        .to_string()
        .into()
    }

    fn call(&self, method: MethodName, payload: Blob) -> RResult<Blob, RString> {
        match method.as_str() {
            PHYSICS_SERVICE_METHOD_INFO => Self::ok_json(&self.info),
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
            unknown => RResult::RErr(
                format!("physics service: unknown method '{unknown}'").into(),
            ),
        }
    }
}

fn parse_backend_config(blob: &ConfigBlobV1) -> Result<PhysicsPluginConfig, String> {
    if blob.bytes.is_empty() {
        return Ok(PhysicsPluginConfig::default());
    }
    let parsed = parse_json_object(blob.bytes.as_slice(), "Bullet config")?;
    let mut config = PhysicsPluginConfig::default();
    if let Some(value) = parsed.get("debug_text").and_then(serde_json::Value::as_str) {
        config.debug_text = value.to_owned();
    }
    if let Some(value) = parsed.get("max_bodies").and_then(serde_json::Value::as_u64) {
        config.max_bodies = value.min(u32::MAX as u64) as u32;
        config.max_bodies = config.max_bodies.clamp(128, 1_048_576);
    }
    if let Some(value) = parsed
        .get("max_queries_per_frame")
        .and_then(serde_json::Value::as_u64)
    {
        config.max_queries_per_frame = value.min(u32::MAX as u64) as u32;
        config.max_queries_per_frame = config.max_queries_per_frame.clamp(1, 65_536);
    }
    Ok(config)
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

/// Returns the Bullet physics plugin signature for the stable ABI boundary.
///
/// # Safety
///
/// The caller must load this symbol from an ABI-compatible plugin binary and
/// consume the returned value according to the NewEngine plugin host contract.
#[no_mangle]
pub unsafe extern "C" fn newengine_plugin_signature_v1() -> PluginSignatureV1 {
    PluginSignatureV1 {
        id: PHYSICS_BACKEND_ID.into(),
        name: PHYSICS_BACKEND_NAME.into(),
        version: PHYSICS_BACKEND_VERSION.into(),
        kind: PluginKind::Runtime,
        bootstrap_phase: PluginBootstrapPhase::Engine,
    }
}

export_newengine_plugin_descriptor_v2!(crate::plugin_definition::descriptor_v2);
export_newengine_plugin!(module = PhysicsPlugin::default());
