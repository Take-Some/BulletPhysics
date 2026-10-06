use super::*;

fn value(result: RResult<Blob, RString>) -> serde_json::Value {
    match result {
        RResult::ROk(blob) => serde_json::from_slice(blob.as_slice()).unwrap(),
        RResult::RErr(error) => panic!("service call failed: {error}"),
    }
}

#[test]
fn metrics_service_reports_tuning_and_shared_backend_lifecycle() {
    let settings = PhysicsPluginConfig {
        query_batch_size: 64,
        profile_steps: true,
        ..Default::default()
    };
    let info = PhysicsBackendInfo {
        backend_id: PHYSICS_BACKEND_ID.into(),
        backend_name: PHYSICS_BACKEND_NAME.into(),
        backend_version: PHYSICS_BACKEND_VERSION.into(),
        debug_text: settings.debug_text.clone(),
        capabilities: capabilities::backend_capabilities(
            settings.max_bodies,
            settings.max_queries_per_frame,
        ),
        protocol_version: PhysicsApiVersion::default(),
    };
    let service = PhysicsBackendService::new(info, settings);
    let metrics = value(service.call("bullet.metrics.v1".into(), Blob::from(Vec::new())));
    assert_eq!(metrics["initialized"], false);
    assert_eq!(metrics["settings"]["query_batch_size"], 64);
    assert!(metrics["frame"].is_null());
    let input = PhysicsServiceRequest::StepFrame(PhysicsFrameInput::empty(3, 12, 1.0 / 60.0));
    let output = value(service.call(
        PHYSICS_SERVICE_METHOD_INVOKE.into(),
        Blob::from(encode_json(&input).unwrap()),
    ));
    assert_eq!(output["FrameOutput"]["fixed_tick"], 12);
    let metrics = value(service.call("bullet.metrics.v1".into(), Blob::from(Vec::new())));
    assert_eq!(metrics["initialized"], true);
    assert_eq!(metrics["frame"]["fixed_tick"], 12);
    assert!(metrics["frame"]["step_ms"].as_f64().unwrap() >= 0.0);
    assert!(matches!(
        service.clone().call(
            PHYSICS_SERVICE_METHOD_SHUTDOWN_V1.into(),
            Blob::from(Vec::new())
        ),
        RResult::ROk(_)
    ));
    assert_eq!(
        value(service.call("bullet.metrics.v1".into(), Blob::from(Vec::new())))["initialized"],
        false
    );
}
