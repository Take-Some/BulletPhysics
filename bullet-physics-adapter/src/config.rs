use newviso_compat_abi::provider::ConfigBlobV1;
use serde::{Deserialize, Serialize};

pub(crate) const DEFAULT_SETTINGS_JSON: &str = r#"{"debug_text":"North Star | Bullet Physics","max_bodies":16384,"max_queries_per_frame":4096,"query_batch_size":256,"query_threads":1,"profile_steps":false}"#;

#[derive(Debug, Clone, Deserialize, Serialize)]
#[serde(default)]
pub(crate) struct PhysicsPluginConfig {
    pub debug_text: String,
    pub max_bodies: u32,
    pub max_queries_per_frame: u32,
    pub query_batch_size: u32,
    pub query_threads: u32,
    pub profile_steps: bool,
}

impl Default for PhysicsPluginConfig {
    fn default() -> Self {
        Self {
            debug_text: "North Star | Bullet Physics".into(),
            max_bodies: 16 * 1024,
            max_queries_per_frame: 4096,
            query_batch_size: 256,
            query_threads: 1,
            profile_steps: false,
        }
    }
}

pub(crate) fn parse_backend_config(blob: &ConfigBlobV1) -> Result<PhysicsPluginConfig, String> {
    if blob.bytes.is_empty() {
        return Ok(PhysicsPluginConfig::default());
    }
    let value = crate::parse_json_object(blob.bytes.as_slice(), "Bullet config")?;
    let mut config: PhysicsPluginConfig =
        serde_json::from_value(value).map_err(|error| format!("Bullet config invalid: {error}"))?;
    // Keep the existing body/query clamping contract for older project configs.
    config.max_bodies = config.max_bodies.clamp(128, 1_048_576);
    config.max_queries_per_frame = config.max_queries_per_frame.clamp(1, 65_536);
    if !(1..=256).contains(&config.query_batch_size) {
        return Err("Bullet query_batch_size must be between 1 and 256".into());
    }
    if !(1..=64).contains(&config.query_threads) {
        return Err("Bullet query_threads must be between 1 and 64".into());
    }
    Ok(config)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn parse(raw: &str) -> Result<PhysicsPluginConfig, String> {
        parse_backend_config(&ConfigBlobV1 {
            bytes: raw.as_bytes().to_vec().into(),
            content_type: "application/json".into(),
            format_version: 1,
        })
    }
    #[test]
    fn legacy_configs_keep_defaults_and_limits() {
        let config = parse(r#"{"max_bodies":64,"debug_text":"test"}"#).unwrap();
        assert_eq!(config.max_bodies, 128);
        assert_eq!(config.debug_text, "test");
        assert_eq!(config.query_batch_size, 256);
        assert_eq!(config.query_threads, 1);
        assert!(!config.profile_steps);
    }
    #[test]
    fn rejects_invalid_tuning_and_wrong_types() {
        for raw in [
            r#"{"query_batch_size":0}"#,
            r#"{"query_batch_size":257}"#,
            r#"{"query_threads":0}"#,
            r#"{"query_threads":65}"#,
            r#"{"max_bodies":"many"}"#,
            r#"{"profile_steps":1}"#,
        ] {
            assert!(parse(raw).is_err(), "accepted {raw}");
        }
    }
}
