use std::path::{Path, PathBuf};

use crate::error::{ArxError, Result};

pub fn default_hw_cfg_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("config/profiles/px6d/alicia_m_can_dm.yaml")
}

pub fn resolve_hw_cfg_path(hw_yaml: Option<&Path>) -> Result<PathBuf> {
    match hw_yaml {
        None => {
            let path = default_hw_cfg_path();
            if !path.exists() {
                return Err(ArxError::Config(format!("{} not found", path.display())));
            }
            Ok(path)
        }
        Some(path) if path.is_absolute() => {
            if !path.exists() {
                return Err(ArxError::Config(format!(
                    "hardware config not found: {}",
                    path.display()
                )));
            }
            Ok(path.to_path_buf())
        }
        Some(path) => {
            let resolved = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
                .join("config")
                .join(path);
            if !resolved.exists() {
                return Err(ArxError::Config(format!(
                    "hardware config not found: {}",
                    resolved.display()
                )));
            }
            Ok(resolved)
        }
    }
}
