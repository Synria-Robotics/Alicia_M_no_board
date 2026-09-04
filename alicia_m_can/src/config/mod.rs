pub mod defaults;
pub mod schema;

pub use defaults::resolve_hw_cfg_path;
pub use schema::{
    default_config, load_cfg, ArxMCanConfig, GripperButtonConfig, GripperMapping, JointCfg,
    JointMotorConfig, LoadedCfg,
};
