use thiserror::Error;

#[derive(Debug, Error)]
pub enum ArxError {
    #[error("motor error: {0}")]
    Motor(#[from] motor_core::error::MotorError),

    #[error("io error: {0}")]
    Io(#[from] std::io::Error),

    #[error("yaml error: {0}")]
    Yaml(#[from] serde_yaml::Error),

    #[error("config error: {0}")]
    Config(String),

    #[error("not connected")]
    NotConnected,

    #[error("not enabled")]
    NotEnabled,

    #[error("runtime error: {0}")]
    Runtime(String),

    #[error("invalid argument: {0}")]
    InvalidArgument(String),
}

pub type Result<T> = std::result::Result<T, ArxError>;
