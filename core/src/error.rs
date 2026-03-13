/// Top-level error type for Tri-Link core.
#[derive(Debug, thiserror::Error)]
pub enum TriError {
    #[error("pose not found for timestamp {0} µs (outside buffer tolerance)")]
    PoseNotFound(u64),

    #[error("inference error: {0}")]
    Inference(String),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("config error: {0}")]
    Config(String),

    #[cfg(feature = "sqlite")]
    #[error("SQLite error: {0}")]
    Sqlite(#[from] rusqlite::Error),
}
