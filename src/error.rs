/// Top-level error type for Tri-Link core.
#[derive(Debug, thiserror::Error)]
pub enum TriError {
    #[error("pose not found for timestamp {0} µs (outside buffer tolerance)")]
    PoseNotFound(u64),

    #[error("I/O error: {0}")]
    Io(#[from] std::io::Error),

    #[error("config error: {0}")]
    Config(String),
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_not_found_message_includes_timestamp() {
        let err = TriError::PoseNotFound(123_456_789);
        assert!(err.to_string().contains("123456789"));
    }

    #[test]
    fn config_error_message_includes_detail() {
        let err = TriError::Config("missing field: base_url".to_string());
        assert!(err.to_string().contains("missing field: base_url"));
    }

    #[test]
    fn io_error_converts_via_from() {
        let io_err = std::io::Error::new(std::io::ErrorKind::NotFound, "file missing");
        let tri_err = TriError::from(io_err);
        assert!(matches!(tri_err, TriError::Io(_)));
        assert!(tri_err.to_string().contains("file missing"));
    }
}
