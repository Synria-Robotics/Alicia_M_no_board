use std::process::Command;

use crate::error::{ArxError, Result};

#[derive(Debug, Clone)]
pub struct ScanOptions {
    pub start_id: u16,
    pub end_id: u16,
    pub model: String,
    pub timeout_ms: u32,
    pub feedback_base: String,
}

impl Default for ScanOptions {
    fn default() -> Self {
        Self {
            start_id: 1,
            end_id: 16,
            model: "4340P".to_string(),
            timeout_ms: 30,
            feedback_base: "0x10".to_string(),
        }
    }
}

pub fn scan_ids(port: &str, baud: u32, opts: &ScanOptions) -> Result<Vec<u16>> {
    let output = Command::new(std::env::var("PYTHON").unwrap_or_else(|_| "python3".to_string()))
        .args([
            "-m",
            "motorbridge.cli",
            "scan",
            "--vendor",
            "damiao",
            "--transport",
            "dm-serial",
            "--serial-port",
            port,
            "--serial-baud",
            &baud.to_string(),
            "--model",
            &opts.model,
            "--start-id",
            &opts.start_id.to_string(),
            "--end-id",
            &opts.end_id.to_string(),
            "--feedback-base",
            &opts.feedback_base,
            "--timeout-ms",
            &opts.timeout_ms.to_string(),
        ])
        .output()
        .map_err(|e| ArxError::Runtime(format!("motorbridge scan failed to start: {e}")))?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        let stdout = String::from_utf8_lossy(&output.stdout);
        return Err(ArxError::Runtime(if !stderr.trim().is_empty() {
            stderr.trim().to_string()
        } else if !stdout.trim().is_empty() {
            stdout.trim().to_string()
        } else {
            "motorbridge scan failed".to_string()
        }));
    }

    let stdout = String::from_utf8_lossy(&output.stdout);
    Ok(parse_scan_hits(&stdout))
}

fn parse_scan_hits(stdout: &str) -> Vec<u16> {
    let mut hits = Vec::new();
    for line in stdout.lines() {
        if !line.contains("[hit]") {
            continue;
        }
        let Some(hex) = line.split("probe=0x").nth(1) else {
            continue;
        };
        let token = hex.split_whitespace().next().unwrap_or(hex);
        if let Ok(id) = u16::from_str_radix(token.trim_end_matches(':'), 16) {
            hits.push(id);
        }
    }
    hits
}

#[cfg(test)]
mod tests {
    use super::parse_scan_hits;

    #[test]
    fn parse_scan_hits_ignores_misses_and_summary() {
        let stdout = "\
[hit] vendor=damiao probe=0x01 esc_id=0x1 mst_id=0x1\n\
[.. ] vendor=damiao probe=0x02 no reply\n\
scan done: 1 motor(s) found\n\
  probe=0x01 vendor=damiao esc_id=0x1 mst_id=0x1\n\
[hit] vendor=damiao probe=0x03 esc_id=0x3 mst_id=0x3\n";
        assert_eq!(parse_scan_hits(stdout), vec![1, 3]);
    }
}
