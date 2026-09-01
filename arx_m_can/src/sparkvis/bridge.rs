use std::collections::HashMap;
use std::io::{Read, Write};
use std::net::TcpStream;
use std::sync::{Arc, Mutex};
use std::thread::{self, JoinHandle};
use std::time::Instant;

use crate::error::{ArxError, Result};
use crate::robot::ArxMCanRobot;
use crate::sparkvis::parse::{merge_joint_target, parse_joint_values, NUM_ARM_JOINTS};
use crate::utils::speed_mapping::ALICIA_SPEED_MAX;

#[derive(Debug, Clone)]
pub struct SparkVisConfig {
    pub ws_url: String,
    pub http_base: String,
    pub speed: f64,
    pub gripper_speed: f64,
    pub command_hz: f64,
    pub watchdog_sec: f64,
    pub control_gripper: bool,
    pub dry_run: bool,
    pub sync_initial: bool,
    pub max_joint_accel_rad_s2: Option<f64>,
    pub max_joint_jerk_rad_s3: Option<f64>,
}

impl Default for SparkVisConfig {
    fn default() -> Self {
        Self {
            ws_url: "ws://127.0.0.1:8000/ws/teleop".to_string(),
            http_base: "http://127.0.0.1:8000".to_string(),
            speed: 100.0,
            gripper_speed: 100.0,
            command_hz: 50.0,
            watchdog_sec: 0.35,
            control_gripper: false,
            dry_run: false,
            sync_initial: true,
            max_joint_accel_rad_s2: None,
            max_joint_jerk_rad_s3: None,
        }
    }
}

struct BridgeState {
    q: [f64; NUM_ARM_JOINTS],
    gripper: Option<f64>,
    pending: bool,
    armed: bool,
    stop: bool,
    last_msg: Option<Instant>,
}

pub struct SparkVisBridge {
    config: SparkVisConfig,
    robot: Option<ArxMCanRobot>,
    cmd_robot: Option<Arc<Mutex<Option<ArxMCanRobot>>>>,
    state: Arc<Mutex<BridgeState>>,
    cmd_handle: Option<JoinHandle<()>>,
    ws_handle: Option<JoinHandle<()>>,
}

impl SparkVisBridge {
    pub fn new(config: SparkVisConfig) -> Self {
        Self {
            config,
            robot: None,
            cmd_robot: None,
            state: Arc::new(Mutex::new(BridgeState {
                q: [0.0; NUM_ARM_JOINTS],
                gripper: None,
                pending: false,
                armed: false,
                stop: false,
                last_msg: None,
            })),
            cmd_handle: None,
            ws_handle: None,
        }
    }

    pub fn connect_robot(&mut self, robot: ArxMCanRobot) -> Result<()> {
        if self.config.dry_run {
            return Ok(());
        }
        let q = robot.get_robot_state_joint()?;
        let mut arr = [0.0; NUM_ARM_JOINTS];
        for (i, v) in q.iter().take(NUM_ARM_JOINTS).enumerate() {
            arr[i] = *v;
        }
        self.state.lock().unwrap().q = arr;
        if self.config.sync_initial {
            publish_joint_state(&self.config.http_base, &arr)?;
        }
        self.robot = Some(robot);
        Ok(())
    }

    pub fn arm(&self) {
        self.state.lock().unwrap().armed = true;
    }

    pub fn disarm(&self) {
        let mut st = self.state.lock().unwrap();
        st.armed = false;
        st.pending = false;
    }

    pub fn apply_joint_update(&self, values: HashMap<String, f64>) {
        let (partial, gripper) = parse_joint_values(&values);
        if partial.is_empty() && gripper.is_none() {
            return;
        }
        let mut st = self.state.lock().unwrap();
        merge_joint_target(&mut st.q, &partial);
        if gripper.is_some() {
            st.gripper = gripper;
        }
        st.pending = true;
        st.last_msg = Some(Instant::now());
    }

    pub fn start(&mut self) -> Result<()> {
        if self.config.dry_run {
            self.arm();
        }
        let state = self.state.clone();
        let config = self.config.clone();
        let robot = Arc::new(Mutex::new(self.robot.take()));
        self.cmd_robot = Some(robot.clone());
        self.cmd_handle = Some(thread::spawn(move || command_loop(state, config, robot)));
        let state = self.state.clone();
        let ws_url = self.config.ws_url.clone();
        self.ws_handle = Some(thread::spawn(move || {
            let _ = ws_loop(&ws_url, state);
        }));
        Ok(())
    }

    pub fn shutdown(&mut self) {
        self.state.lock().unwrap().stop = true;
        if let Some(h) = self.cmd_handle.take() {
            let _ = h.join();
        }
        if let Some(h) = self.ws_handle.take() {
            let _ = h.join();
        }
        if let Some(cmd_robot) = self.cmd_robot.take() {
            if let Ok(mut slot) = cmd_robot.lock() {
                if let Some(mut robot) = slot.take() {
                    let _ = robot.disconnect();
                }
            }
        } else if let Some(mut robot) = self.robot.take() {
            let _ = robot.disconnect();
        }
    }
}

impl Drop for SparkVisBridge {
    fn drop(&mut self) {
        self.shutdown();
    }
}

fn command_loop(
    state: Arc<Mutex<BridgeState>>,
    config: SparkVisConfig,
    robot: Arc<Mutex<Option<ArxMCanRobot>>>,
) {
    let period = std::time::Duration::from_secs_f64(1.0 / config.command_hz.max(1.0));
    let speed = config.speed.clamp(0.0, ALICIA_SPEED_MAX);
    let gripper_speed = config.gripper_speed.clamp(0.0, ALICIA_SPEED_MAX);
    loop {
        let cmd = {
            let mut st = state.lock().unwrap();
            if st.stop {
                return;
            }
            if !st.armed || !st.pending {
                None
            } else if st
                .last_msg
                .is_some_and(|t| t.elapsed().as_secs_f64() > config.watchdog_sec)
            {
                None
            } else {
                st.pending = false;
                Some((st.q, st.gripper))
            }
        };
        if let Some((q, gripper)) = cmd {
            if config.dry_run {
                println!("[sparkvis dry-run] q={q:?} gripper={gripper:?}");
            } else if let Ok(mut slot) = robot.lock() {
                if let Some(robot) = slot.as_mut() {
                    let _ = robot.set_robot_state(
                        Some(&q),
                        if config.control_gripper { gripper } else { None },
                        speed,
                        gripper_speed,
                        false,
                        None,
                        None,
                        config.max_joint_accel_rad_s2,
                        config.max_joint_jerk_rad_s3,
                        true,
                        None,
                        None,
                    );
                }
            }
        }
        thread::sleep(period);
    }
}

fn ws_loop(ws_url: &str, state: Arc<Mutex<BridgeState>>) -> Result<()> {
    let (host, port, path) = parse_ws_url(ws_url)?;
    let mut stream = TcpStream::connect(format!("{host}:{port}"))
        .map_err(|e| ArxError::Runtime(format!("connect ws: {e}")))?;
    ws_handshake(&mut stream, &host, port, &path)?;
    loop {
        if state.lock().unwrap().stop {
            return Ok(());
        }
        if let Some(text) = read_ws_text_frame(&mut stream)? {
            if let Ok(msg) = serde_json::from_str::<serde_json::Value>(&text) {
                if msg.get("type").and_then(|v| v.as_str()) == Some("joint_update") {
                    if let Some(obj) = msg.get("joint_values").and_then(|v| v.as_object()) {
                        let mut map = HashMap::new();
                        for (k, v) in obj {
                            if let Some(n) = v.as_f64() {
                                map.insert(k.clone(), n);
                            }
                        }
                        let (partial, gripper) = parse_joint_values(&map);
                        if !partial.is_empty() || gripper.is_some() {
                            let mut st = state.lock().unwrap();
                            merge_joint_target(&mut st.q, &partial);
                            if gripper.is_some() {
                                st.gripper = gripper;
                            }
                            st.pending = true;
                            st.last_msg = Some(Instant::now());
                        }
                    }
                }
            }
        }
    }
}

fn parse_ws_url(ws_url: &str) -> Result<(String, u16, String)> {
    let rest = ws_url
        .strip_prefix("ws://")
        .ok_or_else(|| ArxError::Runtime("expected ws:// URL".into()))?;
    let (host_port, path) = match rest.split_once('/') {
        Some((hp, p)) => (hp, format!("/{p}")),
        None => (rest, "/ws/teleop".to_string()),
    };
    if let Some((host, port)) = host_port.split_once(':') {
        Ok((host.to_string(), port.parse().map_err(|_| ArxError::Runtime("bad port".into()))?, path))
    } else {
        Ok((host_port.to_string(), 8000, path))
    }
}

fn publish_joint_state(http_base: &str, q: &[f64; NUM_ARM_JOINTS]) -> Result<()> {
    let url = format!("{}/api/sparkvis/joint_state", http_base.trim_end_matches('/'));
    ureq::post(&url)
        .set("Content-Type", "application/json")
        .send_json(serde_json::json!({
            "values": q,
            "joint_name_style": "title",
        }))
        .map_err(|e| ArxError::Runtime(format!("joint_state sync failed: {e}")))?;
    Ok(())
}

fn ws_handshake(stream: &mut TcpStream, host: &str, port: u16, path: &str) -> Result<()> {
    let key = base64_encode(&[7u8; 16]);
    let req = format!(
        "GET {path} HTTP/1.1\r\nHost: {host}:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n"
    );
    stream
        .write_all(req.as_bytes())
        .map_err(|e| ArxError::Runtime(e.to_string()))?;
    let mut buf = [0u8; 4096];
    let n = stream
        .read(&mut buf)
        .map_err(|e| ArxError::Runtime(e.to_string()))?;
    let resp = String::from_utf8_lossy(&buf[..n]);
    if !resp.contains("101") {
        return Err(ArxError::Runtime(format!("websocket handshake failed: {resp}")));
    }
    Ok(())
}

fn base64_encode(data: &[u8]) -> String {
    const TABLE: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    let mut out = String::new();
    for chunk in data.chunks(3) {
        let b0 = chunk[0] as u32;
        let b1 = chunk.get(1).copied().unwrap_or(0) as u32;
        let b2 = chunk.get(2).copied().unwrap_or(0) as u32;
        let n = (b0 << 16) | (b1 << 8) | b2;
        out.push(TABLE[((n >> 18) & 63) as usize] as char);
        out.push(TABLE[((n >> 12) & 63) as usize] as char);
        out.push(if chunk.len() > 1 {
            TABLE[((n >> 6) & 63) as usize] as char
        } else {
            '='
        });
        out.push(if chunk.len() > 2 {
            TABLE[(n & 63) as usize] as char
        } else {
            '='
        });
    }
    out
}

fn read_ws_text_frame(stream: &mut TcpStream) -> Result<Option<String>> {
    let mut header = [0u8; 2];
    stream
        .read_exact(&mut header)
        .map_err(|e| ArxError::Runtime(e.to_string()))?;
    let opcode = header[0] & 0x0F;
    let masked = header[1] & 0x80 != 0;
    let mut len = (header[1] & 0x7F) as usize;
    if len == 126 {
        let mut ext = [0u8; 2];
        stream.read_exact(&mut ext).map_err(|e| ArxError::Runtime(e.to_string()))?;
        len = u16::from_be_bytes(ext) as usize;
    }
    if masked {
        let mut mask = [0u8; 4];
        stream.read_exact(&mut mask).map_err(|e| ArxError::Runtime(e.to_string()))?;
    }
    let mut payload = vec![0u8; len];
    if len > 0 {
        stream
            .read_exact(&mut payload)
            .map_err(|e| ArxError::Runtime(e.to_string()))?;
    }
    match opcode {
        0x1 => Ok(Some(String::from_utf8_lossy(&payload).into_owned())),
        0x8 => Err(ArxError::Runtime("websocket closed".into())),
        _ => Ok(None),
    }
}
