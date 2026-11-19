use std::sync::{Arc, Mutex};

use alicia_m_can::{
    alicia_d_rad_to_alicia_m_rad, default_config, scan_ids as rust_scan_ids, ArxError, ArxMCanArm,
    ArxMCanRobot, ArxMCanState, GravityCompConfig, LeaderSample, ScanOptions, SparkVisBridge,
    SparkVisConfig, TeleopConfig, TeleopSession, DEFAULT_ARM_CONTROL_MODE, DEFAULT_BAUD,
    DEFAULT_CONTROL_HZ, DEFAULT_MAX_JOINT_ACCEL_RAD_S2, DEFAULT_MAX_JOINT_JERK_RAD_S3,
    DEFAULT_MIT_CONTROL_HZ, DEFAULT_PORT, DEFAULT_WAIT_TOLERANCE_RAD,
};
use pyo3::exceptions::{PyRuntimeError, PyValueError};
use pyo3::prelude::*;
use pyo3::types::{PyDict, PyList};

fn map_err(err: ArxError) -> PyErr {
    PyRuntimeError::new_err(err.to_string())
}

fn state_to_py(py: Python<'_>, state: &ArxMCanState) -> PyResult<PyObject> {
    let dict = PyDict::new(py);
    let arm = PyDict::new(py);
    arm.set_item("names", state.arm.names.clone())?;
    arm.set_item("positions", state.arm.positions.clone())?;
    arm.set_item("velocities", state.arm.velocities.clone())?;
    arm.set_item("torques", state.arm.torques.clone())?;
    dict.set_item("arm", arm)?;

    if let Some(gripper) = &state.gripper {
        let g = PyDict::new(py);
        g.set_item("name", gripper.name.clone())?;
        g.set_item("motor_id", gripper.motor_id)?;
        g.set_item("feedback_id", gripper.feedback_id)?;
        g.set_item("position", gripper.position)?;
        g.set_item("velocity", gripper.velocity)?;
        g.set_item("torque", gripper.torque)?;
        dict.set_item("gripper", g)?;
    } else {
        dict.set_item("gripper", py.None())?;
    }
    Ok(dict.into())
}

fn grasp_to_py(py: Python<'_>, result: &alicia_m_can::GraspResult) -> PyResult<PyObject> {
    let out = PyDict::new(py);
    out.set_item("success", result.success)?;
    out.set_item("contact_position", result.contact_position)?;
    out.set_item("contact_torque_nm", result.contact_torque_nm)?;
    out.set_item("hold_torque_nm", result.hold_torque_nm)?;
    out.set_item("peak_temperature_c", result.peak_temperature_c)?;
    out.set_item("reason", result.reason.clone())?;
    Ok(out.into())
}

fn leader_field<'py, T: FromPyObject<'py>>(state: &Bound<'py, PyAny>, name: &str) -> PyResult<T> {
    if let Ok(dict) = state.downcast::<PyDict>() {
        if let Some(value) = dict.get_item(name)? {
            return value.extract();
        }
    }
    state.getattr(name)?.extract()
}

/// One fresh Alicia-D frame per tick, same as SDK ``Teleoperation._control_loop``.
///
/// The call blocks only until that reply arrives. The 500 Hz MIT loop keeps
/// holding the previous pose while this thread waits, so the wait does not brake.
fn parse_leader_sample(
    _py: Python<'_>,
    leader: &Bound<'_, PyAny>,
    last_sequence: u64,
    timeout_s: f64,
) -> PyResult<Option<LeaderSample>> {
    let streaming = leader.hasattr("wait_for_next_robot_state")?;
    let state = if streaming {
        leader.call_method1("wait_for_next_robot_state", (last_sequence, timeout_s))?
    } else {
        leader.call_method1("get_robot_state", ("joint_gripper",))?
    };
    if state.is_none() {
        return Ok(None);
    }
    let angles: Vec<f64> = leader_field(&state, "angles")?;
    let gripper: f64 = leader_field(&state, "gripper")?;
    if angles.len() < 6 {
        return Err(PyValueError::new_err("leader must provide 6 joint angles"));
    }
    let mut joints = [0.0; 6];
    for (i, v) in angles.iter().take(6).enumerate() {
        joints[i] = *v;
    }
    let sequence = if streaming {
        leader_field(&state, "sequence")?
    } else {
        last_sequence.saturating_add(1)
    };
    let captured_at_ns = if streaming {
        leader_field(&state, "timestamp_ns")?
    } else {
        0
    };
    Ok(Some(LeaderSample {
        joints,
        gripper,
        sequence,
        captured_at_ns,
    }))
}

#[pyclass(name = "Robot")]
struct PyRobot {
    inner: Option<Arc<Mutex<ArxMCanRobot>>>,
}

#[pymethods]
impl PyRobot {
    #[new]
    #[pyo3(signature = (
        port=None,
        baud=DEFAULT_BAUD,
        control_hz=DEFAULT_CONTROL_HZ,
        auto_enable=true,
        arm_control_mode=DEFAULT_ARM_CONTROL_MODE,
        wait_tolerance_rad=DEFAULT_WAIT_TOLERANCE_RAD,
        wait_timeout_sec=7.5,
        use_gravity_ff=true,
        gravity_model_path=None,
        gravity_joint_scales=None,
        config_path=None,
    ))]
    fn new(
        port: Option<&str>,
        baud: u32,
        control_hz: f64,
        auto_enable: bool,
        arm_control_mode: &str,
        wait_tolerance_rad: f64,
        wait_timeout_sec: f64,
        use_gravity_ff: bool,
        gravity_model_path: Option<&str>,
        gravity_joint_scales: Option<Vec<f64>>,
        config_path: Option<&str>,
    ) -> PyResult<Self> {
        let port = port.unwrap_or(DEFAULT_PORT);
        let mut gravity = GravityCompConfig {
            enabled: use_gravity_ff,
            model_path: gravity_model_path.map(str::to_string),
            ..GravityCompConfig::default()
        };
        if let Some(scales) = gravity_joint_scales {
            if scales.len() != 6 {
                return Err(PyValueError::new_err(
                    "gravity_joint_scales must have exactly 6 values",
                ));
            }
            gravity.joint_scales = [
                scales[0], scales[1], scales[2], scales[3], scales[4], scales[5],
            ];
        }
        let effective_hz = if use_gravity_ff {
            control_hz.max(DEFAULT_MIT_CONTROL_HZ)
        } else {
            control_hz
        };
        let hw_yaml = config_path.map(std::path::Path::new);
        let robot = ArxMCanRobot::new(
            port,
            baud,
            effective_hz,
            None,
            wait_tolerance_rad,
            wait_timeout_sec,
            auto_enable,
            false,
            Some(arm_control_mode),
            hw_yaml,
            Some(gravity),
        )
        .map_err(map_err)?;
        Ok(Self {
            inner: Some(Arc::new(Mutex::new(robot))),
        })
    }

    fn arm_control_mode(&self) -> PyResult<String> {
        Ok(self.inner_ref()?.arm_control_mode().to_string())
    }

    fn enabled(&self) -> bool {
        self.inner
            .as_ref()
            .and_then(|robot| robot.lock().ok())
            .map(|robot| robot.enabled())
            .unwrap_or(false)
    }

    fn enable(&mut self) -> PyResult<()> {
        self.inner_mut()?.enable().map_err(map_err)
    }

    fn disable(&mut self) -> PyResult<()> {
        self.inner_mut()?.disable().map_err(map_err)
    }

    fn get_joint_positions(&self) -> PyResult<Vec<f64>> {
        self.inner_ref()?.get_robot_state_joint().map_err(map_err)
    }

    fn get_commanded_joints(&self) -> PyResult<Vec<f64>> {
        Ok(self.inner_ref()?.get_ik_seed_joints())
    }

    fn get_arm_state(&self, py: Python<'_>, request_feedback: bool) -> PyResult<PyObject> {
        let state = self
            .inner_ref()?
            .get_arm_state(request_feedback)
            .map_err(map_err)?;
        state_to_py(py, &state)
    }

    #[pyo3(signature = (request_feedback=true))]
    fn get_gripper_value(&self, request_feedback: bool) -> PyResult<Option<f64>> {
        self.inner_ref()?
            .get_gripper_value(request_feedback)
            .map_err(map_err)
    }

    fn gripper_close(&mut self, py: Python<'_>) -> PyResult<PyObject> {
        let result = self.inner_mut()?.close_gripper().map_err(map_err)?;
        grasp_to_py(py, &result)
    }

    fn gripper_open(&mut self, target: f64) -> PyResult<()> {
        self.inner_mut()?.open_gripper(target).map_err(map_err)
    }

    fn gripper_hold(&mut self) -> PyResult<()> {
        self.inner_mut()?.hold_gripper().map_err(map_err)
    }

    fn gripper_release(&mut self) -> PyResult<()> {
        self.inner_mut()?.release_gripper().map_err(map_err)
    }

    #[pyo3(signature = (request_feedback=true))]
    fn get_motor_diagnostics(&self, py: Python<'_>, request_feedback: bool) -> PyResult<PyObject> {
        let diagnostics = self
            .inner_ref()?
            .get_motor_diagnostics(request_feedback)
            .map_err(map_err)?;
        let out = PyList::empty(py);
        for diagnostic in diagnostics {
            let item = PyDict::new(py);
            item.set_item("name", diagnostic.name)?;
            item.set_item("motor_id", diagnostic.motor_id)?;
            item.set_item("feedback_id", diagnostic.feedback_id)?;
            item.set_item("status_code", diagnostic.status_code)?;
            item.set_item("status_name", diagnostic.status_name)?;
            item.set_item("position", diagnostic.position)?;
            item.set_item("velocity", diagnostic.velocity)?;
            item.set_item("torque", diagnostic.torque)?;
            item.set_item("mos_temperature_c", diagnostic.mos_temperature_c)?;
            item.set_item("rotor_temperature_c", diagnostic.rotor_temperature_c)?;
            out.append(item)?;
        }
        Ok(out.into())
    }

    fn get_control_loop_stats(&self, py: Python<'_>) -> PyResult<PyObject> {
        let robot = self.inner_ref()?;
        let (sent_commands, send_errors) = robot.get_control_loop_stats();
        let (overwritten_commands, overruns, missed_deadlines) =
            robot.get_control_loop_timing_stats();
        let out = PyDict::new(py);
        out.set_item("sent_commands", sent_commands)?;
        out.set_item("send_errors", send_errors)?;
        out.set_item("overwritten_commands", overwritten_commands)?;
        out.set_item("overruns", overruns)?;
        out.set_item("missed_deadlines", missed_deadlines)?;
        Ok(out.into())
    }

    fn get_arm_mit_gains(&self) -> PyResult<(Vec<f64>, Vec<f64>)> {
        self.inner_ref()?.get_arm_mit_gains().map_err(map_err)
    }

    #[pyo3(signature = (kp=None, kd=None))]
    fn set_arm_mit_gains(&mut self, kp: Option<Vec<f64>>, kd: Option<Vec<f64>>) -> PyResult<()> {
        self.inner_mut()?.set_arm_mit_gains(kp, kd).map_err(map_err)
    }

    #[pyo3(signature = (target_joints=None, gripper_value=None, speed=400.0, gripper_speed=100.0, wait_for_completion=false, max_joint_accel_rad_s2=None, max_joint_jerk_rad_s3=None, joint_tolerance=None, converge_timeout=None, stream=false, gripper_tolerance=None, gripper_timeout_sec=None))]
    fn set_robot_state(
        &mut self,
        target_joints: Option<Vec<f64>>,
        gripper_value: Option<f64>,
        speed: f64,
        gripper_speed: f64,
        wait_for_completion: bool,
        max_joint_accel_rad_s2: Option<f64>,
        max_joint_jerk_rad_s3: Option<f64>,
        joint_tolerance: Option<f64>,
        converge_timeout: Option<f64>,
        stream: bool,
        gripper_tolerance: Option<f64>,
        gripper_timeout_sec: Option<f64>,
    ) -> PyResult<bool> {
        self.inner_mut()?
            .set_robot_state(
                target_joints.as_deref(),
                gripper_value,
                speed,
                gripper_speed,
                wait_for_completion,
                joint_tolerance,
                converge_timeout,
                max_joint_accel_rad_s2,
                max_joint_jerk_rad_s3,
                stream,
                gripper_tolerance,
                gripper_timeout_sec,
            )
            .map_err(map_err)
    }

    #[pyo3(signature = (target_joints, speed=80.0))]
    fn hold_arm_joints(&mut self, target_joints: Vec<f64>, speed: f64) -> PyResult<()> {
        self.inner_mut()?
            .hold_arm_joints(&target_joints, speed)
            .map_err(map_err)
    }

    #[pyo3(signature = (speed=40.0))]
    fn go_home(&mut self, speed: f64) -> PyResult<()> {
        self.inner_mut()?.go_home(speed).map_err(map_err)
    }

    fn disconnect(&mut self) -> PyResult<()> {
        if let Some(robot) = self.inner.as_ref() {
            robot
                .lock()
                .map_err(|_| PyRuntimeError::new_err("robot mutex poisoned"))?
                .disconnect()
                .map_err(map_err)?;
        }
        Ok(())
    }
}

impl PyRobot {
    fn inner_ref(&self) -> PyResult<std::sync::MutexGuard<'_, ArxMCanRobot>> {
        self.inner
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("robot already disconnected"))?
            .lock()
            .map_err(|_| PyRuntimeError::new_err("robot mutex poisoned"))
    }

    fn inner_mut(&mut self) -> PyResult<std::sync::MutexGuard<'_, ArxMCanRobot>> {
        self.inner_ref()
    }

    fn shared(&self) -> PyResult<Arc<Mutex<ArxMCanRobot>>> {
        self.inner
            .clone()
            .ok_or_else(|| PyRuntimeError::new_err("robot already disconnected"))
    }

    fn take_inner(&mut self) -> PyResult<ArxMCanRobot> {
        let arc = self
            .inner
            .take()
            .ok_or_else(|| PyRuntimeError::new_err("robot already disconnected"))?;
        match Arc::try_unwrap(arc) {
            Ok(mutex) => mutex
                .into_inner()
                .map_err(|_| PyRuntimeError::new_err("robot mutex poisoned")),
            Err(arc) => {
                self.inner = Some(arc);
                Err(PyRuntimeError::new_err("robot is in use by teleoperation"))
            }
        }
    }
}

impl Drop for PyRobot {
    fn drop(&mut self) {
        let _ = self.disconnect();
    }
}

#[pyclass(name = "Arm")]
struct PyArm {
    inner: ArxMCanArm,
}

#[pymethods]
impl PyArm {
    #[new]
    #[pyo3(signature = (port=None, baud=DEFAULT_BAUD, enable_gripper=false, arm_control_mode=DEFAULT_ARM_CONTROL_MODE, config_path=None))]
    fn new(
        port: Option<&str>,
        baud: u32,
        enable_gripper: bool,
        arm_control_mode: &str,
        config_path: Option<&str>,
    ) -> PyResult<Self> {
        let hw_yaml = config_path.map(std::path::Path::new);
        let config =
            default_config(port, Some(baud), 100.0, arm_control_mode, hw_yaml).map_err(map_err)?;
        Ok(Self {
            inner: ArxMCanArm::new(config, enable_gripper).map_err(map_err)?,
        })
    }

    fn connect(&mut self) -> PyResult<()> {
        self.inner.connect().map_err(map_err)
    }

    #[pyo3(signature = (disable=true))]
    fn close(&mut self, disable: bool) -> PyResult<()> {
        self.inner.close(disable).map_err(map_err)
    }

    fn connected(&self) -> bool {
        self.inner.connected()
    }

    #[pyo3(signature = (request_feedback=true))]
    fn read_state(&self, py: Python<'_>, request_feedback: bool) -> PyResult<PyObject> {
        let state = self.inner.read_state(request_feedback).map_err(map_err)?;
        state_to_py(py, &state)
    }

    #[pyo3(signature = (request_feedback=true))]
    fn get_gripper_value(&self, request_feedback: bool) -> PyResult<Option<f64>> {
        self.inner
            .get_gripper_value(request_feedback)
            .map_err(map_err)
    }

    #[pyo3(signature = (start_id=1, end_id=16, model="4340P", timeout_ms=30, feedback_base="0x10"))]
    fn scan_ids(
        &self,
        start_id: u16,
        end_id: u16,
        model: &str,
        timeout_ms: u32,
        feedback_base: &str,
    ) -> PyResult<Vec<u16>> {
        self.inner
            .scan_ids(start_id, end_id, model, timeout_ms, feedback_base)
            .map_err(map_err)
    }

    fn set_zero(&mut self) -> PyResult<()> {
        self.inner.set_zero().map_err(map_err)
    }
}

impl Drop for PyArm {
    fn drop(&mut self) {
        let _ = self.inner.close(false);
    }
}

#[pyfunction]
#[pyo3(signature = (port=None, baud=DEFAULT_BAUD, start_id=1, end_id=16, model="4340P", timeout_ms=30, feedback_base="0x10"))]
fn scan_ids(
    port: Option<&str>,
    baud: u32,
    start_id: u16,
    end_id: u16,
    model: &str,
    timeout_ms: u32,
    feedback_base: &str,
) -> PyResult<Vec<u16>> {
    let port = port.unwrap_or(DEFAULT_PORT);
    rust_scan_ids(
        port,
        baud,
        &ScanOptions {
            start_id,
            end_id,
            model: model.to_string(),
            timeout_ms,
            feedback_base: feedback_base.to_string(),
        },
    )
    .map_err(map_err)
}

#[pyclass(name = "Teleoperation")]
struct PyTeleoperation {
    session: Option<TeleopSession>,
}

#[pymethods]
impl PyTeleoperation {
    #[new]
    #[pyo3(signature = (follower, frequency_hz=100.0, follower_speed=400.0, gripper_speed=400.0, gripper_scale=1.0, use_joint_mapping=true, max_joint_accel_rad_s2=None, max_joint_jerk_rad_s3=None, shape_min_s=0.006, shape_max_s=0.015, stale_hold_s=0.050))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        follower: &PyRobot,
        frequency_hz: f64,
        follower_speed: f64,
        gripper_speed: f64,
        gripper_scale: f64,
        use_joint_mapping: bool,
        max_joint_accel_rad_s2: Option<f64>,
        max_joint_jerk_rad_s3: Option<f64>,
        shape_min_s: f64,
        shape_max_s: f64,
        stale_hold_s: f64,
    ) -> PyResult<Self> {
        if !frequency_hz.is_finite() || frequency_hz <= 0.0 {
            return Err(PyValueError::new_err(
                "frequency_hz must be positive and finite",
            ));
        }
        if !shape_min_s.is_finite()
            || !shape_max_s.is_finite()
            || shape_min_s <= 0.0
            || shape_min_s > shape_max_s
        {
            return Err(PyValueError::new_err(
                "shape_min_s must be > 0 and <= shape_max_s",
            ));
        }
        if !stale_hold_s.is_finite() || stale_hold_s <= shape_max_s {
            return Err(PyValueError::new_err(
                "stale_hold_s must be greater than shape_max_s",
            ));
        }
        Ok(Self {
            session: Some(TeleopSession::from_shared(
                follower.shared()?,
                TeleopConfig {
                    frequency_hz,
                    follower_speed,
                    gripper_speed,
                    gripper_scale,
                    use_joint_mapping,
                    max_joint_accel_rad_s2,
                    max_joint_jerk_rad_s3,
                    shape_min_s,
                    shape_max_s,
                    stale_hold_s,
                },
            )),
        })
    }

    #[pyo3(signature = (leader, on_state=None))]
    fn start(&mut self, leader: PyObject, on_state: Option<PyObject>) -> PyResult<()> {
        let session = self
            .session
            .as_mut()
            .ok_or_else(|| PyRuntimeError::new_err("teleop session already stopped"))?;
        let leader = Python::with_gil(|py| leader.clone_ref(py));
        let on_state = on_state.map(|callback| Python::with_gil(|py| callback.clone_ref(py)));
        let frequency_hz = session.config().frequency_hz;
        let mut last_sequence = 0_u64;
        let timeout_s = (2.0 / frequency_hz.max(1.0)).clamp(0.02, 0.2);
        session
            .start(move || {
                Python::with_gil(|py| -> Result<Option<LeaderSample>, ArxError> {
                    let bound = leader.bind(py);
                    let sample = parse_leader_sample(py, bound, last_sequence, timeout_s)
                        .map_err(|e| ArxError::Runtime(e.to_string()))?;
                    if let (Some(callback), Some(frame)) = (on_state.as_ref(), sample.as_ref()) {
                        if let Ok(joints) = PyList::new(py, frame.joints) {
                            let _ = callback.call1(py, (joints, frame.gripper));
                        }
                    }
                    if let Some(frame) = sample.as_ref() {
                        last_sequence = frame.sequence;
                    }
                    Ok(sample)
                })
            })
            .map_err(map_err)
    }

    fn get_stats(&self, py: Python<'_>) -> PyResult<PyObject> {
        let session = self
            .session
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("teleop session already stopped"))?;
        let stats = session.stats();
        let out = PyDict::new(py);
        out.set_item("leader_samples", stats.leader_samples)?;
        out.set_item("duplicate_samples", stats.duplicate_samples)?;
        out.set_item("read_errors", stats.read_errors)?;
        out.set_item("stale_events", stats.stale_events)?;
        out.set_item("interval_mean_ms", stats.interval_mean_ms)?;
        out.set_item("interval_max_ms", stats.interval_max_ms)?;
        out.set_item("last_sample_age_ms", stats.last_sample_age_ms)?;
        out.set_item("control_sent", stats.control_sent)?;
        out.set_item("control_send_errors", stats.control_send_errors)?;
        out.set_item("mailbox_overwrites", stats.mailbox_overwrites)?;
        out.set_item("control_overruns", stats.control_overruns)?;
        out.set_item("control_missed_deadlines", stats.control_missed_deadlines)?;
        Ok(out.into())
    }

    fn stop(&mut self, py: Python<'_>) {
        if let Some(session) = self.session.as_mut() {
            // The teleop worker calls back into Python to read Alicia-D.  It
            // may therefore be waiting for the GIL while stop() joins it.  Do
            // not hold the GIL across that join or Python and Rust deadlock.
            py.allow_threads(|| session.stop());
        }
    }

    fn disconnect_follower(&mut self, py: Python<'_>) -> PyResult<()> {
        if let Some(mut session) = self.session.take() {
            py.allow_threads(move || session.disconnect_follower())
                .map_err(map_err)?;
        }
        Ok(())
    }
}

impl Drop for PyTeleoperation {
    fn drop(&mut self) {
        if let Some(mut session) = self.session.take() {
            session.stop();
            let _ = session.disconnect_follower();
        }
    }
}

#[pyclass(name = "SparkVisBridge")]
struct PySparkVisBridge {
    inner: SparkVisBridge,
}

#[pymethods]
impl PySparkVisBridge {
    #[new]
    #[pyo3(signature = (
        ws_url="ws://127.0.0.1:8000/ws/teleop",
        http_base="http://127.0.0.1:8000",
        speed=100.0,
        gripper_speed=100.0,
        command_hz=50.0,
        watchdog_sec=0.35,
        control_gripper=false,
        dry_run=false,
        sync_initial=true,
        max_joint_accel_rad_s2=None,
        max_joint_jerk_rad_s3=None,
    ))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        ws_url: &str,
        http_base: &str,
        speed: f64,
        gripper_speed: f64,
        command_hz: f64,
        watchdog_sec: f64,
        control_gripper: bool,
        dry_run: bool,
        sync_initial: bool,
        max_joint_accel_rad_s2: Option<f64>,
        max_joint_jerk_rad_s3: Option<f64>,
    ) -> Self {
        Self {
            inner: SparkVisBridge::new(SparkVisConfig {
                ws_url: ws_url.to_string(),
                http_base: http_base.to_string(),
                speed,
                gripper_speed,
                command_hz,
                watchdog_sec,
                control_gripper,
                dry_run,
                sync_initial,
                max_joint_accel_rad_s2,
                max_joint_jerk_rad_s3,
            }),
        }
    }

    fn connect_robot(&mut self, robot: &mut PyRobot) -> PyResult<()> {
        let inner = robot.take_inner()?;
        self.inner.connect_robot(inner).map_err(map_err)
    }

    fn arm(&self) {
        self.inner.arm();
    }

    fn disarm(&self) {
        self.inner.disarm();
    }

    fn start(&mut self) -> PyResult<()> {
        self.inner.start().map_err(map_err)?;
        Ok(())
    }

    fn shutdown(&mut self) {
        self.inner.shutdown();
    }
}

impl Drop for PySparkVisBridge {
    fn drop(&mut self) {
        self.inner.shutdown();
    }
}

#[pyfunction]
fn map_alicia_d_joints(joints: Vec<f64>) -> PyResult<Vec<f64>> {
    if joints.len() < 6 {
        return Err(PyValueError::new_err("expected 6 Alicia-D joint angles"));
    }
    let mut q = [0.0; 6];
    for (i, value) in joints.iter().take(6).enumerate() {
        q[i] = *value;
    }
    Ok(alicia_d_rad_to_alicia_m_rad(&q).to_vec())
}

#[pymodule]
fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRobot>()?;
    m.add_class::<PyArm>()?;
    m.add_class::<PyTeleoperation>()?;
    m.add_class::<PySparkVisBridge>()?;
    m.add_function(wrap_pyfunction!(scan_ids, m)?)?;
    m.add_function(wrap_pyfunction!(map_alicia_d_joints, m)?)?;
    m.add("DEFAULT_PORT", DEFAULT_PORT)?;
    m.add("DEFAULT_BAUD", DEFAULT_BAUD)?;
    m.add("DEFAULT_CONTROL_HZ", DEFAULT_CONTROL_HZ)?;
    m.add("DEFAULT_MIT_CONTROL_HZ", DEFAULT_MIT_CONTROL_HZ)?;
    m.add("DEFAULT_ARM_CONTROL_MODE", DEFAULT_ARM_CONTROL_MODE)?;
    m.add(
        "DEFAULT_MAX_JOINT_ACCEL_RAD_S2",
        DEFAULT_MAX_JOINT_ACCEL_RAD_S2,
    )?;
    m.add(
        "DEFAULT_MAX_JOINT_JERK_RAD_S3",
        DEFAULT_MAX_JOINT_JERK_RAD_S3,
    )?;
    m.add("DEFAULT_WAIT_TOLERANCE_RAD", DEFAULT_WAIT_TOLERANCE_RAD)?;
    m.add("VERSION", env!("CARGO_PKG_VERSION"))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn map_err_formats_message() {
        let err = map_err(ArxError::NotConnected);
        assert!(
            err.to_string().contains("NotConnected") || err.to_string().contains("not connected")
        );
    }
}
