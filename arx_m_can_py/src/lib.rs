use arx_m_can::{
    scan_ids as rust_scan_ids, ArxMCanArm, ArxMCanRobot, ArxMCanState, ArxError, GravityCompConfig,
    LeaderSample, ScanOptions, SparkVisBridge, SparkVisConfig, TeleopConfig, TeleopSession,
    DEFAULT_ARM_CONTROL_MODE, DEFAULT_BAUD, DEFAULT_CONTROL_HZ, DEFAULT_MAX_JOINT_ACCEL_RAD_S2,
    DEFAULT_MAX_JOINT_JERK_RAD_S3, DEFAULT_MIT_CONTROL_HZ, DEFAULT_PORT, DEFAULT_WAIT_TOLERANCE_RAD,
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

fn leader_field<'py, T: FromPyObject<'py>>(state: &Bound<'py, PyAny>, name: &str) -> PyResult<T> {
    if let Ok(dict) = state.downcast::<PyDict>() {
        if let Some(value) = dict.get_item(name)? {
            return value.extract();
        }
    }
    state.getattr(name)?.extract()
}

fn parse_leader_sample(_py: Python<'_>, leader: &Bound<'_, PyAny>) -> PyResult<Option<LeaderSample>> {
    let state = leader.call_method1("get_robot_state", ("joint_gripper",))?;
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
    Ok(Some(LeaderSample {
        joints,
        gripper,
    }))
}

#[pyclass(name = "Robot")]
struct PyRobot {
    inner: Option<ArxMCanRobot>,
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
        wait_timeout_sec=15.0,
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
            inner: Some(robot),
        })
    }

    fn arm_control_mode(&self) -> PyResult<String> {
        Ok(self.inner_ref()?.arm_control_mode().to_string())
    }

    fn enabled(&self) -> bool {
        self.inner.as_ref().map(|r| r.enabled()).unwrap_or(false)
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
        let state = self.inner_ref()?.get_arm_state(request_feedback).map_err(map_err)?;
        state_to_py(py, &state)
    }

    #[pyo3(signature = (request_feedback=true))]
    fn get_motor_diagnostics(
        &self,
        py: Python<'_>,
        request_feedback: bool,
    ) -> PyResult<PyObject> {
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
        let (sent_commands, send_errors) = self.inner_ref()?.get_control_loop_stats();
        let out = PyDict::new(py);
        out.set_item("sent_commands", sent_commands)?;
        out.set_item("send_errors", send_errors)?;
        Ok(out.into())
    }

    fn get_arm_mit_gains(&self) -> PyResult<(Vec<f64>, Vec<f64>)> {
        self.inner_ref()?.get_arm_mit_gains().map_err(map_err)
    }

    #[pyo3(signature = (kp=None, kd=None))]
    fn set_arm_mit_gains(
        &mut self,
        kp: Option<Vec<f64>>,
        kd: Option<Vec<f64>>,
    ) -> PyResult<()> {
        self.inner_mut()?
            .set_arm_mit_gains(kp, kd)
            .map_err(map_err)
    }

    #[pyo3(signature = (target_joints=None, gripper_value=None, speed=400.0, gripper_speed=100.0, wait_for_completion=false, max_joint_accel_rad_s2=None, max_joint_jerk_rad_s3=None, joint_tolerance=None, converge_timeout=None))]
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
            )
            .map_err(map_err)
    }

    #[pyo3(signature = (command=None, value=None, speed=100.0, wait_for_completion=true))]
    fn set_gripper_target(
        &mut self,
        command: Option<&str>,
        value: Option<f64>,
        speed: f64,
        wait_for_completion: bool,
    ) -> PyResult<bool> {
        self.inner_mut()?
            .set_gripper_target(command, value, speed, wait_for_completion)
            .map_err(map_err)
    }

    #[pyo3(signature = (speed=40.0))]
    fn go_home(&mut self, speed: f64) -> PyResult<()> {
        self.inner_mut()?.go_home(speed).map_err(map_err)
    }

    fn disconnect(&mut self) -> PyResult<()> {
        if let Some(mut robot) = self.inner.take() {
            robot.disconnect().map_err(map_err)?;
        }
        Ok(())
    }
}

impl PyRobot {
    fn inner_ref(&self) -> PyResult<&ArxMCanRobot> {
        self.inner
            .as_ref()
            .ok_or_else(|| PyRuntimeError::new_err("robot already disconnected"))
    }

    fn inner_mut(&mut self) -> PyResult<&mut ArxMCanRobot> {
        self.inner
            .as_mut()
            .ok_or_else(|| PyRuntimeError::new_err("robot already disconnected"))
    }

    fn take_inner(&mut self) -> PyResult<ArxMCanRobot> {
        self.inner
            .take()
            .ok_or_else(|| PyRuntimeError::new_err("robot already disconnected"))
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
    #[pyo3(signature = (port=None, baud=DEFAULT_BAUD, enable_gripper=false, arm_control_mode=DEFAULT_ARM_CONTROL_MODE))]
    fn new(
        port: Option<&str>,
        baud: u32,
        enable_gripper: bool,
        arm_control_mode: &str,
    ) -> PyResult<Self> {
        Ok(Self {
            inner: ArxMCanArm::with_defaults(port, Some(baud), arm_control_mode, enable_gripper)
                .map_err(map_err)?,
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
    #[pyo3(signature = (follower, frequency_hz=60.0, follower_speed=400.0, gripper_speed=400.0, gripper_scale=1.0, use_joint_mapping=true, max_joint_accel_rad_s2=None, max_joint_jerk_rad_s3=None))]
    #[allow(clippy::too_many_arguments)]
    fn new(
        follower: &mut PyRobot,
        frequency_hz: f64,
        follower_speed: f64,
        gripper_speed: f64,
        gripper_scale: f64,
        use_joint_mapping: bool,
        max_joint_accel_rad_s2: Option<f64>,
        max_joint_jerk_rad_s3: Option<f64>,
    ) -> PyResult<Self> {
        let robot = follower.take_inner()?;
        Ok(Self {
            session: Some(TeleopSession::new(
                robot,
                TeleopConfig {
                    frequency_hz,
                    follower_speed,
                    gripper_speed,
                    gripper_scale,
                    use_joint_mapping,
                    max_joint_accel_rad_s2,
                    max_joint_jerk_rad_s3,
                },
            )),
        })
    }

    fn start(&mut self, leader: PyObject) -> PyResult<()> {
        let session = self
            .session
            .as_mut()
            .ok_or_else(|| PyRuntimeError::new_err("teleop session already stopped"))?;
        let leader = Python::with_gil(|py| leader.clone_ref(py));
        session
            .start(move || {
                Python::with_gil(|py| {
                    let bound = leader.bind(py);
                    parse_leader_sample(py, bound)
                        .map_err(|e| ArxError::Runtime(e.to_string()))
                })
            })
            .map_err(map_err)
    }

    fn stop(&mut self) {
        if let Some(session) = self.session.as_mut() {
            session.stop();
        }
    }

    fn disconnect_follower(&mut self) -> PyResult<()> {
        if let Some(mut session) = self.session.take() {
            session.disconnect_follower().map_err(map_err)?;
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

#[pymodule]
fn _native(m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRobot>()?;
    m.add_class::<PyArm>()?;
    m.add_class::<PyTeleoperation>()?;
    m.add_class::<PySparkVisBridge>()?;
    m.add_function(wrap_pyfunction!(scan_ids, m)?)?;
    m.add("DEFAULT_PORT", DEFAULT_PORT)?;
    m.add("DEFAULT_BAUD", DEFAULT_BAUD)?;
    m.add("DEFAULT_CONTROL_HZ", DEFAULT_CONTROL_HZ)?;
    m.add("DEFAULT_MIT_CONTROL_HZ", DEFAULT_MIT_CONTROL_HZ)?;
    m.add("DEFAULT_ARM_CONTROL_MODE", DEFAULT_ARM_CONTROL_MODE)?;
    m.add("DEFAULT_MAX_JOINT_ACCEL_RAD_S2", DEFAULT_MAX_JOINT_ACCEL_RAD_S2)?;
    m.add("DEFAULT_MAX_JOINT_JERK_RAD_S3", DEFAULT_MAX_JOINT_JERK_RAD_S3)?;
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
