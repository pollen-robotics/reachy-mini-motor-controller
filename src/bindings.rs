use pyo3::prelude::*;
use pyo3_stub_gen::{
    define_stub_info_gatherer,
    derive::{gen_stub_pyclass, gen_stub_pymethods},
};
use crate::ReachyMiniMotorController as Controller;

#[gen_stub_pyclass]
#[pyclass(frozen)]
struct ReachyMiniMotorController {
    inner: std::sync::Mutex<Controller>,
}

#[gen_stub_pymethods]
#[pymethods]
impl ReachyMiniMotorController {
    #[new]
    fn new(serialport: String) -> PyResult<Self> {
        let inner = Controller::new(&serialport)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(ReachyMiniMotorController {
            inner: std::sync::Mutex::new(inner),
        })
    }

    #[pyo3(text_signature = "($self)")]
    /// Enable torque on all motors
    fn enable_torque(&self) -> PyResult<()> {
        with_controller!(self, |inner| inner.enable_torque())?;
        Ok(())
    }

    #[pyo3(text_signature = "($self)")]
    /// Disable torque on all motors
    fn disable_torque(&self) -> PyResult<()> {
        with_controller!(self, |inner| inner.disable_torque())?;
        Ok(())
    }

    #[pyo3(text_signature = "($self)")]
    /// Read current positions of all 9 motors
    /// Returns: [body_rotation, antenna_left, antenna_right, stewart_1..6]
    fn read_all_positions(&self) -> PyResult<[f64; 9]> {
        with_controller!(self, |inner| inner.read_all_positions())
    }

    #[pyo3(text_signature = "($self, positions)")]
    /// Set goal positions for all motors
    /// positions: [body_rotation, antenna_left, antenna_right, stewart_1..6]
    fn read_stewart_platform_current(&self) -> PyResult<[i16; 6]> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .read_stewart_platform_current()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn read_stewart_platform_operating_mode(&self) -> PyResult<[u8; 6]> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .read_stewart_platform_operating_mode()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_all_goal_positions(&self, positions: [f64; 9]) -> PyResult<()> {
        with_controller!(self, |inner| inner.set_all_goal_positions(positions))?;
        Ok(())
    }

    #[pyo3(text_signature = "($self, positions)")]
    /// Set goal positions for antenna motors
    fn set_antennas_positions(&self, positions: [f64; 2]) -> PyResult<()> {
        with_controller!(self, |inner| inner.set_antennas_positions(positions))?;
        Ok(())
    }

    #[pyo3(text_signature = "($self, position)")]
    /// Set goal positions for Stewart platform motors
    fn set_stewart_platform_position(&self, position: [f64; 6]) -> PyResult<()> {
        with_controller!(self, |inner| inner.set_stewart_platform_position(position))?;
        Ok(())
    }

    #[pyo3(text_signature = "($self, position)")]
    /// Set body rotation position
    fn set_body_rotation(&self, position: f64) -> PyResult<()> {
        with_controller!(self, |inner| inner.set_body_rotation(position))?;
        Ok(())
    }

    #[pyo3(text_signature = "($self, current)")]
    /// Set goal current for Stewart platform motors
    fn set_stewart_platform_goal_current(&self, current: [i16; 6]) -> PyResult<()> {
        with_controller!(self, |inner| inner.set_stewart_platform_goal_current(current))?;
        Ok(())
    }

    fn set_stewart_platform_operating_mode(&self, mode: u8) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_stewart_platform_operating_mode(mode)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    fn set_antennas_operating_mode(&self, mode: u8) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_antennas_operating_mode(mode)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }



    fn set_body_rotation_operating_mode(&self, mode: u8) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_body_rotation_operating_mode(mode)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    fn enable_body_rotation(&self, enable: bool) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_body_rotation(enable)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    fn enable_antennas(&self, enable: bool) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_antennas(enable)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    fn enable_stewart_platform(&self, enable: bool) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_stewart_platform(enable)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

}

macro_rules! with_controller {
    ($self:expr, |$inner:ident| $body:expr) => {{
        let mut $inner = $self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;
        $body.map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }};
}

#[pyo3::pymodule]
fn reachy_mini_motor_controller(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();
    m.add_class::<ReachyMiniMotorController>()?;
    Ok(())
}

define_stub_info_gatherer!(stub_info);
