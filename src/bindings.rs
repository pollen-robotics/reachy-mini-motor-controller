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

    fn enable_torque(&self) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_torque()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    fn disable_torque(&self) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .disable_torque()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    fn read_all_positions(&self) -> PyResult<[f64; 9]> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .read_all_positions()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_all_goal_positions(&self, positions: [f64; 9]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_all_goal_positions(positions)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    fn set_antennas_positions(&self, positions: [f64; 2]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_antennas_positions(positions)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    fn set_stewart_platform_position(&self, position: [f64; 6]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_stewart_platform_position(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    fn set_body_rotation(&self, position: f64) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_body_rotation(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }
}

#[pyo3::pymodule]
fn reachy_mini_motor_controller(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();

    m.add_class::<ReachyMiniMotorController>()?;

    Ok(())
}

define_stub_info_gatherer!(stub_info);
