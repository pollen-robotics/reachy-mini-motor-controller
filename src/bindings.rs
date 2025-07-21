use std::time::Duration;

use crate::control_loop::{LastPosition, MotorCommand, ReachyMiniControlLoop};

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

    fn set_stewart_platform_goal_current(&self, current: [i16; 6]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_stewart_platform_goal_current(current)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
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

#[gen_stub_pyclass]
#[pyclass]
struct ReachyMiniPyControlLoop {
    inner: std::sync::Arc<ReachyMiniControlLoop>,
}

#[gen_stub_pymethods]
#[pymethods]
impl ReachyMiniPyControlLoop {
    #[new]
    fn new(serialport: String, freq: f64, retries: u64) -> PyResult<Self> {
        let control_loop =
            ReachyMiniControlLoop::new(serialport, Duration::from_secs_f64(1.0 / freq), retries)
                .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(ReachyMiniPyControlLoop {
            inner: std::sync::Arc::new(control_loop),
        })
    }

    fn get_last_position(&self) -> PyResult<LastPosition> {
        self.inner
            .get_last_position()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_all_goal_positions(&self, positions: [f64; 9]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetAllGoalPositions { positions })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_stewart_platform_position(&self, position: [f64; 6]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetStewartPlatformPosition { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_body_rotation(&self, position: f64) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetBodyRotation { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_antennas_positions(&self, positions: [f64; 2]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetAntennasPositions { positions })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn enable_torque(&self) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableTorque())
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn disable_torque(&self) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::DisableTorque())
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_stewart_platform_goal_current(&self, current: [i16; 6]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetStewartPlatformGoalCurrent { current })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_stewart_platform_operating_mode(&self, mode: u8) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetStewartPlatformOperatingMode { mode })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_antennas_operating_mode(&self, mode: u8) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetAntennasOperatingMode { mode })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn set_body_rotation_operating_mode(&self, mode: u8) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetBodyRotationOperatingMode { mode })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn enable_stewart_platform(&self, enable: bool) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableStewartPlatform { enable })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn enable_body_rotation(&self, enable: bool) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableBodyRotation { enable })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    fn enable_antennas(&self, enable: bool) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableAntennas { enable })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }
}

#[pyo3::pymodule]
fn reachy_mini_motor_controller(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();

    m.add_class::<ReachyMiniMotorController>()?;
    m.add_class::<ReachyMiniPyControlLoop>()?;
    m.add_class::<LastPosition>()?;

    Ok(())
}

define_stub_info_gatherer!(stub_info);
