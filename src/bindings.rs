use std::{collections::HashMap, time::Duration};

use crate::control_loop::{
    ControlLoopStats, FullBodyPosition, MotorCommand, ReachyMiniControlLoop,
};

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
    /// Create a new motor controller for the given serial port.
    ///
    /// # Arguments
    /// * `serialport` - Path to (Unix) or COM ID (Windows) of the serial port device.
    #[new]
    fn new(serialport: String) -> PyResult<Self> {
        let inner = Controller::new(&serialport)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(ReachyMiniMotorController {
            inner: std::sync::Mutex::new(inner),
        })
    }

    /// Is torque enabled on all motors
    fn is_torque_enabled(&self) -> PyResult<bool> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .is_torque_enabled()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable torque on all motors.
    fn enable_torque(&self) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_torque()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Disable torque on all motors.
    fn disable_torque(&self) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .disable_torque()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Read all motor positions as a 9-element array.
    fn read_all_positions(&self) -> PyResult<[f64; 9]> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .read_all_positions()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Read the current for the Stewart platform motors.
    fn read_stewart_platform_current(&self) -> PyResult<[i16; 6]> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .read_stewart_platform_current()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Read the operating mode for the Stewart platform motors.
    fn read_stewart_platform_operating_mode(&self) -> PyResult<[u8; 6]> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .read_stewart_platform_operating_mode()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for all motors (9 values).
    ///
    /// # Arguments
    /// * `positions` - Array of 9 goal positions (body_yaw, stewart, antennas).
    fn set_all_goal_positions(&self, positions: [f64; 9]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_all_goal_positions(positions)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set goal positions for the antennas (2 values).
    ///
    /// # Arguments
    /// * `positions` - Array of 2 goal positions for antennas.
    fn set_antennas_positions(&self, positions: [f64; 2]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_antennas_positions(positions)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set goal positions for the Stewart platform (6 values).
    ///
    /// # Arguments
    /// * `position` - Array of 6 goal positions for Stewart platform.
    fn set_stewart_platform_position(&self, position: [f64; 6]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_stewart_platform_position(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set goal position for the body rotation motor.
    ///
    /// # Arguments
    /// * `position` - Goal position for body rotation motor.
    fn set_body_rotation(&self, position: f64) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_body_rotation(position)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set goal current for the Stewart platform motors.
    ///
    /// # Arguments
    /// * `current` - Array of 6 goal currents for Stewart platform motors.
    fn set_stewart_platform_goal_current(&self, current: [i16; 6]) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_stewart_platform_goal_current(current)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set operating mode for all Stewart platform motors.
    ///
    /// # Arguments
    /// * `mode` - Operating mode value for Stewart platform motors.
    fn set_stewart_platform_operating_mode(&self, mode: u8) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_stewart_platform_operating_mode(mode)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set operating mode for both antennas.
    ///
    /// # Arguments
    /// * `mode` - Operating mode value for antennas.
    fn set_antennas_operating_mode(&self, mode: u8) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_antennas_operating_mode(mode)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Set operating mode for the body rotation motor.
    ///
    /// # Arguments
    /// * `mode` - Operating mode value for body rotation motor.
    fn set_body_rotation_operating_mode(&self, mode: u8) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .set_body_rotation_operating_mode(mode)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Enable or disable the body rotation motor.
    ///
    /// # Arguments
    /// * `enable` - Set to true to enable, false to disable.
    fn enable_body_rotation(&self, enable: bool) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_body_rotation(enable)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Enable or disable the antennas.
    ///
    /// # Arguments
    /// * `enable` - Set to true to enable, false to disable.
    fn enable_antennas(&self, enable: bool) -> PyResult<()> {
        let mut inner = self.inner.lock().map_err(|_| {
            pyo3::exceptions::PyRuntimeError::new_err("Failed to lock motor controller")
        })?;

        inner
            .enable_antennas(enable)
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(())
    }

    /// Enable or disable the Stewart platform motors.
    ///
    /// # Arguments
    /// * `enable` - Set to true to enable, false to disable.
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
    /// Create a new control loop for the motor controller.
    ///
    /// # Arguments
    /// * `serialport` - Path to (Unix) or COM ID (Windows) of the serial port device.
    /// * `update_loop_period` - Period between control loop updates.
    /// * `allowed_retries` - Number of allowed retries for reading positions.
    /// * `init_timeout` - Timeout for initial position read.
    /// * `stats_pub_period` - Optional period for publishing stats.
    #[new]
    #[pyo3(signature = (
        serialport,
        read_position_loop_period,
        allowed_retries = 5,
        stats_pub_period = None,
        voltage_rampup_timeout = Duration::from_secs(30),
    ))]
    fn new(
        serialport: String,
        read_position_loop_period: Duration,
        allowed_retries: u64,
        stats_pub_period: Option<Duration>,
        voltage_rampup_timeout: Duration,
    ) -> PyResult<Self> {
        let control_loop = ReachyMiniControlLoop::new(
            serialport,
            read_position_loop_period,
            stats_pub_period,
            allowed_retries,
            voltage_rampup_timeout,
        )
        .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))?;
        Ok(ReachyMiniPyControlLoop {
            inner: std::sync::Arc::new(control_loop),
        })
    }

    /// Close the control loop and release resources.
    fn close(&self) -> PyResult<()> {
        self.inner.close();
        Ok(())
    }

    /// Get the id/name motors used in this controller.
    fn get_motor_name_id(&self) -> HashMap<String, u8> {
        self.inner.get_motor_name_id()
    }

    /// Get the last successfully read motor positions.
    fn get_last_position(&self) -> PyResult<FullBodyPosition> {
        self.inner
            .get_last_position()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for all motors (9 values).
    ///
    /// # Arguments
    /// * `positions` - Array of 9 goal positions (body_yaw, stewart, antennas).
    fn set_all_goal_positions(&self, positions: FullBodyPosition) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetAllGoalPositions { positions })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for the Stewart platform (6 values).
    ///
    /// # Arguments
    /// * `position` - Array of 6 goal positions for Stewart platform.
    fn set_stewart_platform_position(&self, position: [f64; 6]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetStewartPlatformPosition { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal position for the body rotation motor.
    ///
    /// # Arguments
    /// * `position` - Goal position for body rotation motor.
    fn set_body_rotation(&self, position: f64) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetBodyRotation { position })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal positions for the antennas (2 values).
    ///
    /// # Arguments
    /// * `positions` - Array of 2 goal positions for antennas.
    fn set_antennas_positions(&self, positions: [f64; 2]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetAntennasPositions { positions })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Check torque enabled status.
    fn is_torque_enabled(&self) -> PyResult<bool> {
        self.inner
            .is_torque_enabled()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable torque on all motors.
    fn enable_torque(&self) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableTorque())
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Disable torque on all motors.
    fn disable_torque(&self) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::DisableTorque())
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set goal current for the Stewart platform motors.
    ///
    /// # Arguments
    /// * `current` - Array of 6 goal currents for Stewart platform motors.
    fn set_stewart_platform_goal_current(&self, current: [i16; 6]) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetStewartPlatformGoalCurrent { current })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Check stewart platform operating mode
    fn get_stewart_platform_operating_mode(&self) -> PyResult<u8> {
        self.inner
            .get_control_mode()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set operating mode for all Stewart platform motors.
    ///
    /// # Arguments
    /// * `mode` - Operating mode value for Stewart platform motors.
    fn set_stewart_platform_operating_mode(&self, mode: u8) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetStewartPlatformOperatingMode { mode })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set operating mode for both antennas.
    ///
    /// # Arguments
    /// * `mode` - Operating mode value for antennas.
    fn set_antennas_operating_mode(&self, mode: u8) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetAntennasOperatingMode { mode })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Set operating mode for the body rotation motor.
    ///
    /// # Arguments
    /// * `mode` - Operating mode value for body rotation motor.
    fn set_body_rotation_operating_mode(&self, mode: u8) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::SetBodyRotationOperatingMode { mode })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable or disable the Stewart platform motors.
    ///
    /// # Arguments
    /// * `enable` - Set to true to enable, false to disable.
    fn enable_stewart_platform(&self, enable: bool) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableStewartPlatform { enable })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable or disable the body rotation motor.
    ///
    /// # Arguments
    /// * `enable` - Set to true to enable, false to disable.
    fn enable_body_rotation(&self, enable: bool) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableBodyRotation { enable })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Enable or disable the antennas.
    ///
    /// # Arguments
    /// * `enable` - Set to true to enable, false to disable.
    fn enable_antennas(&self, enable: bool) -> PyResult<()> {
        self.inner
            .push_command(MotorCommand::EnableAntennas { enable })
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Get the latest control loop statistics, if available.
    fn get_stats(&self) -> PyResult<Option<ControlLoopStats>> {
        self.inner
            .get_stats()
            .map_err(|e| pyo3::exceptions::PyRuntimeError::new_err(e.to_string()))
    }

    /// Perform an asynchronous raw read of motor bytes.
    /// # Arguments
    /// * `id` - Motor ID to read from.
    /// * `addr` - Address to read from.
    /// * `length` - Number of bytes to read.
    fn async_read_raw_bytes(&self, id: u8, addr: u8, length: u8) -> PyResult<Vec<u8>> {
        self.inner
            .async_read_raw_bytes(id, addr, length)
            .map_err(|e| {
                pyo3::exceptions::PyRuntimeError::new_err(format!(
                    "Failed to read raw bytes: {}",
                    e.to_string()
                ))
            })
    }

    /// Perform an asynchronous raw write of motor bytes.
    /// # Arguments
    /// * `id` - Motor ID to write to.
    /// * `addr` - Address to write to.
    /// * `data` - Data bytes to write.
    fn async_write_raw_bytes(&self, id: u8, addr: u8, data: Vec<u8>) -> PyResult<()> {
        self.inner
            .async_write_raw_bytes(id, addr, data)
            .map_err(|e| {
                pyo3::exceptions::PyRuntimeError::new_err(format!(
                    "Failed to write raw bytes: {}",
                    e.to_string()
                ))
            })
    }
}

#[pyo3::pymodule]
fn reachy_mini_motor_controller(m: &Bound<'_, PyModule>) -> PyResult<()> {
    pyo3_log::init();

    m.add_class::<ReachyMiniMotorController>()?;
    m.add_class::<ReachyMiniPyControlLoop>()?;
    m.add_class::<FullBodyPosition>()?;
    m.add_class::<ControlLoopStats>()?;

    Ok(())
}

define_stub_info_gatherer!(stub_info);
