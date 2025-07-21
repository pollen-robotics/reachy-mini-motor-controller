use log::{error, warn};
use pyo3_stub_gen::derive::gen_stub_pyclass;
use std::time::Duration;

use tokio::{
    sync::mpsc::{self, Sender},
    time,
};

use crate::ReachyMiniMotorController;

use std::sync::{Arc, Mutex};

use pyo3::prelude::*;

#[gen_stub_pyclass]
#[pyclass]
#[derive(Debug, Clone, Copy)]
pub struct LastPosition {
    #[pyo3(get)]
    pub body_yaw: f64,
    #[pyo3(get)]
    pub stewart: [f64; 6],
    #[pyo3(get)]
    pub antennas: [f64; 2],
    #[pyo3(get)]
    pub timestamp: f64, // seconds since UNIX epoch
}

pub struct ReachyMiniControlLoop {
    tx: Sender<MotorCommand>,
    last_position: Arc<Mutex<Result<LastPosition, String>>>,
}

#[derive(Debug, Clone)]
pub enum MotorCommand {
    SetAllGoalPositions { positions: [f64; 9] },
    SetStewartPlatformPosition { position: [f64; 6] },
    SetBodyRotation { position: f64 },
    SetAntennasPositions { positions: [f64; 2] },
    EnableTorque(),
    DisableTorque(),
    SetStewartPlatformGoalCurrent { current: [i16; 6] },
    SetStewartPlatformOperatingMode { mode: u8 },
    SetAntennasOperatingMode { mode: u8 },
    SetBodyRotationOperatingMode { mode: u8 },
    EnableStewartPlatform { enable: bool },
    EnableBodyRotation { enable: bool },
    EnableAntennas { enable: bool },
}

impl ReachyMiniControlLoop {
    pub fn new(
        serialport: String,
        read_period: Duration,
        retries: u64,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let (tx, rx) = mpsc::channel(100);
        let last_position = Arc::new(Mutex::new(Err("Not read yet".to_string())));
        let last_position_clone = last_position.clone();

        std::thread::spawn(move || {
            run(serialport, rx, last_position_clone, read_period, retries);
        });

        Ok(ReachyMiniControlLoop { tx, last_position })
    }

    pub fn push_command(
        &self,
        command: MotorCommand,
    ) -> Result<(), mpsc::error::SendError<MotorCommand>> {
        self.tx.blocking_send(command)
    }

    pub fn get_last_position(&self) -> Result<LastPosition, pyo3::PyErr> {
        match &*self.last_position.lock().unwrap() {
            Ok(pos) => Ok(*pos),
            Err(e) => Err(pyo3::exceptions::PyRuntimeError::new_err(e.clone())),
        }
    }
}

fn run(
    serialport: String,
    mut rx: mpsc::Receiver<MotorCommand>,
    last_position: Arc<Mutex<Result<LastPosition, String>>>,
    read_period: Duration,
    retries: u64,
) {
    tokio::runtime::Runtime::new().unwrap().block_on(async {
        let mut c = ReachyMiniMotorController::new(serialport.as_str()).unwrap();
        let mut interval = time::interval(read_period);
        let mut error_count = 0;

        loop {
            tokio::select! {
                maybe_command = rx.recv() => {
                    if let Some(command) = maybe_command {
                        handle_commands(&mut c, command).unwrap();
                    }
                }
                _ = interval.tick() => {
                    match c.read_all_positions() {
                        Ok(positions) => {
                            error_count = 0;
                            if positions.len() == 9 {
                                let now = std::time::SystemTime::now()
                                    .duration_since(std::time::UNIX_EPOCH)
                                    .unwrap_or_else(|_| std::time::Duration::from_secs(0));
                                let last = LastPosition {
                                    body_yaw: positions[0],
                                    stewart: [positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]],
                                    antennas: [positions[7], positions[8]],
                                    timestamp: now.as_secs_f64(),
                                };
                                if let Ok(mut pos) = last_position.lock() {
                                    *pos = Ok(last);
                                }
                            } else {
                                error!("Unexpected positions length: {}", positions.len());
                            }
                        },
                        Err(e) => {
                            error_count += 1;
                            if error_count < retries {
                                warn!("Failed to read positions ({}). Retry {}/{}", e, error_count, retries);
                            } else {
                                error!("Failed to read positions after {} retries: {}", retries, e);
                                if let Ok(mut pos) = last_position.lock() {
                                    *pos = Err(e.to_string());
                                }
                            }
                        },
                    }
                }
            }
        }
    })
}

fn handle_commands(
    controller: &mut ReachyMiniMotorController,
    command: MotorCommand,
) -> Result<(), Box<dyn std::error::Error>> {
    use MotorCommand::*;

    match command {
        SetAllGoalPositions { positions } => controller.set_all_goal_positions(positions),
        SetStewartPlatformPosition { position } => {
            controller.set_stewart_platform_position(position)
        }
        SetBodyRotation { position } => controller.set_body_rotation(position),
        SetAntennasPositions { positions } => controller.set_antennas_positions(positions),
        EnableTorque() => controller.enable_torque(),
        DisableTorque() => controller.disable_torque(),
        SetStewartPlatformGoalCurrent { current } => {
            controller.set_stewart_platform_goal_current(current)
        }
        SetStewartPlatformOperatingMode { mode } => {
            controller.set_stewart_platform_operating_mode(mode)
        }
        SetAntennasOperatingMode { mode } => controller.set_antennas_operating_mode(mode),
        SetBodyRotationOperatingMode { mode } => controller.set_body_rotation_operating_mode(mode),
        EnableStewartPlatform { enable } => controller.enable_stewart_platform(enable),
        EnableBodyRotation { enable } => controller.enable_body_rotation(enable),
        EnableAntennas { enable } => controller.enable_antennas(enable),
    }
}
