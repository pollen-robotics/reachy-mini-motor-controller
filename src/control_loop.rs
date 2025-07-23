use log::{error, warn};
use pyo3::prelude::*;
use pyo3_stub_gen::derive::gen_stub_pyclass;

use std::{
    sync::{Arc, Mutex},
    time::Duration,
};
use tokio::{
    sync::mpsc::{self, Sender},
    time,
};

use crate::ReachyMiniMotorController;

#[gen_stub_pyclass]
#[pyclass]
#[derive(Debug, Clone, Copy)]
pub struct FullBodyPosition {
    #[pyo3(get)]
    pub body_yaw: f64,
    #[pyo3(get)]
    pub stewart: [f64; 6],
    #[pyo3(get)]
    pub antennas: [f64; 2],
    #[pyo3(get)]
    pub timestamp: f64, // seconds since UNIX epoch
}

#[pymethods]
impl FullBodyPosition {
    fn __repr__(&self) -> pyo3::PyResult<String> {
        Ok(format!(
            "FullBodyPosition(body_yaw={:.3}, stewart={:?}, antennas={:?}, timestamp={:.3})",
            self.body_yaw, self.stewart, self.antennas, self.timestamp
        ))
    }
}

pub struct ReachyMiniControlLoop {
    tx: Sender<MotorCommand>,
    last_position: Arc<Mutex<Result<FullBodyPosition, String>>>,
    last_stats: Option<Arc<Mutex<ControlLoopStats>>>,
}

#[derive(Debug, Clone)]
pub enum MotorCommand {
    SetAllGoalPositions { positions: FullBodyPosition },
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

#[gen_stub_pyclass]
#[pyclass]
#[derive(Debug, Clone)]
pub struct ControlLoopStats {
    #[pyo3(get)]
    pub period: Duration,
    #[pyo3(get)]
    pub read_dt: Vec<f64>,
    #[pyo3(get)]
    pub write_dt: Vec<f64>,
}

#[pymethods]
impl ControlLoopStats {
    fn __repr__(&self) -> pyo3::PyResult<String> {
        let read_dt_avg = if !self.read_dt.is_empty() {
            self.read_dt.iter().sum::<f64>() / self.read_dt.len() as f64 * 1000.0
        } else {
            0.0
        };

        let write_dt_avg = if !self.write_dt.is_empty() {
            self.write_dt.iter().sum::<f64>() / self.write_dt.len() as f64 * 1000.0
        } else {
            0.0
        };

        Ok(format!(
            "ControlLoopStats(period={:?}, read_dt={:.2}ms, write_dt={:.2}ms)",
            self.period, read_dt_avg, write_dt_avg
        ))
    }
}

impl ReachyMiniControlLoop {
    pub fn new(
        serialport: String,
        read_period: Duration,
        retries: u64,
        stats_pub_period: Option<Duration>,
        timeout: Duration,
    ) -> Result<Self, Box<dyn std::error::Error>> {
        let (tx, rx) = mpsc::channel(100);

        let last_stats = stats_pub_period.map(|period| {
            Arc::new(Mutex::new(ControlLoopStats {
                period,
                read_dt: Vec::new(),
                write_dt: Vec::new(),
            }))
        });
        let last_stats_clone = last_stats.clone();

        let mut c = ReachyMiniMotorController::new(serialport.as_str()).unwrap();

        let last_position = tries_to_get_one_pos(&mut c, timeout)?;

        let last_position = Arc::new(Mutex::new(Ok(last_position)));
        let last_position_clone = last_position.clone();

        std::thread::spawn(move || {
            run(
                c,
                rx,
                last_position_clone,
                last_stats_clone,
                read_period,
                retries,
            );
        });

        Ok(ReachyMiniControlLoop {
            tx,
            last_position,
            last_stats,
        })
    }

    pub fn push_command(
        &self,
        command: MotorCommand,
    ) -> Result<(), mpsc::error::SendError<MotorCommand>> {
        self.tx.blocking_send(command)
    }

    pub fn get_last_position(&self) -> Result<FullBodyPosition, pyo3::PyErr> {
        match &*self.last_position.lock().unwrap() {
            Ok(pos) => Ok(*pos),
            Err(e) => Err(pyo3::exceptions::PyRuntimeError::new_err(e.clone())),
        }
    }

    pub fn get_stats(&self) -> Result<Option<ControlLoopStats>, pyo3::PyErr> {
        match self.last_stats {
            Some(ref stats) => {
                let stats = stats.lock().unwrap();
                Ok(Some(stats.clone()))
            }
            None => Ok(None),
        }
    }
}

fn run(
    mut c: ReachyMiniMotorController,
    mut rx: mpsc::Receiver<MotorCommand>,
    last_position: Arc<Mutex<Result<FullBodyPosition, String>>>,
    last_stats: Option<Arc<Mutex<ControlLoopStats>>>,
    read_period: Duration,
    retries: u64,
) {
    tokio::runtime::Runtime::new().unwrap().block_on(async {
        let mut interval = time::interval(read_period);
        let mut error_count = 0;

        // Stats related variables
        let mut stats_t0 = std::time::Instant::now();
        let mut read_dt = Vec::new();
        let mut write_dt = Vec::new();

        loop {
            tokio::select! {
                maybe_command = rx.recv() => {
                    if let Some(command) = maybe_command {
                        let write_tick = std::time::Instant::now();
                        handle_commands(&mut c, command).unwrap();
                        if last_stats.is_some() {
                            let elapsed = write_tick.elapsed().as_secs_f64();
                            write_dt.push(elapsed);
                        }
                    }
                }
                _ = interval.tick() => {
                    let read_tick = std::time::Instant::now();

                    match read_pos(&mut c) {
                        Ok(positions) => {
                            error_count = 0;
                                let now = std::time::SystemTime::now()
                                    .duration_since(std::time::UNIX_EPOCH)
                                    .unwrap_or_else(|_| std::time::Duration::from_secs(0));
                                let last = FullBodyPosition {
                                    body_yaw: positions.body_yaw,
                                    stewart: positions.stewart,
                                    antennas: positions.antennas,
                                    timestamp: now.as_secs_f64(),
                                };
                                if let Ok(mut pos) = last_position.lock() {
                                    *pos = Ok(last);
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
                    if last_stats.is_some() {
                        let elapsed = read_tick.elapsed().as_secs_f64();
                        read_dt.push(elapsed);
                    }

                    if let Some(stats) = &last_stats {
                        if stats_t0.elapsed() > stats.lock().unwrap().period {
                            stats.lock().unwrap().read_dt.extend(read_dt.iter().cloned());
                            stats.lock().unwrap().write_dt.extend(write_dt.iter().cloned());

                            read_dt.clear();
                            write_dt.clear();
                            stats_t0 = std::time::Instant::now();
                        }
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
        SetAllGoalPositions { positions } => controller.set_all_goal_positions([
            positions.body_yaw,
            positions.antennas[0],
            positions.antennas[1],
            positions.stewart[0],
            positions.stewart[1],
            positions.stewart[2],
            positions.stewart[3],
            positions.stewart[4],
            positions.stewart[5],
        ]),
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

fn read_pos(c: &mut ReachyMiniMotorController) -> Result<FullBodyPosition, String> {
    match c.read_all_positions() {
        Ok(positions) => {
            if positions.len() == 9 {
                let now = std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .unwrap_or_else(|_| std::time::Duration::from_secs(0));
                Ok(FullBodyPosition {
                    body_yaw: positions[0],
                    stewart: [
                        positions[3],
                        positions[4],
                        positions[5],
                        positions[6],
                        positions[7],
                        positions[8],
                    ],
                    antennas: [positions[1], positions[2]],
                    timestamp: now.as_secs_f64(),
                })
            } else {
                Err(format!("Unexpected positions length: {}", positions.len()))
            }
        }
        Err(e) => Err(e.to_string()),
    }
}

fn tries_to_get_one_pos(
    c: &mut ReachyMiniMotorController,
    timeout: Duration,
) -> Result<FullBodyPosition, Box<dyn std::error::Error>> {
    let start = std::time::Instant::now();
    loop {
        match read_pos(c) {
            Ok(pos) => return Ok(pos),
            Err(e) => warn!("Failed to read positions: {}", e),
        }

        if start.elapsed() > timeout {
            return Err(Box::new(std::io::Error::new(
                std::io::ErrorKind::TimedOut,
                "Timeout while trying to read motor positions",
            )));
        }
    }
}
