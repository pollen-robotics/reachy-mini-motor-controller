use log::{info, warn};
use pyo3::prelude::*;
use pyo3_stub_gen::derive::{gen_stub_pyclass, gen_stub_pymethods};

use std::{
    collections::HashMap,
    fmt::Debug,
    sync::{Arc, Mutex},
    time::{Duration, SystemTime, UNIX_EPOCH},
};
use tokio::{
    sync::mpsc::{self, Receiver, Sender},
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

#[gen_stub_pymethods]
#[pymethods]
impl FullBodyPosition {
    #[new]
    pub fn new(body_yaw: f64, stewart: Vec<f64>, antennas: Vec<f64>) -> Self {
        if stewart.len() != 6 || antennas.len() != 2 {
            panic!("Stewart platform must have 6 positions and antennas must have 2 positions.");
        }
        FullBodyPosition {
            body_yaw,
            stewart: [
                stewart[0], stewart[1], stewart[2], stewart[3], stewart[4], stewart[5],
            ],
            antennas: [antennas[0], antennas[1]],
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or(Duration::from_secs(0))
                .as_secs_f64(),
        }
    }

    fn __repr__(&self) -> pyo3::PyResult<String> {
        Ok(format!(
            "FullBodyPosition(body_yaw={:.3}, stewart={:?}, antennas={:?}, timestamp={:.3})",
            self.body_yaw, self.stewart, self.antennas, self.timestamp
        ))
    }
}

pub struct ReachyMiniControlLoop {
    loop_handle: Arc<Mutex<Option<std::thread::JoinHandle<()>>>>,
    stop_signal: Arc<Mutex<bool>>,
    tx: Sender<MotorCommand>,
    last_position: Arc<Mutex<Result<FullBodyPosition, MotorError>>>,
    last_torque: Arc<Mutex<Result<bool, MotorError>>>,
    last_control_mode: Arc<Mutex<Result<u8, MotorError>>>,
    last_stats: Option<(Duration, Arc<Mutex<ControlLoopStats>>)>,
    rx_raw_bytes: Arc<Mutex<Receiver<Vec<u8>>>>,
    motor_name_id: HashMap<String, u8>,
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

    ReadRawBytes { id: u8, addr: u8, length: u8 },
    WriteRawBytes { id: u8, addr: u8, data: Vec<u8> },
}

#[gen_stub_pyclass]
#[pyclass]
#[derive(Clone)]
pub struct ControlLoopStats {
    #[pyo3(get)]
    pub period: Vec<f64>,
    #[pyo3(get)]
    pub read_dt: Vec<f64>,
    #[pyo3(get)]
    pub write_dt: Vec<f64>,
}

#[pymethods]
impl ControlLoopStats {
    fn __repr__(&self) -> pyo3::PyResult<String> {
        Ok(format!(
            "ControlLoopStats(period=~{:.2?}ms, read_dt=~{:.2?} ms, write_dt=~{:.2?} ms)",
            self.period.iter().sum::<f64>() / self.period.len() as f64 * 1000.0,
            self.read_dt.iter().sum::<f64>() / self.read_dt.len() as f64 * 1000.0,
            self.write_dt.iter().sum::<f64>() / self.write_dt.len() as f64 * 1000.0,
        ))
    }
}

impl std::fmt::Debug for ControlLoopStats {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.__repr__().unwrap())
    }
}

#[derive(Debug, Clone)]
pub enum MotorError {
    MissingMotors(Vec<String>),
    CommunicationError(),
    NoPowerError(),
    VoltageRampUpTimeoutError(u16, Duration),
    PortNotFound(String),
}

impl std::error::Error for MotorError {}
impl std::fmt::Display for MotorError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            MotorError::MissingMotors(names) => {
                write!(f, "Missing motors: {:?}!", names)
            }
            MotorError::CommunicationError() => {
                write!(
                    f,
                    "Motor communication error! Check connections and power supply."
                )
            }
            MotorError::NoPowerError() => {
                write!(
                    f,
                    "No motors detected. Check if the power supply is connected and turned on!"
                )
            }
            MotorError::PortNotFound(port) => {
                write!(
                    f,
                    "Check if your USB cable is connected. Could not find port: {}!",
                    port
                )
            }
            MotorError::VoltageRampUpTimeoutError(voltage, duration) => {
                write!(
                    f,
                    "Voltage did not ramp up to 5V ({}V) within {:?}!",
                    voltage, duration
                )
            }
        }
    }
}

impl ReachyMiniControlLoop {
    pub fn new(
        serialport: String,
        read_position_loop_period: Duration,
        stats_pub_period: Option<Duration>,
        read_allowed_retries: u64,
        voltage_rampup_timeout: Duration,
    ) -> Result<Self, MotorError> {
        let stop_signal = Arc::new(Mutex::new(false));
        let stop_signal_clone = stop_signal.clone();

        let (tx, rx) = mpsc::channel(100);

        let last_stats = stats_pub_period.map(|period| {
            (
                period,
                Arc::new(Mutex::new(ControlLoopStats {
                    period: Vec::new(),
                    read_dt: Vec::new(),
                    write_dt: Vec::new(),
                })),
            )
        });
        let last_stats_clone = last_stats.clone();

        // Validate serial port based on operating system

        // On Unix-like systems, check if the port path exists
        #[cfg(not(windows))]
        if !std::path::Path::new(&serialport).exists() {
            return Err(MotorError::PortNotFound(serialport));
        }
        // On Windows, validate COM port format
        #[cfg(windows)]
        if !serialport.starts_with("COM") {
            return Err(MotorError::PortNotFound(serialport));
        }

        let mut c = ReachyMiniMotorController::new(serialport.as_str()).unwrap();

        match c.check_missing_ids() {
            Ok(missing_ids) if missing_ids.len() == 9 => {
                return Err(MotorError::NoPowerError());
            }
            Ok(missing_ids) if !missing_ids.is_empty() => {
                let id_to_name: HashMap<u8, String> = c
                    .get_motor_name_id()
                    .iter()
                    .map(|(name, id)| (id.clone(), name.clone()))
                    .collect();

                let missing_motors: Vec<String> = missing_ids
                    .iter()
                    .map(|id| {
                        id_to_name
                            .get(id)
                            .unwrap_or(&format!("Unknown({})", id))
                            .clone()
                    })
                    .collect();
                return Err(MotorError::MissingMotors(missing_motors));
            }
            Ok(_) => {}
            Err(_) => return Err(MotorError::CommunicationError()),
        }

        // Wait until voltage is stable at 5V
        info!("Waiting for voltage to be stable at 5V...");
        let mut current_voltage = read_volt_with_retries(&mut c, read_allowed_retries)?;
        let start_time = SystemTime::now();
        while current_voltage
            .iter()
            .any(|&v| v < 45 && start_time.elapsed().unwrap() < voltage_rampup_timeout)
        {
            std::thread::sleep(Duration::from_millis(100));
            current_voltage = read_volt_with_retries(&mut c, read_allowed_retries)?;
        }
        if current_voltage.iter().any(|&v| v < 45) {
            return Err(MotorError::VoltageRampUpTimeoutError(
                current_voltage.iter().cloned().min().unwrap_or(0),
                voltage_rampup_timeout,
            ));
        }
        info!(
            "Voltage is stable at ~5V: {:?} (took {:?})",
            current_voltage,
            start_time.elapsed().unwrap()
        );

        let motor_name_id = c.get_motor_name_id();

        // Reboot all motors on error status
        c.reboot(true)
            .map_err(|_| MotorError::CommunicationError())?;

        // Init last position by trying to read current positions
        // If the init fails, it probably means we have an hardware issue
        // so it's better to fail.
        let last_position = read_pos_with_retries(&mut c, read_allowed_retries)?;
        let last_torque = read_torque_with_retries(&mut c, read_allowed_retries)?;
        let last_control_mode = read_control_mode_with_retries(&mut c, read_allowed_retries)?;

        let last_position = Arc::new(Mutex::new(Ok(last_position)));
        let last_position_clone = last_position.clone();

        let last_torque = Arc::new(Mutex::new(Ok(last_torque)));
        let last_torque_clone = last_torque.clone();
        let last_control_mode = Arc::new(Mutex::new(Ok(last_control_mode)));
        let last_control_mode_clone = last_control_mode.clone();

        let (tx_raw_bytes, rx_raw_bytes) = mpsc::channel(1);

        let loop_handle = std::thread::spawn(move || {
            run(
                c,
                stop_signal_clone,
                rx,
                last_position_clone,
                last_torque_clone,
                last_control_mode_clone,
                last_stats_clone,
                read_position_loop_period,
                read_allowed_retries,
                tx_raw_bytes,
            );
        });

        let rx_raw_bytes = Arc::new(Mutex::new(rx_raw_bytes));

        Ok(ReachyMiniControlLoop {
            loop_handle: Arc::new(Mutex::new(Some(loop_handle))),
            stop_signal,
            tx,
            last_position,
            last_torque,
            last_control_mode,
            last_stats,
            rx_raw_bytes,
            motor_name_id,
        })
    }

    pub fn close(&self) {
        if let Ok(mut stop) = self.stop_signal.lock() {
            *stop = true;
        }
        if let Some(handle) = self.loop_handle.lock().unwrap().take() {
            handle.join().unwrap();
        }
    }

    pub fn get_motor_name_id(&self) -> HashMap<String, u8> {
        self.motor_name_id.clone()
    }

    pub fn push_command(
        &self,
        command: MotorCommand,
    ) -> Result<(), mpsc::error::SendError<MotorCommand>> {
        self.tx.blocking_send(command)
    }

    pub fn get_last_position(&self) -> Result<FullBodyPosition, MotorError> {
        match &*self.last_position.lock().unwrap() {
            Ok(pos) => Ok(*pos),
            Err(e) => Err(e.clone()),
        }
    }

    pub fn is_torque_enabled(&self) -> Result<bool, MotorError> {
        match &*self.last_torque.lock().unwrap() {
            Ok(enabled) => Ok(*enabled),
            Err(e) => Err(e.clone()),
        }
    }

    pub fn get_control_mode(&self) -> Result<u8, MotorError> {
        match &*self.last_control_mode.lock().unwrap() {
            Ok(mode) => Ok(*mode),
            Err(e) => Err(e.clone()),
        }
    }

    pub fn get_stats(&self) -> Result<Option<ControlLoopStats>, MotorError> {
        match self.last_stats {
            Some((_, ref stats)) => {
                let stats = stats.lock().unwrap();
                Ok(Some(stats.clone()))
            }
            None => Ok(None),
        }
    }

    pub fn async_read_raw_bytes(
        &self,
        id: u8,
        addr: u8,
        length: u8,
    ) -> Result<Vec<u8>, MotorError> {
        let command = MotorCommand::ReadRawBytes { id, addr, length };
        self.push_command(command)
            .map_err(|_| MotorError::CommunicationError())?;
        let data = self
            .rx_raw_bytes
            .lock()
            .unwrap()
            .blocking_recv()
            .ok_or(MotorError::CommunicationError())?;
        Ok(data)
    }

    pub fn async_write_raw_bytes(&self, id: u8, addr: u8, data: Vec<u8>) -> Result<(), MotorError> {
        let command = MotorCommand::WriteRawBytes { id, addr, data };
        self.push_command(command)
            .map_err(|_| MotorError::CommunicationError())?;
        Ok(())
    }
}

impl Drop for ReachyMiniControlLoop {
    fn drop(&mut self) {
        self.close();
    }
}

fn run(
    mut c: ReachyMiniMotorController,
    stop_signal: Arc<Mutex<bool>>,
    mut rx: mpsc::Receiver<MotorCommand>,
    last_position: Arc<Mutex<Result<FullBodyPosition, MotorError>>>,
    last_torque: Arc<Mutex<Result<bool, MotorError>>>,
    last_control_mode: Arc<Mutex<Result<u8, MotorError>>>,
    last_stats: Option<(Duration, Arc<Mutex<ControlLoopStats>>)>,
    read_position_loop_period: Duration,
    read_allowed_retries: u64,
    tx_raw_bytes: Sender<Vec<u8>>,
) {
    tokio::runtime::Runtime::new().unwrap().block_on(async {
        let mut interval = time::interval(read_position_loop_period);
        let mut error_count = 0;

        // Stats related variables
        let mut stats_t0 = std::time::Instant::now();
        let mut read_dt = Vec::new();
        let mut write_dt = Vec::new();

        let mut last_read_tick = std::time::Instant::now();

        loop {
            tokio::select! {
                maybe_command = rx.recv() => {
                    if let Some(command) = maybe_command {
                        let write_tick = std::time::Instant::now();
                        let res = handle_commands(&mut c, last_torque.clone(), last_control_mode.clone(), command).unwrap();

                        if let Some(data) = res {
                           // This means we had a ReadRawBytes command
                            tx_raw_bytes.send(data).await.unwrap();
                        }

                        if last_stats.is_some() {
                            let elapsed = write_tick.elapsed().as_secs_f64();
                            write_dt.push(elapsed);
                        }
                    }
                }
                _ = interval.tick() => {
                    let read_tick = std::time::Instant::now();
                    if let Some((_, stats)) = &last_stats {
                        stats.lock().unwrap().period.push(read_tick.duration_since(last_read_tick).as_secs_f64());
                        last_read_tick = read_tick;
                    }

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
                            if error_count >= read_allowed_retries && let Ok(mut pos) = last_position.lock() {
                                *pos = Err(e);
                            }
                        },
                    }
                    if last_stats.is_some() {
                        let elapsed = read_tick.elapsed().as_secs_f64();
                        read_dt.push(elapsed);
                    }

                    if let Some((period, stats)) = &last_stats
                        && stats_t0.elapsed() > *period {
                            stats.lock().unwrap().read_dt.extend(read_dt.iter().cloned());
                            stats.lock().unwrap().write_dt.extend(write_dt.iter().cloned());

                            read_dt.clear();
                            write_dt.clear();
                            stats_t0 = std::time::Instant::now();
                    }
                }
            }

            if *stop_signal.lock().unwrap() {
                // Drain the command channel before exiting
                loop {
                    if rx.is_empty() {
                        break;
                    }
                    if let Some(command) = rx.recv().await {
                        handle_commands(&mut c, last_torque.clone(), last_control_mode.clone(), command).unwrap();
                    }
                }
                break;
            }
        }
    })
}

fn handle_commands(
    controller: &mut ReachyMiniMotorController,
    last_torque: Arc<Mutex<Result<bool, MotorError>>>,
    last_control_mode: Arc<Mutex<Result<u8, MotorError>>>,
    command: MotorCommand,
) -> Result<Option<Vec<u8>>, Box<dyn std::error::Error>> {
    use MotorCommand::*;

    match command {
        SetAllGoalPositions { positions } => controller
            .set_all_goal_positions([
                positions.body_yaw,
                positions.stewart[0],
                positions.stewart[1],
                positions.stewart[2],
                positions.stewart[3],
                positions.stewart[4],
                positions.stewart[5],
                positions.antennas[0],
                positions.antennas[1],
            ])
            .map(|_| None),
        SetStewartPlatformPosition { position } => controller
            .set_stewart_platform_position(position)
            .map(|_| None),
        SetBodyRotation { position } => controller.set_body_rotation(position).map(|_| None),
        SetAntennasPositions { positions } => {
            controller.set_antennas_positions(positions).map(|_| None)
        }
        EnableTorque() => {
            let res = controller.enable_torque();
            if res.is_ok()
                && let Ok(mut torque) = last_torque.lock()
            {
                *torque = Ok(true);
            }
            res.map(|_| None)
        }
        DisableTorque() => {
            let res = controller.disable_torque();
            if res.is_ok()
                && let Ok(mut torque) = last_torque.lock()
            {
                *torque = Ok(false);
            }
            res.map(|_| None)
        }
        SetStewartPlatformGoalCurrent { current } => controller
            .set_stewart_platform_goal_current(current)
            .map(|_| None),
        SetStewartPlatformOperatingMode { mode } => {
            let res = controller.set_stewart_platform_operating_mode(mode);
            if res.is_ok()
                && let Ok(mut control_mode) = last_control_mode.lock()
            {
                *control_mode = Ok(mode);
            }
            res.map(|_| None)
        }
        SetAntennasOperatingMode { mode } => {
            controller.set_antennas_operating_mode(mode).map(|_| None)
        }
        SetBodyRotationOperatingMode { mode } => controller
            .set_body_rotation_operating_mode(mode)
            .map(|_| None),
        EnableStewartPlatform { enable } => {
            controller.enable_stewart_platform(enable).map(|_| None)
        }
        EnableBodyRotation { enable } => controller.enable_body_rotation(enable).map(|_| None),
        EnableAntennas { enable } => controller.enable_antennas(enable).map(|_| None),
        ReadRawBytes { id, addr, length } => {
            let data = controller.read_raw_bytes(id, addr, length)?;
            Ok(Some(data))
        }
        WriteRawBytes { id, addr, data } => {
            controller.write_raw_bytes(id, addr, &data).map(|_| None)
        }
    }
}

pub fn read_pos(c: &mut ReachyMiniMotorController) -> Result<FullBodyPosition, MotorError> {
    match c.read_all_positions() {
        Ok(positions) => {
            let now = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_else(|_| std::time::Duration::from_secs(0));
            Ok(FullBodyPosition {
                body_yaw: positions[0],
                stewart: [
                    positions[1],
                    positions[2],
                    positions[3],
                    positions[4],
                    positions[5],
                    positions[6],
                ],
                antennas: [positions[7], positions[8]],
                timestamp: now.as_secs_f64(),
            })
        }
        Err(_) => Err(MotorError::CommunicationError()),
    }
}

pub fn read_volt(c: &mut ReachyMiniMotorController) -> Result<[u16; 9], String> {
    match c.read_all_voltages() {
        Ok(voltages) => {
            if voltages.len() == 9 {
                Ok(voltages)
            } else {
                Err(format!("Unexpected voltages length: {}", voltages.len()))
            }
        }
        Err(e) => Err(e.to_string()),
    }
}

fn read_pos_with_retries(
    c: &mut ReachyMiniMotorController,
    retries: u64,
) -> Result<FullBodyPosition, MotorError> {
    for i in 0..retries {
        match read_pos(c) {
            Ok(pos) => return Ok(pos),
            Err(e) => {
                warn!(
                    "Failed to read positions: {:?}. Retrying... {}/{}",
                    e,
                    i + 1,
                    retries
                );
            }
        }
    }
    Err(MotorError::CommunicationError())
}

fn read_volt_with_retries(
    c: &mut ReachyMiniMotorController,
    retries: u64,
) -> Result<[u16; 9], MotorError> {
    for i in 0..retries {
        match read_volt(c) {
            Ok(voltages) => return Ok(voltages),
            Err(e) => {
                warn!(
                    "Failed to read voltages: {}. Retrying... {}/{}",
                    e,
                    i + 1,
                    retries
                );
            }
        }
    }
    Err(MotorError::CommunicationError())
}

fn read_torque_with_retries(
    c: &mut ReachyMiniMotorController,
    retries: u64,
) -> Result<bool, MotorError> {
    for i in 0..retries {
        match c.is_torque_enabled() {
            Ok(enabled) => {
                return Ok(enabled);
            }
            Err(e) => {
                warn!(
                    "Failed to read torque status: {}. Retrying... {}/{}",
                    e,
                    i + 1,
                    retries
                );
            }
        }
    }
    Err(MotorError::CommunicationError())
}

fn read_control_mode_with_retries(
    c: &mut ReachyMiniMotorController,
    retries: u64,
) -> Result<u8, MotorError> {
    for i in 0..retries {
        match c.read_stewart_platform_operating_mode() {
            Ok(mode) => {
                return Ok(mode[0]);
            }
            Err(e) => {
                warn!(
                    "Failed to read operating mode: {}. Retrying... {}/{}",
                    e,
                    i + 1,
                    retries
                );
            }
        }
    }
    Err(MotorError::CommunicationError())
}
