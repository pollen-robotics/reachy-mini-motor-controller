use std::{collections::HashMap, time::Duration};

use log::warn;
use rustypot::servo::dynamixel::xl330;

pub struct ReachyMiniMotorController {
    dph_v2: rustypot::DynamixelProtocolHandler,
    serial_port: Box<dyn serialport::SerialPort>,
    all_ids: [u8; 9],
}

const ANTENNAS_IDS: [u8; 2] = [17, 18]; // Right and Left antennas
const STEWART_PLATFORM_IDS: [u8; 6] = [11, 12, 13, 14, 15, 16];
const BODY_ROTATION_ID: u8 = 10;

impl ReachyMiniMotorController {
    pub fn new(serialport: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let dph_v2 = rustypot::DynamixelProtocolHandler::v2();

        let serial_port = serialport::new(serialport, 1_000_000)
            .timeout(Duration::from_millis(10))
            .open()?;

        let all_ids = [
            BODY_ROTATION_ID,
            STEWART_PLATFORM_IDS[0],
            STEWART_PLATFORM_IDS[1],
            STEWART_PLATFORM_IDS[2],
            STEWART_PLATFORM_IDS[3],
            STEWART_PLATFORM_IDS[4],
            STEWART_PLATFORM_IDS[5],
            ANTENNAS_IDS[0],
            ANTENNAS_IDS[1],
        ];

        Ok(Self {
            dph_v2,
            serial_port,
            all_ids,
        })
    }

    pub fn get_motor_name_id(&self) -> HashMap<String, u8> {
        let mut motor_id_name = HashMap::new();
        motor_id_name.insert("body_rotation".to_string(), BODY_ROTATION_ID);
        motor_id_name.insert("stewart_1".to_string(), STEWART_PLATFORM_IDS[0]);
        motor_id_name.insert("stewart_2".to_string(), STEWART_PLATFORM_IDS[1]);
        motor_id_name.insert("stewart_3".to_string(), STEWART_PLATFORM_IDS[2]);
        motor_id_name.insert("stewart_4".to_string(), STEWART_PLATFORM_IDS[3]);
        motor_id_name.insert("stewart_5".to_string(), STEWART_PLATFORM_IDS[4]);
        motor_id_name.insert("stewart_6".to_string(), STEWART_PLATFORM_IDS[5]);
        motor_id_name.insert("right_antenna".to_string(), ANTENNAS_IDS[0]);
        motor_id_name.insert("left_antenna".to_string(), ANTENNAS_IDS[1]);
        motor_id_name
    }

    pub fn reboot(
        &mut self,
        on_error_status_only: bool,
        reboot_timeout: Duration,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let mut error_status = Vec::new();

        if on_error_status_only {
            error_status = xl330::sync_read_hardware_error_status(
                &self.dph_v2,
                self.serial_port.as_mut(),
                &self.all_ids,
            )?;
        }

        let faulty_ids: Vec<u8> = if on_error_status_only {
            self.all_ids
                .iter()
                .zip(error_status.iter())
                // Ignore input voltage error (status == 1) for reboot decision.
                .filter_map(|(&id, &status)| {
                    if status != 0 && status != 1 {
                        Some(id)
                    } else {
                        None
                    }
                })
                .collect()
        } else {
            self.all_ids.to_vec()
        };

        let name2id = self.get_motor_name_id();
        let id2name: HashMap<u8, String> =
            name2id.into_iter().map(|(name, id)| (id, name)).collect();

        for id in &faulty_ids {
            let name = id2name.get(id).unwrap();
            warn!("Rebooting motor {} (id={})", name, id);
            self.dph_v2.reboot(self.serial_port.as_mut(), *id as u8)?;
        }

        let mut missing_ids = faulty_ids.clone();
        let start_time = std::time::Instant::now();
        while !missing_ids.is_empty() && start_time.elapsed() < reboot_timeout {
            std::thread::sleep(Duration::from_millis(100));
            missing_ids = missing_ids
                .into_iter()
                .filter(|id| {
                    let ping_result = self.dph_v2.ping(self.serial_port.as_mut(), *id);
                    match ping_result {
                        Ok(res) => !res,
                        Err(_) => true,
                    }
                })
                .collect();
        }
        for id in &missing_ids {
            let name = id2name.get(id).unwrap();
            warn!(
                "Motor {} (id={}) did not respond after reboot within timeout",
                name, id
            );
        }

        if missing_ids.is_empty() {
            Ok(())
        } else {
            let names = missing_ids
                .iter()
                .map(|id| id2name.get(id).unwrap().to_string())
                .collect::<Vec<String>>();
            Err(format!(
                "Some motors did not respond after reboot ({:?} - ids: {:?})",
                names, missing_ids
            )
            .into())
        }
    }

    pub fn check_missing_ids(&mut self) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        let mut missing_ids = Vec::new();

        for id in self.all_ids {
            if xl330::read_id(&self.dph_v2, self.serial_port.as_mut(), id).is_err() {
                missing_ids.push(id);
            }
        }

        Ok(missing_ids)
    }

    /// Read the current input voltage of all servos.
    /// Returns an array of 9 input voltages in the following order:
    /// [body_rotation, stewart_1, stewart_2, stewart_3, stewart_4, stewart_5, stewart_6, antenna_right, antenna_left]
    pub fn read_all_voltages(&mut self) -> Result<[u16; 9], Box<dyn std::error::Error>> {
        let volt = xl330::sync_read_present_input_voltage(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &self.all_ids,
        )?;
        
        volt.try_into()
            .map_err(|_| "Invalid voltage array length: expected 9 elements".into())
    }
        

    /// Read the current position of all servos.
    /// Returns an array of 9 positions in the following order:
    /// [body_rotation, stewart_1, stewart_2, stewart_3, stewart_4, stewart_5, stewart_6, antenna_right, antenna_left]
    pub fn read_all_positions(&mut self) -> Result<[f64; 9], Box<dyn std::error::Error>> {
        let pos = xl330::sync_read_present_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &self.all_ids,
        )?;
        
        pos.try_into()
            .map_err(|_| "Invalid position array length: expected 9 elements".into())
    }

    /// Set the goal position of all servos.
    /// The positions array must be in the following order:
    /// [body_rotation, stewart_1, stewart_2, stewart_3, stewart_4, stewart_5, stewart_6, antenna_right, antenna_left]
    pub fn set_all_goal_positions(
        &mut self,
        positions: [f64; 9],
    ) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_goal_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &self.all_ids,
            &positions,
        )?;

        Ok(())
    }

    pub fn set_antennas_positions(
        &mut self,
        positions: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_goal_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &ANTENNAS_IDS,
            &positions,
        )?;

        Ok(())
    }

    pub fn set_stewart_platform_position(
        &mut self,
        position: [f64; 6],
    ) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_goal_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
            &position,
        )?;

        Ok(())
    }
    pub fn set_body_rotation(&mut self, position: f64) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_goal_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &[BODY_ROTATION_ID],
            &[position],
        )?;

        Ok(())
    }

    pub fn is_torque_enabled(&mut self) -> Result<bool, Box<dyn std::error::Error>> {
        let xl_torque =
            xl330::sync_read_torque_enable(&self.dph_v2, self.serial_port.as_mut(), &self.all_ids)?;

        Ok(xl_torque.iter().all(|&x| x))
    }

    pub fn enable_torque(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque(true)
    }

    pub fn enable_torque_on_ids(&mut self, ids: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque_on_ids(ids, true)
    }

    pub fn disable_torque(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque(false)
    }

    pub fn disable_torque_on_ids(&mut self, ids: &[u8]) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque_on_ids(ids, false)
    }

    fn set_torque(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_torque_enable(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &self.all_ids,
            &[enable; 9],
        )?;

        Ok(())
    }

    fn set_torque_on_ids(
        &mut self,
        ids: &[u8],
        enable: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let enables = vec![enable; ids.len()];
        xl330::sync_write_torque_enable(&self.dph_v2, self.serial_port.as_mut(), ids, &enables)?;

        Ok(())
    }

    pub fn set_stewart_platform_goal_current(
        &mut self,
        current: [i16; 6],
    ) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_goal_current(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
            &current,
        )?;

        Ok(())
    }

    pub fn read_stewart_platform_current(
        &mut self,
    ) -> Result<[i16; 6], Box<dyn std::error::Error>> {
        let currents = xl330::sync_read_present_current(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
        )?;

        currents.try_into()
            .map_err(|_| "Invalid current array length: expected 6 elements".into())
    }

    pub fn set_stewart_platform_operating_mode(
        &mut self,
        mode: u8,
    ) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_operating_mode(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
            &[mode; 6],
        )?;

        Ok(())
    }

    pub fn read_stewart_platform_operating_mode(
        &mut self,
    ) -> Result<[u8; 6], Box<dyn std::error::Error>> {
        let modes = xl330::sync_read_operating_mode(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
        )?;

        modes.try_into()
            .map_err(|_| "Invalid mode array length: expected 6 elements".into())
    }

    pub fn set_antennas_operating_mode(
        &mut self,
        mode: u8,
    ) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_operating_mode(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &ANTENNAS_IDS,
            &[mode; 2],
        )?;

        Ok(())
    }

    pub fn set_body_rotation_operating_mode(
        &mut self,
        mode: u8,
    ) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_operating_mode(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &[BODY_ROTATION_ID],
            &[mode],
        )?;

        Ok(())
    }

    pub fn enable_body_rotation(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_torque_enable(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &[BODY_ROTATION_ID],
            &[enable],
        )?;

        Ok(())
    }

    pub fn enable_antennas(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_torque_enable(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &ANTENNAS_IDS,
            &[enable; 2],
        )?;

        Ok(())
    }

    pub fn enable_stewart_platform(
        &mut self,
        enable: bool,
    ) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_torque_enable(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
            &[enable; 6],
        )?;

        Ok(())
    }

    pub fn read_raw_bytes(
        &mut self,
        id: u8,
        address: u8,
        length: u8,
    ) -> Result<Vec<u8>, Box<dyn std::error::Error>> {
        self.dph_v2
            .read(self.serial_port.as_mut(), id, address, length)
    }

    pub fn write_raw_bytes(
        &mut self,
        id: u8,
        address: u8,
        data: &[u8],
    ) -> Result<(), Box<dyn std::error::Error>> {
        self.dph_v2
            .write(self.serial_port.as_mut(), id, address, data)
    }

    pub fn write_raw_packet(&mut self, data: &[u8]) -> Result<Vec<u8>, std::io::Error> {
        self.serial_port.write_all(data)?;
        self.serial_port.flush()?;

        let mut n = self.serial_port.bytes_to_read()? as usize;
        let start = std::time::Instant::now();
        while n == 0 && start.elapsed() < Duration::from_millis(10) {
            std::thread::sleep(Duration::from_millis(5));
            n = self.serial_port.bytes_to_read()? as usize;
        }
        let mut buff = vec![0u8; n];
        self.serial_port.read_exact(&mut buff)?;

        Ok(buff)
    }
}
