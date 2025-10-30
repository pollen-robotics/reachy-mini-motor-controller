use std::{collections::HashMap, time::Duration};

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

    pub fn reboot(&mut self, on_error_status_only: bool) -> Result<(), Box<dyn std::error::Error>> {
        let mut error_status = Vec::new();
        if on_error_status_only {
            error_status = xl330::sync_read_hardware_error_status(
                &self.dph_v2,
                self.serial_port.as_mut(),
                &self.all_ids,
            )?;
        }
        //println!("Error status: {:?}", error_status);

        let mut last_reboot_id = self.all_ids[0];
        for (pos, id) in self.all_ids.iter().enumerate() {
            if !on_error_status_only || (on_error_status_only && error_status[pos] == 1) {
                self.dph_v2.reboot(self.serial_port.as_mut(), *id as u8)?;
                last_reboot_id = *id;
            }
        }
        while !self
            .dph_v2
            .ping(self.serial_port.as_mut(), last_reboot_id)?
        {
            std::thread::sleep(Duration::from_millis(100));
        }
        Ok(())
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
        let mut volt = Vec::new();

        volt.extend(xl330::sync_read_present_input_voltage(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &self.all_ids,
        )?);

        Ok(volt.try_into().unwrap())
    }

    /// Read the current position of all servos.
    /// Returns an array of 9 positions in the following order:
    /// [body_rotation, stewart_1, stewart_2, stewart_3, stewart_4, stewart_5, stewart_6, antenna_right, antenna_left]
    pub fn read_all_positions(&mut self) -> Result<[f64; 9], Box<dyn std::error::Error>> {
        let mut pos = Vec::new();

        pos.extend(xl330::sync_read_present_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &self.all_ids,
        )?);

        Ok(pos.try_into().unwrap())
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
    pub fn disable_torque(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque(false)
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

        Ok(currents.try_into().unwrap())
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

        Ok(modes.try_into().unwrap())
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
}
