use std::time::Duration;

use rustypot::servo::{dynamixel::xl330, feetech::sts3215};

pub struct ReachyMiniMotorController {
    dph_v1: rustypot::DynamixelProtocolHandler,
    dph_v2: rustypot::DynamixelProtocolHandler,
    serial_port: Box<dyn serialport::SerialPort>,
}

impl ReachyMiniMotorController {
    pub fn new(serialport: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let dph_v1 = rustypot::DynamixelProtocolHandler::v1();
        let dph_v2 = rustypot::DynamixelProtocolHandler::v2();

        let serial_port = serialport::new(serialport, 1_000_000)
            .timeout(Duration::from_millis(10))
            .open()?;

        Ok(Self {
            dph_v1,
            dph_v2,
            serial_port,
        })
    }

    pub fn read_dxl_hardware_error_status(
        &mut self,
    ) -> Result<[u8; 6], Box<dyn std::error::Error>> {
        let err = xl330::sync_read_hardware_error_status(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![1, 2, 3, 4, 5, 6],
        )?;

        Ok(err.try_into().unwrap())
    }

    pub fn set_dxl_operating_mode(&mut self, mode: u8) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_operating_mode(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![1, 2, 3, 4, 5, 6],
            &[mode, mode, mode, mode, mode, mode],
        )?;

        Ok(())
    }

    pub fn get_dxl_operating_mode(&mut self) -> Result<[u8; 6], Box<dyn std::error::Error>> {
        let m = xl330::sync_read_operating_mode(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![1, 2, 3, 4, 5, 6],
        )?;

        Ok(m.try_into().unwrap())
    }

    pub fn set_dxl_current_limit(&mut self, limit: u16) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_current_limit(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![1, 2, 3, 4, 5, 6],
            &[limit, limit, limit, limit, limit, limit],
        )?;

        Ok(())
    }

    pub fn get_dxl_current_limit(&mut self) -> Result<[u16; 6], Box<dyn std::error::Error>> {
        let l = xl330::sync_read_current_limit(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![1, 2, 3, 4, 5, 6],
        )?;

        Ok(l.try_into().unwrap())
    }

    pub fn set_dxl_goal_current(&mut self, goal: i16) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_goal_current(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![1, 2, 3, 4, 5, 6],
            &[goal, goal, goal, goal, goal, goal],
        )?;

        Ok(())
    }

    pub fn get_dxl_goal_current(&mut self) -> Result<[i16; 6], Box<dyn std::error::Error>> {
        let l = xl330::sync_read_goal_current(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![1, 2, 3, 4, 5, 6],
        )?;

        Ok(l.try_into().unwrap())
    }

    pub fn read_all_positions(&mut self) -> Result<[f64; 9], Box<dyn std::error::Error>> {
        let mut pos = Vec::new();

        pos.extend(sts3215::sync_read_present_position(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &vec![11, 21, 22],
        )?);
        pos.extend(xl330::sync_read_present_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![1, 2, 3, 4, 5, 6],
        )?);

        Ok(pos.try_into().unwrap())
    }

    pub fn set_all_goal_positions(
        &mut self,
        positions: [f64; 9],
    ) -> Result<(), Box<dyn std::error::Error>> {
        sts3215::sync_write_goal_position(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &vec![11, 21, 22],
            &positions[0..3],
        )?;
        xl330::sync_write_goal_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![1, 2, 3, 4, 5, 6],
            &positions[3..9],
        )?;

        Ok(())
    }

    pub fn set_antennas_positions(
        &mut self,
        positions: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        sts3215::sync_write_goal_position(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &vec![21, 22],
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
            &[1, 2, 3, 4, 5, 6],
            &position,
        )?;

        Ok(())
    }
    pub fn set_body_rotation(&mut self, position: f64) -> Result<(), Box<dyn std::error::Error>> {
        sts3215::sync_write_goal_position(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[11],
            &[position],
        )?;

        Ok(())
    }

    pub fn enable_torque(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque(true)
    }
    pub fn disable_torque(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque(false)
    }

    fn set_torque(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        sts3215::sync_write_torque_enable(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[11, 21, 22],
            &[enable; 3],
        )?;
        xl330::sync_write_torque_enable(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &[1, 2, 3, 4, 5, 6],
            &[enable; 6],
        )?;

        Ok(())
    }
}
