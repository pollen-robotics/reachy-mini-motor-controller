use std::time::Duration;

use rustypot::servo::dynamixel::xl330;

pub struct ReachyMiniMotorController {
    dph_v2: rustypot::DynamixelProtocolHandler,
    serial_port: Box<dyn serialport::SerialPort>,
}

impl ReachyMiniMotorController {
    pub fn new(serialport: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let dph_v2 = rustypot::DynamixelProtocolHandler::v2();

        let serial_port = serialport::new(serialport, 1_000_000)
            .timeout(Duration::from_millis(10))
            .open()?;

        Ok(Self {
            dph_v2,
            serial_port,
        })
    }

    pub fn read_all_positions(&mut self) -> Result<[f64; 9], Box<dyn std::error::Error>> {
        let mut pos = Vec::new();

        pos.extend(xl330::sync_read_present_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![1, 2, 3, 4, 5, 6, 11, 21, 22],
        )?);

        Ok(pos.try_into().unwrap())
    }

    pub fn set_all_goal_positions(
        &mut self,
        positions: [f64; 9],
    ) -> Result<(), Box<dyn std::error::Error>> {
        xl330::sync_write_goal_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &vec![11, 21, 22, 1, 2, 3, 4, 5, 6],
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
        xl330::sync_write_goal_position(
            &self.dph_v2,
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
        xl330::sync_write_torque_enable(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &[1, 2, 3, 4, 5, 6, 11, 21, 22],
            &[enable; 9],
        )?;

        Ok(())
    }
}
