use std::time::Duration;
use rustypot::servo::{dynamixel::xl330, feetech::sts3215};

// Constants for motor IDs
const BODY_ROTATION_ID: u8 = 11;
const ANTENNA_LEFT_ID: u8 = 21;
const ANTENNA_RIGHT_ID: u8 = 22;
const STEWART_PLATFORM_IDS: [u8; 6] = [1, 2, 3, 4, 5, 6];

// Serial configuration
const DEFAULT_BAUD_RATE: u32 = 1_000_000;
const DEFAULT_TIMEOUT_MS: u64 = 10;

// Motor limits
const POSITION_MIN: f64 = -180.0;
const POSITION_MAX: f64 = 180.0;
const MAX_RETRIES: u8 = 3;
const RETRY_DELAY_MS: u64 = 5;

/// Motor controller for Reachy Mini
/// 
/// This struct is NOT thread-safe. For concurrent access, wrap in a Mutex.
/// The Python bindings handle this automatically.
pub struct ReachyMiniMotorController {
    dph_v1: rustypot::DynamixelProtocolHandler,
    dph_v2: rustypot::DynamixelProtocolHandler,
    serial_port: Box<dyn serialport::SerialPort>,
}

impl ReachyMiniMotorController {
    pub fn new(serialport: &str) -> Result<Self, Box<dyn std::error::Error>> {
        let dph_v1 = rustypot::DynamixelProtocolHandler::v1();
        let dph_v2 = rustypot::DynamixelProtocolHandler::v2();
        let serial_port = serialport::new(serialport, DEFAULT_BAUD_RATE)
            .timeout(Duration::from_millis(DEFAULT_TIMEOUT_MS))
            .open()?;
        
        let mut controller = Self {
            dph_v1,
            dph_v2,
            serial_port,
        };
        
        // Ping all motors on startup
        controller.ping_all_motors()?;
        
        Ok(controller)
    }
    
    /// Ping all motors to ensure they're responsive
    pub fn ping_all_motors(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        // Check STS3215 motors
        for &id in &[BODY_ROTATION_ID, ANTENNA_LEFT_ID, ANTENNA_RIGHT_ID] {
            let mut retries = 0;
            loop {
                match sts3215::ping(&self.dph_v1, self.serial_port.as_mut(), id) {
                    Ok(_) => break,
                    Err(_) if retries < MAX_RETRIES => {
                        retries += 1;
                        std::thread::sleep(Duration::from_millis(RETRY_DELAY_MS));
                    }
                    Err(e) => {
                        return Err(format!("Motor {} not responding after {} retries: {}", id, retries, e).into());
                    }
                }
            }
        }
        
        // Check XL330 motors
        for &id in &STEWART_PLATFORM_IDS {
            let mut retries = 0;
            loop {
                match xl330::ping(&self.dph_v2, self.serial_port.as_mut(), id) {
                    Ok(_) => break,
                    Err(_) if retries < MAX_RETRIES => {
                        retries += 1;
                        std::thread::sleep(Duration::from_millis(RETRY_DELAY_MS));
                    }
                    Err(e) => {
                        return Err(format!("Motor {} not responding after {} retries: {}", id, retries, e).into());
                    }
                }
            }
        }
        
        Ok(())
    }
    
    /// Validate that positions are within acceptable bounds
    fn validate_positions(positions: &[f64]) -> Result<(), Box<dyn std::error::Error>> {
        for (i, &pos) in positions.iter().enumerate() {
            if pos < POSITION_MIN || pos > POSITION_MAX {
                return Err(format!(
                    "Position {} out of bounds: {} (must be between {} and {})",
                    i, pos, POSITION_MIN, POSITION_MAX
                ).into());
            }
        }
        Ok(())
    }
    
    /// Read current positions of all motors
    /// Returns positions in order: [body_rotation, antenna_left, antenna_right, stewart_1..6]
    pub fn read_all_positions(&mut self) -> Result<[f64; 9], Box<dyn std::error::Error>> {
        let mut pos = Vec::new();
        
        pos.extend(sts3215::sync_read_present_position(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[BODY_ROTATION_ID, ANTENNA_LEFT_ID, ANTENNA_RIGHT_ID],
        )?);
        
        pos.extend(xl330::sync_read_present_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
        )?);
        
        pos.try_into()
            .map_err(|v: Vec<f64>| format!("Expected 9 positions, got {}", v.len()).into())
    }
    
    /// Set goal positions for all motors
    /// positions: [body_rotation, antenna_left, antenna_right, stewart_1..6]
    pub fn set_all_goal_positions(
        &mut self,
        positions: [f64; 9],
    ) -> Result<(), Box<dyn std::error::Error>> {
        Self::validate_positions(&positions)?;
        
        sts3215::sync_write_goal_position(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[BODY_ROTATION_ID, ANTENNA_LEFT_ID, ANTENNA_RIGHT_ID],
            &positions[0..3],
        )?;
        
        xl330::sync_write_goal_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
            &positions[3..9],
        )?;
        
        Ok(())
    }
    
    /// Set goal positions for antenna motors
    pub fn set_antennas_positions(
        &mut self,
        positions: [f64; 2],
    ) -> Result<(), Box<dyn std::error::Error>> {
        Self::validate_positions(&positions)?;
        
        sts3215::sync_write_goal_position(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[ANTENNA_LEFT_ID, ANTENNA_RIGHT_ID],
            &positions,
        )?;
        Ok(())
    }
    
    /// Set goal positions for Stewart platform motors
    pub fn set_stewart_platform_position(
        &mut self,
        position: [f64; 6],
    ) -> Result<(), Box<dyn std::error::Error>> {
        Self::validate_positions(&position)?;
        
        xl330::sync_write_goal_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
            &position,
        )?;
        Ok(())
    }
    
    /// Set body rotation position
    pub fn set_body_rotation(&mut self, position: f64) -> Result<(), Box<dyn std::error::Error>> {
        Self::validate_positions(&[position])?;
        
        sts3215::sync_write_goal_position(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[BODY_ROTATION_ID],
            &[position],
        )?;
        Ok(())
    }
    
    /// Enable torque on all motors
    pub fn enable_torque(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque(true)
    }
    
    /// Disable torque on all motors
    pub fn disable_torque(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.set_torque(false)
    }
    
    fn set_torque(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        sts3215::sync_write_torque_enable(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[BODY_ROTATION_ID, ANTENNA_LEFT_ID, ANTENNA_RIGHT_ID],
            &[enable; 3],
        )?;
        
        xl330::sync_write_torque_enable(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
            &[enable; 6],
        )?;
        
        Ok(())
    }
    
    /// Set goal current for Stewart platform motors
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
    
    // Additional methods for API symmetry
    
    /// Read only body rotation position
    pub fn read_body_rotation(&mut self) -> Result<f64, Box<dyn std::error::Error>> {
        sts3215::read_present_position(&self.dph_v1, self.serial_port.as_mut(), BODY_ROTATION_ID)
            .map_err(|e| e.into())
    }
    
    /// Read only antenna positions
    pub fn read_antenna_positions(&mut self) -> Result<[f64; 2], Box<dyn std::error::Error>> {
        let positions = sts3215::sync_read_present_position(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[ANTENNA_LEFT_ID, ANTENNA_RIGHT_ID],
        )?;
        
        positions.try_into()
            .map_err(|v: Vec<f64>| format!("Expected 2 antenna positions, got {}", v.len()).into())
    }
    
    /// Read Stewart platform positions
    pub fn read_stewart_platform_positions(&mut self) -> Result<[f64; 6], Box<dyn std::error::Error>> {
        let positions = xl330::sync_read_present_position(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &STEWART_PLATFORM_IDS,
        )?;
        
        positions.try_into()
            .map_err(|v: Vec<f64>| format!("Expected 6 stewart positions, got {}", v.len()).into())
      
    pub fn read_stewart_platform_current(
        &mut self,
    ) -> Result<[i16; 6], Box<dyn std::error::Error>> {
        let currents = xl330::sync_read_present_current(
            &self.dph_v2,
            self.serial_port.as_mut(),
            &[1, 2, 3, 4, 5, 6],
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
            &[1, 2, 3, 4, 5, 6],
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
            &[1, 2, 3, 4, 5, 6],
        )?;

        Ok(modes.try_into().unwrap())
    }

    pub fn set_antennas_operating_mode(
        &mut self,
        mode: u8,
    ) -> Result<(), Box<dyn std::error::Error>> {
        sts3215::sync_write_mode(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[21, 22],
            &[mode; 2],
        )?;

        Ok(())
    }

    pub fn set_body_rotation_operating_mode(
        &mut self,
        mode: u8,
    ) -> Result<(), Box<dyn std::error::Error>> {
        sts3215::sync_write_mode(&self.dph_v1, self.serial_port.as_mut(), &[11], &[mode])?;

        Ok(())
    }

    pub fn enable_body_rotation(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        sts3215::sync_write_torque_enable(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[11],
            &[enable],
        )?;

        Ok(())
    }

    pub fn enable_antennas(&mut self, enable: bool) -> Result<(), Box<dyn std::error::Error>> {
        sts3215::sync_write_torque_enable(
            &self.dph_v1,
            self.serial_port.as_mut(),
            &[21, 22],
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
            &[1, 2, 3, 4, 5, 6],
            &[enable; 6],
        )?;

        Ok(())
 main
    }
}
