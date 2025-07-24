use std::f64::consts::PI;
use std::time::Duration;
use std::io::{self, Write};
use reachy_mini_motor_controller::ReachyMiniMotorController;

const DEFAULT_PORTS: &[&str] = &[
    "/dev/tty.usbmodem58FA0959031",
    "/dev/ttyACM0", 
    "/dev/ttyUSB0",
];

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    
    // Try to find a working serial port
    let serial_port = std::env::args()
        .nth(1)
        .or_else(|| {
            DEFAULT_PORTS.iter()
                .find(|port| {
                    println!("Trying port {}...", port);
                    std::path::Path::new(port).exists()
                })
                .map(|s| s.to_string())
        })
        .ok_or("No serial port found. Please specify one as argument: sin /dev/ttyUSB0")?;
    
    println!("Connected to {}", serial_port);
    let mut c = ReachyMiniMotorController::new(&serial_port)?;
    
    c.enable_torque()?;
    println!("Running sine wave motion. Press Ctrl+C to stop.");
    
    // Set up Ctrl+C handler
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })?;
    
    let t0 = std::time::Instant::now();
    let amp = 30.0_f64.to_radians();
    let freq = 0.25;
    
    while running.load(std::sync::atomic::Ordering::SeqCst) {
        let t = t0.elapsed().as_secs_f64();
        let pos = (2.0 * PI * freq * t).sin() * amp;
        
        c.set_all_goal_positions([pos; 9])?;
        
        let cur = c.read_all_positions()?;
        let errors: Vec<f64> = cur
            .iter()
            .map(|&cur_pos| (cur_pos - pos).abs())
            .collect();
        
        let max_error = errors.iter().cloned().fold(0.0, f64::max);
        let mean_error = errors.iter().sum::<f64>() / errors.len() as f64;
        
        print!("\rTime: {:6.2}s | Goal: {:6.1}° | Max Error: {:5.2}° | Mean Error: {:5.2}°",
               t, pos.to_degrees(), max_error.to_degrees(), mean_error.to_degrees());
        io::stdout().flush()?;
        
        std::thread::sleep(Duration::from_millis(10));
    }
    
    println!("\n\nShutting down gracefully...");
    c.disable_torque()?;
    println!("Motors disabled. Goodbye!");
    
    Ok(())
}
