use std::{thread::sleep, time::Duration, io::{self, Write}};
use reachy_mini_motor_controller::ReachyMiniMotorController;

const DEFAULT_PORTS: &[&str] = &[
    "/dev/ttyACM0",
    "/dev/tty.usbmodem5A460830791",
    "/dev/ttyUSB0",
];

struct TestPositions {
    lower: [f64; 9],
    upper: [f64; 9],
}

impl Default for TestPositions {
    fn default() -> Self {
        Self {
            lower: [
                0.0015339807878858025,
                3.0434178831651124,
                -3.043417883165112,
                -0.8620972027917304,
                0.7608544707912781,
                -0.3758252930319821,
                0.3221359654559848,
                -0.7470486437003072,
                0.7930680673368764,
            ],
            upper: [
                -0.004601942363656963,
                3.0434178831651124,
                -3.043417883165112,
                0.44792239006260726,
                -0.44945637085049306,
                0.5829126993965437,
                -0.5062136600022615,
                0.37889325460775325,
                -0.4862719097597483,
            ],
        }
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    
    // Try to find a working serial port
    let serial_port = std::env::args()
        .nth(1)
        .or_else(|| {
            DEFAULT_PORTS.iter()
                .find(|port| std::path::Path::new(port).exists())
                .map(|s| s.to_string())
        })
        .ok_or("No serial port found. Please specify one as argument: debug /dev/ttyUSB0")?;
    
    println!("Using serial port: {}", serial_port);
    let mut c = ReachyMiniMotorController::new(&serial_port)?;
    
    let positions = TestPositions::default();
    
    c.enable_torque()?;
    println!("Motors enabled. Press Ctrl+C to stop.");
    
    // Set up Ctrl+C handler
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })?;
    
    let mut cycle = 0;
    while running.load(std::sync::atomic::Ordering::SeqCst) {
        print!("\rCycle {}: moving to lower position... ", cycle);
        io::stdout().flush()?;
        c.set_all_goal_positions(positions.lower)?;
        sleep(Duration::from_millis(1000));
        
        print!("\rCycle {}: moving to upper position... ", cycle);
        io::stdout().flush()?;
        c.set_all_goal_positions(positions.upper)?;
        sleep(Duration::from_millis(1000));
        
        cycle += 1;
    }
    
    println!("\n\nShutting down gracefully...");
    c.disable_torque()?;
    println!("Motors disabled. Goodbye!");
    
    Ok(())
}
