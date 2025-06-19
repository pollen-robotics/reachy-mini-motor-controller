use std::{thread::sleep, time::Duration};

use reachy_mini_motor_controller::ReachyMiniMotorController;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // let serial_port = "/dev/ttyACM0"; // Adjust this to your serial port
    let serial_port = "/dev/tty.usbmodem5A460830791";
    let mut c = ReachyMiniMotorController::new(serial_port)?;

    let lower = [
        0.0015339807878858025,
        3.0434178831651124,
        -3.043417883165112,
        -0.8620972027917304,
        0.7608544707912781,
        -0.3758252930319821,
        0.3221359654559848,
        -0.7470486437003072,
        0.7930680673368764,
    ];
    let upper = [
        -0.004601942363656963,
        3.0434178831651124,
        -3.043417883165112,
        0.44792239006260726,
        -0.44945637085049306,
        0.5829126993965437,
        -0.5062136600022615,
        0.37889325460775325,
        -0.4862719097597483,
    ];

    c.enable_torque()?;

    loop {
        c.set_all_goal_positions(lower)?;
        sleep(Duration::from_millis(1000));
        c.set_all_goal_positions(upper)?;
        sleep(Duration::from_millis(1000));
    }
}
