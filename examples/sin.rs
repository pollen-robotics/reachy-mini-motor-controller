use std::f64::consts::PI;

use reachy_mini_motor_controller::ReachyMiniMotorController;

fn main() {
    let serialport = "/dev/tty.usbmodem58FA0959031";
    let mut c = ReachyMiniMotorController::new(serialport).unwrap();

    c.enable_torque().unwrap();

    let t0 = std::time::Instant::now();

    let amp = 30.0_f64.to_radians();
    let freq = 0.25;

    loop {
        let t = t0.elapsed().as_secs_f64();
        let pos = (2.0 * PI * freq * t).sin() * amp;

        c.set_all_goal_positions([pos; 9]).unwrap();

        let cur = c.read_all_positions().unwrap();

        let errors = cur
            .iter()
            .zip([pos; 9].iter())
            .map(|(cur, goal)| (cur - goal).abs())
            .collect::<Vec<_>>();
        println!("Errors: {:?}", errors);
    }
}