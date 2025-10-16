use std::{f64::consts::PI, thread::sleep, time::Duration};

// Create args struct
use clap::Parser;
use reachy_mini_motor_controller::control_loop::{
    FullBodyPosition, MotorCommand, ReachyMiniControlLoop,
};

#[derive(Parser, Debug)]
#[clap(author, version, about, long_about = None)]
struct Args {
    /// Serial port to use
    #[clap(short, long)]
    port: String,

    /// Frequency for reading positions
    #[clap(short, long)]
    read_frequency: f64,

    #[clap(short, long)]
    write_frequency: f64,
}

fn main() {
    let args = Args::parse();

    let loop_controller = ReachyMiniControlLoop::new(
        args.port,
        Duration::from_secs_f64(1.0 / args.read_frequency),
        Some(Duration::from_secs(1)),
        5,
        Duration::from_secs(30),
    )
    .unwrap();

    let write_period = Duration::from_secs_f64(1.0 / args.write_frequency);

    let mut last_stats_tick = std::time::Instant::now();
    let sin_t0 = std::time::Instant::now();

    let f = 0.25;
    let amp = 10.0_f64.to_radians();

    loop_controller
        .push_command(MotorCommand::EnableTorque())
        .unwrap();

    loop {
        let t = sin_t0.elapsed().as_secs_f64();
        let target_pos = (2.0 * PI * f * t).sin() * amp;

        loop_controller
            .push_command(MotorCommand::SetAllGoalPositions {
                positions: FullBodyPosition {
                    body_yaw: target_pos,
                    antennas: [target_pos; 2],
                    stewart: [target_pos; 6],
                    timestamp: 0.0,
                },
            })
            .unwrap();

        if last_stats_tick.elapsed() > Duration::from_secs(1)
            && let Ok(Some(stats)) = loop_controller.get_stats()
        {
            println!("{:?}", stats);
            last_stats_tick = std::time::Instant::now();
        }

        sleep(write_period);
    }
}
