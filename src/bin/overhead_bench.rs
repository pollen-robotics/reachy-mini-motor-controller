use std::time::Duration;

use reachy_mini_motor_controller::{ReachyMiniMotorController, control_loop::read_pos};
use rustypot::servo::{dynamixel::xl330, feetech::sts3215};

const N: usize = 1000;

fn main() {
    let serialport = "/dev/ttyACM0";

    {
        let mut c = ReachyMiniMotorController::new(serialport).unwrap();

        let tic = std::time::Instant::now();
        for _ in 0..N {
            let _ = read_pos(&mut c);
        }
        let elapsed = tic.elapsed();
        println!("Full loop read elapsed time: {:?}", elapsed);

        let tic = std::time::Instant::now();
        for _ in 0..N {
            let _ = c.read_all_positions();
        }
        let elapsed = tic.elapsed();
        println!("Controller read all positions elapsed time: {:?}", elapsed);
    }
    {
        let dph_v1 = rustypot::DynamixelProtocolHandler::v1();
        let dph_v2 = rustypot::DynamixelProtocolHandler::v2();

        let mut serial_port = serialport::new(serialport, 1_000_000)
            .timeout(Duration::from_millis(10))
            .open()
            .unwrap();

        let tic = std::time::Instant::now();
        for _ in 0..N {
            let _ =
                sts3215::sync_read_present_position(&dph_v1, serial_port.as_mut(), &[11, 21, 22]);
            let _ = xl330::sync_read_present_position(
                &dph_v2,
                serial_port.as_mut(),
                &[1, 2, 3, 4, 5, 6],
            );
        }
        let elapsed = tic.elapsed();
        println!("Dynamixel read elapsed time: {:?}", elapsed);

        {
            let sts_msg = [255, 255, 254, 7, 130, 56, 2, 11, 21, 22, 8];
            let xl_msg = [
                255, 255, 253, 0, 254, 13, 0, 130, 132, 0, 4, 0, 1, 2, 3, 4, 5, 6, 178, 155,
            ];
            let mut sts_buf = [0u8; 8];
            let mut xl_buf = [0u8; 15];

            let tic = std::time::Instant::now();
            for _ in 0..N {
                serial_port.write_all(&sts_msg).unwrap();
                for _ in 0..3 {
                    serial_port.read_exact(&mut sts_buf).unwrap();
                }
                serial_port.write_all(&xl_msg).unwrap();
                for _ in 0..6 {
                    serial_port.read_exact(&mut xl_buf).unwrap();
                }
            }
            let elapsed = tic.elapsed();
            println!("Serial port read elapsed time: {:?}", elapsed);

            let mut sts_buf = [0u8; 8 * 3];
            let mut xl_buf = [0u8; 15 * 6];
            let tic = std::time::Instant::now();
            for _ in 0..N {
                serial_port.write_all(&sts_msg).unwrap();
                serial_port.read_exact(&mut sts_buf).unwrap();
                serial_port.write_all(&xl_msg).unwrap();
                serial_port.read_exact(&mut xl_buf).unwrap();
            }
            let elapsed = tic.elapsed();
            println!("Raw raw serial port read elapsed time: {:?}", elapsed);
        }
    }
}
