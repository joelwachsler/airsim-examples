extern crate mavlink;
use std::sync::Arc;
use std::thread;
use std::time::Duration;

use mavlink::common;
use mavlink::common::MavMessage;

/**
 * Creates a new thread which sends a heartbeat every second.
 */
fn start_heartbeats(
    vehicle: Arc<Box<mavlink::MavConnection + std::marker::Send + std::marker::Sync>>,
) {
    thread::spawn(move || {
        let heartbeat_msg = MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
            mavtype: 2,     // MAV_TYPE_QUADROTOR
            autopilot: 8,   // MAV_AUTOPILOT_INVALID
            base_mode: 128, // MAV_MODE_FLAG_SAFETY_ARMED
            custom_mode: 0,
            system_status: 0,
            mavlink_version: 0x3,
        });

        loop {
            println!("Sending a heartbeat!");
            vehicle.send(&heartbeat_msg).ok();

            thread::sleep(Duration::from_secs(1));
        }
    });
}

/**
 * Updates the pvm of the four motor controls.
 */
fn update_motor_pwm(
    motor_pwm: [f32; 4],
    vehicle: &Arc<Box<mavlink::MavConnection + std::marker::Send + std::marker::Sync>>,
) {
    let controls: Vec<f32> = vec![
        motor_pwm[0],
        motor_pwm[1],
        motor_pwm[2],
        motor_pwm[3],
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
        -1.0,
    ];

    let control_msg = MavMessage::HIL_ACTUATOR_CONTROLS(common::HIL_ACTUATOR_CONTROLS_DATA {
        time_usec: 0,
        flags: 0,
        controls,
        mode: 0,
    });

    vehicle.send(&control_msg).ok();
}

/**
 * Blocks until a HIL_GPS message is received.
 */
fn get_gps_msg(
    vehicle: &Arc<Box<mavlink::MavConnection + std::marker::Send + std::marker::Sync>>,
) -> common::HIL_GPS_DATA {
    loop {
        match vehicle.recv().unwrap() {
            MavMessage::HIL_GPS(gps_data) => return gps_data,
            _ => continue,
        }
    }
}

fn main() {
    // Create a thread-safe reference-counting pointer which is destroyed when
    // all references to it is destroyed.
    let vehicle = Arc::new(mavlink::connect("udpin:127.0.0.1:14560").unwrap());

    start_heartbeats(vehicle.clone());

    // Let's initialize the ground variable
    let ground: i32 = get_gps_msg(&vehicle).alt;
    // Altitude, in meters * 1000 (positive for up)
    let target_alt = 2500;

    // Take off and tries to get an altitude of 2.5m
    loop {
        // Blocking until a MAVLink message is received
        let height = get_gps_msg(&vehicle).alt;
        let diff = height - ground;

        // Variable for holding the current motor pwm.
        let pwm: f32;

        // Print formatting: 2500 -> 2.50
        println!("Current height: {:.2}m", diff as f32 / 1000.0);

        if diff > target_alt {
            pwm = 0.48;
        } else {
            pwm = 0.50;
        }

        println!("Updating the pwm to {:.2}", pwm);
        update_motor_pwm([pwm; 4], &vehicle);
    }
}
