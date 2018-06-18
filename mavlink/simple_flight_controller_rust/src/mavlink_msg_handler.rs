extern crate mavlink;

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use std::mem::replace;

use std::marker;

use mavlink::common;
use mavlink::common::MavMessage;

use drone::Drone;

const MOTOR_SEND_HZ: u32 = 100;

/**
 * Creates a new thread which sends a heartbeat every second.
 */
pub fn send_heartbeat_messages(
    vehicle_connection: Arc<Box<mavlink::MavConnection + marker::Send + marker::Sync>>
) {
    println!("Heartbeat sender initialized!");
    thread::spawn(move || {
        let heartbeat_msg = MavMessage::HEARTBEAT(common::HEARTBEAT_DATA {
            mavtype: 2,     // MAV_TYPE_QUADROTOR
            autopilot: 12,  // MAV_AUTOPILOT_PX4
            base_mode: 128, // MAV_MODE_FLAG_SAFETY_ARMED
            custom_mode: 0,
            system_status: 4,
            mavlink_version: 0x3,
        });

        loop {
            vehicle_connection.send(&heartbeat_msg).ok();
            thread::sleep(Duration::from_secs(1));
        }
    });
}

pub fn msg_listener(
    vehicle_connection: Arc<Box<mavlink::MavConnection + marker::Send + marker::Sync>>,
    gps: Arc<Mutex<common::HIL_GPS_DATA>>,
    sensor: Arc<Mutex<common::HIL_SENSOR_DATA>>
) {
    println!("Message listener started\n");

    thread::spawn(move || {
        loop {
            match vehicle_connection.recv().unwrap() {
                MavMessage::HIL_GPS(gps_data) => {
                    let mut data = gps.lock().unwrap();
                    replace(&mut *data, gps_data);
                },
                MavMessage::HIL_SENSOR(sensor_data) => {
                    let mut data = sensor.lock().unwrap();
                    replace(&mut *data, sensor_data);
                },
                _ => continue
            }
        }
    });
}

pub fn send_motor_state(
    vehicle_connection: Arc<Box<mavlink::MavConnection + marker::Send + marker::Sync>>,
    drone: Arc<Drone>
) {
    thread::spawn(move || {
        println!("Motor state sender started");
        loop {
            let motors = drone.get_motor_state();

            let controls: Vec<f32> = vec![
                motors[0],
                motors[1],
                motors[2],
                motors[3],
                -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0, -1.0 
            ];

            let message = MavMessage::HIL_ACTUATOR_CONTROLS(common::HIL_ACTUATOR_CONTROLS_DATA {
                time_usec: 0,
                flags: 0,
                controls,
                mode: 0,
            });

            vehicle_connection.send(&message).ok();

            thread::sleep(Duration::from_secs(1) / MOTOR_SEND_HZ);
        }
    });
}
