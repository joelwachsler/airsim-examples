extern crate mavlink;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use mavlink::common;

mod mavlink_msg_handler;
mod calculate_pids;
mod cmd_handler;

mod drone;
use drone::Drone;

fn main() {
    let vehicle_connection = Arc::new(mavlink::connect("udpin:127.0.0.1:14560").unwrap());

    let gps = Arc::new(Mutex::new(common::HIL_GPS_DATA {
        time_usec: 0,
        lat: 0,
        lon: 0,
        alt: 0,
        eph: 0,
        epv: 0,
        vel: 0,
        vn: 0,
        ve: 0,
        vd: 0,
        cog: 0,
        fix_type: 0,
        satellites_visible: 0,
    }));

    let sensor = Arc::new(Mutex::new(common::HIL_SENSOR_DATA {
        time_usec: 0,
        xacc: 0.0,
        yacc: 0.0,
        zacc: 0.0,
        xgyro: 0.0,
        ygyro: 0.0,
        zgyro: 0.0,
        xmag: 0.0,
        ymag: 0.0,
        zmag: 0.0,
        abs_pressure: 0.0,
        diff_pressure: 0.0,
        pressure_alt: 0.0,
        temperature: 0.0,
        fields_updated: 0,
    }));

    mavlink_msg_handler::send_heartbeat_messages(vehicle_connection.clone());
    mavlink_msg_handler::msg_listener(vehicle_connection.clone(), gps.clone(), sensor.clone());

    let drone = Arc::new(Drone::new());

    mavlink_msg_handler::send_motor_state(vehicle_connection.clone(), drone.clone());
    calculate_pids::calculate_pids(drone.clone(), sensor.clone(), gps.clone());

    cmd_handler::start_cmd_handler(drone.clone());

    thread::sleep(Duration::from_secs(100))
}

