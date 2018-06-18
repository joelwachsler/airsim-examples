extern crate mavlink;

use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;
use std::f32;
use mavlink::common::{HIL_GPS_DATA, HIL_SENSOR_DATA};

mod pid;
use self::pid::PID;
use drone::Drone;

const PID_CALC_HZ: u32 = 10;

fn wait_for_sensor_values(
    sensor: &Arc<Mutex<HIL_SENSOR_DATA>>,
    gps: &Arc<Mutex<HIL_GPS_DATA>>,
) {
    let mut sensor_data = sensor.lock().unwrap().clone();
    let mut gps_data = gps.lock().unwrap().clone();

    while sensor_data.xacc == 0.0 && sensor_data.yacc == 0.0 && sensor_data.zacc == 0.0 || gps_data.alt == 0 {
        println!("Waiting for sensor data...");

        thread::sleep(Duration::from_secs(1) / 10);

        sensor_data = sensor.lock().unwrap().clone();
        gps_data = gps.lock().unwrap().clone();
    }

    println!("Got sensor data!");
}

/// Calculates the drone pointing using the provided magnometer values.
fn calculate_drone_pointing(xmag: f32, ymag: f32) -> f32 {
    ymag.atan2(xmag)
}

fn to_rad(deg: f64) -> f64 {
    deg * 0.0174533
}

fn to_deg(rad: f64) -> f64 {
    rad * 57.2958
}

fn calc_angle_pitch_roll_yaw(sensors: &HIL_SENSOR_DATA, angle_pitch: &mut f64, angle_roll: &mut f64, angle_yaw: &mut f64, dt: f64) {
    *angle_pitch += sensors.ygyro as f64 * dt;
    *angle_roll += sensors.xgyro as f64 * dt;
    *angle_yaw += sensors.zgyro as f64 * dt;

    *angle_roll -= *angle_pitch * (sensors.zgyro as f64 * dt).sin();
    *angle_pitch += *angle_roll * (sensors.zgyro as f64 * dt).sin();

    let acc_total_vector = ((sensors.xacc * sensors.xacc) + (sensors.yacc * sensors.yacc) + (sensors.zacc * sensors.zacc)).sqrt();
    let angle_pitch_acc = (sensors.xacc / acc_total_vector).asin();
    let angle_roll_acc = (sensors.yacc / acc_total_vector).asin();
    // TODO: may not be correct
    let angle_yaw_acc = (sensors.zacc / acc_total_vector).asin();

    // Using a complementary filter to reduce drag
    // TODO: maybe update the values taken from each sensor?
    *angle_pitch = *angle_pitch * 0.9996 + angle_pitch_acc as f64 * 0.0004;
    *angle_roll = *angle_roll * 0.9996 + angle_roll_acc as f64 * 0.0004;
    *angle_yaw = *angle_yaw * 0.9996 + angle_yaw_acc as f64 * 0.0004;
}

pub fn calculate_pids(
    drone: Arc<Drone>,
    sensor: Arc<Mutex<HIL_SENSOR_DATA>>,
    gps: Arc<Mutex<HIL_GPS_DATA>>,
) {
    thread::spawn(move || {
        const P_GAIN_RP: f64 = 0.1125;
        const I_GAIN_RP: f64 = 0.001;
        const D_GAIN_RP: f64 = 0.8;
        const MAX_RP: f64 = 0.2;

        let mut pitch = PID::new(P_GAIN_RP, I_GAIN_RP, D_GAIN_RP, MAX_RP);
        let mut roll = PID::new(P_GAIN_RP, I_GAIN_RP, D_GAIN_RP, MAX_RP);
        let mut yaw = PID::new(5.0, 0.1, 35.0, 0.2);

        const ALTITUDE_SCALE: f64 = 10_000.0;
        let mut altitude = PID::new(2.0, 0.0005525, 25.0, 0.2 * ALTITUDE_SCALE);

        const LAT_LON_P: f64 = 1000.0;
        const LAT_LON_I: f64 = 0.0;
        const LAT_LON_D: f64 = 75_000.0;
        const LAT_LON_MAX: f64 = 0.1;
        
        let mut lat_pid = PID::new(LAT_LON_P, LAT_LON_I, LAT_LON_D, LAT_LON_MAX);
        let mut lon_pid = PID::new(LAT_LON_P, LAT_LON_I, LAT_LON_D, LAT_LON_MAX);

        wait_for_sensor_values(&sensor, &gps);

        {
            let gps_data = gps.lock().unwrap().clone();
            drone.set_ground_alt(gps_data.alt);

            let lat = gps_data.lat as f64 / 1E7;
            let lon = gps_data.lon as f64 / 1E7;

            drone.set_home_coordinates(lat, lon);
            drone.set_current_coordinates(lat, lon);
            drone.set_target_coordinates(lat, lon);
        }

        let mut angle_pitch: f64 = 0.0;
        let mut angle_roll: f64 = 0.0;
        let mut angle_yaw: f64 = 0.0;

        // TODO: find a way to use the constant instead
        const DT: f32 = 1.0 / PID_CALC_HZ as f32;

        loop {
            while !drone.is_armed() {
                thread::sleep(Duration::from_secs(1) / 10);
            }

            let gps_data = gps.lock().unwrap().clone();
            let sensor_data = sensor.lock().unwrap().clone();

            let alt = gps_data.alt - drone.get_ground_alt();
            drone.set_current_alt(alt);
            // Update target altitude setpoint
            altitude.setpoint = drone.get_target_alt() as f64;

            let lat = gps_data.lat as f64 / 1E7;
            let lon = gps_data.lon as f64 / 1E7;
            drone.set_current_coordinates(lat, lon);

            yaw.setpoint = calculate_drone_pointing(sensor_data.xmag, sensor_data.ymag) as f64;

            calc_angle_pitch_roll_yaw(&sensor_data, &mut angle_pitch, &mut angle_roll, &mut angle_yaw, DT as f64);

            let (target_lat, target_lon) = drone.get_target_coordinates();

            lat_pid.setpoint = target_lat;
            lon_pid.setpoint = target_lon;

            pitch.setpoint = lat_pid.calculate_pid(lat);
            roll.setpoint = lon_pid.calculate_pid(lon) * -1.0;

            let pid_output_pitch = pitch.calculate_pid(angle_pitch);
            let pid_output_roll = roll.calculate_pid(angle_roll);
            let pid_output_yaw = yaw.calculate_pid(angle_yaw);
            let mut pid_output_altitude_throttle = altitude.calculate_pid(alt as f64);

            // Throttle normalization
            pid_output_altitude_throttle *= -1.0;
            pid_output_altitude_throttle /= ALTITUDE_SCALE;
            pid_output_altitude_throttle += 0.48;

            // Update the motor state
            let mut new_motor_state: [f32; 4] = [0.0; 4];

            const NE_MOTOR: usize = 0; // CCW
            const SW_MOTOR: usize = 1; // CCW
            const NW_MOTOR: usize = 2; // CW
            const SE_MOTOR: usize = 3; // CW

            new_motor_state[NE_MOTOR] = (pid_output_altitude_throttle - pid_output_pitch + pid_output_roll - pid_output_yaw) as f32; // CCW
            new_motor_state[SE_MOTOR] = (pid_output_altitude_throttle + pid_output_pitch + pid_output_roll + pid_output_yaw) as f32; // CW
            new_motor_state[SW_MOTOR] = (pid_output_altitude_throttle + pid_output_pitch - pid_output_roll - pid_output_yaw) as f32; // CCW
            new_motor_state[NW_MOTOR] = (pid_output_altitude_throttle - pid_output_pitch - pid_output_roll + pid_output_yaw) as f32; // CW

            drone.set_motor_state(new_motor_state);

            thread::sleep(Duration::from_secs(1) / PID_CALC_HZ);
        }
    });
}
