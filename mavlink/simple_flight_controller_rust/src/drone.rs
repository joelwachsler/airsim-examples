use std::sync::Mutex;

pub struct Drone {
    motors: Mutex<[f32; 4]>,
    armed: Mutex<bool>,
    current_lat: Mutex<f64>,
    current_lon: Mutex<f64>,
    home_lat: Mutex<f64>,
    home_lon: Mutex<f64>,
    target_lat: Mutex<f64>,
    target_lon: Mutex<f64>,
    ground_alt: Mutex<i32>,
    target_alt: Mutex<i32>,
    current_alt: Mutex<i32>,
}

impl Drone {
    pub fn new() -> Drone {
        Drone {
            motors: Mutex::new([-1.0; 4]),
            armed: Mutex::new(false),
            current_lat: Mutex::new(0.0),
            current_lon: Mutex::new(0.0),
            home_lat: Mutex::new(0.0),
            home_lon: Mutex::new(0.0),
            target_lat: Mutex::new(0.0),
            target_lon: Mutex::new(0.0),
            ground_alt: Mutex::new(0),
            target_alt: Mutex::new(0),
            current_alt: Mutex::new(0),
        }
    }

    pub fn is_armed(&self) -> bool {
        self.armed.lock().unwrap().clone()
    }

    /// Arms the drone.
    pub fn arm(&self) {
        *self.armed.lock().unwrap() = true;
    }

    /// Disarms the drone.
    pub fn disarm(&self) {
        *self.armed.lock().unwrap() = false;

        self.set_motor_state([-1.0; 4]);
    }

    /// Retrieves the motor state in a thread-safe way.
    pub fn get_motor_state(&self) -> [f32; 4] {
        self.motors.lock().unwrap().clone()
    }

    pub fn set_motor_state(&self, new_motor_pwm: [f32; 4]) {
        *self.motors.lock().unwrap() = new_motor_pwm;
    }

    pub fn get_target_coordinates(&self) -> (f64, f64) {
        (self.target_lat.lock().unwrap().clone(), self.target_lon.lock().unwrap().clone())
    }

    pub fn set_target_coordinates(&self, lat: f64, lon: f64) {
        println!("Setting the target to: {}, {}", lat, lon);

        *self.target_lat.lock().unwrap() = lat;
        *self.target_lon.lock().unwrap() = lon;
    }

    pub fn get_current_coordinates(&self) -> (f64, f64) {
        (self.current_lat.lock().unwrap().clone(), self.current_lon.lock().unwrap().clone())
    }

    pub fn set_current_coordinates(&self, lat: f64, lon: f64) {
        *self.current_lat.lock().unwrap() = lat;
        *self.current_lon.lock().unwrap() = lon;
    }

    pub fn get_home_coordinates(&self) -> (f64, f64) {
        (self.home_lat.lock().unwrap().clone(), self.home_lon.lock().unwrap().clone())
    }

    pub fn set_home_coordinates(&self, lat: f64, lon: f64) {
        *self.home_lat.lock().unwrap() = lat;
        *self.home_lon.lock().unwrap() = lon;
    }

    pub fn get_ground_alt(&self) -> i32 {
        self.ground_alt.lock().unwrap().clone()
    }

    pub fn set_ground_alt(&self, alt: i32) {
        *self.ground_alt.lock().unwrap() = alt;
    }

    pub fn get_target_alt(&self) -> i32 {
        self.target_alt.lock().unwrap().clone()
    }

    pub fn set_target_alt(&self, target_alt: i32) {
        *self.target_alt.lock().unwrap() = target_alt;
    }

    pub fn get_current_alt(&self) -> i32 {
        self.current_alt.lock().unwrap().clone()
    }

    pub fn set_current_alt(&self, current_alt: i32) {
        *self.current_alt.lock().unwrap() = current_alt;
    }
}
