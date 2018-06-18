/// Represents an instance of a PID.
pub struct PID {
    p_gain: f64,
    i_gain: f64,
    d_gain: f64,
    max: f64,
    pub setpoint: f64,
    i_mem: f64,
    last_d_error: f64,
}

impl PID {
    pub fn new(p_gain: f64, i_gain: f64, d_gain: f64, max: f64) -> PID {
        PID {
            p_gain,
            i_gain,
            d_gain,
            max,
            setpoint: 0.0,
            i_mem: 0.0,
            last_d_error: 0.0,
        }
    }

    pub fn calculate_pid(&mut self, input: f64) -> f64 {
        let pid_error_temp = input - self.setpoint;

        self.i_mem += self.i_gain * pid_error_temp;

        if self.i_mem > self.max {
            self.i_mem = self.max;
        } else if self.i_mem < self.max * -1.0 {
            self.i_mem = self.max * -1.0;
        }

        let mut pid_output = self.p_gain * pid_error_temp + self.i_mem + self.d_gain * (pid_error_temp - self.last_d_error);

        if pid_output > self.max {
            pid_output = self.max;
        } else if pid_output < self.max * -1.0 {
            pid_output = self.max * -1.0;
        }

        self.last_d_error = pid_error_temp;

        pid_output
    }
}
