pub trait Controller {
    fn update(&mut self, state: &[f32; 4]) -> [f32; 2];
    fn set_setpoint(&mut self, setpoint: [f32; 4]);
}
pub struct PIDController {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub setpoint: [f32; 4],
    pub integral: [f32; 4],
    pub previous_error: [f32; 4],
}

impl Controller for PIDController {
    fn update(&mut self, state: &[f32; 4]) -> [f32; 2] {
        let mut output = [0.0; 2];
        for i in 0..4 {
            let error = self.setpoint[i] - state[i];
            self.integral[i] += error;
            let derivative = error - self.previous_error[i];
            output[0] += self.kp * error + self.ki * self.integral[i] + self.kd * derivative;
            output[1] += self.kp * error + self.ki * self.integral[i] + self.kd * derivative;
            self.previous_error[i] = error;
        }
        output
    }
    fn set_setpoint(&mut self, setpoint: [f32; 4]) {
        self.setpoint = setpoint;
    }
}
