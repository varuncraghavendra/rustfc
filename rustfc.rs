use std::time::{Duration, Instant};
use std::thread::sleep;

const R2D: f32 = 57.29578;
const D2R: f32 = 0.0174533;

const MOTOR1_PIN: u8 = 8;
const MOTOR2_PIN: u8 = 27;
const MOTOR3_PIN: u8 = 2;
const MOTOR4_PIN: u8 = 1;

const RC_CH1_PIN: u8 = 11;
const RC_CH2_PIN: u8 = 12;
const RC_CH3_PIN: u8 = 15;
const RC_CH4_PIN: u8 = 13;
const RC_CH5_PIN: u8 = 14;

const GYRO_PRESET: f32 = 0.0175;
const ACC_PRESET: f32 = 0.000244;

const LOOP_RATE: u64 = 250;

const CF_K: f32 = 0.997;
const EWMA_K1: f32 = 0.7;
const EWMA_K2: f32 = 0.9;

const KP_ROLL_RATE: f32 = 1.2;
const KI_ROLL_RATE: f32 = 0.02;
const KD_ROLL_RATE: f32 = 4.5;

const KP_ROLL: f32 = 1.87;

const KP_YAW_RATE: f32 = 3.0;
const KI_YAW_RATE: f32 = 0.02;
const KD_YAW_RATE: f32 = 0.0;

const PID_MAX_OUTPUT: f32 = 400.0;

struct UAV {
    imu_start: bool,
    rc_pins_prev_data: [bool; 5],
    raw_gyro_data: [i16; 3],
    raw_acc_data: [i16; 3],
    uav_start_flag: i32,
    esc_pwm: [i32; 4],
    rc_channels: [i32; 5],
    acc_data: [f32; 3],
    gyro_data: [f32; 3],
    gyro_euler: [f32; 2],
    acc_euler: [f32; 2],
    euler_angles: [f32; 2],
    gyro_euler_dps: [f32; 3],
    rate_setpoints: [f32; 3],
    euler_corrections: [f32; 2],
    pid_error_euler: [f32; 3],
    pid_prev_error_euler: [f32; 3],
    pid_integrator_euler: [f32; 3],
    pid_output: [f32; 3],
    gyro_offset: [i64; 3],
    loop_timer: Instant,
}

impl UAV {
    fn new() -> Self {
        Self {
            imu_start: false,
            rc_pins_prev_data: [false; 5],
            raw_gyro_data: [0; 3],
            raw_acc_data: [0; 3],
            uav_start_flag: 0,
            esc_pwm: [1000; 4],
            rc_channels: [1500, 1500, 1000, 1500, 1000],
            acc_data: [0.0; 3],
            gyro_data: [0.0; 3],
            gyro_euler: [0.0; 2],
            acc_euler: [0.0; 2],
            euler_angles: [0.0; 2],
            gyro_euler_dps: [0.0; 3],
            rate_setpoints: [0.0; 3],
            euler_corrections: [0.0; 2],
            pid_error_euler: [0.0; 3],
            pid_prev_error_euler: [0.0; 3],
            pid_integrator_euler: [0.0; 3],
            pid_output: [0.0; 3],
            gyro_offset: [0; 3],
            loop_timer: Instant::now(),
        }
    }

    fn setup(&mut self) {
        println!("Initializing UAV...");
        sleep(Duration::from_secs(5));
        self.calibrate_imu();
        self.loop_timer = Instant::now();
    }

    fn calibrate_imu(&mut self) {
        println!("Calibrating IMU...");
        self.gyro_offset = [0, 0, 0];
    }

    fn read_raw_imu(&mut self) {
        println!("Reading raw IMU data...");
    }

    fn pid_controller(
        &mut self,
        setpoint: f32,
        current_angle: f32,
        rate_of_change: f32,
        axis: &str,
    ) -> f32 {
        let error = setpoint - current_angle;
        let rate_error = -rate_of_change; // Negative sign to compensate for rate direction
        
        // Determine which PID constants to use based on the axis
        let (kp, ki, kd, prev_error, integrator, prev_integral) = match axis {
            "roll" => (
                KP_ROLL,
                KI_ROLL_RATE,
                KD_ROLL_RATE,
                &mut self.pid_prev_error_euler[0],
                &mut self.pid_integrator_euler[0],
                &mut self.pid_prev_error_euler[0],
            ),
            "yaw" => (
                KP_YAW_RATE,
                KI_YAW_RATE,
                KD_YAW_RATE,
                &mut self.pid_prev_error_euler[1],
                &mut self.pid_integrator_euler[1],
                &mut self.pid_prev_error_euler[1],
            ),
            _ => panic!("Invalid axis for PID controller"),
        };

        // Proportional term
        let p_term = kp * error;

        // Integral term
        *integrator += error;
        let i_term = ki * *integrator;

        // Derivative term
        let d_term = kd * rate_error;

        // PID output
        let pid_output = p_term + i_term + d_term;

        // Limit the output to prevent saturation
        let pid_output = pid_output.clamp(-PID_MAX_OUTPUT, PID_MAX_OUTPUT);

        // Update the previous error and integrator for the next cycle
        *prev_error = error;
        *prev_integral = *integrator;

        pid_output
    }

    fn loop_iteration(&mut self) {
        // Example setpoints (you can modify these based on your logic)
        let roll_setpoint: f32 = 0.0;  // Target roll angle (e.g., level flight)
        let pitch_setpoint: f32 = 0.0; // Target pitch angle
        let yaw_setpoint: f32 = 0.0;   // Target yaw angle

        // Read raw IMU data
        self.read_raw_imu();

        // Calculate the current roll, pitch, yaw from the IMU data
        let roll_angle = self.euler_angles[0];  // Roll angle from gyro/accelerometer
        let yaw_angle = self.euler_angles[1];   // Yaw angle from gyro/accelerometer
        let roll_rate = self.gyro_data[0];       // Roll rate from gyro
        let yaw_rate = self.gyro_data[2];        // Yaw rate from gyro

        // Use PID controller for each axis
        let roll_pid = self.pid_controller(roll_setpoint, roll_angle, roll_rate, "roll");
        let yaw_pid = self.pid_controller(yaw_setpoint, yaw_angle, yaw_rate, "yaw");

        // Apply the PID outputs to control the motors (ESCs)
        self.esc_pwm[0] += roll_pid as i32; // For simplicity, applying directly
        self.esc_pwm[1] -= roll_pid as i32;
        self.esc_pwm[2] -= yaw_pid as i32;
        self.esc_pwm[3] += yaw_pid as i32;

        // Print debug info
        println!("Roll PID: {}", roll_pid);
        println!("Yaw PID: {}", yaw_pid);

        let elapsed = self.loop_timer.elapsed().as_micros();
        println!("Loop iteration: {} µs", elapsed);

        // Reset loop timer
        self.loop_timer = Instant::now();
    }
}

fn main() {
    let mut uav = UAV::new();
    uav.setup();
    loop {
        uav.loop_iteration();
        sleep(Duration::from_millis(1000 / LOOP_RATE));
    }
}
