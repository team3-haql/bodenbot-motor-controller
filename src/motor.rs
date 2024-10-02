//! The motor encoder spawns a new motor task loop. The task
//! will manage the pid motor controller for the given pins.
//!
//! The current state of the driver such as target value and
//! measured value can be read over i2c.
use crate::encoder::{Direction, Encoder};
use crate::pwm::{PwmSignal, TOP_CLOCK};
use crate::utils::Mutex;
use embassy_rp::gpio::{AnyPin, Level, Output};
use embassy_rp::rom_data::float_funcs::fexp;
use embassy_time::{Duration, Instant, Ticker};
use num_traits::float::FloatCore;
use once_cell::sync::Lazy;

/// Proportional PID controller constant
const K_P: f32 = 1.0;
/// Integral PID controller constant
const K_I: f32 = 0.0;
/// Derivative PID controller constant
const K_D: f32 = 0.0;
const FREQUENCY: u64 = 100;

pub type DriverMutex = Mutex<Lazy<Driver>>;

macro_rules! motor_drivers {
    ($($motor:ident),*$(,)?) => {
        $(
        #[doc = concat!("The global singleton for motor driver ", stringify!($motor))]
        pub static $motor: DriverMutex = DriverMutex::new(Lazy::new(|| {
            Driver::default()
        }));
        )*
    };
}

#[rustfmt::skip]
motor_drivers!(
    MOTOR0,
    MOTOR1,
    MOTOR2,
    MOTOR3,
    MOTOR4,
    MOTOR5,
);

struct ExponentialAverage {
    value: f32,
    alpha: f32,
}

impl ExponentialAverage {
    fn new(alpha: f32) -> Self {
        Self { value: 0.0, alpha }
    }
    fn add(&mut self, value: f32) {
        self.value = self.alpha * value + (1.0 - self.alpha) * self.value;
    }
    fn average(&self) -> f32 {
        self.value
    }
}

/// PID controller for motor speed control
pub struct Driver {
    target_value: f32,
    previous_error: f32,
    integral: f32,
    output: f32,
    data: ExponentialAverage,
}

impl Default for Driver {
    fn default() -> Self {
        Self {
            target_value: 0.0,
            previous_error: 0.0,
            integral: 0.0,
            output: 0.0,
            data: ExponentialAverage::new(1.0 - fexp(-1.0 / (FREQUENCY as f32 * 0.05))),
        }
    }
}

impl Driver {
    /// Read the target value for the PID controller
    pub fn set_target(&mut self, target: f32) {
        self.target_value = target
    }

    /// Read the current target value for the PID controller
    pub fn get_target(&self) -> f32 {
        self.target_value
    }

    /// Read the last measured_value for the PID controller
    pub fn get_measure_value(&self) -> f32 {
        self.data.average()
    }

    /// Update the PID controller with a new measured_value and time delta.
    pub fn update(&mut self, radians: f32, delta: Duration) -> f32 {
        // should not be longer than a few milliseconds
        // Divide by 1000000 to convert micros to seconds
        let dt = (delta.as_micros() as f32) / 1e6;
        if dt.abs() < 1e-6 {
            return self.output;
        }

        self.data.add(radians / dt);
        let measured_value = self.data.average();

        let error = (self.target_value - measured_value).clamp(-100.0, 100.0);
        let proportional = error; // Proportional term
        self.integral += error * dt; // Integral term
                                     // Windup guard
                                     // 80 RPM -> 0.00838 rad/ms
        self.integral = self.integral.clamp(-100.0, 100.0);

        let derivative = (error - self.previous_error) / dt; // Derivative term
        self.previous_error = error;

        let accel = (K_P * proportional) + (K_I * self.integral) + (K_D * derivative);

        self.output += accel * dt;
        self.output = self.output.clamp(-1.0, 1.0);
        if self.output.is_nan() {
            self.output = 0.0;
        }
        self.output
    }
}

/// Motor driver task runs a loop that reads the encoder
/// value and updates the motor driver
pub async fn motor_driver<'a, D>(
    pwm_signal: &'a PwmSignal,
    driver: &'a DriverMutex,
    direction: D,
    mut encoder: Encoder<'a>,
) -> !
where
    D: Into<AnyPin>,
{
    defmt::info!("Starting motor driver");
    log::info!("Starting motor driver");
    let mut direction = Output::new(direction.into(), Level::Low);
    let mut last_update = Instant::now();
    let mut ticker = Ticker::every(Duration::from_hz(FREQUENCY));
    loop {
        let value = encoder.read_and_reset().await;
        // make sure the time step doesn't become too long
        // reading from encoder may stall when not moving.
        let elapsed = last_update.elapsed();
        let control = driver.lock().await.update(value, elapsed);

        // Use a threshold to set encoder direction to avoid oscillation
        // when switching directions
        if control > 0.1 {
            direction.set_low();
            encoder.set_direction(Direction::Forward).await;
        } else if control < -0.1 {
            direction.set_high();
            encoder.set_direction(Direction::Backward).await;
        } else {
            encoder.set_direction(Direction::None).await;
        }

        pwm_signal.signal((control * TOP_CLOCK as f32) as u16);

        last_update = Instant::now();

        // use Timer instead of Ticker so time steps remain constant
        // even if the loop takes longer due to encoder stall
        ticker.next().await;
    }
}
