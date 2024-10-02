#![no_std]

mod encoder;
mod i2c;
pub mod motor;
mod pwm;
// mod serial;
mod serial_motor;
pub mod utils;

use embassy_executor::Spawner;
use embassy_rp::gpio;
use embassy_rp::watchdog::Watchdog;
use embassy_time::{Duration, Ticker, Timer};
use gpio::{AnyPin, Input, Output};

use {defmt_rtt as _, panic_probe as _};

pub async fn motor_control(spawner: &Spawner) {
    let p = embassy_rp::init(Default::default());
    defmt::info!("Start Motor Controller!");

    // start serial
    spawner.must_spawn(serial_motor::serial_motor_task(p.USB));
    spawner.must_spawn(i2c::device_task(p.I2C1, p.PIN_26, p.PIN_27));

    // Set to watchdog to reset if it's not fed within 1.05 seconds, and start it
    let mut watchdog = Watchdog::new(p.WATCHDOG);
    // watchdog.start(Duration::from_secs(5));

    let mut enc = encoder::EncoderBuilder::new(p.PIO0, p.PIO1);

    join!(
        // create pwm workers
        // pwm::slice_worker_b(p.PWM_CH0, p.PIN_1, &pwm::MOTOR0_PWM),
        pwm::slice_worker_ab(p.PWM_SLICE0, p.PIN_16, &pwm::MOTOR5, p.PIN_1, &pwm::MOTOR0),
        pwm::slice_worker_ab(
            p.PWM_SLICE1,
            p.PIN_18,
            &pwm::LED0_PWM,
            p.PIN_19,
            &pwm::FAN0_PWM
        ),
        pwm::slice_worker_ab(
            p.PWM_SLICE2,
            p.PIN_4,
            &pwm::MOTOR1,
            p.PIN_21,
            &pwm::FAN1_PWM
        ),
        pwm::slice_worker_b(p.PWM_SLICE3, p.PIN_7, &pwm::MOTOR2),
        pwm::slice_worker_b(p.PWM_SLICE5, p.PIN_11, &pwm::MOTOR3),
        pwm::slice_worker_a(p.PWM_SLICE7, p.PIN_14, &pwm::MOTOR4),
        // start motor drivers
        motor::motor_driver(&pwm::MOTOR0, &motor::MOTOR0, p.PIN_0, enc.spawn(p.PIN_2),),
        motor::motor_driver(&pwm::MOTOR1, &motor::MOTOR1, p.PIN_3, enc.spawn(p.PIN_5),),
        motor::motor_driver(&pwm::MOTOR2, &motor::MOTOR2, p.PIN_6, enc.spawn(p.PIN_8),),
        motor::motor_driver(&pwm::MOTOR3, &motor::MOTOR3, p.PIN_9, enc.spawn(p.PIN_10),),
        motor::motor_driver(&pwm::MOTOR4, &motor::MOTOR4, p.PIN_12, enc.spawn(p.PIN_13),),
        motor::motor_driver(&pwm::MOTOR5, &motor::MOTOR5, p.PIN_15, enc.spawn(p.PIN_17),),
        // read button
        async {
            let mut btn = Input::new(AnyPin::from(p.PIN_28), gpio::Pull::Up);
            loop {
                btn.wait_for_falling_edge().await;
                defmt::info!("Button pressed!");
                // wait to debounce
                Timer::after_millis(50).await;
                btn.wait_for_rising_edge().await;
                Timer::after_millis(50).await;
            }
        },
        // blink LED
        async {
            let mut led = Output::new(AnyPin::from(p.PIN_25), gpio::Level::Low);
            let mut ticker = Ticker::every(Duration::from_hz(1));
            loop {
                led.set_high();
                Timer::after_millis(200).await;
                led.set_low();
                ticker.next().await;
            }
        },
        async {
            // feed watchdog
            loop {
                Timer::after_millis(100).await;
                watchdog.feed();
            }
        },
    )
    .await;
}
