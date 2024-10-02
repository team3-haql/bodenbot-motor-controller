#![no_std]
#![no_main]

use bodenbot_motor_controller as motor_control;
use embassy_executor::Spawner;
// use embassy_time::Timer;
// use fixed::traits::ToFixed;
// use motor_control::join;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    motor_control::motor_control(&spawner).await;

    // let motors = [
    //     &motor_control::motor::MOTOR0,
    //     &motor_control::motor::MOTOR1,
    //     &motor_control::motor::MOTOR2,
    //     &motor_control::motor::MOTOR3,
    //     &motor_control::motor::MOTOR4,
    //     &motor_control::motor::MOTOR5,
    // ];
    //
    // join!(motor_control::motor_control(&spawner), async {
    //     loop {
    //         for (i, motor) in motors.iter().enumerate() {
    //             log::info!(
    //                 "motor{}: {}",
    //                 i,
    //                 motor.lock().await.get_measure_value().to_num::<f32>()
    //             );
    //             motor.lock().await.set_target(1.to_fixed());
    //         }
    //         Timer::after_millis(500).await;
    //     }
    // },)
    // .await;
}
