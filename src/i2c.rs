//! This i2c task can receives messages with a register.
//! Writing to the register updates the appropriate values within
//! a controller.
//!
//! The motor registers can be written to and read from. Writing to the motor
//! register expects a number in the `I16F16` fixed point format and updates the
//! associated controller's target value. Reading from the motor will return the
//! current measured value in the same format.
//!
//! The `LED` and `FAN` registers use a `u16` for a compare value a pwm clock. The value
//! can only be written to.
use crate::motor;
use crate::motor::DriverMutex;
use crate::pwm;
use embassy_rp::peripherals::{self as p, I2C1};
use embassy_rp::{bind_interrupts, i2c, i2c_slave};

bind_interrupts!(struct Irqs {
    I2C1_IRQ => i2c::InterruptHandler<I2C1>;
});

/// The device address for the I2C device
const DEV_ADDR: u8 = 0x22;

/// Macro to define registers for I2C communication
macro_rules! registers {
    ($($name:ident: $value:expr),* $(,)?) => {
        pub mod registers {
            $(
            #[doc = concat!("The register value for ", stringify!($name))]
            pub const $name: u8 = $value;
            )*
        }
    }
}

registers!(
    MOTOR0: 0x00,
    MOTOR1: 0x01,
    MOTOR2: 0x02,
    MOTOR3: 0x03,
    MOTOR4: 0x04,
    MOTOR5: 0x05,
    LED0: 0x10,
    FAN0: 0x20,
    FAN1: 0x21,
);

use registers::*;

fn read_motor(motor: &DriverMutex) -> [u8; 4] {
    let value = if let Ok(m) = motor.try_lock() {
        m.get_measure_value().to_le_bytes()
        // m.get_target().to_le_bytes()
    } else {
        [0, 0, 0, 0]
    };
    value
}

async fn write_motor(motor: &DriverMutex, value: [u8; 4]) {
    let value = f32::from_le_bytes(value);
    motor.lock().await.set_target(value);
}

#[embassy_executor::task]
pub async fn device_task(i2c: I2C1, d_sda: p::PIN_26, d_scl: p::PIN_27) -> ! {
    defmt::info!("I2C start");
    log::info!("I2C Device start");

    let mut config = i2c_slave::Config::default();
    config.addr = DEV_ADDR as u16;
    config.general_call = true;
    let mut dev = i2c_slave::I2cSlave::new(i2c, d_scl, d_sda, Irqs, config);

    loop {
        let mut buf = [0u8; 128];
        // only the write and write then read commands are used since each
        // transaction always expects an associated register.
        match dev.listen(&mut buf).await {
            Ok(i2c_slave::Command::GeneralCall(len)) => {
                defmt::info!("Device received general call write: {}", buf[..len])
            }
            Ok(i2c_slave::Command::Read) => loop {
                defmt::warn!("Received Read command");
                match dev.respond_to_read(&[0x73]).await {
                    Ok(x) => match x {
                        i2c_slave::ReadStatus::Done => {
                            log::info!("Finished responding to read");
                            break;
                        }
                        i2c_slave::ReadStatus::NeedMoreBytes => (),
                        i2c_slave::ReadStatus::LeftoverBytes(x) => {
                            defmt::info!("tried to write {} extra bytes", x);
                            break;
                        }
                    },
                    Err(e) => {
                        defmt::error!("error while responding {}", e);
                        log::error!("error while responding");
                    }
                }
            },
            Ok(i2c_slave::Command::Write(_len)) => {
                match buf[0] {
                    // Set the state
                    MOTOR0 => write_motor(&motor::MOTOR0, [buf[1], buf[2], buf[3], buf[4]]).await,
                    MOTOR1 => write_motor(&motor::MOTOR1, [buf[1], buf[2], buf[3], buf[4]]).await,
                    MOTOR2 => write_motor(&motor::MOTOR2, [buf[1], buf[2], buf[3], buf[4]]).await,
                    MOTOR3 => write_motor(&motor::MOTOR3, [buf[1], buf[2], buf[3], buf[4]]).await,
                    MOTOR4 => write_motor(&motor::MOTOR4, [buf[1], buf[2], buf[3], buf[4]]).await,
                    MOTOR5 => write_motor(&motor::MOTOR5, [buf[1], buf[2], buf[3], buf[4]]).await,
                    LED0 => pwm::LED0_PWM.signal(u16::from_le_bytes([buf[1], buf[2]])),
                    FAN0 => pwm::FAN0_PWM.signal(u16::from_le_bytes([buf[1], buf[2]])),
                    FAN1 => pwm::FAN1_PWM.signal(u16::from_le_bytes([buf[1], buf[2]])),
                    // error
                    _ => {
                        defmt::error!("Invalid Write {:x}", buf[0]);
                        log::error!("Invalid Write");
                    }
                }
            }
            Ok(i2c_slave::Command::WriteRead(_len)) => {
                macro_rules! motor_read {
                    ($driver:expr) => {{
                        let values = read_motor($driver);
                        match dev.respond_and_fill(&values, 0x00).await {
                            Ok(_read_status) => {}
                            Err(e) => defmt::error!("error while responding {}", e),
                        }
                    }};
                }
                match buf[0] {
                    MOTOR0 => motor_read!(&motor::MOTOR0),
                    MOTOR1 => motor_read!(&motor::MOTOR1),
                    MOTOR2 => motor_read!(&motor::MOTOR2),
                    MOTOR3 => motor_read!(&motor::MOTOR3),
                    MOTOR4 => motor_read!(&motor::MOTOR4),
                    MOTOR5 => motor_read!(&motor::MOTOR5),
                    LED0 => defmt::todo!("Read LED"),
                    FAN0 => defmt::todo!("Read FAN 0"),
                    FAN1 => defmt::todo!("Read FAN 1"),
                    // error
                    x => {
                        defmt::error!("Invalid Write Read {:x}", x);
                        log::error!("Invalid Write Read");
                    }
                }
            }
            Err(e) => defmt::error!("{}", e),
        }
    }
}
