//! Since 2 pwn pins are tied to a single slice, it is difficult
//! to manage pins that are not used together, but part of the same
//! pwm slice. To get around this, we create a worker task that runs the
//! pwm slice, and a signal channel that communicates with this task.
//! This way we do not directly interact with the pins.
use embassy_futures::select::select;
use embassy_futures::select::Either;
use embassy_rp::pwm::{ChannelAPin, ChannelBPin, Config, Pwm, Slice};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::signal::Signal;

/// The signal type for setting pwm signals.
pub type PwmSignal = Signal<ThreadModeRawMutex, u16>;

/// The top clock value for the pwm slice, the pwm clock resets
/// when it reaches this value.
pub const TOP_CLOCK: u16 = 0x8000;

macro_rules! pwm_signals {
    ($($signal:ident),*$(,)?) => {
        $(
        /// A global singleton that can be used to set PWM compare values.
        pub static $signal: PwmSignal = PwmSignal::new();
        )*
    };
}

#[rustfmt::skip]
pwm_signals!(
    MOTOR0,
    MOTOR1,
    MOTOR2,
    MOTOR3,
    MOTOR4,
    MOTOR5,
    LED0_PWM,
    FAN0_PWM,
    FAN1_PWM,
);

/// Create a worker task that runs the pwm slice for two pins
/// on the a and b channels.
pub async fn slice_worker_ab<C, U, V>(
    chan: C,
    p_a: U,
    sig_a: &'static PwmSignal,
    p_b: V,
    sig_b: &'static PwmSignal,
) -> !
where
    C: Slice,
    U: ChannelAPin<C>,
    V: ChannelBPin<C>,
{
    let mut config = Config::default();
    config.top = TOP_CLOCK;
    let mut pwm = Pwm::new_output_ab(chan, p_a, p_b, config.clone());
    loop {
        match select(sig_a.wait(), sig_b.wait()).await {
            Either::First(c) => {
                config.compare_a = c;
            }
            Either::Second(c) => {
                config.compare_b = c;
            }
        };
        pwm.set_config(&config);
    }
}

/// Create a worker task that runs the pwm slice for a single pin
/// on the a channel.
pub async fn slice_worker_a<C, U>(chan: C, p: U, sig: &'static PwmSignal) -> !
where
    C: Slice,
    U: ChannelAPin<C>,
{
    let mut config = Config::default();
    config.top = TOP_CLOCK;
    let mut pwm = Pwm::new_output_a(chan, p, config.clone());
    loop {
        config.compare_a = sig.wait().await;
        pwm.set_config(&config);
    }
}

/// Create a worker task that runs the pwm slice for a single pin
/// on the b channel.
pub async fn slice_worker_b<C, U>(chan: C, p: U, sig: &'static PwmSignal) -> !
where
    C: Slice,
    U: ChannelBPin<C>,
{
    let mut config = Config::default();
    config.top = TOP_CLOCK;
    let mut pwm = Pwm::new_output_b(chan, p, config.clone());
    loop {
        config.compare_b = sig.wait().await;
        pwm.set_config(&config);
    }
}
