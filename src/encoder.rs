//! We use the PIO for the encoder to allow high throughput without
//! interrupting the CPU. The PIO state machine is configured to
//! count the encoder pin pulses until a signal is sent to push the
//! pulse count to the RX FIFO which is read by the CPU.
use embassy_rp::gpio::Pull;
use embassy_rp::peripherals::{PIO0, PIO1};
use embassy_rp::{bind_interrupts, pio};
use embassy_time::Duration;
use fixed::traits::ToFixed;
use pio::{Common, Config, Instance, InterruptHandler, LoadedProgram, Pio, PioPin, StateMachine};

/// The gear ratio of the bodenbot motors
const RATIO: i32 = 100;

/// The pulses per revolution of the encoder
const PPR: i32 = 16 * RATIO;

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => InterruptHandler<PIO0>;
    PIO1_IRQ_0 => InterruptHandler<PIO1>;
});

/// The internal encoder PIO state machine
struct PioEncoderInner<'d, T: Instance, const SM: usize> {
    sm: StateMachine<'d, T, SM>,
}

/// The encoder direction
#[allow(dead_code)]
#[derive(Debug, Copy, Clone, PartialEq, Eq)]
pub enum Direction {
    Forward,
    Backward,
    None,
}

/// A wrapper for the PIO encoder state machine to allow
/// multiple generic instances of the PIO state machine to be
/// used in a non generic function.
#[allow(dead_code)]
enum PioEncoder<'d> {
    P0SM0(PioEncoderInner<'d, PIO0, 0>),
    P0SM1(PioEncoderInner<'d, PIO0, 1>),
    P0SM2(PioEncoderInner<'d, PIO0, 2>),
    P0SM3(PioEncoderInner<'d, PIO0, 3>),
    P1SM0(PioEncoderInner<'d, PIO1, 0>),
    P1SM1(PioEncoderInner<'d, PIO1, 1>),
    P1SM2(PioEncoderInner<'d, PIO1, 2>),
    P1SM3(PioEncoderInner<'d, PIO1, 3>),
}

/// The public interface for reading the encoder..
pub struct Encoder<'d> {
    direction: Direction,
    pulses: i32,
    pio: PioEncoder<'d>,
}

#[allow(dead_code)]
struct PioData<'d, T: Instance> {
    common: Common<'d, T>,
    sm0: Option<StateMachine<'d, T, 0>>,
    sm1: Option<StateMachine<'d, T, 1>>,
    sm2: Option<StateMachine<'d, T, 2>>,
    sm3: Option<StateMachine<'d, T, 3>>,
    program: LoadedProgram<'d, T>,
}

impl<'d, T: Instance> PioData<'d, T> {
    fn new(mut pio: Pio<'d, T>) -> Self {
        let program = build_program(&mut pio.common);
        Self {
            common: pio.common,
            sm0: Some(pio.sm0),
            sm1: Some(pio.sm1),
            sm2: Some(pio.sm2),
            sm3: Some(pio.sm3),
            program,
        }
    }
}

pub struct EncoderBuilder<'d> {
    pio0: PioData<'d, PIO0>,
    pio1: PioData<'d, PIO1>,
    encoder_count: u32,
}

fn build_program<'d, T: Instance>(pio: &mut Common<'d, T>) -> LoadedProgram<'d, T> {
    #[rustfmt::skip]
    let pio_program = pio_proc::pio_asm!(
        "start:",
        "  set y (-1)", // negate y so counter is accurate
        "  set x 0",
        "loop:",
        "  wait 0 pin 0 [8]",
        "  wait 1 pin 0 [8]",
        "  jmp y-- test",
        "test:",
        "  pull noblock",
        "  out x 32",
        "  jmp !x loop",
        "output:",
        "  mov isr ~y",
        "  push",
    );
    pio.try_load_program(&pio_program.program)
        .expect("Failed to load PIO program.")
}

impl<'d> EncoderBuilder<'d> {
    pub fn new(pio0: PIO0, pio1: PIO1) -> Self {
        let pio0 = Pio::new(pio0, Irqs);
        let pio1 = Pio::new(pio1, Irqs);

        Self {
            pio0: PioData::new(pio0),
            pio1: PioData::new(pio1),
            encoder_count: 0,
        }
    }

    /// Creates and sets up a new encoder state machine.
    pub fn spawn(&mut self, clk_pin: impl PioPin) -> Encoder<'d> {
        // This macro creates a PIO encoder state machine and wraps it in
        // an enum for the proper encoder.
        macro_rules! pio_encoder {
            ($machine:ident: ($pio:ident, $sm:ident)) => {{
                let common = &mut self.$pio.common;
                let sm = self
                    .$pio
                    .$sm
                    .take()
                    .expect("Failed to take PIO state machine in encoder builder");
                let program = &self.$pio.program;

                let pio_encoder = PioEncoderInner::new(common, sm, clk_pin, program);
                defmt::info!("Starting encoder {}", stringify!($machine));
                log::info!("Starting encoder {}", stringify!($machine));

                PioEncoder::$machine(pio_encoder)
            }};
        }

        // each time this function is called a new PIO machine has
        // to be used. The `ENCODER_COUNT` is keeping track of
        // how many have been created
        let machine = match self.encoder_count {
            0 => pio_encoder!(P0SM0: (pio0, sm0)),
            1 => pio_encoder!(P0SM1: (pio0, sm1)),
            2 => pio_encoder!(P0SM2: (pio0, sm2)),
            3 => pio_encoder!(P0SM3: (pio0, sm3)),
            4 => pio_encoder!(P1SM0: (pio1, sm0)),
            5 => pio_encoder!(P1SM1: (pio1, sm1)),
            6 => pio_encoder!(P1SM2: (pio1, sm2)),
            7 => pio_encoder!(P1SM3: (pio1, sm3)),
            _ => panic!("Exceeded the number of supported encoders"),
        };

        self.encoder_count += 1;

        let encoder = Encoder::new(machine);
        encoder
    }
}

impl<'d, T: Instance, const SM: usize> PioEncoderInner<'d, T, SM> {
    fn new(
        pio: &mut Common<'d, T>,
        mut sm: StateMachine<'d, T, SM>,
        pulse_pin: impl PioPin,
        program: &LoadedProgram<'d, T>,
    ) -> Self {
        // The PIO program is written in PIO assembly
        // it decrements the y register using a jmp instuction
        // since that is the only PIO instruction that allows
        // arithmetic operations. The value is negated before pushing
        // to the RX FIFO to fix the negative value.
        //
        // x must be set to 0 before `pull noblock` since that instruction
        // defaults to reading the x register if there is nothing in the TX FIFO.
        //
        // if x is not zero after the pull instruction, that signals that a read command
        // occured and the pulse count is pushed to the CPU.
        //
        // the arithmetic logic is based on x + y = ~(~x - y)
        let mut cfg = Config::default();

        let mut pulse_pin = pio.make_pio_pin(pulse_pin);
        pulse_pin.set_pull(Pull::None);
        sm.set_pin_dirs(pio::Direction::In, &[&pulse_pin]);
        cfg.set_in_pins(&[&pulse_pin]);
        // limit clock speed to avoid reading jitter
        cfg.clock_divider = 10.to_fixed();

        cfg.use_program(program, &[]);
        sm.set_config(&cfg);
        sm.set_enable(true);
        Self { sm }
    }

    async fn read(&mut self) -> u32 {
        // signal a read to the PIO state machine
        if self.sm.tx().empty() && self.sm.rx().empty() {
            self.sm.tx().push(5);
        }

        embassy_time::block_for(Duration::from_nanos(50));

        if let Some(value) = self.sm.rx().try_pull() {
            value
        } else {
            0
        }
    }
}

impl PioEncoder<'_> {
    pub async fn read(&mut self) -> u32 {
        match self {
            PioEncoder::P0SM0(sm) => sm.read().await,
            PioEncoder::P0SM1(sm) => sm.read().await,
            PioEncoder::P0SM2(sm) => sm.read().await,
            PioEncoder::P0SM3(sm) => sm.read().await,
            PioEncoder::P1SM0(sm) => sm.read().await,
            PioEncoder::P1SM1(sm) => sm.read().await,
            PioEncoder::P1SM2(sm) => sm.read().await,
            PioEncoder::P1SM3(sm) => sm.read().await,
        }
    }
}

impl<'d> Encoder<'d> {
    const fn new(pio: PioEncoder<'d>) -> Self {
        Self {
            direction: Direction::None,
            pulses: 0,
            pio,
        }
    }

    /// Read the encoder angle in radians
    async fn read(&mut self) -> f32 {
        self.update().await;
        (self.pulses as f32) * (core::f32::consts::PI * 2.0) / (PPR as f32)
    }

    /// Read the encoder angle in radians
    pub async fn read_and_reset(&mut self) -> f32 {
        let rot = self.read().await;
        self.reset_pulse_count();
        rot
    }

    fn reset_pulse_count(&mut self) {
        self.pulses = 0;
    }

    /// The encoder direction needs to be manually set since
    /// we are using a single pulse encoder.
    pub async fn set_direction(&mut self, direction: Direction) {
        // Update the encoder before changing the direction so that
        // previous encoder pulses are counted correctly
        self.update().await;
        self.direction = direction;
    }

    /// Updates the interrnal pulse count depending on its set direction
    async fn update(&mut self) {
        let pulses = self.pio.read().await;
        match self.direction {
            Direction::Forward => self.pulses += pulses as i32,
            Direction::Backward => self.pulses -= pulses as i32,
            Direction::None => (),
        }
    }
}
