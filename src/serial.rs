//! This create a serial usb logger task and allows the use for the `log`
//! crate to write to serial for debuggin.

use embassy_rp::bind_interrupts;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::{Driver, InterruptHandler};
use {defmt_rtt as _, panic_probe as _};

bind_interrupts!(struct Irqs {
    USBCTRL_IRQ => InterruptHandler<USB>;
});

#[embassy_executor::task]
pub async fn serial_task(usb: USB) {
    defmt::info!("Starting USB serial logger");
    // Create the driver, from the HAL.
    let driver = Driver::new(usb, Irqs);

    embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
}
