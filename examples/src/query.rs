//! Basic Embassy example for the Raspberry Pi Pico 2
//! Showcase querying things from the DFPlayer Mini
//!
//! Assumes the following connections:
//!
//! - UART0 TX/RX on GPIO 17/16
//! - Busy Pin on GPIO 18
//!
//! Assumes the following for the DFPlayer Mini:
//!
//! - Powered and ground connected
//! - TX/RX connected
//! - Busy Pin connected
//! - Speaker connected
//! - Using SD card for audio files. Place 3 mp3 files on the SD card into a folder structure as documented in the DFPlayer Mini manual. For example:
//!     - no folders, use root directory
//!         - 0001-whatevernameX.mp3
//!         - 0002-whatevernameY.mp3
//!         - 0003-whatevernameZ.mp3

#![no_std]
#![no_main]

use defmt::{Debug2Format, error, info};
use dfplayer_serial::{DfPlayer, Equalizer, TimeSource};
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    block::ImageDef,
    config::Config,
    peripherals::UART0,
    uart::{
        BufferedInterruptHandler, BufferedUart, Config as UartConfig, DataBits,
        Parity, StopBits,
    },
};
use embassy_time::{Delay, Duration, Instant, Timer};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

/// Firmware image type for bootloader
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

bind_interrupts!(pub struct Irqs {
    UART0_IRQ => BufferedInterruptHandler<UART0>;

});

/// Firmware entry point
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Config::default());

    // Initialize the DFPlayer Mini

    // We need a UART port to communicate with the DFPlayer Mini, defaults should be fine. But these valuesHere do work:
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 9600;
    uart_config.data_bits = DataBits::DataBits8;
    uart_config.stop_bits = StopBits::STOP1;
    uart_config.parity = Parity::ParityNone;

    // Create a buffered UART instance.
    // Buffer size for query commands must be way bigger than what we need for the other commands. The DFPlayer Mini is finnicky with timing and will sometimes send responses in multiple chunks.
    // Also we sometimes see responses transmitted in kind of random order, forcing the driver to buffer the responses until we find the one we expect.
    // In this example here 32 bytes is not enough, we see frequent issues with the query commands. 64 bytes seems to work fine here.
    static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 64])[..];
    static RX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 64])[..];
    let mut uart = BufferedUart::new(
        p.UART0,
        Irqs,
        p.PIN_16,
        p.PIN_17,
        tx_buf,
        rx_buf,
        uart_config,
    );

    // now we can create the DFPlayer Mini instance. See driver documentation for more information on the parameters.
    let feedback_enable = false;
    let timeout_ms = 1000;
    let delay = Delay;
    let reset_duration_override = None;

    // Implement the TimeSource trait for the DFPlayer Mini. The driver is hardware-agnostic and requires a TimeSource implementation to work.
    // That way you can use any time source you want, as long as it implements the TimeSource trait. This example uses the embassy_time crate for the implementation.
    struct MyTimeSource;
    impl TimeSource for MyTimeSource {
        type Instant = Instant;

        fn now(&self) -> Self::Instant {
            Instant::now()
        }

        fn is_elapsed(&self, since: Self::Instant, timeout_ms: u64) -> bool {
            Instant::now().duration_since(since)
                >= Duration::from_millis(timeout_ms)
        }
    }

    // Create the DFPlayer Mini instance, handing in the UART instance and the TimeSource implementation as well as the other parameters.
    let mut dfplayer = match DfPlayer::try_new(
        &mut uart,
        feedback_enable,
        timeout_ms,
        MyTimeSource,
        delay,
        reset_duration_override,
    )
    .await
    {
        Ok(dfplayer) => dfplayer,
        Err(e) => {
            error!("Error initializing DFPlayer Mini: {}", Debug2Format(&e));
            return;
        }
    };

    // Wait for the DFPlayer Mini to initialize
    Timer::after(Duration::from_millis(500)).await;

    // Now we can start sending commands to the DFPlayer Mini

    // Get the number of tracks on the SD card
    info!("Querying number of tracks on SD card...");
    match dfplayer.query_tracks_sd().await {
        Ok(tracks) => info!("Number of tracks on SD card: {}", tracks),
        Err(e) => error!(
            "Failed to query number of tracks on SD card: {}",
            Debug2Format(&e)
        ),
    }

    // The DFPlayer can be finnicky with timing, so we wait a bit before sending the next command
    Timer::after(Duration::from_millis(100)).await;

    // get the current equalizer setting
    info!("Querying current equalizer setting...");
    match dfplayer.query_eq().await {
        Ok(eq) => {
            let equalizer: Equalizer = match eq {
                0 => Equalizer::Normal,
                1 => Equalizer::Pop,
                2 => Equalizer::Rock,
                3 => Equalizer::Jazz,
                4 => Equalizer::Classic,
                5 => Equalizer::Bass,
                6..=u8::MAX => Equalizer::Normal,
            };
            info!("Current equalizer setting: {}", Debug2Format(&equalizer)); // Equalizer does not implement Debug itself
        }
        Err(e) => {
            error!("Failed to query equalizer setting: {}", Debug2Format(&e))
        }
    }

    // The DFPlayer can be finnicky with timing, so we wait a bit before sending the next command
    Timer::after(Duration::from_millis(100)).await;

    // get the current volume setting
    info!("Querying current volume setting...");
    match dfplayer.query_volume().await {
        Ok(volume) => info!("Current volume setting: {}", volume),
        Err(e) => {
            error!("Failed to query volume setting: {}", Debug2Format(&e))
        }
    }

    // Finish here
    info!("Finished querying DFPlayer Mini");
}
