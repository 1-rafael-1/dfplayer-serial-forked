//! Basic Embassy example for the Raspberry Pi Pico 2
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

use defmt::{error, info};
use dfplayer_serial::{
    Command, DfPlayer, Equalizer, PlayBackMode, PlayBackSource, Source,
    TimeSource,
};
use embassy_executor::Spawner;
use embassy_rp::{
    bind_interrupts,
    block::ImageDef,
    config::Config,
    gpio::{Input, Pull},
    peripherals::UART0,
    uart::{
        self, BufferedInterruptHandler, BufferedUart, Config as UartConfig,
        DataBits, InterruptHandler, Parity, StopBits, Uart,
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
    // UART0_IRQ => InterruptHandler<UART0>;

});

/// Firmware entry point
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_rp::init(Config::default());

    // Initialize the DFPlayer Mini
    // The modules usually have a busy pin that can be used to determine if the module is currently playing audio. Low if busy, high if not busy.
    let busy = Input::new(p.PIN_18, Pull::None);

    // We need a UART port to communicate with the DFPlayer Mini, defaults should be fine. But these valuesHere do work:
    let mut uart_config = UartConfig::default();
    uart_config.baudrate = 9600;
    uart_config.data_bits = DataBits::DataBits8;
    uart_config.stop_bits = StopBits::STOP1;
    uart_config.parity = Parity::ParityNone;

    static TX_BUF: StaticCell<[u8; 32]> = StaticCell::new();
    let tx_buf = &mut TX_BUF.init([0; 32])[..];
    static RX_BUF: StaticCell<[u8; 32]> = StaticCell::new();
    let rx_buf = &mut RX_BUF.init([0; 32])[..];

    let mut uart = BufferedUart::new(
        p.UART0,
        Irqs,
        p.PIN_16,
        p.PIN_17,
        tx_buf,
        rx_buf,
        uart_config,
    );

    // now we can create the DFPlayer Mini instance
    let feedback_enable = true;
    let timeout_ms = 1000;
    let delay = Delay;
    let reset_duration_override = None;

    // Implement the TimeSource trait for the DFPlayer Mini
    struct EmbassyTimeSource;
    impl TimeSource for EmbassyTimeSource {
        type Instant = Instant;

        fn now(&self) -> Self::Instant {
            Instant::now()
        }

        fn is_elapsed(&self, since: Self::Instant, timeout_ms: u64) -> bool {
            Instant::now().duration_since(since)
                >= Duration::from_millis(timeout_ms)
        }
    }

    // Create the DFPlayer Mini instance
    let mut dfplayer = match DfPlayer::try_new(
        &mut uart,
        feedback_enable,
        timeout_ms,
        EmbassyTimeSource,
        delay,
        reset_duration_override,
    )
    .await
    {
        Ok(dfplayer) => dfplayer,
        Err(e) => {
            error!("Error initializing DFPlayer Mini: {}", e);
            return;
        }
    };

    // Now we can start sending commands to the DFPlayer Mini
    // Set the volume to 5, because we tinker late at night and don't want to wake up the neighbors
    info!("Setting volume to 2");
    match dfplayer.set_volume(2).await {
        Ok(_) => info!("Volume set successfully"),
        Err(e) => error!("Failed to set volume: {}", e),
    }

    // Set the equalizer to rock
    info!("Setting equalizer to rock");
    match dfplayer.set_equalizer(Equalizer::Rock).await {
        Ok(_) => info!("Equalizer set successfully"),
        Err(e) => error!("Failed to set equalizer: {}", e),
    }

    // Try to play the first track
    info!("Attempting to play first track");
    match dfplayer.play(1).await {
        Ok(_) => info!("Play command sent successfully"),
        Err(e) => error!("Failed to play track: {}", e),
    }

    // Main loop
    info!("Entering main loop");
    loop {
        // Check if the DFPlayer is busy
        let is_busy = busy.is_low();
        info!("DFPlayer Mini is busy: {}", is_busy);

        // If not busy, try playing a track
        if !is_busy {
            info!("Device idle, sending next track command");
            match dfplayer.next().await {
                Ok(_) => info!("Next track command sent successfully"),
                Err(e) => error!("Failed to send next track command: {}", e),
            }
        }

        info!("Waiting 5s before next check");
        Timer::after(Duration::from_millis(5000)).await;
    }
}
