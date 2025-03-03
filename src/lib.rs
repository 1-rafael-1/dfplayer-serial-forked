//! A no_std async library for interfacing with DFPlayer Mini MP3 modules
//!
//! This crate provides an async interface to control DFPlayer Mini MP3 modules
//! using embedded-hal-async compatible serial interfaces. It handles the binary
//! protocol, error checking, and timeout management required when communicating
//! with these devices.
//!
//! ## Features
//!
//! - Async/await API for embedded systems
//! - Full command support for DFPlayer Mini and compatible modules
//! - Proper error handling and timeout management
//! - no_std compatible
//! - Robust initialization sequence with fallback mechanisms
//! - Efficient non-blocking I/O patterns for embedded environments
//!
//! ## Example
//!
//! ```rust,no_run
//! use dfplayer_serial::{DfPlayer, PlayBackSource, TimeSource};
//! use embassy_time::{Duration, Instant, Delay};
//!
//! // Define a time source for the DFPlayer
//! struct MyTimeSource;
//! impl TimeSource for MyTimeSource {
//!     type Instant = Instant;
//!     fn now(&self) -> Self::Instant { Instant::now() }
//!     fn is_elapsed(&self, since: Self::Instant, timeout_ms: u64) -> bool {
//!         Instant::now().duration_since(since) >= Duration::from_millis(timeout_ms)
//!     }
//! }
//!
//! // In your async function:
//! async fn example(mut uart: impl embedded_io_async::Read + embedded_io_async::Write + embedded_io_async::ReadReady) {
//!     let mut dfplayer = DfPlayer::try_new(
//!         &mut uart,      // UART port (9600 baud, 8N1)
//!         false,          // feedback_enable
//!         1000,           // timeout_ms
//!         MyTimeSource,   // time source
//!         Delay,          // delay provider
//!         None,           // reset_duration_override
//!     ).await.expect("Failed to initialize DFPlayer");
//!
//!     // Play first track
//!     dfplayer.play(1).await.expect("Failed to play track");
//! }
//! ```
//!
//! This crate optionally supports logging via the defmt framework.
//! Enable the "defmt" feature to activate logging.

#![no_std]

use embedded_hal_async::delay::DelayNs;
use embedded_io_async::{Read, ReadReady, Write};

#[cfg(feature = "defmt")]
use defmt::{Debug2Format, info};

// Protocol constants
const START_BYTE: u8 = 0x7E;
const END_BYTE: u8 = 0xEF;
const VERSION: u8 = 0xFF;
const MSG_LEN: u8 = 0x06;
const DATA_FRAME_SIZE: usize = 10;

// Message byte indices
const INDEX_VERSION: usize = 1;
const INDEX_CMD: usize = 3;
const INDEX_FEEDBACK_ENABLE: usize = 4;
const INDEX_PARAM_H: usize = 5;
const INDEX_PARAM_L: usize = 6;
const INDEX_CHECKSUM_H: usize = 7;
const INDEX_CHECKSUM_L: usize = 8;

/// Minimal time provider trait for timeout tracking. Implement this for your platform.
pub trait TimeSource {
    /// Monotonic time point type
    type Instant: Copy + Clone + PartialEq + PartialOrd;

    /// Get the current time
    fn now(&self) -> Self::Instant;

    /// Check if a timeout has occurred
    fn is_elapsed(&self, since: Self::Instant, timeout_ms: u64) -> bool;
}

/// Represents available media sources on the DFPlayer
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Source {
    /// Internal USB flash storage
    USBFlash = 0b001,
    /// SD card inserted in the module
    SDCard = 0b010,
    /// External USB device connected to the module
    USBHost = 0b100,
}

impl TryFrom<u8> for Source {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b001 => Ok(Source::USBFlash),
            0b010 => Ok(Source::SDCard),
            0b100 => Ok(Source::USBHost),
            _ => Err(()),
        }
    }
}

/// Error codes reported by the DFPlayer module
#[derive(Debug, Clone, Copy)]
#[repr(u8)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ModuleError {
    /// Module is currently busy
    Busy = 1,
    /// Module is in sleep mode
    Sleeping = 2,
    /// Serial receive error occurred
    SerialRxError = 3,
    /// Checksum validation failed
    Checksum = 4,
    /// Requested track is out of valid range
    TrackNotInScope = 5,
    /// Track was not found on the media
    TrackNotFound = 6,
    /// Error inserting file/track
    InsertionError = 7,
    /// Module entering sleep mode
    EnterSleep = 8,
}

impl TryFrom<u8> for ModuleError {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            1 => Ok(ModuleError::Busy),
            2 => Ok(ModuleError::Sleeping),
            3 => Ok(ModuleError::SerialRxError),
            4 => Ok(ModuleError::Checksum),
            5 => Ok(ModuleError::TrackNotInScope),
            6 => Ok(ModuleError::TrackNotFound),
            7 => Ok(ModuleError::InsertionError),
            8 => Ok(ModuleError::EnterSleep),
            _ => Err(()),
        }
    }
}

/// Errors that can occur when operating the DFPlayer
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error<SerialError> {
    /// Initialization failed
    Init,
    /// Serial port communication error
    SerialPort(SerialError),
    /// Delay implementation failed
    DelayError,
    /// Command was not acknowledged when feedback was enabled
    FailedAck,
    /// Connection to the module was lost
    Connection,
    /// Error reported by the module itself
    ModuleError(ModuleError),
    /// Unknown error occurred
    Unknown,
    /// Received message was corrupted or incomplete
    BrokenMessage,
    /// Operation timed out
    UserTimeout,
    /// Command parameter was invalid
    BadParameter,
}

/// Data structure representing a message to/from the DFPlayer
#[derive(PartialEq, Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct MessageData {
    command: Command,
    param_h: u8,
    param_l: u8,
}

impl MessageData {
    /// Create a new message with the specified command and parameters
    pub const fn new(command: Command, param_h: u8, param_l: u8) -> Self {
        Self {
            command,
            param_h,
            param_l,
        }
    }
}

/// Message used for acknowledging commands
const ACK_MESSAGE_DATA: MessageData =
    MessageData::new(Command::NotifyReply, 0, 0);

/// Commands supported by the DFPlayer module
#[repr(u8)]
#[derive(PartialEq, Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Command {
    /// Play next file
    Next = 0x01,
    /// Play previous file
    Previous = 0x02,
    /// Specify Track number to play (1-2999)
    PlayTrack = 0x03,
    /// Increase Volume
    VolumeUp = 0x04,
    /// Decrease Volume
    VolumeDown = 0x05,
    /// Set specific volume value, 0-30
    SetVolume = 0x06,
    /// Select equalizer
    SetEQ = 0x07,
    /// Select track to play in loop
    PlayLoopTrack = 0x08,
    /// Select the Playback source (USDB/SD)
    SetPlaybackSource = 0x09,
    /// Enter Sleep/StandBy Mode
    EnterSleepMode = 0x0A,
    /// Normal mode per DFRobot, but it's reported it does nothing
    EnterNormalMode = 0x0B,
    /// Reset the device
    Reset = 0x0C,
    /// Start Playback
    Play = 0x0D,
    /// Pause Current Playback
    Pause = 0x0E,
    /// Specify Track to play in a folder, 99 folders 255 tracks each max
    PlayTrackInFolder = 0x0F,
    /// Configure the audio amplifier gain settings, MSB enables amp, LS sets
    /// gain 0-31
    ConfigAudioAmp = 0x10,
    /// Play all tracks in a loop
    PlayLoopAll = 0x11,
    /// Play track number in MP3 folder, max 65536 tracks, 3000 recommended max
    PlayTrackInMp3Folder = 0x12,
    /// Play track number in ADVERT folder, max 3000 tracks
    StartAdvertisement = 0x13,
    /// Play track of large folder, 1-3000 valid names
    PlayTrackLargeFolder = 0x14,
    /// Stop playing ADVERT track if one is active
    StopAdvertisement = 0x15,
    /// Stop all playback including advertisement
    Stop = 0x16,
    /// Play tracks from a folder on repeat, max 99 folders, 255 files each
    PlayLoopFolder = 0x17,
    /// Play random tracks from all media available in the current source
    PlayRandom = 0x18,
    /// If a track is playing, control loop playback enable (0x1 enable, 0x0 disable)
    LoopCurrentTrack = 0x19,
    /// Control whether the DAC is powered on or off
    SetDAC = 0x1a,
    /// Only sent by module when media is connected
    NotifyPushMedia = 0x3A,
    /// Only sent by module when media is removed
    NotifyPullOutMedia = 0x3B,
    /// Only sent by module when track in USB Flash finished playing
    NotifyFinishTrackUSBFlash = 0x3C,
    /// Only sent by module when track in SD card finished playing
    NotifyFinishTrackSD = 0x3D,
    /// Only sent by module when track in USB Host link stopped playing
    NotifyFinishTrackUSBHost = 0x3E,
    /// List the available sources. For the DFPlayer Mini, it's essentially
    /// only the SD card
    QueryAvailableSources = 0x3F,
    /// Only sent by module when an error occurs
    NotifyError = 0x40,
    /// Sent as ACK response when feedback is enabled
    NotifyReply = 0x41,
    /// Returns status of module
    QueryStatus = 0x42,
    /// Returns current volume setting
    QueryVolume = 0x43,
    /// Returns current EQ setting
    QueryEQ = 0x44,
    /// Returns current playback mode
    ReservedQueryPlaybackMode = 0x45,
    /// Returns software version
    ReservedQuerySwVersion = 0x46,
    /// Returns number of tracks on USB storage
    QueryTrackCntUSB = 0x47,
    /// Returns number of tracks on SD card
    QueryTrackCntSD = 0x48,
    /// Returns number of tracks on PC
    ReservedQueryTrackCntPC = 0x49,
    /// Query the keep-on setting
    ReservedQueryKeepOn = 0x4A,
    /// Returns current track number on USB flash
    QueryCurrentTrackUSBFlash = 0x4B,
    /// Returns current track number on SD card
    QueryCurrentTrackSD = 0x4C,
    /// Returns current track number on USB host
    QueryCurrentTrackUSBHost = 0x4D,
    /// Returns number of tracks in current folder
    QueryFolderTrackCnt = 0x4E,
    /// Returns number of folders available
    QueryFolderCnt = 0x4F,
}

impl TryFrom<u8> for Command {
    type Error = ();
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0x01 => Ok(Command::Next),
            0x02 => Ok(Command::Previous),
            0x03 => Ok(Command::PlayTrack),
            0x04 => Ok(Command::VolumeUp),
            0x05 => Ok(Command::VolumeDown),
            0x06 => Ok(Command::SetVolume),
            0x07 => Ok(Command::SetEQ),
            0x08 => Ok(Command::PlayLoopTrack),
            0x09 => Ok(Command::SetPlaybackSource),
            0x0A => Ok(Command::EnterSleepMode),
            0x0B => Ok(Command::EnterNormalMode),
            0x0C => Ok(Command::Reset),
            0x0D => Ok(Command::Play),
            0x0E => Ok(Command::Pause),
            0x0F => Ok(Command::PlayTrackInFolder),
            0x10 => Ok(Command::ConfigAudioAmp),
            0x11 => Ok(Command::PlayLoopAll),
            0x12 => Ok(Command::PlayTrackInMp3Folder),
            0x13 => Ok(Command::StartAdvertisement),
            0x14 => Ok(Command::PlayTrackLargeFolder),
            0x15 => Ok(Command::StopAdvertisement),
            0x16 => Ok(Command::Stop),
            0x17 => Ok(Command::PlayLoopFolder),
            0x18 => Ok(Command::PlayRandom),
            0x19 => Ok(Command::LoopCurrentTrack),
            0x1A => Ok(Command::SetDAC),
            0x3A => Ok(Command::NotifyPushMedia),
            0x3B => Ok(Command::NotifyPullOutMedia),
            0x3C => Ok(Command::NotifyFinishTrackUSBFlash),
            0x3D => Ok(Command::NotifyFinishTrackSD),
            0x3E => Ok(Command::NotifyFinishTrackUSBHost),
            0x3F => Ok(Command::QueryAvailableSources),
            0x40 => Ok(Command::NotifyError),
            0x41 => Ok(Command::NotifyReply),
            0x42 => Ok(Command::QueryStatus),
            0x43 => Ok(Command::QueryVolume),
            0x44 => Ok(Command::QueryEQ),
            0x45 => Ok(Command::ReservedQueryPlaybackMode),
            0x46 => Ok(Command::ReservedQuerySwVersion),
            0x47 => Ok(Command::QueryTrackCntUSB),
            0x48 => Ok(Command::QueryTrackCntSD),
            0x49 => Ok(Command::ReservedQueryTrackCntPC),
            0x4A => Ok(Command::ReservedQueryKeepOn),
            0x4B => Ok(Command::QueryCurrentTrackUSBFlash),
            0x4C => Ok(Command::QueryCurrentTrackSD),
            0x4D => Ok(Command::QueryCurrentTrackUSBHost),
            0x4E => Ok(Command::QueryFolderTrackCnt),
            0x4F => Ok(Command::QueryFolderCnt),
            _ => Err(()),
        }
    }
}

/// Equalizer settings available on the DFPlayer
#[repr(u8)]
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Equalizer {
    /// Normal (flat) equalizer setting
    Normal = 0x0,
    /// Pop music equalizer preset
    Pop = 0x1,
    /// Rock music equalizer preset
    Rock = 0x2,
    /// Jazz music equalizer preset
    Jazz = 0x3,
    /// Classical music equalizer preset
    Classic = 0x4,
    /// Bass boost equalizer preset
    Bass = 0x5,
}

/// Playback modes supported by the DFPlayer
#[repr(u8)]
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PlayBackMode {
    /// Repeat all tracks
    Repeat = 0x0,
    /// Repeat tracks in current folder
    FolderRepeat = 0x1,
    /// Repeat single track
    SingleRepeat = 0x2,
    /// Play tracks in random order
    Random = 0x3,
}

/// Media sources supported by the DFPlayer
#[repr(u8)]
#[derive(Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PlayBackSource {
    /// USB storage device
    USB = 0x0,
    /// SD card
    SDCard = 0x1,
    /// Auxiliary input
    Aux = 0x2,
    /// Sleep mode (no source)
    Sleep = 0x3,
    /// Flash memory
    Flash = 0x4,
}

/// Calculate the checksum for a DFPlayer message
///
/// The checksum is calculated by summing all bytes from version to parameters
/// and then taking the two's complement of the sum.
pub fn checksum(buffer: &[u8]) -> u16 {
    let mut checksum = 0;
    for &b in buffer {
        checksum += u16::from(b);
    }
    if buffer[2] == 0x0 {
        checksum += 2
    };
    0u16.wrapping_sub(checksum)
}

/// Main driver for interfacing with DFPlayer Mini modules
pub struct DfPlayer<'a, S, T, D>
where
    S: Read + Write + ReadReady,
    T: TimeSource,
    D: DelayNs,
{
    port: &'a mut S,
    feedback_enable: bool,
    last_command: MessageData,
    last_response: MessageData,
    last_cmd_acknowledged: bool,
    timeout_ms: u64,
    time_source: T,
    delay: D,
}

/// Structure for interacting with the device
impl<'a, S, T, D> DfPlayer<'a, S, T, D>
where
    S: Read + Write + ReadReady,
    T: TimeSource,
    D: DelayNs,
{
    /// Create a new DFPlayer interface
    ///
    /// This initializes the driver and performs a robust startup sequence for the DFPlayer module.
    /// The serial port must be configured with 9600 baud, 8N1 format before calling this function.
    ///
    /// The initialization sequence:
    /// 1. Clears any pending data in the receive buffer
    /// 2. Sends a reset command and waits for the device to restart
    /// 3. Configures SD card as the default media source
    /// 4. Sets the volume to a moderate level (15 out of 30)
    ///
    /// The function will attempt to continue even if certain initialization steps fail,
    /// making it more resilient to communication issues common with these modules.
    ///
    /// # Arguments
    /// * `port` - Serial port connected to the DFPlayer module
    /// * `feedback_enable` - Whether to enable command acknowledgement (set to false if you're having reliability issues)
    /// * `timeout_ms` - Timeout for operations in milliseconds
    /// * `time_source` - Source of time for timeout tracking
    /// * `delay` - Delay provider for timing operations
    /// * `reset_duration_override` - Optional override for reset delay duration (ms)
    pub async fn try_new(
        port: &'a mut S,
        feedback_enable: bool,
        timeout_ms: u64,
        time_source: T,
        delay: D,
        reset_duration_override: Option<u64>,
    ) -> Result<Self, Error<S::Error>> {
        let mut player = Self {
            port,
            feedback_enable,
            last_command: MessageData::new(Command::EnterNormalMode, 0, 0),
            last_response: MessageData::new(Command::EnterNormalMode, 0, 0),
            last_cmd_acknowledged: false,
            timeout_ms,
            time_source,
            delay,
        };

        // Clear any pending data in the receive buffer first
        #[cfg(feature = "defmt")]
        info!("Clearing initial receive buffer");
        let _ = player.clear_receive_buffer().await;

        // Send reset command with longer timeout for initialization
        #[cfg(feature = "defmt")]
        info!("Sending reset command");

        // Store original timeout and use a longer one for reset
        let original_timeout = player.timeout_ms;
        player.timeout_ms = 2000; // Longer timeout for reset

        // Send the reset command
        let reset_result = player
            .send_command(MessageData::new(Command::Reset, 0, 0))
            .await;

        // Wait for device reset regardless of command result
        let wait_ms = reset_duration_override.unwrap_or(1500);
        #[cfg(feature = "defmt")]
        info!("Waiting {}ms for device reset", wait_ms);
        player.delay.delay_ms(wait_ms as u32).await;

        // Clear any data that might have arrived during reset
        let _ = player.clear_receive_buffer().await;

        // Restore original timeout
        player.timeout_ms = original_timeout;

        // Continue even if reset command had issues
        if let Err(e) = reset_result {
            #[cfg(feature = "defmt")]
            info!("Reset error: {:?} - continuing anyway", Debug2Format(&e));
        }

        // Configure SD card as the default media source
        #[cfg(feature = "defmt")]
        info!("Setting playback source to SD card");

        // Try to select SD card source, but continue even if it fails
        let source_result =
            player.set_playback_source(PlayBackSource::SDCard).await;
        if let Err(e) = source_result {
            #[cfg(feature = "defmt")]
            info!(
                "Source select warning: {:?} - continuing anyway",
                Debug2Format(&e)
            );
        }

        // Set initial volume to a moderate level
        #[cfg(feature = "defmt")]
        info!("Setting initial volume");

        // Set volume to 15 (half scale)
        let vol_result = player.set_volume(15).await;
        if let Err(e) = vol_result {
            #[cfg(feature = "defmt")]
            info!(
                "Volume set warning: {:?} - continuing anyway",
                Debug2Format(&e)
            );
        }

        #[cfg(feature = "defmt")]
        info!("DFPlayer initialization complete");

        Ok(player)
    }

    /// Read and process a message from the DFPlayer module
    ///
    /// This function handles the binary protocol parsing, message validation,
    /// and stores the last response. It uses a robust state machine to assemble
    /// complete messages from potentially fragmented reads, with proper timeout
    /// handling and error recovery.
    ///
    /// Special handling is provided for reset commands, which often don't receive responses.
    ///
    /// Returns `Ok(())` if a valid message was received and processed, or an error
    /// otherwise. If a module error response was received, returns that specific error.
    pub async fn read_last_message(&mut self) -> Result<(), Error<S::Error>> {
        let timeout_start = self.time_source.now();

        // State tracking for message assembly
        let mut current_index = 0;
        let mut message = [0u8; DATA_FRAME_SIZE];
        let mut receive_buffer = [0u8; 32];

        // Continue trying to read until timeout
        while !self.time_source.is_elapsed(timeout_start, self.timeout_ms) {
            // Try to read data with graceful fallback if read_ready isn't reliable
            let bytes_read = match self.port.read_ready() {
                Ok(true) => match self.port.read(&mut receive_buffer).await {
                    Ok(n) => n,
                    Err(e) => {
                        #[cfg(feature = "defmt")]
                        info!("Read error, will retry: {:?}", Debug2Format(&e));
                        self.delay.delay_ms(10).await;
                        continue;
                    }
                },
                // If read_ready says not ready or errors, we'll still try a read
                // This helps with UART implementations where read_ready isn't reliable
                _ => {
                    match self.port.read(&mut receive_buffer).await {
                        Ok(n) => {
                            if n == 0 {
                                // No data available, wait briefly and retry
                                self.delay.delay_ms(10).await;
                                continue;
                            }
                            n
                        }
                        Err(e) => {
                            #[cfg(feature = "defmt")]
                            info!("Read error: {:?}", Debug2Format(&e));
                            self.delay.delay_ms(10).await;
                            continue;
                        }
                    }
                }
            };

            #[cfg(feature = "defmt")]
            if bytes_read > 0 {
                info!(
                    "Read {} bytes: {:?}",
                    bytes_read,
                    &receive_buffer[..bytes_read]
                );
            }

            // Process each byte through our state machine
            for i in 0..bytes_read {
                let byte = receive_buffer[i];

                match current_index {
                    0 => {
                        // State 0: Looking for start byte (0x7E)
                        if byte == START_BYTE {
                            message[current_index] = byte;
                            current_index = 1;
                        }
                    }
                    9 => {
                        // State 9: We have 9 bytes and are looking for the end byte (0xEF)
                        message[current_index] = byte;

                        if byte == END_BYTE {
                            // Complete message received, validate and process
                            #[cfg(feature = "defmt")]
                            info!("Complete message: {:?}", message);

                            // Validate checksum
                            let read_checksum = ((message[7] as u16) << 8)
                                | (message[8] as u16);
                            let calc_checksum = checksum(&message[1..7]);

                            if read_checksum == calc_checksum {
                                // Valid message - extract command and parameters
                                if let Ok(cmd) = message[3].try_into() {
                                    self.last_response.command = cmd;
                                    self.last_response.param_h = message[5];
                                    self.last_response.param_l = message[6];

                                    // Check if this is an ACK message
                                    if self.last_response == ACK_MESSAGE_DATA {
                                        self.last_cmd_acknowledged = true;
                                    }

                                    // Check if this is an error response
                                    if self.last_response.command
                                        == Command::NotifyError
                                    {
                                        if let Ok(err) = self
                                            .last_response
                                            .param_l
                                            .try_into()
                                        {
                                            return Err(Error::ModuleError(
                                                err,
                                            ));
                                        } else {
                                            return Err(Error::Unknown);
                                        }
                                    }

                                    // Valid message processed successfully
                                    return Ok(());
                                }
                            } else {
                                #[cfg(feature = "defmt")]
                                info!(
                                    "Checksum mismatch: expected {:04X}, got {:04X}",
                                    calc_checksum, read_checksum
                                );
                            }
                        }

                        // Reset parser state whether valid or not
                        current_index = 0;
                    }
                    _ => {
                        // States 1-8: Collecting the message body
                        message[current_index] = byte;
                        current_index += 1;
                    }
                }
            }
        }

        // If we reached here, we timed out with no valid response
        #[cfg(feature = "defmt")]
        info!("Timeout waiting for response");

        // Special case for reset command - just return success
        if self.last_command.command == Command::Reset {
            #[cfg(feature = "defmt")]
            info!("Ignoring timeout for reset command");
            return Ok(());
        }

        // Timeout error for other commands
        Err(Error::UserTimeout)
    }

    /// Send a command to the DFPlayer module
    ///
    /// This constructs a properly formatted message, sends it to the device,
    /// and then waits for a response or acknowledgement if feedback is enabled.
    ///
    /// The method calculates the appropriate checksum and handles all aspects of
    /// the binary communication protocol.
    ///
    /// # Arguments
    /// * `command_data` - The command and parameters to send
    pub async fn send_command(
        &mut self,
        command_data: MessageData,
    ) -> Result<(), Error<S::Error>> {
        // Format the command message according to protocol
        let mut out_buffer = [
            START_BYTE, VERSION, MSG_LEN, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
            END_BYTE,
        ];

        // Set feedback flag if enabled
        if self.feedback_enable {
            out_buffer[INDEX_FEEDBACK_ENABLE] = 0x1;
        }

        // Set command and parameters
        out_buffer[INDEX_CMD] = command_data.command as u8;
        out_buffer[INDEX_PARAM_H] = command_data.param_h;
        out_buffer[INDEX_PARAM_L] = command_data.param_l;

        // Calculate and set checksum
        let checksum = checksum(&out_buffer[INDEX_VERSION..INDEX_CHECKSUM_H]);
        out_buffer[INDEX_CHECKSUM_H] = (checksum >> 8) as u8;
        out_buffer[INDEX_CHECKSUM_L] = checksum as u8;

        // Log the message being sent
        #[cfg(feature = "defmt")]
        info!("tx {}", out_buffer);

        // Send the message to the device
        self.port
            .write_all(&out_buffer)
            .await
            .map_err(Error::SerialPort)?;

        // Store the command for reference
        self.last_command = command_data;

        // Reset acknowledgment flag before reading response
        self.last_cmd_acknowledged = false;

        // Wait for and process the response
        self.read_last_message().await?;

        // Check for acknowledgement if feedback is enabled
        if self.feedback_enable && (command_data.command != Command::Reset) {
            if !self.last_cmd_acknowledged {
                #[cfg(feature = "defmt")]
                info!(
                    "Expected ACK not received: {} {}",
                    self.last_command, self.last_response
                );
                return Err(Error::FailedAck);
            }
        } else {
            // Check for more data (needed for certain commands)
            if let Ok(true) = self.port.read_ready() {
                let _ = self.read_last_message().await;
            }
        }

        Ok(())
    }

    /// Clear any pending data in the receive buffer
    ///
    /// This method safely reads and discards any data waiting in the receive buffer
    /// without blocking indefinitely. It uses non-blocking I/O patterns to avoid
    /// hanging when no data is available.
    ///
    /// This is used during initialization and after certain commands to ensure
    /// a clean state for subsequent communications.
    async fn clear_receive_buffer(&mut self) -> Result<(), Error<S::Error>> {
        let mut buffer = [0u8; 32];
        let start = self.time_source.now();

        // Try reading a few times with timeouts
        for _ in 0..5 {
            // First check if data is available to avoid blocking
            let has_data = match self.port.read_ready() {
                Ok(ready) => ready,
                Err(_) => false, // Assume no data on error
            };

            if !has_data {
                // No data available, don't block
                #[cfg(feature = "defmt")]
                info!("No data to clear");

                // Short delay and continue
                self.delay.delay_ms(10).await;
                continue;
            }

            // Data is available, read and discard it
            match self.port.read(&mut buffer).await {
                Ok(0) => {
                    // No data was actually read
                    self.delay.delay_ms(10).await;
                }
                Ok(n) => {
                    #[cfg(feature = "defmt")]
                    info!("Cleared {} bytes: {:?}", n, &buffer[..n]);
                    // Short delay and try again
                    self.delay.delay_ms(10).await;
                }
                Err(e) => {
                    #[cfg(feature = "defmt")]
                    info!("Clear buffer read error: {:?}", Debug2Format(&e));
                    self.delay.delay_ms(10).await;
                }
            }

            // Check if we've been trying too long
            if self.time_source.is_elapsed(start, 100) {
                break;
            }
        }

        Ok(())
    }

    /// Play the next track
    ///
    /// Sends the command to play the next track in sequence.
    pub async fn next(&mut self) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(Command::Next, 0, 0))
            .await
    }

    /// Reset the DFPlayer module
    ///
    /// Sends a reset command to the module and waits for it to restart.
    /// The reset command typically causes the module to restart its processor
    /// and reinitialize its state.
    ///
    /// Special handling is provided for the reset command, as it often won't
    /// receive a response from the module.
    ///
    /// # Arguments
    /// * `reset_duration_override` - Optional override for reset delay duration (ms)
    pub async fn reset(
        &mut self,
        reset_duration_override: Option<u64>,
    ) -> Result<(), Error<S::Error>> {
        // Send reset command
        self.send_command(MessageData::new(Command::Reset, 0, 0))
            .await?;

        // Wait for the device to complete the reset
        let wait_ms = reset_duration_override.unwrap_or(1500); // Default timeout based on typical M16P init time
        self.delay.delay_ms(wait_ms as u32).await;

        // Try to read a response (though one may not come after reset)
        let _ = self.read_last_message().await;

        Ok(())
    }

    /// Set the media source for playback
    ///
    /// Configures which media source the DFPlayer should use for audio files.
    /// This method includes an additional delay after sending the command to
    /// allow the module time to switch sources and initialize the file system.
    ///
    /// # Arguments
    /// * `playback_source` - The media source to use (SD card, USB, etc.)
    pub async fn set_playback_source(
        &mut self,
        playback_source: PlayBackSource,
    ) -> Result<(), Error<S::Error>> {
        // Send the command to set playback source
        let cmd_result = self
            .send_command(MessageData::new(
                Command::SetPlaybackSource,
                0,
                playback_source as u8,
            ))
            .await;

        // Wait for the module to initialize the source
        // This delay is needed regardless of command success
        self.delay.delay_ms(200).await;

        cmd_result
    }

    /// Set the volume level (0-30)
    ///
    /// Configures the output volume of the DFPlayer module.
    /// Valid range is 0 (silent) to 30 (maximum volume).
    ///
    /// # Arguments
    /// * `volume` - Volume level from 0 (silent) to 30 (maximum)
    ///
    /// # Errors
    /// Returns `Error::BadParameter` if the volume is greater than 30.
    pub async fn set_volume(
        &mut self,
        volume: u8,
    ) -> Result<(), Error<S::Error>> {
        if volume > 30 {
            return Err(Error::BadParameter);
        }
        self.send_command(MessageData::new(Command::SetVolume, 0, volume))
            .await
    }

    /// Play a specific track by its index number
    ///
    /// Starts playback of a track by its numerical index.
    /// Track numbers typically start at 1, not 0.
    ///
    /// # Arguments
    /// * `track` - Track number (1-2999)
    ///
    /// # Errors
    /// Returns `Error::BadParameter` if the track number is greater than 2999.
    pub async fn play(&mut self, track: u16) -> Result<(), Error<S::Error>> {
        if track > 2999 {
            return Err(Error::BadParameter);
        }
        self.send_command(MessageData::new(
            Command::PlayTrack,
            (track >> 8) as u8,
            track as u8,
        ))
        .await
    }

    /// Set the equalizer mode
    ///
    /// Configures the audio equalizer preset on the DFPlayer.
    ///
    /// # Arguments
    /// * `equalizer` - Equalizer preset to use
    pub async fn set_equalizer(
        &mut self,
        equalizer: Equalizer,
    ) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(Command::SetEQ, 0, equalizer as u8))
            .await
    }

    /// Set whether to loop all tracks
    ///
    /// Enables or disables looping through all tracks.
    ///
    /// # Arguments
    /// * `enable` - Whether to enable looping of all tracks
    pub async fn set_loop_all(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(
            Command::PlayLoopAll,
            0,
            if enable { 1 } else { 0 },
        ))
        .await
    }

    /// Pause the current playback
    ///
    /// Pauses any currently playing track.
    pub async fn pause(&mut self) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(Command::Pause, 0, 0))
            .await
    }

    /// Resume playback
    ///
    /// Resumes playback of a paused track.
    pub async fn resume(&mut self) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(Command::Play, 0, 0))
            .await
    }

    /// Play the previous track
    ///
    /// Plays the track before the current one in sequence.
    pub async fn previous(&mut self) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(Command::Previous, 0, 0))
            .await
    }

    /// Stop all playback
    ///
    /// Stops any current playback, including advertisements.
    pub async fn stop(&mut self) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(Command::Stop, 0, 0))
            .await
    }

    /// Play a track from a specific folder
    ///
    /// The DFPlayer supports organizing tracks in folders.
    /// This command plays a specific track from a specific folder.
    ///
    /// # Arguments
    /// * `folder` - Folder number (1-99)
    /// * `track` - Track number within the folder (1-255)
    ///
    /// # Errors
    /// Returns `Error::BadParameter` if parameters are out of range.
    pub async fn play_from_folder(
        &mut self,
        folder: u8,
        track: u8,
    ) -> Result<(), Error<S::Error>> {
        if folder == 0 || folder > 99 || track == 0 {
            return Err(Error::BadParameter);
        }

        self.send_command(MessageData::new(
            Command::PlayTrackInFolder,
            folder,
            track,
        ))
        .await
    }

    /// Play tracks in random order
    ///
    /// Starts playback in random order from the current source.
    pub async fn play_random(&mut self) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(Command::PlayRandom, 0, 0))
            .await
    }

    /// Set whether to loop the current track
    ///
    /// Enables or disables looping of the currently playing track.
    ///
    /// # Arguments
    /// * `enable` - Whether to enable looping of the current track
    pub async fn set_loop_current_track(
        &mut self,
        enable: bool,
    ) -> Result<(), Error<S::Error>> {
        self.send_command(MessageData::new(
            Command::LoopCurrentTrack,
            0,
            if enable { 1 } else { 0 },
        ))
        .await
    }

    /// Query the total number of tracks on the SD card
    ///
    /// Returns the number of tracks on the SD card or an error.
    pub async fn query_tracks_sd(&mut self) -> Result<u16, Error<S::Error>> {
        self.send_command(MessageData::new(Command::QueryTrackCntSD, 0, 0))
            .await?;

        // Extract the track count from the response
        let count = ((self.last_response.param_h as u16) << 8)
            | (self.last_response.param_l as u16);
        Ok(count)
    }

    /// Query the current volume setting
    ///
    /// Returns the current volume level (0-30) or an error.
    pub async fn query_volume(&mut self) -> Result<u8, Error<S::Error>> {
        self.send_command(MessageData::new(Command::QueryVolume, 0, 0))
            .await?;

        Ok(self.last_response.param_l)
    }

    /// Query the current equalizer setting
    ///
    /// Returns the current equalizer setting or an error.
    pub async fn query_eq(&mut self) -> Result<u8, Error<S::Error>> {
        self.send_command(MessageData::new(Command::QueryEQ, 0, 0))
            .await?;

        Ok(self.last_response.param_l)
    }
}
