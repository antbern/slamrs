//! Module for handling rotary encoders.
//! Basically a Rust port of https://github.com/adamgreen/QuadratureDecoder

use core::ptr::addr_of_mut;

use rp_pico::hal::{
    self,
    dma::single_buffer,
    gpio::{
        bank0::{Gpio21, Gpio22},
        FunctionPio0, Pin, PullDown,
    },
    pac,
    pio::{PIOBuilder, PIOExt},
};

/// Global variable to store the encoder value
/// This is safe because the DMA channel will only write to this location
/// and we will only read from it.
static mut ENCODER_VALUE: u32 = 0;

/// A simple DMA target that writes to a single memory location without incrementing
struct OverwriteTarget {
    address: *const u32,
}
impl OverwriteTarget {
    /// Create a new instance of the ReplacableTarget.
    ///
    /// Safety: The caller must ensure that the address is valid and points to a valid memory location.
    pub fn new(address: *mut u32) -> Self {
        Self { address }
    }
}
unsafe impl hal::dma::WriteTarget for OverwriteTarget {
    type TransmittedWord = u32;

    fn tx_treq() -> Option<u8> {
        None
    }

    fn tx_address_count(&mut self) -> (u32, u32) {
        (self.address as u32, u32::MAX)
    }

    fn tx_increment(&self) -> bool {
        false
    }
}

pub fn initialize_encoders(
    pio0: pac::PIO0,
    resets: &mut pac::RESETS,
    dma: hal::dma::Channel<hal::dma::CH0>,

    in_a: Pin<Gpio21, FunctionPio0, PullDown>,
    in_b: Pin<Gpio22, FunctionPio0, PullDown>,
) {
    let program = pio_proc::pio_file!("pio/encoder.pio");
    // Initialize and start PIO
    let (mut pio, sm0, _, _, _) = pio0.split(resets);
    let installed = pio.install(&program.program).unwrap();
    let (mut sm, pio_rx, _tx) = PIOBuilder::from_program(installed)
        .in_pin_base(in_a.id().num)
        .autopush(false)
        .push_threshold(32)
        .in_shift_direction(hal::pio::ShiftDirection::Left)
        .build(sm0);

    // The GPIO pin needs to be configured as inputs.
    sm.set_pindirs([
        (in_a.id().num, hal::pio::PinDir::Input),
        (in_b.id().num, hal::pio::PinDir::Input),
    ]);

    sm.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::SET {
            destination: pio::SetDestination::X,
            data: 0,
        },
        delay: 0,
        side_set: None,
    });
    sm.exec_instruction(pio::Instruction {
        operands: pio::InstructionOperands::MOV {
            destination: pio::MovDestination::Y,
            op: pio::MovOperation::None,
            source: pio::MovSource::PINS,
        },
        delay: 0,
        side_set: None,
    });

    // setup the DMA channel to continously read the PIO FIFO into a global variable
    // With some calculation, it seems the max transfers of 2^32 will not run out within 1000 years
    // if we assume 2400 updates / second (1 rev / second). So we should ony need to
    // start the DMA transfer once.
    single_buffer::Config::new(
        dma,
        pio_rx,
        OverwriteTarget::new(unsafe { addr_of_mut!(ENCODER_VALUE) }),
    )
    .start();

    sm.start();
}

pub fn get_encoder_value() -> i32 {
    // interpret the value as a signed integer
    (unsafe { ENCODER_VALUE }) as i32
}
