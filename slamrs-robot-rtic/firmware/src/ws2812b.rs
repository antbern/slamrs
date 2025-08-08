use rp2040_hal::{
    fugit::HertzU32,
    gpio::{bank0::Gpio11, FunctionPio1, Pin, PullDown},
    pac,
    pio::{Buffers, PIOExt, PinDir, Running, ShiftDirection, StateMachine, Tx, PIO, SM0},
};

pub struct WS2812B {
    _pio: PIO<pac::PIO1>,
    _sm: StateMachine<(pac::PIO1, SM0), Running>,
    _tx: Tx<(pac::PIO1, SM0)>,
    _pin: Pin<Gpio11, FunctionPio1, PullDown>,
}

impl WS2812B {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        pio1: pac::PIO1,
        resets: &mut pac::RESETS,
        pin: Pin<Gpio11, FunctionPio1, PullDown>,
        rgbw: bool,
        sys_freq: HertzU32,
    ) -> Self {
        // Setup the PIO do output WS2812B data signals
        // Taken from `pico-examples`: https://github.com/raspberrypi/pico-examples/blob/1c5d9aa567598e6e3eadf6d7f2d8a9342b44dab4/pio/ws2812/ws2812.pio

        // Create and start the I2S pio program
        let program = pio_proc::pio_asm!(
            ".side_set 1",
            "",
            ".define public T1 2",
            ".define public T2 5",
            ".define public T3 3",
            "",
            ".wrap_target",
            "bitloop:",
            "    out x, 1       side 0 [T3 - 1] ; Side-set still takes place when instruction stalls",
            "    jmp !x do_zero side 1 [T1 - 1] ; Branch on the bit we shifted out. Positive pulse",
            "do_one:",
            "    jmp  bitloop   side 1 [T2 - 1] ; Continue driving high, for a long pulse",
            "do_zero:",
            "    nop            side 0 [T2 - 1] ; Or drive low, for a short pulse",
            ".wrap",
        );

        // frequency is hard-coded 800kHz
        let freq = 800000.0;

        let (mut pio, sm0, _, _, _) = pio1.split(resets);

        // install the program into PIO instruction memory
        let installed = pio.install(&program.program).unwrap();

        // calculate the clock divider values
        let cycles_per_bit =
            program.public_defines.T1 + program.public_defines.T2 + program.public_defines.T3;
        let div = sys_freq.to_Hz() as f32 / (freq * cycles_per_bit as f32);

        let (div_int, div_frac) = pio_calculate_clkdiv_from_float(div);

        // configure the state machine
        let (mut sm, _rx, tx) = rp2040_hal::pio::PIOBuilder::from_installed_program(installed)
            .side_set_pin_base(pin.id().num)
            .clock_divisor_fixed_point(div_int, div_frac)
            .out_shift_direction(ShiftDirection::Left)
            .autopull(true)
            .pull_threshold(if rgbw { 32 } else { 24 })
            .buffers(Buffers::OnlyTx)
            .build(sm0);

        sm.set_pindirs([(pin.id().num, PinDir::Output)]);

        // attach the GPIO to the PIO peripheral
        //let pin: Pin<I, Function<P>> = pin.into_mode();

        // enable
        let sm = sm.start();

        Self {
            _pio: pio,
            _sm: sm,
            _tx: tx,
            _pin: pin,
        }
    }

    pub fn set_color(&mut self, r: u8, g: u8, b: u8) {
        // write the data to the PIO buffer
        self._tx.write(urgb_u32(r, g, b) << 8);
    }
}

fn urgb_u32(r: u8, g: u8, b: u8) -> u32 {
    ((r as u32) << 8) | ((g as u32) << 16) | (b as u32)
}

// from https://github.com/raspberrypi/pico-sdk/blob/2ccab115de0d42d31d6611cca19ef0cd0d2ccaa7/src/rp2_common/hardware_pio/include/hardware/pio.h#L223
fn pio_calculate_clkdiv_from_float(div: f32) -> (u16, u8) {
    // debug_assert!(div >= 1.0 && div <= 65536.0);
    // TODO: should really test more here
    // something like https://github.com/rp-rs/ws2812-pio-rs/blob/0b0dbb035e220954300475784ac6fe3ab7f4ab9b/src/lib.rs#L116
    let div_int = div as u16;

    // if div_int == 65536 {
    //     div_int = 0;
    // }

    let div_frac = if div_int == 0 {
        0
    } else {
        ((div - div_int as f32) * (1u16 << 8) as f32) as u8
    };

    (div_int, div_frac)
}
