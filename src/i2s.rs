use pio::{Instruction, InstructionOperands, OutDestination};
use rp_pico::{
    hal::{
        self, gpio::{
            FunctionNull, FunctionPio0, Pin, PinId, PullNone, PullType, ValidFunction
        }, pio::{
            PIOBuilder, PIOExt, Running,
            StateMachine, Tx, SM0,
        }
    },
    pac::{PIO0, RESETS},
};

/// Setup PIO0 to act as I2S master. Difference between LRCK and BCK pin
/// numebers must be exactly 1. E.g. BCK = 16, LRCK = 17
pub fn setup_i2s<I1, I2, I3, P>(
    pio0: PIO0,
    resets: &mut RESETS,
    bclk_gpio: Pin<I1, FunctionNull, P>,
    lrck_gpio: Pin<I2, FunctionNull, P>,
    din_gpio: Pin<I3, FunctionNull, P>,
    clock_rate: usize,
    sample_rate: usize,
    bit_depth: u32,
) -> (StateMachine<(PIO0, SM0), Running>, Tx<(PIO0, SM0)>)
where
    I1: PinId + ValidFunction<FunctionPio0>,
    I2: PinId + ValidFunction<FunctionPio0>,
    I3: PinId + ValidFunction<FunctionPio0>,
    P: PullType,
{
    let (mut pio0, sm0, _, _, _) = pio0.split(resets);

    // Thanks malacalypse for PIO program which I find in his
    // rp2040_i2s_example GitHub repository.
    let program_with_defines = pio_proc::pio_asm!(
        "; This block also outputs the word clock (also called frame or LR clock) and",
        "; the bit clock.",
        ";",
        "; Set register x to (bit depth - 2) (e.g. for 24 bit audio, set to 22).",
        "; Note that if this is needed to be synchronous with the SCK module,",
        "; it is not possible to run 24-bit frames with an SCK of 256x fs. You must either",
        "; run SCK at 384x fs (if your codec permits this) or use 32-bit frames, which",
        "; work fine with 24-bit codecs.",
        "",
        ".side_set 2",
        "",
        "public entry_point:",
        ";                        /--- LRCLK",
        ";                        |/-- BCLK",
        "frameL:                ; ||",
        "mov x, y          side 0b00 ; start of Left frame",
        "pull noblock      side 0b01 ; One clock after edge change with no data",
        "dataL:",
        "out pins, 1       side 0b00",
        "jmp x-- dataL     side 0b01",
        "",
        "frameR:",
        "mov x, y          side 0b10",
        "pull noblock      side 0b11 ; One clock after edge change with no data",
        "dataR:",
        "out pins, 1       side 0b10",
        "jmp x-- dataR     side 0b11",
    );
    let program = program_with_defines.program;
    let installed = pio0.install(&program).unwrap();

    let bck_pin_id = bclk_gpio.id().num;
    let _bck: Pin<_, FunctionPio0, PullNone> = bclk_gpio.reconfigure();

    let lrck_pin_id = lrck_gpio.id().num;
    let _lrck: Pin<_, FunctionPio0, PullNone> = lrck_gpio.reconfigure();

    let din_pin_id = din_gpio.id().num;
    let _din: Pin<_, FunctionPio0, PullNone> = din_gpio.reconfigure();

    let (int, frac) = clock_divisor(clock_rate, sample_rate, bit_depth);
    let (mut sm, _, mut tx) = PIOBuilder::from_installed_program(installed)
        .clock_divisor_fixed_point(int, frac)
        .side_set_pin_base(bck_pin_id)
        .out_pins(din_pin_id, 1)
        .autopull(false)
        .out_shift_direction(hal::pio::ShiftDirection::Left)
        .build(sm0);

    sm.set_pindirs([
        (bck_pin_id, hal::pio::PinDir::Output),
        (lrck_pin_id, hal::pio::PinDir::Output),
        (din_pin_id, hal::pio::PinDir::Output),
    ]);

    (set_bit_depth(sm.start(), &mut tx, bit_depth), tx)
}

/// Sets the bit depth of an audio stream. For future.
pub fn set_bit_depth(
    i2s_sm: StateMachine<(PIO0, SM0), Running>,
    i2s_tx: &mut Tx<(PIO0, SM0)>,
    bit_depth: u32,
) -> StateMachine<(PIO0, SM0), Running> {
    while !i2s_tx.is_empty() {}

    let mut i2s_sm = i2s_sm.stop();
    i2s_tx.write(bit_depth - 2);    // As stated in PIO commentary

    i2s_sm.exec_instruction(Instruction {
        operands: InstructionOperands::PULL {
            if_empty: false,
            block: false,
        },
        delay: 0,
        side_set: Some(0b01),
    });
    i2s_sm.exec_instruction(Instruction {
        operands: InstructionOperands::OUT {
            destination: OutDestination::Y,
            bit_count: 32,
        },
        delay: 0,
        side_set: Some(0b00),
    });

    i2s_sm.start()
}

fn clock_divisor(
    clock_rate: usize,
    sample_rate: usize,
    bit_depth: u32,
) -> (u16, u8) {
    let bck_rate = sample_rate * bit_depth as usize * 2 * 2;
    let ratio = (clock_rate as f32) / (bck_rate as f32);
    (
        ratio as u16,
        (ratio * 256. % 256.) as u8,
    )
}
