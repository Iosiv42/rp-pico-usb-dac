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
    bck_gpio: Pin<I1, FunctionNull, P>,
    lrck_gpio: Pin<I2, FunctionNull, P>,
    din_gpio: Pin<I3, FunctionNull, P>,
    bit_depth: u32,
) -> (StateMachine<(PIO0, SM0), Running>, Tx<(PIO0, SM0)>)
where
    I1: PinId + ValidFunction<FunctionPio0>,
    I2: PinId + ValidFunction<FunctionPio0>,
    I3: PinId + ValidFunction<FunctionPio0>,
    P: PullType,
{
    let (mut pio0, sm0, _, _, _) = pio0.split(resets);

    let program_with_defines = pio_proc::pio_asm!(
        ".side_set 2",
        ".wrap_target",
        "mov x, y side 0b01",
        "left:",
            "out pins, 1 side 0b00",
            "jmp x--, left side 0b01",
        "out pins, 1 side 0b10",
        "mov x, y side 0b11",
        "right:",
            "out pins, 1 side 0b10",
            "jmp x--, right side 0b11",
        "out pins, 1 side 0b00",
        ".wrap",
    );
    let program = program_with_defines.program;
    let installed = pio0.install(&program).unwrap();

    let bck_pin_id = bck_gpio.id().num;
    let _bck: Pin<_, FunctionPio0, PullNone> = bck_gpio.reconfigure();

    let lrck_pin_id = lrck_gpio.id().num;
    let _lrck: Pin<_, FunctionPio0, PullNone> = lrck_gpio.reconfigure();

    let din_pin_id = din_gpio.id().num;
    let _din: Pin<_, FunctionPio0, PullNone> = din_gpio.reconfigure();

    let (mut sm, _, mut tx) = PIOBuilder::from_installed_program(installed)
        .clock_divisor_fixed_point(42, 248)
        .side_set_pin_base(bck_pin_id)
        .out_pins(din_pin_id, 1)
        .autopull(true)
        .out_shift_direction(hal::pio::ShiftDirection::Left)
        .build(sm0);

    sm.set_pindirs([
        (bck_pin_id, hal::pio::PinDir::Output),
        (lrck_pin_id, hal::pio::PinDir::Output),
        (din_pin_id, hal::pio::PinDir::Output),
    ]);

    (set_bit_depth(sm.start(), &mut tx, bit_depth), tx)
}

/// Sets the bit depth of audio stream. Note that PCM must be tightly packed
/// without any padding. I.e. it's not allowed for 24 bit depth to be
/// [123, 12, 55, 0]. It must be [123, 12, 55].
pub fn set_bit_depth(
    i2s_sm: StateMachine<(PIO0, SM0), Running>,
    i2s_tx: &mut Tx<(PIO0, SM0)>,
    bit_depth: u32,
) -> StateMachine<(PIO0, SM0), Running> {
    while !i2s_tx.is_empty() {}

    let mut i2s_sm = i2s_sm.stop();
    i2s_tx.write(bit_depth - 2);

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
