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

/// Setup PIO0 to act as I2S master. Difference between LRCK and BCK pins'
/// numebers must be exactly 1. E.g. BCK = 16, LRCK = 17
pub fn setup_i2s<I1, I2, I3, P>(
    pio0: PIO0,
    resets: &mut RESETS,
    bck_gpio: Pin<I1, FunctionNull, P>,
    lrck_gpio: Pin<I2, FunctionNull, P>,
    din_gpio: Pin<I3, FunctionNull, P>,
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
        "set y, 14 side 0b11",
        "left:",
            "out pins, 1 side 0b10",
            "jmp y--, left side 0b11",
        "out pins, 1 side 0b00",
        "set y, 14 side 0b01",
        "right:",
            "out pins, 1 side 0b00",
            "jmp y--, right side 0b01",
        "out pins, 1 side 0b10",
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

    let (mut sm, _, tx) = PIOBuilder::from_installed_program(installed)
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

    (sm.start(), tx)
}
