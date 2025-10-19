//! The LEDs are on the IO expander peripheral: [the TCA9554A](https://www.ti.com/product/TCA9554A).
//!  We therefore write a partial I2C driver for this device so we can easily work the LEDs.
//!
//!  Note that the device is reset to its default state by cycling the power supply and causing a power-on reset
//!  (from the data sheet). This means that once we flash a new program *and* cycle the power off then back on,
//!  we can see the device has reverted to its default state.

use embedded_hal::i2c::I2c;
use esp_idf_svc::hal::prelude::*;

//Baudrate Max clock frequency in KHz (depending if using standard or fast mode).
#[allow(unused)]
pub const BAUDRATE_STANDARD: KiloHertz = KiloHertz(100);
pub const BAUDRATE_FAST: KiloHertz = KiloHertz(400);

//Note I2c has the following definition:
// pub trait I2c<A: AddressMode = SevenBitAddress>: ErrorType  {..}
// So we don't need to specify SevenBitAddress here.

/// Note that this Register enum is also all possibilites for the command byte
/// as it is only sent for *write* operations
/// (see 8.6.2 Control Register and Command Byte in the Data Sheet).
/// When sent to the IO expander after the addressing byte, this byte tells the IO expander
/// we wish to write to the corresponding register.
/// All these registers are 8-bit.
///
/// If we wanted to read a register instead, we would simply use the
/// [I2c::write_read] method, which also matches the IO expander data sheet.
#[allow(unused)]
enum Register {
    InputPort = 0,
    OutputPort = 1,
    PolarityInversion = 2,
    Configuration = 3,
}

pub enum LEDs {
    Red,
    Blue,
}

pub struct LedDriver<I2C: I2c> {
    i2c: I2C,

    //Caching the state of the output port register is a convenience for flipping the LEDs.
    //It saves us having to actually read it from the IO expander to know the state of the LEDs.
    output_value: u8,
}

impl<I2C: I2c> LedDriver<I2C> {
    //Associated Constants

    ///From the [Korvo schematic]
    /// the I2C Address (of this io expander)：0'b 0111 000x .
    /// This address is specified in the **right-aligned** form as specified
    /// [here](https://docs.rs/embedded-hal/latest/src/embedded_hal/i2c.rs.html#296).
    ///
    /// [Korvo schematic]: https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-Korvo-2_V3.1.2_20240116.pdf
    const ADDR: u8 = 0b00111000;

    ///This is called P6 for the Korvo. On the IO expander it is pin 11.
    /// Push-pull design structure.
    const RED_LED_PIN_ON: u8 = 0b01_000000;

    ///This is called P7 for the Korvo. On the IO expander it is pin 12.
    /// Push-pull design structure.
    const BLUE_LED_PIN_ON: u8 = 0b1_0000000;

    ///The default value of the output port on power up.
    const DEFAULT_OUTPUT: u8 = 0b11111111;

    ///The byte in each register is of the form: P7 P6 P5 P4 P3 P2 P1 P0 (MSB -> LSB).
    /// So to clear P6 and P7 in the configuration register (sets the LEDs as output)
    /// we need to write this byte
    const SET_LEDS_OUTPUT: u8 = 0b00111111;

    ///Creates and initializes the LED driver.
    pub fn build(mut i2c: I2C) -> Self {
        //At power on, the IO expander device has all pins being inputs.
        // We set the LED pins as outputs.
        // We do this by clearing the relevant bits from the
        //Configuration register (which starts with all bits set).
        i2c.write(
            Self::ADDR,
            &[Register::Configuration as u8, Self::SET_LEDS_OUTPUT],
        )
        .expect("Setting LEDs as output should not fail");

        //Once we enable the LEDs pins as outputs, their default values on power-up will turn them on.
        //So we turn them off here.
        //This also ensures that after a reset (but not a power cycle),
        //we would be in a consistent state.

        let mut driver = Self {
            i2c,
            output_value: Self::DEFAULT_OUTPUT,
        };

        driver.flip_led(LEDs::Red);
        driver.flip_led(LEDs::Blue);

        driver
    }

    ///Turn the led of choice on or off.
    pub fn flip_led(&mut self, led: LEDs) {
        match led {
            LEDs::Red => {
                self.output_value ^= Self::RED_LED_PIN_ON;
            }
            LEDs::Blue => {
                self.output_value ^= Self::BLUE_LED_PIN_ON;
            }
        }

        self.i2c
            .write(Self::ADDR, &[Register::OutputPort as u8, self.output_value])
            .expect("Flipping the LEDs should not fail");
    }

    ///Returns (is_Red_led_on, is_Blue_led_on)
    pub fn get_led_states(&self) -> (bool, bool) {
        (
            (self.output_value & Self::RED_LED_PIN_ON) != 0,
            (self.output_value & Self::BLUE_LED_PIN_ON) != 0,
        )
    }
}
