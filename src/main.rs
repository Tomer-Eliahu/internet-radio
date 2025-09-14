use std::time::Duration;

#[allow(unused)]
use embedded_svc::{
    http::{Method, client::Client},
    io::{Read, Write},
};

//Note esp_idf_svc re-exports esp_idf_hal so we can just use that hal from here.
#[allow(unused)]
use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    hal::{
        delay::FreeRtos, //if not using embassy timer
        gpio::{InterruptType, PinDriver, Pull},
        i2c::{I2cConfig, I2cDriver},
        io::EspIOError,
        prelude::{Peripherals, *},
        task::notification::Notification,
        task::queue::Queue,
    },
    http::client::{Configuration, EspHttpConnection},
    sys::esp_random, //generate a random number (if needed)
    wifi::{AsyncWifi, EspWifi},
};

#[allow(unused)]
use embedded_hal::{delay::DelayNs, digital::OutputPin};

use crate::{buttons::diagonse, led::LedDriver};

/*debug_assertions is by default Enabled for non-release builds
and disabled for release builds.
#[cfg(debug_assertions)]
{
    if x > 40 {
        log::info!("x is {x}"); //This works fine.
    }

    //panic!("TEST panic"); //This also works-- we get a backtrace! But the MCU restarts automatically for us.
    //We can change that by adding CONFIG_ESP_SYSTEM_PANIC_GDBSTUB=y to sdkconfig.defaults.
}

A better approach (if you just need to conditionally complie in log messages)
is to just adjust the log-level allowed for release builds by setting the cargo-feature you want.
see https://docs.rs/log/latest/log/#compile-time-filters .
*/

//Note: to use the 2 system LEDs (that are user programmable) I either
// need to use I2C to talk to the IO expander peripheral to set its pins (it holds the LEDs)
// OR use this C thing https://github.com/espressif/esp-bsp/blob/master/bsp/esp32_s3_korvo_2/API.md#bulb-leds
// (maybe I can call into that from Rust??)

//######## JUST use the I2C driver as shown below (it implements the I2C trait of embedded hal) ############
//The io expander is TCA9554A (there is a *partial* driver for it [async only] in Rust)-- but I can write my own
//(not too much work;
//can even make it a sync implementation of embedded hal I2C trait and contribute it as a pull request
//to https://github.com/ladvoc/tca9554-rs?tab=readme-ov-file).
//From the Korvo schematic I2C Address (of this io expander)：0'b 0111 000x .
//Note that if this address is wrong, then this is the right one:
// https://esp32.com/viewtopic.php?t=28578
// Recall some devices have a WhoAmI register ( To check if the device is addressed correctly,
//read its device ID and print the value. --this one does not though)
//######## JUST use the I2C driver as shown below (it implements the I2C trait of embedded hal) ############

//Note that pin gpio17 in Rust corresponds the pin named IO17 in the ESP docs (which might be connected
//to a different physical pin on the board; but that is not relevant as the IO17 for us is an alias for that).

//Note: I will also need to use this IO expander to use the LCD anyway (but *just* for initilization!)...
//The LCD uses SPI and I2C (primarly SPI-- which is much faster due to underlying hardware differences [this
//is a general fact])

//Note: from esp-idf-hal README.md: The following pins are not recommended for use by you:
//ESP32-S3:	26 - 32, 33 - 37* (*= When using Octal Flash and/or Octal PSRAM)

//Note from googling around (https://esp32.com/viewtopic.php?t=34179),
// it appears for the esp-idf you can pick whichever GPIO pins you want
//to be primary and secondary I2C.

/*You probably want to do something like this (NOTE this is not for MY BOARD!)
    let sda = peripherals.pins.gpio10;
    let scl = peripherals.pins.gpio8;
    let i2c = peripherals.i2c0;
    let config = I2cConfig::new().baudrate(100.kHz().into());
    let i2c = I2cDriver::new(i2c, sda, scl, &config)?;

*/

fn main() -> ! {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let peripherals = Peripherals::take().unwrap();

    let sda = peripherals.pins.gpio17; //serial data line
    let scl = peripherals.pins.gpio18; //serial clock line
    let i2c = peripherals.i2c0;
    //IMPORTANT NOTE -- if need bus sharing, remember there are things in the hal for that
    //(if I can't make using i2c1 work).


    //CRITICAL NOTE:
    //I have no idea why but if you put the LCD screen on where it is supposed to rest,
    //the I2C times out. Literally, restarting the *same* program via the terminal (CTRL + R),
    //if the screen hangs off and down over the buttons (so the silver back of the screen covers the
    //buttons), it works, if the screen rests where it is supposed to, it does not.

    //So for this project, make sure you disconnect the LCD screen from the Korvo.

    let config = I2cConfig::new().baudrate(led::BAUDRATE_FAST.into());
    //for trouble shooting, maybe enable the following line
    //config =config.scl_enable_pullup(true).sda_enable_pullup(true).timeout(Duration::from_millis(20).into());
    let i2c = I2cDriver::new(i2c, sda, scl, &config).unwrap();
    let mut led_driver = LedDriver::build(i2c);

    led_driver.flip_led(led::LEDs::Blue);

    let mut x = 7;
    let y = 35;
    x += y;

    log::info!("Hello, world!");

    log::info!("x is {x}"); //This works fine.

    let adc1 = peripherals.adc1;
    let adc_pin = peripherals.pins.gpio5;

    //Add call to diagnose here which never returns
    diagonse(adc1, adc_pin);

    loop {
        std::thread::sleep(Duration::from_millis(100));
    }
}

///The LEDs are on the IO expander peripheral: [the TCA9554A](https://www.ti.com/product/TCA9554A).
/// We therefore write a partial I2C driver for this device so we can easily work the LEDs.
///
/// Note that the device is reset to its default state by cycling the power supply and causing a power-on reset
/// (from the data sheet). This means that once we flash a new program *and* cycle th power off then back on,
/// we can see the device has reverted to its default state.
pub mod led {
    use embedded_hal::i2c::I2c;
    use esp_idf_svc::hal::prelude::*;

    ///Baudrate Max clock frequency in KHz (depending if using standard or fast mode).
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
    /// [write_read method](https://docs.rs/embedded-hal/latest/embedded_hal/i2c/trait.I2c.html#method.write_read)
    /// which also matches the IO expander data sheet.
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

    pub struct LedDriver<I2C> {
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
        ///This is called P7 for the Korvo. On the IO expander it is pin 11.
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

            let mut driver  = Self {
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
}

///On the ESP32-S3-Korvo-2 V3.1 all buttons (Except the Boot and Reset buttons) share just 1 pin, the GPIO 5 pin.
/// This is known as a resistor ladder. The schematic is on page 6
/// [here](https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-Korvo-2_V3.1.2_20240116.pdf).
///
/// Each button press creates a different, distinct voltage level that can be read by the microcontroller's ADC pin.
/// The microcontroller measures the analog voltage and compares it to a predefined range of values (like the mid points
/// in voltage between each 2 buttons).
///Each range then corresponds to a specific button.
/// For example, a reading between 0.0V and 0.5V could mean Button 1, while 1.0V to 1.5V means Button 2, and so on.
///
/// The con of this is that
/// we cannot reliably detect multiple *simultaneous* button presses
/// because the resulting voltage would be a combination of the individual button voltages.
///
/// REMEMBER TO IMPLEMENT DEBOUNCING!
/// Debouncing: When a mechanical button is pressed or released,
/// it can "bounce," creating multiple fast, spurious electrical signals. You must implement software debouncing
/// (e.g., waiting a few milliseconds and re-reading the pin state) to ensure a single press is registered.
///
/// We want to record the voltage on the Pin (change the station; stop and play, adjust the volume).
///
///
/// the esp std book used notifications, which only give the latest value, so if the interrupt is triggered multiple
///times before the value of the notification is read, you will only be able to read the latest one.
///Queues, on the other hand, allow receiving multiple values. See esp_idf_hal::task::queue::Queue for more details
///
///
/// CRITICAL: Debouncing and noise filtering must also be handled in your code,
/// as this is not provided by the hardware or esp-idf-hal in Rust.
/// You can use multisampling (Take multiple ADC readings in quick succession and average them.
/// This reduces the impact of random noise on any single reading
/// and provides a more stable value for button detection)
///
/// Polling: we will periodically measure the adc input pin in (for instance) an embassy task.
/// Or set up a timer interrupt every 100ms.
///This is the simplest way (we also get debouncing for "free" here). 
/// It also is power efficent as we can go low power for the rest of the time 
/// (the embassy exector goes low power for us automatically when the mcu has nothing to do).
/// **[This is Espressif's approach to their own button component](https://docs.espressif.com/projects/esp-iot-solution/en/latest/input_device/button.html)**.
/// **Final decision: I decided to go with this approach!!**.
pub mod buttons {

    //If going embassy polling approach:
    //MAYBE By taking multiple readings a few milliseconds apart, 
    //your polling routine can confirm a stable button press.
    //I.E. use multisampling. This is what espressif does as well (with adc one shot mode)

    use std::thread;
    use std::time::Duration;

    use esp_idf_svc::hal::{
        adc::{
            ADC1,
            attenuation::DB_11,
            oneshot::{config::AdcChannelConfig, *},
        },
        gpio::Gpio5,
    };

    ///Note that ADC2 is also used by the Wifi ([source]), so we use ADC1.
    ///
    ///[source]: https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/adc_oneshot.html#hardware-limitations.
    ///
    ///[schematic]: https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-Korvo-2_V3.1.2_20240116.pdf
    ///
    /// Values I got from calling diagonse (these are different than the board [schematic]):
    ///                 
    /// No button presss: 3100
    ///                 
    /// Vol + about 320
    ///                 
    /// Vol - about 720
    ///                 
    /// Set about 990
    ///                 
    /// Play about 1500
    ///                 
    /// Mute about 1810
    ///                 
    /// Rec about 2210
    pub fn diagonse(adc1: ADC1, gpio_pin: Gpio5) -> ! {
        let adc = AdcDriver::new(adc1).unwrap();

        // configuring pin to analog read, you can regulate the adc input voltage range depending on your need
        // we use the attenuation of 11db which sets the input voltage range to around 0-3.1V
        let config = AdcChannelConfig {
            attenuation: DB_11,
            ..Default::default()
        };

        let mut adc_pin = AdcChannelDriver::new(&adc, gpio_pin, &config).unwrap();

        loop {
            // you can change the sleep duration depending on how often you want to sample
            thread::sleep(Duration::from_millis(100));
            //adc.read should *NOT* be called in ISR context. It returns the voltage of the pin.
            log::info!("ADC value: {}", adc.read(&mut adc_pin).unwrap());
        }
    }
}
