//!  On the ESP32-S3-Korvo-2 V3.1 all buttons (Except the Boot and Reset buttons) share just 1 pin, the GPIO 5 pin.
//!  This is known as a resistor ladder. The schematic is on page 6
//!  [here](https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-Korvo-2_V3.1.2_20240116.pdf).
//!
//!  Each button press creates a different, distinct voltage level (that varies somewhat)
//!  that can be read by the microcontroller's ADC pin.
//!  Each range then corresponds to a specific button.
//!  For example, a reading between 0.0V and 0.5V could mean Button 1, while 1.0V to 1.5V means Button 2, and so on.
//!
//!  Because a resistor ladder is used,
//!  we cannot reliably detect multiple *simultaneous* button presses
//!  because the resulting voltage would be a combination of the individual button voltages.
//!  Luckily, that does not really matter for us.

use std::thread;
use std::time::Duration;

use esp_idf_svc::hal::{
    adc::{
        attenuation::DB_11,
        oneshot::{config::AdcChannelConfig, *},
        ADC1,
    },
    gpio::Gpio5,
};

///Figure out to what voltage values each button press corresponds.
///
///Note that ADC2 is also used by the Wifi ([source]), so we use ADC1.
///
/// Values I got from calling this function (these are different than the board [schematic]):
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
///
///
///[source]: https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/adc_oneshot.html#hardware-limitations.
///
///[schematic]: https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-Korvo-2_V3.1.2_20240116.pdf
pub fn diagnose(adc1: ADC1, gpio_pin: Gpio5) -> ! {
    let adc = AdcDriver::new(adc1).unwrap();

    // Configuring pin to analog read, you can regulate the adc input voltage range depending on your need.
    // We use the attenuation of 11db which sets the input voltage range to around 0-3.1V on the esp32-S3.
    let config = AdcChannelConfig {
        attenuation: DB_11,
        ..Default::default()
    };

    let mut adc_pin = AdcChannelDriver::new(&adc, gpio_pin, &config).unwrap();

    loop {
        thread::sleep(Duration::from_millis(100));
        //adc.read should *NOT* be called in ISR context. It returns the voltage of the pin.
        log::info!("ADC value: {}", adc.read(&mut adc_pin).unwrap());
    }
}
