use std::time::Duration;

#[allow(unused)]
use embedded_svc::{
    http::{Method, client::Client},
    io::{Read, Write},
};

use esp_idf_svc::{hal::i2s::I2sDriver, io::utils::try_read_full};
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
    timer::EspTaskTimerService
    //Alternative Timer approach if needed
    //https://github.com/esp-rs/esp-idf-hal/blob/master/examples/blinky_async.rs
};


//use embedded_hal::{digital::OutputPin};

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


/*From the Embassy website

No busy-loop polling: CPU sleeps when there’s no work to do, using WFI.

from: https://docs.embassy.dev/embassy-executor/git/cortex-m/index.html#embassy-executor

**NOT TRUE FOR STD**

*/

use embassy_executor::Spawner;
use embedded_hal_async::delay::DelayNs;
use  embassy_futures::select::select;

const STATION_URLS: [&'static str;1] = ["https://18063.live.streamtheworld.com/977_CLASSROCK.mp3"];


#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();
    

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    let timer_service= EspTaskTimerService::new().unwrap();
    let mut async_timer = timer_service.timer_async().unwrap();

    let peripherals = Peripherals::take().unwrap();
    

    let sda = peripherals.pins.gpio17; //serial data line
    let scl = peripherals.pins.gpio18; //serial clock line
    let i2c = peripherals.i2c0;
    //IMPORTANT NOTE -- if need bus sharing, remember there are things in the hal for that
    //(if I can't make using i2c1 work).

    /* from https://docs.rs/embedded-hal/latest/embedded_hal/i2c/index.html:
    The embedded-hal-bus crate provides several implementations for sharing I2C buses. 
    You can use them to take an exclusive instance you’ve received from a HAL 
    and “split” it into multiple shared ones, 
    to instantiate several drivers on the same bus. */


    //CRITICAL NOTE:
    //I have no idea why but if you put the LCD screen on where it is supposed to rest,
    //the I2C times out. Literally, restarting the *same* program via the terminal (CTRL + R),
    //if the screen hangs off and down over the buttons (so the silver back of the screen covers the
    //buttons), it works, if the screen rests where it is supposed to, it does not.
    //Maybe if the LCD is connected and you are using esp-idf, then that I2C is reserved?

    //So for this project, make sure you disconnect the LCD screen from the Korvo.

    let config = I2cConfig::new().baudrate(led::BAUDRATE_FAST.into());
    //for trouble shooting, maybe enable the following line
    //config =config.scl_enable_pullup(true).sda_enable_pullup(true).timeout(Duration::from_millis(20).into());
    let i2c = I2cDriver::new(i2c, sda, scl, &config).unwrap();
    let mut led_driver: LedDriver<I2cDriver<'_>> = LedDriver::build(i2c);

    led_driver.flip_led(led::LEDs::Blue);

    let mut x = 7;
    let y = 35;
    x += y;

    log::info!("Hello, world!");

    log::info!("x is {x}"); //This works fine.

    let adc1 = peripherals.adc1;
    let adc_pin = peripherals.pins.gpio5;

    //CONT. FROM HERE
    
    /*CRITICAL NOTE:
    TODO:
    Confirm that your I2S master (e.g., an ESP32 or other microcontroller) 
    is configured to output the necessary clock signals (BCLK and LRCK) at the correct rates.
    The clocks must be synchronised - so one should be derived from the other.
    
    */
    let sound  = peripherals.i2s0;//TODO: look into this from docs and more.
    //Set mclk to None I think (should not be needed). ws is word select which is LRCK.
    //let i2s= I2sDriver::new_std_tx(i2s, config, bclk, dout, mclk, ws);
    
    let pa_ctrl= peripherals.pins.gpio48;
    
    let modem= peripherals.modem;
    //let mut client = wifi::setup_wifi(modem).await.unwrap();
    //let _ = wifi::test_get_request(&mut client);

    let blink_while_setting_wifi = async { 
        loop {
            led_driver.flip_led(led::LEDs::Blue);
            async_timer.delay_ms(500).await;
        }

    };

    //TODO: Use concurrently while searching for connection
    let mut client = match select(blink_while_setting_wifi,  wifi::setup_wifi(modem)).await
    {
        embassy_futures::select::Either::First(_) => unreachable!(),
        embassy_futures::select::Either::Second(res) => {res.unwrap()},
    };

    let _ = wifi::test_get_request(&mut client).expect("Test should not fail");

    log::info!("Wifi should be set up. Blue should stop blinking");

    //As the Blue light could have been off at this time we switch it on if need be.
    match led_driver.get_led_states() {
        (_, false) => led_driver.flip_led(led::LEDs::Blue),
        _ => ()
    };

    
    // loop {
    //     led_driver.flip_led(led::LEDs::Red);
    //     async_timer.delay_ms(500).await;
    // }

    //Could spawn everything from here below in an async Task.

    //Start streaming audio from the station
    //For STATION_URL[0] we get around 3.66 kb per 200ms 
    //start with 1 second buffer

    //TODO: maybe change this to a different type? Make this a dynamic buffer?
    //let mut buf = [0u8; 1024 * 4 * 5];
    
    {

        // Send request
        //This GET request is equivalent to GET /977_CLASSROCK.mp3 HTTP/1.1 Accept: */* 
        let request = client.get( STATION_URLS[0]).unwrap();
        log::info!("-> GET {}", STATION_URLS[0]);
        let mut response = request.submit().unwrap();

        // Process response
        let status = response.status();
        //TODO: actually check if status is valid, and if not add retries? or just straight up panic?
        log::info!("Status is: {status}");

        let stream_source = audio_stream::new_stream(response);
      
        

        




        //From Reading Symphonia: the MediaSourceStream has an internal ring buffer 
        //where we specify the max size for it (by default around 64kB max size). 
        //It reads from the source (our response), only when needed.
        //When that happens, we read from the underlying HTTP stream.

        


        
        //TODO: Fill up the reader with audio content for 200ms on startup (to prevent jitter)?

        //TODO: Use embassy_sync::pipe to communicate between this writer and the reader that passes data
        //into I2S stream?

        
    }

    //Add call to diagnose here which never returns
    diagonse(adc1, adc_pin);

    //Can Join futures where one of them detects button presses in a loop
    //So it is impl Future<Output =!> . If we need to change to change the station we break out the loop.
    //and make a new get request

    loop {
        std::thread::sleep(Duration::from_millis(100));
    }
}


// let executor = Executor::new();

// executor.run(|spawner| {
//     spawner.spawn(run(led_driver));
// });
    
// #[embassy_executor::task]
// async fn run(led_driver: LedDriver<I2cDriver<'_>>) {
//     loop {
//         led_driver.flip_led(led::LEDs::Blue);
//         Timer::after_millis(500).await;
//     }
// }

pub mod audio_stream {
    use std::io::Error;
    use symphonia::core::io::ReadOnlySource;
    use esp_idf_svc::http::client::{EspHttpConnection, Response};
    use std::cell::RefCell;
    use std::sync::Mutex;
    /// A wrapper around the Response from the GET request.
    /// It implements std::io:Read by calling into the inner read which itself calls 
    /// the embedded::io::Read implementation on the underlying connection.
    /// This allows us to bridge the gap from the HTTPS Response to Symphonia's MediaSource.
    pub struct StreamSource<'a> {
        //We wrap StreamSourceInner in a mutex because we needed the stream source
        // to impl Sync as well so we would have
        //impl<R: Read + Send + Sync> MediaSource for ReadOnlySource<R> as needed.
        inner: Mutex<StreamSourceInner<'a>>
    }

    struct StreamSourceInner<'a> {
        inner: RefCell<Response<&'a mut EspHttpConnection>>
    }

    impl std::io::Read for StreamSource<'_> {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, std::io::Error> {
            let mut lock = self
                .inner
                .lock()
                .expect("Only one thread/async task at a time should read the stream source");
            //Note that since the Mutex guarantees this thread to have exclusive access
            //to StreamSourceInner, we do not need to pay the additional performance cost of RefCell
            lock.inner.get_mut().read(buf)
            .inspect(|read_bytes| log::trace!("read {} bytes!", read_bytes))
            .map_err(|e| Error::other(e))
        }
    }

    
    pub fn new_stream<'a> (
        response: Response<&'a mut EspHttpConnection>,
    ) -> ReadOnlySource<StreamSource<'a>> {
        ReadOnlySource::new(StreamSource {
            inner: Mutex::new(StreamSourceInner {
                inner: RefCell::new(response),
            }),
        })
    }
    

    ///SAFTEY: We use a RefCell (which dynamically checkes the borrowing rules) 
    /// to make sure we access the underlying HTTPS stream in a unique way.
    /// This ensures this implementation of Send is sound.
    /// 
    /// We needed to implement this because EspHttpConnection has the following field:
    ///raw_client: *mut esp_http_client
    unsafe impl<'a> Send for StreamSourceInner<'a> {}
    
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
/// We want to record the voltage on the Pin (change the station: 
/// use SET for station back and REC for station foward; stop (Mute key) and Play, 
/// adjust the volume: Vol+ and Vol-).
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
/// Or set up a timer interrupt to fire every 100ms or so to update a global atomic, to tell the main function
/// it is time to poll the pin again.
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
        // we use the attenuation of 11db which sets the input voltage range to around 0-3.1V on the esp32-S3.
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



pub mod wifi {

    use embedded_svc::{
        http::{client::Client as HttpClient, Method},
        wifi::{AuthMethod, ClientConfiguration, Configuration},
    };

    use esp_idf_svc::{
        eventloop::EspSystemEventLoop, http::client::{Configuration as HttpConfiguration, EspHttpConnection}, 
        io::{self, utils::try_read_full}, 
        nvs::EspDefaultNvsPartition, sys::EspError, 
        timer::EspTaskTimerService, wifi::{AsyncWifi, EspWifi}
    };

    pub mod config;

    static WIFI: std::sync::Mutex<Option<AsyncWifi<EspWifi<'static>>>> = std::sync::Mutex::new(None);

    pub async fn setup_wifi(modem: esp_idf_svc::hal::modem::Modem) -> Result<HttpClient<EspHttpConnection>, EspError>
    {
        let sys_loop = EspSystemEventLoop::take()?;
        //TODO: nvs is optional. We could just use None in EspWifi::new. Maybe change to that!
        let nvs = EspDefaultNvsPartition::take()?;
        let timer_service = EspTaskTimerService::new()?;

        let mut wifi: AsyncWifi<EspWifi<'_>> = AsyncWifi::wrap(
            EspWifi::new(modem, sys_loop.clone(), Some(nvs))?,
            sys_loop,
            timer_service
        )?;

        let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
            ssid: config::WIFI_NAME.try_into().unwrap(),
            bssid: None,
            auth_method: AuthMethod::WPA2Personal,
            password: config::WIFI_PASSWORD.try_into().unwrap(),
            channel: None,
            ..Default::default()
        });

        wifi.set_configuration(&wifi_configuration)?;

        wifi.start().await?;
        log::info!("Wifi started");

        wifi.connect().await?;
        log::info!("Wifi connected");

        wifi.wait_netif_up().await?;
        log::info!("Wifi network interface up");

        //We need to hold on to wifi (once it is dropped, it terminates).
        {
            *WIFI.lock().unwrap() = Some(wifi);
        }
        
        // Create HTTP client
        //
        // Note: To send a request to an HTTPS server, you can do:        
        let config = &HttpConfiguration {
            crt_bundle_attach: Some(esp_idf_svc::sys::esp_crt_bundle_attach),
            use_global_ca_store: true,
            ..Default::default()
        };
        
        let client = HttpClient::wrap(
            EspHttpConnection::new(&config)?);

        Ok(client)
    }


    //TODO REWRITE THE STUFF BELOW
    /// Test sending an HTTP GET request.
    pub fn test_get_request(client: &mut HttpClient<EspHttpConnection>) -> Result<(), io::EspIOError> {
        // Prepare headers and URL
        let headers = [("accept", "text/plain")];
        let url = "http://ifconfig.net/";

        // Send request
        //
        // Note: If you don't want to pass in any headers, you can also use `client.get(url, headers)`.
        let request = client.request(Method::Get, url, &headers)?;
        log::info!("-> GET {url}");
        let mut response = request.submit()?;

        // Process response
        let status = response.status();
        log::info!("<- {status}");
        let mut buf = [0u8; 1024];


        //This try_read_full calls an underlying function which Reads data from http stream.
        //While an async version of this function exists, the necessary traits to call it
        //are not implemented on any type so we can't use it.
        let bytes_read = try_read_full(&mut response, &mut buf)
        .map_err(|e| e.0)?;
    
        log::info!("Read {bytes_read} bytes");
        match std::str::from_utf8(&buf[0..bytes_read]) {
        Ok(body_string) => log::info!(
                "Response body (truncated to {} bytes): {:?}",
                buf.len(),
                body_string
            ),
            Err(e) => log::error!("Error decoding response body: {e}"),
        };

        Ok(())
    }

}




///The board has an ES8311 low-power mono audio codec chip.
/// The data sheet is available [here](https://www.lcsc.com/datasheet/C962342.pdf).
///
/// The extended data sheet (this includes register descriptions) is available 
/// [here](https://www.lcdwiki.com/res/PublicFile/ES8311_DS.pdf).
/// There is a lot more information in the 
/// [user guide](https://files.waveshare.com/wiki/common/ES8311.user.Guide.pdf).
/// 
/// We can completely configure this device using I2C.
/// 
/// Here is some of the relevant information:
/// 
///* DAC: 24-bit, 8 to 96 kHz sampling frequency.
/// 
///* The analog output path includes mono DAC, programmable volume control, 
/// a fully differential output and headphone amplifier.
/// The mono audio DAC supports sampling rates from 8 kHz to 96 kHz and includes 
/// programmable digital filtering and Dynamic Range Compression in the DAC path (from the user guide).
///     
///* Ideally we want the device to work in its default clock mode (where we supply
/// LRCK (Left/Right data alignment clock) and SCL (Bit clock for synchronisation)). 
/// This mode sets our MCU as the "master" for the I2S.
/// 
///* I2S capable of up to 24 bits.
/// 
///* The device can work in 2 speed modes: 
///     - Single Speed (Fs normally range from 8 kHz to 48 kHz).
///     - Double Speed (Fs normally range from 64 kHz to 96 kHz).
/// 
///**Note: (only applicable to master mode; not the mode we will be using)** 
/// The DAC only works in single speed mode.
/// the ratio between internal DAC clock and LRCK must be equal or greater than 256, 
/// and this ratio must be integral multiple of sixteen.
/// 
///* We can increase/decrease volume and even mute using this device.
/// 
///* The DAC is also capable of DSP/PCM mode serial audio data format instead of I2S. 
/// With this mode we can use 32-bit depth (but there is no *audible* benefit of using that).
/// 
///* We can enable Dynamic Range Control (a.k.a. Dynamic Range Compression) feature of the DAC, 
/// but since we are using it to listen to music here,
/// we will leave it at its default setting of disabled.
/// 
///* The DAC has an equalizer, but we are not going to use it (it is bypassed by default).
/// 
///* ES8311 has an internal power-on reset.
///
pub mod speaker {

 
    //!ESP_IO48 is connected to PA_CTRL (from Korvo schematics).
    //!IMPORTANT: we must use this to power up this power amplifier!
    //![PA data sheet](https://www.alldatasheet.com/html-pdf/1131841/ETC1/NS4150/929/8/NS4150.html).
    //!PA_CTRL (simple High or Low setting; H: Open mode (i.e on), L: Shutdown, i.e off) 
    //!is power down control terminal.
    //!It used to make power consumption more efficent (draw lower power when on standby).
    //!So we want to drive this pin High before use, and drive it low post use 
    //!(maybe have driving it low be a part of a drop implementation).



    //For the codec (ES8311), our MCU pins we are interested in are ([source]):
    //IO16 = I2S0_MCLK (Master Clock) (this is actually not relevant for us; maybe it is-- give it as a reference
    //in addition to the 2 clock lines below? is this required? it seems it is needed from the following statement:
    //" In slave mode, LRCK divider is inactive and ES8311 will detect MCLK/LRCK ratio automatically").
    //We need to make sure MCLK and LRCK are synced???

    //We need to supply the following:
    //IO9=  I2S0_SCLK
    //IO45= I2S0_LRCK
    //IO8= I2S0_DSDIN

    //These I2C pins (note that they might also be used to talk to different devices, so we might need bus sharing)
    //IO17= I2C_SDA
    //IO18= I2C_CLK
    //
    //[source]: https://docs.espressif.com/projects/esp-adf/en/latest/design-guide/dev-boards/user-guide-esp32-s3-korvo-2.html#pin-allocation-summary



    use std::time::Duration;

    use embedded_hal::i2c::I2c;
    
    
    use esp_idf_svc::hal::{gpio::{Gpio48, PinDriver}, prelude::*};

    ///I2C Baudrate Max clock frequency in KHz.
    pub const BAUDRATE: KiloHertz = KiloHertz(400);
    pub const BAUDRATE_SLOW: KiloHertz = KiloHertz(100);

    pub struct Volume(u8);

    impl TryFrom<u8> for Volume {
        type Error = &'static str;
        
        fn try_from(value: u8) -> Result<Self, Self::Error> {
            if value <= 100 {
                Ok(Volume(value))
            }
            else {
                Err("Volume only accepts 0<= integral values <=100")
            }
        }

    }
    

   
    pub struct SpeakerDriver<I2C: I2c>  {
        i2c: I2C,
        ///TODO: Note this type has async methods, use them!
        i2s: esp_idf_svc::hal::i2s::I2sDriver<'static, esp_idf_svc::hal::i2s::I2sTx>,
        pa_ctrl_pin: PinDriver<'static, Gpio48, esp_idf_svc::hal::gpio::Output>,
        vol: Volume
    }

    impl<I2C: I2c> SpeakerDriver<I2C>
    {  
        ///The I2C address is a seven-bit chip address. 
        ///The chip address must be 0011 00y, where y equals CE.
        ///From the [Korvo schematic], CE is 0 for us (it is just connected to Ground).
        ///So the 7-bit chip address is 0011 000.
        /// 
        /// 
        /// This address is specified in the **right-aligned** form as specified
        /// [here](https://docs.rs/embedded-hal/latest/src/embedded_hal/i2c.rs.html#296).
        ///
        /// [Korvo schematic]: https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-Korvo-2_V3.1.2_20240116.pdf 
        pub const I2C_ADDR: u8 = 0b00011_000;


        ///Creates and initlizes the Speaker Driver.
        /// This function initilizes the codec and also turns on the power amplifier (PA).
        pub fn build(i2c: I2C, i2s: esp_idf_svc::hal::i2s::I2sDriver<'static, esp_idf_svc::hal::i2s::I2sTx>, 
        pa_ctrl_pin: Gpio48) -> Self {

            let pa_ctrl_pin = PinDriver::output(pa_ctrl_pin).unwrap();
            let mut speaker_driver = Self{ i2c, i2s, pa_ctrl_pin, vol: Volume(0) };
            
            speaker_driver.codec_software_reset();

            //For Debug builds, verify this device is indeed the codec.
            #[cfg(debug_assertions)]
            {
                let mut read_buf: [u8; 1] = [0];

                speaker_driver.i2c.write_read(Self::I2C_ADDR, 
                    &[Register::ChipID1 as u8], read_buf.as_mut()).unwrap();
                
                assert_eq!(read_buf[0], Register::ChipID1.default(), 
                "MISTAKEN IDENTITY: expected this to be the codec");

                speaker_driver.i2c.write_read(Self::I2C_ADDR, 
                    &[Register::ChipID2 as u8], read_buf.as_mut()).unwrap();

                assert_eq!(read_buf[0], Register::ChipID2.default(), 
                "MISTAKEN IDENTITY: expected this to be the codec");

            }

            //We power up everything except ADC stuff
            speaker_driver.i2c.write(Self::I2C_ADDR, 
                &[Register::SysPwrMgt as u8 , 0b00110101]).unwrap();

            //Power up DAC
            speaker_driver.i2c.write(Self::I2C_ADDR, 
                &[Register::SysEnableDac as u8 , 0]).unwrap();
            
            //Enable speaker drive
            let write_in = const {Register::SysEnableSpeakerDrive.default() | (1<<4)};
            
            speaker_driver.i2c.write(Self::I2C_ADDR, 
                &[Register::SysEnableSpeakerDrive as u8 , write_in]).unwrap();

            
            //Power up the power amplifier
            speaker_driver.pa_ctrl_pin.set_high().unwrap();

            //set the volume to 80% as an initial value
            speaker_driver.set_vol(Volume::try_from(80).unwrap());

            
            //The speaker driver is all set!
            speaker_driver
        }



        /// From the [user guide]: 
        /// It is suggested that releasing a software reset operation to clear 
        /// the internal state while codec power up. 
        /// The following is proposal procedure of software reset operation: 
        /// set the reset bits to ‘1’ to release reset signal and clear CSM_ON to ‘0’ to powerdown state machine, 
        /// then delay a short time, such as several milliseconds, clear reset bits to ‘0’ 
        /// and set CMS_ON to ‘1’ at last.
        ///  
        /// Please set all reset bits (bits 0-4 inclusive) to ‘1’ 
        /// and clear CSM_ON to ‘0’ (for us simply return the register to its default value!)
        /// to minimize the power consumption when codec is ready for standby or sleep (We do this in 
        /// the drop implementation).
        /// 
        ///[user guide]: https://files.waveshare.com/wiki/common/ES8311.user.Guide.pdf
        fn codec_software_reset(&mut self) {
           
            //reset everything and power down
            self.i2c.write(Self::I2C_ADDR, &[Register::Reset as u8, Register::Reset.default()]).unwrap();
            
            std::thread::sleep(Duration::from_millis(100));

            //no reset and power up
            self.i2c.write(Self::I2C_ADDR, &[Register::Reset as u8, (1 << 7)]).unwrap();

        }


        pub fn mute(&mut self) {

            self.i2c.write(Self::I2C_ADDR, 
                &[Register::DacMute as u8, 0b1110_0000]).unwrap();

        }

        pub fn unmute(&mut self) {

            self.i2c.write(Self::I2C_ADDR, 
                &[Register::DacMute as u8, 0]).unwrap();

        }

        ///Sets the volume. Takes an integral precentage of volume desired.
        pub fn set_vol(&mut self, vol: Volume) {

            //the min is 0 which corresponds to -95.5dB 
            let max: f32 = 191.0; //191 = 0xBF which corresponds to 0dB
            
            let desired = (max * ((vol.0 as f32)/100.0)) as u8;

            self.i2c.write(Self::I2C_ADDR,
                &[Register::DacVol as u8, desired]).unwrap();
            

            self.vol = vol;

        }
    }

    impl<I2C> Drop for SpeakerDriver<I2C>
    where I2C: I2c
    {
        fn drop(&mut self) {

            //reset everything and power down. Minimizes power consumption.
            self.i2c.write(Self::I2C_ADDR, 
                &[Register::Reset as u8, Register::Reset.default()]).unwrap();

            //Power down the PA.
            self.pa_ctrl_pin.set_low().unwrap();

        }
    }


    ///Registers (just the ones we need).
    /// 
    /// **Important**: bits are numbered starting from 0.
    #[non_exhaustive]
    enum Register {

        ///The Reset register.
        ///This register has default value: 0001 1111.
        /// 
        ///CSM_ON (bit 7) must be set to ‘1’ to start up state machine in normal mode (i.e turn on this codec), 
        /// so we need to flip that bit.
        ///See page 18 of the user guide for more info (we follow the power up and power down procedure
        /// suggested there).
        Reset = 0,

        ///Power Management. See page 18 in the user guide for more info.
        /// C stuff just writes 0x1 to here (enables everything an startups VMID in normal mode).
        /// default 0b1111_1100.
        SysPwrMgt = 0x0D,

        //default 0000 0010 . The C code writes 0 to this (does *not* enable internal reference DAC circuit)
        ///This register enables and disables the DAC. It has a default value of 0000 0010 (DAC powered down 
        /// and internal reference circuits for DAC disabled).
        /// We want to write 0 into this to power up the DAC.
        SysEnableDac = 0x12,

        ///The C code writes 0x10 i.e. 16 into this to enable heaphone drive.
        /// Has default 0100 0000. 
        /// We want to set bit 4 to 1 (has default 0), to enable output to HP (or speaker) drive.
        SysEnableSpeakerDrive = 0x13,


        ///Write 1 to bits 5 to 7 inclusive to cleanly mute the DAC (according to the user guide page 29).
        /// Has default value of 0.
        DacMute = 0x31,

        ///DAC_VOLUME is an 8-bit digital volume control for DAC 
        /// with range from -95.5dB to +32dB in 0.5dB/step resolution.
        ///used to control DAC volume. 
        /// Default value of 0 which is -95.5dB. Max value 0xFF which corresponds to +32dB.
        DacVol = 0x32,

        ///READ ONLY. Should have a value of 0x83.
        ChipID1 = 0xFD,

        ///READ ONLY. Should have a value of 0x11.
        ChipID2 = 0xFE,

        //-----------Maybe add--------------
        //In C, they adjust the default of register 0x10 to set  bits 2 and 3 to 1 as well
        //(highest bias level). The rest of the register they keep the same as the default.



        //es8311_write_reg(codec, ES8311_DAC_REG37 (this is 0x37: DAC equalizer), 0x08). 
        //This is C stuff when starting the codec 
        //(it is disabling the equlizer which is already disabled by default); 


        //----- Currently not needed----------------

        //Serial Digital Port In
        //bit 6 set to 1 mutes. Set to 0 (default) is unmute.
        // The other default values mean 24 bit I2S, Left channel to DAC.
        // See page 12 of the user guide for more info.
        // While this mute function should work, there is a cleaner way to do this, that does not result
        // in artifact sounds.
        //SdpIn = 0x09, NOT NEEDED

        //Clock Manager.
        // Need to set bit 4 to 1 to turn on Bit Clock (ACTUALLY: I am fairly sure this not needed in slave mode;
        // just in master mode where this pin is output instead of input).
        // default value of register = 0
        //ClockManager = 0x01,

    }

    impl Register {

        ///Returns the default value of the entire register
        const fn default(&self) -> u8 {
            match *self 
            {
                Register::Reset => 0b0001_1111,
                Register::SysPwrMgt => 0b1111_1100,
                Register::SysEnableDac => 0b0000_0010,
                Register::SysEnableSpeakerDrive => 0b0100_0000,
                Register::DacMute | Register::DacVol => 0,
                Register::ChipID1 => 0x83,
                Register::ChipID2 => 0x11,
            }
        }
    }


}