use std::sync::Mutex;

#[allow(unused)]
use embedded_svc::{
    http::{Method, client::Client},
    io::{Read, Write},
};

use esp_idf_svc::hal::i2s::{self, config::StdConfig};
#[allow(unused)]
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
use symphonia::core::{audio::{AudioBuffer, RawSampleBuffer}, codecs::Decoder, 
    formats::FormatOptions, io::MediaSourceStream, meta::MetadataOptions, probe::Hint};
use symphonia::core::errors::Error as symphonia_Error;

//use embedded_hal::{digital::OutputPin};
#[allow(unused)]
use crate::{buttons::diagnose, led::LedDriver, speaker::SpeakerDriver};
use embedded_hal::i2c::I2c;

use embedded_hal_bus::i2c::MutexDevice;

//use symphonia_bundle_mp3::layer3::BitResevoir;


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

**NOT TRUE FOR STD (ACTUALLY I AM NOT SURE; Maybe it is not True here where STD is built on FREERTOS.
It is true for general std I think (but I am not sure)**

*/

use embassy_executor::Spawner;
use embedded_hal_async::delay::DelayNs;
use  embassy_futures::select::select;
use static_cell::StaticCell;

//TODO: the first 2 stations are mp3, Add some that are aac or something else
const STATION_URLS: [&'static str;3] = ["https://18063.live.streamtheworld.com/977_CLASSROCK.mp3",
"https://puma.streemlion.com:3130/stream",
"https://streamer.radio.co/s52d0fa340/listen"];

//Mentions location simply means says name of station/ talks in a way that is identifiable
//
//977_CLASSROCK is 70's Rock - HitsRadio (North Carolina, USA) (Has ads so maybe get rid?)
//
//puma is Classic Rock Legends Radio (LA, CA, USA) (Mentions location) (audio/mpeg):
// https://puma.streemlion.com:3130/stream
// (https://www.radio.net/s/classicrocklegends)
//
//Smooth Radio London (mentions location):  https://media-ice.musicradio.com/SmoothLondonMP3
//(https://www.radio.net/s/smoothradiolondon)
//
//OzInDi Radio Australia (mentions location): https://streamer.radio.co/s52d0fa340/listen
//THIS IS audio/aac it is aac!
//(from https://www.radio.net/s/ozindiradio)
//
//Radiowave NZ (mentions location): https://stream.radiowavenz.com/stream (audio/mpeg)
//(from https://www.radio.net/s/radiowave-nz)
//Another NZ station (also audio/mpeg), this one is from TuneIN instead of radio.net:
//https://stream.kixfm.co.nz:8178/stream
//
//Cool Radio Canada (audio/aac) (mentions location): https://live.leanstream.co/CKUEF2
//(from https://www.radio.net/s/coolradiocanada)
//
//Timewarp Ireland (mentions location): https://broadcast.shoutstream.co.uk:8052/stream (audio/mpeg)
//(from https://www.radio.net/s/timewarpireland)
//
//Note I don't think aacp (different than aac) should work, 
//but if you want to try here is a good station for it:
//https://www.radio.net/s/station-x-xrn-australia 

//We need to do this because otherwise Rust thinks that client drops before
//our media_stream (which borrows the client mutablly). 
static CLIENT: StaticCell<Client<EspHttpConnection>> = StaticCell::new();

use esp_idf_svc::sys::{MALLOC_CAP_8BIT, MALLOC_CAP_SPIRAM, MALLOC_CAP_EXEC, MALLOC_CAP_DMA};

use symphonia_bundle_mp3::MpaDecoder;
use symphonia_bundle_mp3::decoder::State;


#[embassy_executor::main]
async fn main(_spawner: Spawner) -> ! {

    /* TODO: 
        maybe add:
    
        CONFIG_LWIP_L2_TO_L3_COPY --ALSO IMPORTANT MAYBE
    
    */

    /* 
        TODO: if watchdog is causing a problem        

        Might be relevant (watchdog for bootloader-- maybe increase):
        https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig-reference.html#config-bootloader-wdt-time-ms
        
        also see:
        https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig-reference.html#config-esp-int-wdt-timeout-ms
        and related settings.

        especially see:
        https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/kconfig-reference.html#config-esp-wifi-enterprise-support

        It says to adjust watch dog timer if using wifi https!
    */

    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();
    

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();

    //unsafe{assert!(esp_idf_svc::sys::heap_caps_check_integrity_all(true));}

     unsafe{log::warn!("START: have {} largest free block size in bytes and total free heap mem is {}",
        esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_8BIT as _),
        esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_8BIT as _));
        
        // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_8BIT as _);    // DRAM
        // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_SPIRAM as _);  // PSRAM (if available)
        // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_EXEC as _);    // IRAM

        log::warn!("DMA mem is:");
        esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_DMA as _); //DMA
    
    }

    
    let timer_service= EspTaskTimerService::new().unwrap();
    let mut led_async_timer = timer_service.timer_async().unwrap();

    let peripherals = Peripherals::take().unwrap();
    

    let sda = peripherals.pins.gpio17; //I2C serial data line
    let scl = peripherals.pins.gpio18; //I2C serial clock line
    let i2c = peripherals.i2c0;

    let config = I2cConfig::new().baudrate(led::BAUDRATE_FAST.into());
    //for trouble shooting, maybe enable the following line
    //config =config.scl_enable_pullup(true).sda_enable_pullup(true).timeout(Duration::from_millis(20).into());
    let i2c = Mutex::new(I2cDriver::new(i2c, sda, scl, &config).unwrap());

    //Note we share an i2c bus with mutliple devices (the IO expander for LED control and also 
    //the codec to control the speaker later). However since we only use 1 device at a time this is fine.
    let shared_i2c = MutexDevice::new(&i2c);
    let mut led_driver = LedDriver::build(shared_i2c);

    led_driver.flip_led(led::LEDs::Blue);

    let mut x = 7;
    let y = 35;
    x += y;

    log::info!("Hello, world!");

    log::info!("x is {x}"); //This works fine.

    

    let modem= peripherals.modem;

    let blink_while_setting_wifi = async { 
        loop {
            led_driver.flip_led(led::LEDs::Blue);
            led_async_timer.delay_ms(500).await;
        }

    };

    let mut client = 
        match select(blink_while_setting_wifi,  wifi::setup_wifi(modem)).await
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

    
    unsafe{log::warn!("POST WIFI: have {} largest free block size in bytes and total free heap mem is {}",
        esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_8BIT as _),
        esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_8BIT as _));
        
        
        // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_8BIT as _);    // DRAM
        // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_SPIRAM as _);  // PSRAM (if available)
        // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_EXEC as _);    // IRAM

        log::warn!("DMA mem is:");
        esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_DMA as _); //DMA
    
    
    }

    //Can put this LED blinking in a seperate async task from here on out
    // loop {
    //     led_driver.flip_led(led::LEDs::Red);
    //     led_async_timer.delay_ms(500).await;
    // }

    let client = CLIENT.init(client);
    let mut current_station: usize = 0;

    //Initilizes the speaker hardware (codec & power amplifier)
    let shared_i2c = MutexDevice::new(&i2c);
    let pa_ctrl_pin= peripherals.pins.gpio48;
    let mut speaker_controller = SpeakerDriver::build(shared_i2c, pa_ctrl_pin);

    let mut adc1 = peripherals.adc1;
    let mut adc_pin = peripherals.pins.gpio5;
    
    let poll_buttons = 
    speaker_control(&mut speaker_controller, current_station, &mut adc1, &mut adc_pin);
    
    //If you want to diagnose the button values, uncomment the following:
    //Add call to diagnose here which never returns
    //diagnose(adc1, adc_pin);



    //We will need these later for the I2S driver. We just want to borrow these.

    let mut i2s0 = peripherals.i2s0;
    
    //We need to supply the following:
    //IO9=  I2S0_SCLK (this is the bit clock or BCLK)
    //IO45= I2S0_LRCK (ws is word select which is LRCK)
    //IO8= I2S0_DSDIN
    let (mut bclk, mut dout, mut ws) = (peripherals.pins.gpio9, peripherals.pins.gpio8, peripherals.pins.gpio45);

    //I2S0_MCLK would be gpio16. TODO: maybe set it to Some(gpio16)?
    //Master clock line. It is an optional signal depending on the slave side,
    // mainly used for offering a reference clock to the I2S slave device.
    //To set it to None use Option::None::<esp_idf_svc::hal::gpio::Gpio16>.
    let mut mclk = Some(peripherals.pins.gpio16);


    /* I2S notes ************************************************************************************/
    
    //Note that
    //i2s_driver.write_all_async(data);
    //i2s_driver.write_async(data);
    // will block if the DMA buffers are full!

    //Once a DMA buffer is full, it will be sent to the speaker for us.
    //dma_buffer_size = dma_frame_num * slot_num * slot_bit_width / 8.
    //(source: https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/i2s.html#data-transport)

    // const DMA_FRAMES_PER_BUFFER: u32 = 240;
    // const DMA_BUFFER_SIZE: usize = DMA_FRAMES_PER_BUFFER as usize * 2 * (24/8); // this is 1440. 
    //Note it is divisable by 3.

    //NOTE can revert to the above values, the problem is the reading from the stream,
    //Not that the DMA buffers are full!
    //TODO revert to the above values maybe

    //Try for smooth playback
    const DMA_FRAMES_PER_BUFFER: u32 = 681;
    const DMA_BUFFER_SIZE: usize = DMA_FRAMES_PER_BUFFER as usize * 2 * (24/8); // this is 4086.
    

    //interrupt_interval(unit: sec) = dma_frame_num / sample_rate
    //This is about 5.4ms of audio here.
    //So if we preload 6 DMA buffers, we have about 32ms of audio ahead of time.
    //lets try this first and see if it works smoothly! 

    //CRITICAL: could change dma_frame_num to 681 (which is divisable by 3).
    //This gives us 15.4 ms per DMA buffer and 92.6ms For all 6 DMA buffers.
    //It has DMA_BUFFER_SIZE= 4086 .
        
    //Source: https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/i2s.html#application-notes
    const {
        assert!(DMA_BUFFER_SIZE <= 4092);
        assert!(DMA_BUFFER_SIZE % 3 ==0);
        assert!(DMA_FRAMES_PER_BUFFER % 3 ==0);
    }

    //FOR DMA_FRAMES_PER_BUFFER = 240:
    //
    //2.5 ms of audio data is 1440 bytes (1 DMA buffer) at 96Khz sample rate (the max of the codec).
    //5ms of audio data is 1440 bytes at 48Khz sample rate (the highest sample rate I expect we will encounter)

    //At a network speed of 60 Mb/s, it would take approximately 4.27 milliseconds, to read 32 KB of data
    //(this is the max size read at a time by the media source by symphonia)
    //
    //So for our playback to never be choppy, we must have:
    //ms_a_dma_buffer_holds > 4.27
    //and we *should* have that.
    
    //If this becomes a problem, we simply set DMA_FRAMES_PER_BUFFER = 681,
    //For this value we have 7ms of audio data is a single DMA buffer (with that many frames) at 96Khz.
    
    //TODO: verify playback is never choopy, if it is increase dma_frames_per buffer (read here above).

    
    //The read of the source of the stream will be in chunks of size 32768/2.
    //This is slightly bigger than 4 DMA buffers of our max size.

    /* I2S notes ************************************************************************************/


    //TODO: decide if to uncomment to make continue 'start_stream below work
    // let raw_client: *mut Client<EspHttpConnection> = client;

    


    //Could spawn everything from here below in an async Task.
    //'start_stream: {

        //TODO: decide if to uncomment to make continue 'start_stream below work
        //SAFTEY: This is fine as we always drop the media stream before we get back here.
        //So there is always ever 1 single mut borrow of client active at any time.
        // let client: &'static mut Client<EspHttpConnection> = unsafe {
        //     &mut *raw_client
        // };


        // Send request
        //This GET request is equivalent to GET /977_CLASSROCK.mp3 HTTP/1.1 Accept: */* 
        let request = client.get( STATION_URLS[current_station]).unwrap();
        log::info!("-> GET {}", STATION_URLS[current_station]);
        let response = request.submit().unwrap();

        // Process response
        let status = response.status();
        //TODO: actually check if status is valid, and if not add retries? or just straight up panic?
        //I think add say 5 retries, and then change station to next one, if total retries exceed 20 (4 stations)
        //panic!
        log::info!("Status is: {status}");

        //MediaSourceStreamOptions has just 1 field: max buffer len which is by default 64kB.
        let media_stream = MediaSourceStream::new(
            Box::new(audio_stream::new_stream(response)), 
            Default::default());
        
        unsafe{
            log::warn!("POST MediaSourceStream: have {} largest free block size in bytes and total free heap mem is {}",
            esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_8BIT as _),
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_8BIT as _));
            
            
            // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_8BIT as _);    // DRAM
            // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_SPIRAM as _);  // PSRAM (if available)
            // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_EXEC as _);    // IRAM

            log::warn!("DMA mem is:");
            esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_DMA as _); //DMA
        
    
        }


        // Create a probe hint using the file's extension
        let mut hint = Hint::new();
        //TODO: some station urls can be like https://puma.streemlion.com:3130/stream .
        //So *maybe* make a station struct that specifies the format for the hint.
        //This is optional as it is ok for the hint to be wrong.

        //Note it is ok for the hint to be wrong.
        hint.mime_type("audio/mpeg");
        hint.with_extension("mp3");

        let format_opts: FormatOptions = Default::default();
        let metadata_opts: MetadataOptions = Default::default();
        
        //Create a probe which is used to automatically detect the media 
        //format and instantiate a compatible FormatReader.
        let mut probe_res = Box::new(
            symphonia::default::get_probe()
            .format(&hint, media_stream, &format_opts, &metadata_opts)
            .expect("Format should be identifiable by the probe"));
        
        log::info!("the metadata is {:#?} and the tracks are {:#?}", 
        probe_res.metadata.get(), probe_res.format.tracks());

        let first_supported_track =  probe_res.format.tracks().iter()
        .find(|t| t.codec_params.codec != symphonia::core::codecs::CODEC_TYPE_NULL)
        .expect("At least one track should be supported");

        log::info!("the id of first supported track is {:#?} and the codec param are {:#?}", 
        first_supported_track.id, first_supported_track.codec_params);

        let track_id = first_supported_track.id;

        /*We got (the other info was all None or Unknown). There was only 1 track and it was supported.
            Track {
            id: 0,
            codec_params: CodecParameters {
                codec: CodecType(
                    4099, //This is CODEC_TYPE_MP3
                ),
                sample_rate: Some(
                    44100,
                ),
                time_base: Some(
                    TimeBase {
                        numer: 1,
                        denom: 44100,
                    },
                ),

            This looks like CD quality: 16-bit/44.1 kHz is the recognized standard for audio CDs.
        */
        
        //TODO: is this default sound or should we just panic?? I am not sure
        //Our default sample rate if we can't find the actual value.
        let mut sample_rate: u32 = 44800;
        
        //TODO: Drop the i2S driver and reconfigure it to use actual sample rate!
        if let Some(actual_sample_rate) = first_supported_track.codec_params.sample_rate {
            
            //We can't reconfigure the driver using function. So we need to drop it and rebuild it.
            log::info!("Found actual sample rate: {}", actual_sample_rate);
            sample_rate = actual_sample_rate;
        }


        //Trying 24 bit depth, 44.1Khz. Can always use 16 bit depth (adjust speaker config in that case!)
        let i2s_config = StdConfig::new(
                        //7 DMA buffers, (note DMA_FRAMES_PER_BUFFER %3 == 0 as needed for 24 bit depth).
                        //auto_clear = true means that there will be silence if we have no new data to send.
                        //Note this acts as our jitter buffer!
            i2s::config::Config::default().auto_clear(true)
            .dma_buffer_count(7)
            .frames_per_buffer(DMA_FRAMES_PER_BUFFER), //Maybe need to adjust this Config in partiuclar

            i2s::config::StdClkConfig::from_sample_rate_hz(sample_rate)
            .mclk_multiple(i2s::config::MclkMultiple::M384),
            
            i2s::config::StdSlotConfig::philips_slot_default(i2s::config::DataBitWidth::Bits24, 
                i2s::config::SlotMode::Stereo),
                
            i2s::config::StdGpioConfig::default()
            );

        //IMPORTANT:
        //Make sure to drop this driver before making a new GET request that needs a different bit-width or 
        //sampling freq. We must drop this as the reconfig functions are not exposed to Rust from C.
        let mut i2s_driver= 
        I2sDriver::new_std_tx(&mut i2s0, &i2s_config, &mut bclk, &mut dout, mclk.as_mut(), &mut ws)
        .unwrap();
        
        unsafe{
            log::warn!("POST I2S driver: have {} largest free block size in bytes and total free heap mem is {}",
            esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_8BIT as _),
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_8BIT as _));
        
        
            // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_8BIT as _);    // DRAM
            // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_SPIRAM as _);  // PSRAM (if available)
            // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_EXEC as _);    // IRAM

            log::warn!("DMA mem is:");
            esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_DMA as _); //DMA
        
    
        }
       
        
        //We want to preload data onto the I2S TX channel at least 1 DMA buffer in size
        let mut preload = true; 
        let mut preloaded_bytes= 0;
        

        unsafe{
            log::warn!("LAST: PRE DECODER: have {} largest free block size in bytes and total free heap mem is {}",
            esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_8BIT as _),
            esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_8BIT as _));

                
            // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_8BIT as _);    // DRAM
            // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_SPIRAM as _);  // PSRAM (if available)
            // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_EXEC as _);    // IRAM

            log::warn!("DMA mem is:");
            esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_DMA as _); //DMA
        }


        //Instantiate a decoder.
        let decoder_options: symphonia::core::codecs::DecoderOptions = symphonia::core::codecs::DecoderOptions::default();
        let mut decoder =              
        symphonia::default::get_codecs()
        .make(&first_supported_track.codec_params, &decoder_options)
        .inspect_err(|_| 
            log::error!("Please only select radio stations with symphonia supported codecs"))
        .expect("Making a Decoder for a supported track should not fail");
        

        /*TODO: 
            If the above fails, then do
        
            based on codec_params type create the right decoder for us:
            /// MPEG Layer 3 (MP3)
            pub const CODEC_TYPE_MP3: CodecType = CodecType(0x1003);
            /// Advanced Audio Coding (AAC)
            pub const CODEC_TYPE_AAC: CodecType = CodecType(0x1004);


            //THIS WORKED!!! (even for all formats!)
            let mut aac_decoder = Box::new(
            <symphonia::default::codecs::AacDecoder as 
            symphonia::core::codecs::Decoder>::try_new(&first_supported_track.codec_params, &decoder_options)
            .expect("AAC decoder should be able to be set up"));
        
            log::warn!("AAC decoder init!");
        
            //THIS WORKED!!! (even for all formats!)
            let mut decoder = Box::new(
            <symphonia::default::codecs::MpaDecoder as 
            symphonia::core::codecs::Decoder>::try_new(&first_supported_track.codec_params, &decoder_options)
            .expect("MP3 decoder should be able to be set up"));

            // loop {            
            //     std::thread::sleep(std::time::Duration::from_millis(100));
            // }
        
        */

        log::info!("Decoder set up");
      
        
        let mut decoded_buf = None;

        //Decode and write audio packets to I2S driver.
        loop {
            match probe_res.format.next_packet()
            {
                Ok(packet) if packet.track_id() == track_id => {
                    //Only read packets that belong to our selected track.

                    match decoder.decode(&packet) {
                        Ok(decoded_audio) => {
                            
                            //We will copy the decoded audio into a raw sample buffer in an
                            // interleaved order. That will make decoded_buf be either some
                            // buffer of bytes (in the correct ordering for I2S), or None.
               
                            if decoded_buf.is_none() {

                                // Get the audio buffer specification.
                                let spec = *decoded_audio.spec();

                                // Get the capacity of the decoded buffer. Note: This is capacity, not length!
                                let duration = decoded_audio.capacity() as u64;
                                
                                //We use i24 as I2S data should be signed.
                                //source: https://en.wikipedia.org/wiki/I%C2%B2S#:~:text=Data%20is%20signed%2C%20encoded%20as,required%20between%20transmitter%20and%20receiver.
                                decoded_buf = Some(RawSampleBuffer::<symphonia::core::sample::i24>::new(duration, spec));
                                
                                //Maybe need to clamp samples to valid range? 
                                //Actually, made irrelavant by copy_interleaved_ref below.

                                //No need if we are expecting the original bit depth to be less or equal
                                // (i.e. 16 or 24), which we do.
                                
                            }

                            // Copy the decoded audio buffer into the sample buffer in an interleaved format.
                            if let Some(buf) = &mut decoded_buf {
                                buf.copy_interleaved_ref(decoded_audio);
                            }
                            
                        }

                        Err(symphonia_Error::DecodeError(_)) | Err(symphonia_Error::IoError(_)) => {continue;},

                        Err(symphonia_Error::ResetRequired) => {

                            //If ResetRequired is returned, consumers of the decoded audio data 
                            //should expect the duration and SignalSpec of the decoded audio buffer to change.

                            decoded_buf = None; 
                            
                            continue;
                        }

                        Err(err) => {panic!("{}", err);}
                    }
                },
                Err(symphonia_Error::ResetRequired) => {
                    // Restart Big loop (new get request with same value here I think)
                    //
                    // Or just do what it says: If ResetRequired is returned, 
                    //then the track list must be re-examined and all Decoders re-created.
                    //Add on Restart required error, to break out of this future with the current station number?
                    //TODO: Decide about if keeping this continue here or doing something else
                    
                    //continue 'start_stream; //fails because media source thinks it has &'static mut on client
                    //We can always uncomment the raw_client stuff to fix this

                    //Maybe the future approach works?
                    
                    //worst case just panic here and that restarts the entire program
                    panic!("RESTRAT REQ -- handel this! ");
                    todo!()
                },
                Err(err) => {panic!("{}", err);},
                Ok(_) => {continue;}
            }


            //Write the decoded_buf to the i2s TX channel.
            match &mut decoded_buf {
                Some(buf) => {
                    if preload {
                        let data = buf.as_bytes();
                        let new_loaded_bytes = i2s_driver.preload_data(data).unwrap();

                        preloaded_bytes += new_loaded_bytes;

                        //If we preloaded 1 DMA_BUFFER of audio data, start transmitting!
                        if preloaded_bytes >= DMA_BUFFER_SIZE {
                            preload = false;
                            i2s_driver.tx_enable().unwrap();
                            log::info!("Music start!")
                        }

                        //If we could not write all of this buf data into our DMA buffers
                        // (because our DMA buffers are full, for instance from previous station audio;
                        // or this buffer has more data than all of our DMA buffers).
                        // Then after enabling transimitting, finish writing this buf data.
                        if new_loaded_bytes < data.len() {

                            //CRITICAL: this will block if the DMA buffers are full!
                            i2s_driver.write_all_async(&data[new_loaded_bytes..]).await.unwrap();

                        }

                    }
                    else {
                        i2s_driver.write_all_async(buf.as_bytes()).await.unwrap();
                    }

                    log::info!("buffer decoded onto DMA!");
                },
                None => unreachable!(),
            }



        }


        //TODO: see if we can make dynamic station work: just from codec params here
        //propely configure the Speaker for I2S. 
        //
        //For espressif all they do is fixed parameter type stuff (they know bit rate and freq and
        //encoding ahead of time).
        // The alternative is that for every station we have a Station struct instance
        //where we would specify the codec used, the bit rate, the frequency and anything else
        //we would need to know to configure the speaker.



        //From Reading Symphonia: the MediaSourceStream has an internal ring buffer 
        //where we specify the max size for it (by default around 64kB max size; 
        //The max size we specify must be >32kB). 
        //It reads from the source (our response), only when needed.
        //When that happens, we read from the underlying HTTP stream.        
        loop {            
            std::thread::sleep(std::time::Duration::from_millis(100));
        }


        //TODO: Use embassy_sync::pipe to communicate between this writer and the reader that passes data
        //into I2S stream?
        
    //}

    //TODO: have in a select: 3 loops that never terminate: 
    //One that puts data from the decoder onto the DMA buffer.
    //Another that polls the buttons and mutes, unmutes, inc/dec vol from that future.
    //Another that blinks the RED led I think.
    //If there is a station change, the second future breakes from the loop (so it terminates),
    //with the value of what the station change should be. 
    //Then the bigger loop of get request to the new station, decoder, etc.. is set up again!
    //Maybe preload some data to DMA before enabling transmit channel.
    //I think I need to preload at least 1 DMA buffer length.


    //Can Join futures where one of them detects button presses in a loop
    //So it is impl Future<Output =!> . If we need to change to change the station we break out the loop.
    //and make a new get request

}


///Control the speaker hardware (not the I2S driver).
/// Polls the buttons for presses and mutes/unmutes, inc/dec volume accordingly.
/// On changing station (pressing the SET or REC button), returns the new station number.
/// 
/// **Note: ** You might want to call buttons::diagnose, and make sure these values also correspond to your buttons.
async fn speaker_control<I2C: I2c>(speaker_controller: &mut SpeakerDriver<I2C>, current_station: usize,
adc1: &mut esp_idf_svc::hal::adc::ADC1, adc_pin: &mut esp_idf_svc::hal::gpio::Gpio5)-> usize {
    let timer_service = EspTaskTimerService::new().unwrap();
    let mut button_async_timer = timer_service.timer_async().unwrap();

    loop{
        todo!()
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
        fn read(&mut self, mut buf: &mut [u8]) -> Result<usize, std::io::Error> {
            let mut lock = self
                .inner
                .lock()
                .expect("Only one thread/async task at a time should read the stream source");
            //TODO: change to trace or get rid of the line below. Change log below back to trace
            log::warn!("READING from Stream source now!");

            //TODO: maybe adjust this amount if needed. 
            //We can't handle reading max buf size that we will be given by Symphonia (32768 bytes!),
            //without a lag in audio playback.
            //So we make sure to read at most half that. Then Symphonia will adjust the buffer it gives
            //us to be that size from that point on.
            let max_internal = 32768/2;
            if buf.len() > max_internal {
                buf = &mut buf[..max_internal];
            }

            //Note that since the Mutex guarantees this thread to have exclusive access
            //to StreamSourceInner, we do not need to pay the additional performance cost of RefCell
            lock.inner.get_mut().read(buf)
            .inspect(|read_bytes| log::warn!("read {} bytes!", read_bytes))
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
    /// Values I got from calling diagnose (these are different than the board [schematic]):
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
    pub fn diagnose(adc1: ADC1, gpio_pin: Gpio5) -> ! {
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
        //It seems like nvs is needed to prevent a crash.
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
        
        //Async client not exposed to Rust (the async connection trait is not impl on EspHttpConnection)
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
/// LRCK (Left/Right data alignment clock) and SCLK (Bit clock for synchronisation)). 
/// This mode sets our MCU as the "master" for the I2S.
/// 
///* I2S capable of up to 24 bits.
/// 
///* The device can work in 2 speed modes: 
///     - Single Speed (Fs normally range from 8 kHz to 48 kHz).
///     - Double Speed (Fs normally range from 64 kHz to 96 kHz).
/// 
/// * The DAC only works in single speed mode.
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
///* The DAC has an equalizer, but we are not going to use it.
/// 
///* ES8311 has an internal power-on reset.
///
/// 
/// ### Implementation Notes
/// Note that we don't rely on being passed MCLK as a reference (that is you can set mclk= None for the I2S
/// driver).Instead we take BCLK as the source for the raw_internal_mclk.
/// 
/// As we can then multiply and divide raw_internal_mclk by values we select, by multiplying it by 8
/// we will have DAC internal clock freq = final internal mclk (post multiplying) freq = BCLK_freq * 8.
/// 
/// Note: The BCLK freq = sample rate * number of channels * bit depth.
/// Also, when using I2S standard stereo mode, LRCK (word select) has the *same* freq as our sample rate!
/// (https://en.wikipedia.org/wiki/I%C2%B2S)
/// 
/// This means for 16-bit depth audio: 
///* DAC internal clock freq = sample_rate * 2 * 16 * 8 = 256 * sample_rate = 256 * LRCK .
/// And for 24-bit depth audio:
///* DAC internal clock freq = sample_rate * 2 * 24 * 8 = 384 * sample_rate = 384 * LRCK .
/// 
/// So this approach sets ideal ratios for each bit depth, and it is completley sample rate agnositc!
/// 
/// Note that we must have internal_DAC_CLOCK_freq. <= 35 MHz (from the user guide page 15) if DVDD is 3.3V 
/// (which is the case for us).
/// 
/// Note that as the max sample rate we expect to encounter is 44800Hz our max internal_DAC_CLOCK_freq
/// will be 384 * 44800 = 17.2032 MHz so we are fine.
///
pub mod speaker {

    
    //!### Power Amplifier Notes
    //!ESP_IO48 is connected to PA_CTRL (from Korvo schematics).
    //!IMPORTANT: we must use this to power up this power amplifier!
    //![PA data sheet](https://www.alldatasheet.com/html-pdf/1131841/ETC1/NS4150/929/8/NS4150.html).
    //!PA_CTRL (simple High or Low setting; H: Open mode (i.e on), L: Shutdown, i.e off) 
    //!is power down control terminal.
    //!It used to make power consumption more efficent (draw lower power when on standby).
    //!So we want to drive this pin High before use, and drive it low post use 
    //!(we have driving it low be a part of the drop implementation).



    //For the codec (ES8311), our MCU pins we are interested in are ([source]):
    //IO16 = I2S0_MCLK (Master Clock) (this is actually not relevant for us; We don't depend on it).

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
    

    ///Note that this type simply controls the hardware via I2C and pa_ctrl_pin.
    /// I2S is managed separately.
    pub struct SpeakerDriver<I2C: I2c>  {
        i2c: I2C,
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
        /// 
        /// Note that SpeakerDriver does not rely on being passed MCLK as a reference 
        /// (that is you can set mclk = None for the I2S driver).
        /// 
        /// Note that the ratio between DAC internal clock and LRCK will be 
        /// 384 for 24 bit audio depth and 256 for 16 bit audio depth.
        pub fn build(i2c: I2C, pa_ctrl_pin: Gpio48) -> Self {

            let pa_ctrl_pin = PinDriver::output(pa_ctrl_pin).unwrap();
            let mut speaker_driver = Self{ i2c, pa_ctrl_pin, vol: Volume(0) };
            
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

                log::warn!("SPEAKER IDENTIFIED");

            }

            //Set up raw internal master clock multiplier
            speaker_driver.i2c.write(Self::I2C_ADDR, 
                &[Register::ClockFactors as u8 , 0b00011000]).unwrap();
            
            //Make sure we don't use DAC Equalizer
            speaker_driver.i2c.write(Self::I2C_ADDR, 
                &[Register::DacEqualizer as u8 , Register::DacEqualizer.default()]).unwrap();

            //We power up everything except ADC stuff

            //TODO: C CODE JUST WRITES 0x1 into this (below values adjusted to keep ADC stuff powered down).
            //Was 0b00110101 (TRY THIS AGAIN). IF THAT FAILS, I think try 0b00110001.
            //Maye try 0b00110101 agian
            speaker_driver.i2c.write(Self::I2C_ADDR, 
                &[Register::SysPwrMgt as u8 , 0b00110101]).unwrap();
            
            //Turn on relevant clocks
            speaker_driver.i2c.write(Self::I2C_ADDR, 
                &[Register::ClockManager as u8 , 0b10110101]).unwrap();

            //Power up DAC
            speaker_driver.i2c.write(Self::I2C_ADDR, 
                &[Register::SysEnableDac as u8 , 0]).unwrap();
            
            //Enable speaker drive
            let write_in = const {Register::SysEnableSpeakerDrive.default() | (1<<4)};
            
            speaker_driver.i2c.write(Self::I2C_ADDR, 
                &[Register::SysEnableSpeakerDrive as u8 , write_in]).unwrap();
            

            //This is the correct config for pa_ctrl_pin (gpio 48).
            //Unfortunately, there doesn't appear to be a way to set this as the config in a more idomatic way.
            let raw_config = esp_idf_svc::sys::gpio_config_t { 
                pin_bit_mask: (1u64 << 48), 
                mode: esp_idf_svc::sys::gpio_mode_t_GPIO_MODE_OUTPUT, 
                pull_up_en: esp_idf_svc::sys::gpio_pullup_t_GPIO_PULLUP_DISABLE,
                pull_down_en: esp_idf_svc::sys::gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
                intr_type: esp_idf_svc::sys::gpio_int_type_t_GPIO_INTR_DISABLE};

            
            unsafe { 
                //Set up the correct config.
                esp_idf_svc::sys::esp!(esp_idf_svc::sys::gpio_config(&raw_config)).unwrap();
            }
            
            log::info!("GPIO 48 should be correctly configured now");

            //Power up the power amplifier
            speaker_driver.pa_ctrl_pin.set_high().unwrap();

            //TODO: I don't think this is_set_high actually does anything
            assert!(speaker_driver.pa_ctrl_pin.is_set_high(), "PA pin not set high. PA is off");

            //set the volume to 60% as an initial value
            speaker_driver.set_vol(Volume::try_from(60).unwrap());

            
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
            
            //We could make this async, but this is not critical (1 time set up cost).
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

        ///Sets the volume. Takes an integral percentage of volume desired.
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



    //TODO: Think if we should change all I2C writes except in software reset,
    //to first read the register value and then write it back with just the bits we needed to adjust
    //changed. That is if we should do read-modify-write operations instead of just writing.
    //I see the C code sometimes does this and sometimes just writes directly into registers (even ones
    //with internal reserved bits-- which I never write into).



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

        ///A clock manager register.
        /// This register has default value of 0.
        /// 
        /// We want to write 10110101 into this register to use BCLK as source for raw_internal_master clock (before
        /// dividing and multiplying) and turn on the clocks we want (DAC).
        /// The C code writes in 1011 1111 into this OR 0011 1111 When using MCLK as a reference.
        ClockManager = 0x1,

        ///This is another clock manager register. 
        /// This register has a default value of 0.
        /// 
        /// We simply use it to set MULT_PRE to all 1's. So we want to write 0001 1000 into this.
        /// This means we will multiply the raw_internal_clock by 8. 
        /// Read the implementation notes of [speaker][self], to see why we do this.
        ClockFactors = 0x2,
        
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

        ///We want to bypass the DAC equalizer. Has default value of 0000 1000.
        /// We want to write that default value into this register as the docs are a bit unclear.
        /// It says it has this default value (so equalizer disabled by default), yet in the register
        /// description it seems to say the DAC equalizer is enabled by default.
        DacEqualizer = 0x37,

        ///READ ONLY. Should have a value of 0x83.
        ChipID1 = 0xFD,

        ///READ ONLY. Should have a value of 0x11.
        ChipID2 = 0xFE,


        //I covered all registers that might also be needed in the notes below!



        //-----------Maybe add--------------
        //In C, they adjust the default of register 0x10  (ES8311_SYSTEM_REG10)
        //to set  bits 2 and 3 to 1 as well
        //(highest bias level). The rest of the register they keep the same as the default.


        //REGISTER 0X04 – CLOCK MANAGER, DEFAULT 0001 0000 (sets DAC over sample rate)
        //C stuff keeps it at the default value for 44.1KHZ and 44.8Khz (and for all higher sample rates)
        //My understanding is that the audio difference is likely not something you'll notice.
        //but maybe set it to 128 instead of 64 and see.

        //The C code does this es8311_write_reg(codec, ES8311_GP_REG45, 0x00);
        //It writes in the default value into this register. might be worth trying.


        //----- Currently not needed----------------

        //Serial Digital Port In
        //bit 6 set to 1 mutes. Set to 0 (default) is unmute.
        // The other default values mean 24 bit I2S, Left channel to DAC.
        // See page 12 of the user guide for more info.
        // While this mute function should work, there is a cleaner way to do this, that does not result
        // in artifact sounds.
        //SdpIn = 0x09, NOT NEEDED

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
                Register::DacMute | Register::DacVol | Register::ClockManager | Register::ClockFactors => 0,
                Register::DacEqualizer => 0b0000_1000,
                Register::ChipID1 => 0x83,
                Register::ChipID2 => 0x11,
            }
        }
    }


}