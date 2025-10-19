use std::sync::Mutex;

use embedded_svc::http::client::Client;

//Note esp_idf_svc re-exports esp_idf_hal so we can just use that hal from here.
use esp_idf_svc::{
    hal::{
        adc::{attenuation::DB_11, 
            oneshot::{config::AdcChannelConfig, AdcChannelDriver, AdcDriver}}, 
            gpio::{Gpio16, Gpio45, Gpio8, Gpio9}, i2c::{I2cConfig, I2cDriver}, 
            i2s::{self, config::StdConfig, I2sDriver}, prelude::Peripherals
    },
    http::{client::EspHttpConnection, status},
    timer::EspTaskTimerService,
};

use symphonia::core::{audio::RawSampleBuffer, 
    formats::FormatOptions, io::MediaSourceStream, meta::MetadataOptions, probe::Hint};

use symphonia::core::errors::Error as symphonia_Error;

use embedded_hal::i2c::I2c;

use embedded_hal_bus::i2c::MutexDevice;

use embassy_executor::Spawner;
use embedded_hal_async::delay::DelayNs;
use embassy_futures::select::select;
use static_cell::StaticCell;


#[allow(unused)]
use crate::{buttons::diagnose, led::LedDriver, speaker::{Volume, SpeakerDriver}};


//-------------------------------------------------------------------------------------

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


//-------------------------------------------------------------------------------------



/*
If you are interested in memory usage information you could do:

#[allow(unused)]
use esp_idf_svc::sys::{MALLOC_CAP_8BIT, MALLOC_CAP_SPIRAM, MALLOC_CAP_EXEC, MALLOC_CAP_DMA};

unsafe{
    log::warn!("have {} largest free block size in bytes and total free heap mem is {}",
    esp_idf_svc::sys::heap_caps_get_largest_free_block(MALLOC_CAP_8BIT as _),
    esp_idf_svc::sys::heap_caps_get_free_size(MALLOC_CAP_8BIT as _));
    
    // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_8BIT as _);    // DRAM
    // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_SPIRAM as _);  // PSRAM
    // esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_EXEC as _);    // IRAM

    log::warn!("DMA mem is:");
    esp_idf_svc::sys::heap_caps_print_heap_info(MALLOC_CAP_DMA as _); //DMA
}
*/




//TODO: move into its own mod?
const STATION_URLS: [&'static str;7] = 
["https://18063.live.streamtheworld.com/977_CLASSROCK.mp3",
"https://puma.streemlion.com:3130/stream",
"https://ais-sa1.streamon.fm/7000_48k.aac",
"https://media-ice.musicradio.com/SmoothLondonMP3",
"https://broadcast.shoutstream.co.uk:8052/stream",
"https://streamer.radio.co/s52d0fa340/listen",
"https://stream.radiowavenz.com/stream",
];

//Mentions location simply means says name of station/ talks in a way that is identifiable
//
//977_CLASSROCK is 70's Rock - HitsRadio (North Carolina, USA) (Has ads so maybe get rid?)
//Also prone to timing out
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
//CKUA (Alberta, Canada) (audio/aac) (mentions location): https://ais-sa1.streamon.fm/7000_48k.aac
//Note it is actually 44.1KHz. (from https://www.radio.net/s/ckua).
//
//Timewarp Ireland (mentions location): https://broadcast.shoutstream.co.uk:8052/stream (audio/mpeg)
//(from https://www.radio.net/s/timewarpireland)



///Blink the red led forever. This is meant to indicate to the user the program has not crashed (is still running).
/// This way, if a certain station is not working, the user knows the problem is with the station,
///not the program.
/// 
/// Note this may hang a bit when switching between stations. The issue is that the GET requests are blocking
/// as the async API has not been exposed to Rust yet. While we could use raw calls to the underlying C bindings
/// to have an async connection, doing that just for this is not worth it.
#[embassy_executor::task]
async fn blink(mut led_driver: LedDriver<MutexDevice<'static, I2cDriver<'static>>>, 
    mut led_async_timer: esp_idf_svc::timer::EspAsyncTimer) {
    loop {
        led_driver.flip_led(led::LEDs::Red);
        led_async_timer.delay_ms(1000).await;
    }
}



//Ensures our client does not drop before
//our media_stream (which borrows the client mutablly). 
static CLIENT: StaticCell<Client<EspHttpConnection>> = StaticCell::new();

static I2C_GLOBAL: StaticCell<Mutex<I2cDriver<'static>>> = StaticCell::new();


///Our i2s driver uses DMA. This constant specifies how many audio frames we have in one DMA buffer.
///           
///Once a DMA buffer is full, it will be sent to the speaker for us (via an interrupt).
/// 
///interrupt_interval(unit: sec) = DMA_FRAMES_PER_BUFFER / sample_rate 
/// (this is relevant for sending audio with i2s as well).
///The bigger the better for performence ([source]).
/// 
///for DMA_FRAMES_PER_BUFFER = 681:
///This gives us at least 14ms of audio per DMA buffer (assuming we don't encounter
///sample rates higher than 48Khz).
/// 
///[source]: https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/i2s.html#data-transport
const DMA_FRAMES_PER_BUFFER: u32 = 681;

///The size of one DMA buffer (we use multiple DMA buffers in our i2s driver).
///dma_buffer_size = dma_frame_num * slot_num * slot_bit_width / 8 ([source]).
/// 
///[source]: https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/i2s.html#data-transport
const DMA_BUFFER_SIZE: usize = DMA_FRAMES_PER_BUFFER as usize * 2 * (24/8); // this is 4086.

///The maximum number of retries to attempt if failing to connect to a given internet radio station.
/// 
/// After this amount of retries, the program panics.
const RETRIES: usize = 5;


#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {

    //These are some *complie* time checks. 
    const { 
        assert!(DMA_BUFFER_SIZE <= 4092); //This is the max allowed size. See source in DMA_BUFFER_SIZE doc.
        assert!(DMA_BUFFER_SIZE % 3 ==0); //Important for 24 bit-depth audio
        assert!(DMA_FRAMES_PER_BUFFER % 3 ==0); //Important for 24 bit-depth audio

        assert!(!STATION_URLS.is_empty());

    }

    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();
    

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();


    let timer_service= EspTaskTimerService::new().unwrap();
    let mut led_async_timer = timer_service.timer_async().unwrap();

    let peripherals = Peripherals::take().unwrap();
    

    let sda = peripherals.pins.gpio17; //I2C serial data line
    let scl = peripherals.pins.gpio18; //I2C serial clock line
    let i2c = peripherals.i2c0;

    let config = I2cConfig::new().baudrate(led::BAUDRATE_FAST.into());
    let i2c: Mutex<I2cDriver<'static>>  = Mutex::new(I2cDriver::new(i2c, sda, scl, &config).unwrap());
    let i2c = I2C_GLOBAL.init(i2c);

    //Note we share an i2c bus with mutliple devices (the IO expander for LED control and also 
    //the codec to control the speaker later). However, since we only use 1 device at a time this is fine.
    let shared_i2c  = MutexDevice::new(i2c);
    let mut led_driver  = LedDriver::build(shared_i2c);

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
    
    //Note we rely in the stream function on having done this here. This makes later code slightly more ergonomic.
    //It also serves as a sanity check.
    wifi::test_get_request(&mut client).expect("wifi get request test should not fail");

    log::info!("Wifi should be set up. Blue led should stop blinking");

    //The Blue light could be off at this time. Thus we switch it on if need be.
    //This indicates to the user the wifi is all set.
    match led_driver.get_led_states() {
        (_, false) => led_driver.flip_led(led::LEDs::Blue),
        _ => ()
    };

    
    //Spawn an async task that continually 
    //blinks the red led to let the user know the program is still running (has not crashed).
    spawner.spawn(blink(led_driver, led_async_timer)).unwrap();

    
    let client = CLIENT.init(client);
    let mut current_station: usize = 0;

    //Initilizes the speaker hardware (codec & power amplifier)
    let shared_i2c = MutexDevice::new(i2c);
    let pa_ctrl_pin= peripherals.pins.gpio48;
    let mut speaker_controller = SpeakerDriver::build(shared_i2c, pa_ctrl_pin);

    
    //We need the following for the I2S driver later. 
    let mut i2s0 = peripherals.i2s0;

    //Pin used for sending serial data out to the codec.
    let mut dout = peripherals.pins.gpio8;

    //The I2S bit clock. Also known as the SCLK (serial clock).
    let mut bclk = peripherals.pins.gpio9;

    //The I2S word select clock. Also known as the LRCK (left-right clock).
    let mut ws = peripherals.pins.gpio45; 

    //Master clock line. It is an optional signal depending on the slave side,
    // mainly used for offering a reference clock to the I2S slave device.
    let mut mclk = Some(peripherals.pins.gpio16);

    
    let raw_client: *mut Client<EspHttpConnection> = client;

    //For polling the buttons (which are connected to a single pin in a resistor ladder).
    let mut adc1 = peripherals.adc1;
    let mut adc_pin = peripherals.pins.gpio5;


    loop {
        
        let poll_buttons = 
        speaker_control(&mut speaker_controller, current_station, &mut adc1, &mut adc_pin);
        //If you want to diagnose the button values, uncomment the following:
        //buttons::diagnose(adc1, adc_pin);

        let stream = 
        stream(raw_client, current_station, &mut i2s0, &mut bclk, &mut dout, &mut ws, &mut mclk);

        
        //Play the stream while polling the buttons. 
        //Update the station on the relevant button press.
        match select( stream,  poll_buttons).await
        {
            embassy_futures::select::Either::First(_) => (),

            embassy_futures::select::Either::Second(new_station_number) =>
             {current_station = new_station_number},
        };
        
    };

}


///Control the speaker hardware (not the I2S driver).
/// Polls the buttons for presses and mutes/unmutes, inc/dec volume accordingly.
/// On changing station (pressing the SET or REC button), returns the new station number.
/// 
/// **Note:** You might want to call [self::buttons::diagnose], 
/// and make sure these values also correspond to your buttons.
async fn speaker_control<I2C: I2c>(speaker_controller: &mut SpeakerDriver<I2C>, current_station: usize,
adc1: &mut esp_idf_svc::hal::adc::ADC1, adc_pin: &mut esp_idf_svc::hal::gpio::Gpio5)-> usize {

    const VOL_ADJUST_AMOUNT: u8 = 5;

    let timer_service = EspTaskTimerService::new().unwrap();
    let mut button_async_timer = timer_service.timer_async().unwrap();


    let adc = AdcDriver::new(adc1).unwrap();

    // Configuring pin to analog read, you can regulate the adc input voltage range depending on your need
    // we use the attenuation of 11db which sets the input voltage range to around 0-3.1V on the esp32-S3.
    let config = AdcChannelConfig {
        attenuation: DB_11,
        ..Default::default()
    };

    let mut adc_pin = AdcChannelDriver::new(&adc, adc_pin, &config).unwrap();

    loop {
        button_async_timer.delay_ms(100).await;
        //adc.read returns the voltage of the pin.
        match adc.read(&mut adc_pin).unwrap() {
            220..=420 => {
                log::info!("VOL+ button pressed!"); 
                let current_vol = speaker_controller.get_current_vol();
                let new_vol = current_vol.saturating_add(VOL_ADJUST_AMOUNT);
                speaker_controller.set_vol(Volume::try_from(new_vol).unwrap());
            },

            620..=820 => {
                log::info!("VOL- button pressed!"); 
                let current_vol = speaker_controller.get_current_vol();
                let new_vol = current_vol.saturating_sub(VOL_ADJUST_AMOUNT);
                speaker_controller.set_vol(Volume::try_from(new_vol).unwrap());
            },

            890..=1090 => {
                log::info!("SET (station back) button pressed!");
                //This calculates the previous station
                break (current_station + (STATION_URLS.len()-1)) % STATION_URLS.len()

            },

            1400..=1600 => {
                log::info!("PLAY (unmute) button pressed!");
                speaker_controller.unmute(); 
            },

            1710..=1910 => {
                log::info!("MUTE button pressed!");
                speaker_controller.mute(); 
            },

            2110..=2310 => {
                log::info!("REC (station forward) button pressed!");
                //Return the new station number
                break (current_station + 1) % STATION_URLS.len()
            },

            3000.. => log::trace!("No button press"),

            res => log::error!("Could not definitively identify which button was pressed. Got value {res}")

        }
       
    }

}


///Stream the audio from an internet radio station. 
/// Returns on [symphonia_Error::ResetRequired]. 
///
/// ## Panics
/// If connecting to the current_station fails RETRIES times, this panics.
/// Also panics on other unrecoverable errors.
/// 
async fn stream(raw_client: *mut Client<EspHttpConnection>, current_station: usize, i2s0: &mut i2s::I2S0, 
 bclk: &mut Gpio9, dout: &mut Gpio8, ws: &mut Gpio45, mclk: &mut Option<Gpio16>) {

    //TODO: move this into its own function (that we call to get reponse)?
    let mut connection_attempts = RETRIES + 1;
    let response= loop {

        if connection_attempts == 0 {
            panic!("Failed to connect to {} with {} retries", STATION_URLS[current_station], RETRIES);
        }

        //SAFTEY:
        //MediaSourceStream demands a 'static lifetime of everything involved in making it.
        //This is fine as we always drop the media stream before the stream function is called again.
        //So there is always ever 1 single mut borrow of client (which was placed in a static cell)
        //active at any time, and that borrow lasts as long as *actually* needed.
        let client: &'static mut Client<EspHttpConnection> = unsafe {
            &mut *raw_client
        };

        //Send GET request
        /*
            As we did a test get request earlier, we can just use this modify_get_request.
            We needed to make this function in a fork of esp-idf-svc as this functionality was not
            exposed in Rust APIs. The alternative was to drop and recreate the EspHttpConnection
            on each station change which is inefficent.

            The problem with existing Rust APIs is that they don't consider the possbility that the HTTP response
            is an endless stream of data, and try to flush the old response on a new get request.
            So for us, it will block forever.
        */
        client.connection().modify_get_request(STATION_URLS[current_station]).unwrap();
        let request = 
        embedded_svc::http::client::Request::wrap(client.connection());

        log::info!("-> GET {}", STATION_URLS[current_station]);
        let response = request.submit().unwrap();

        // Process response
        let status = response.status();
        log::info!("Status is: {status}");

        if status::OK.contains(&status) {
            break response;
        }

        connection_attempts -=1;
    };

    //MediaSourceStreamOptions has just 1 field: max buffer len which is by default 64kB.
    //Unfortunately, that is the smallest allowed size.
    //The MediaSourceStream has an internal ring buffer. 
    //It reads from the source (our response), only when needed.
    //When that happens, we read from the underlying HTTP stream.
    let media_stream = MediaSourceStream::new(
        Box::new(audio_stream::new_stream(response)), 
        Default::default());


    // Create a probe hint.
    //Note it is ok for the hint to be wrong, and it seems like most stations are mp3.
    let mut hint = Hint::new();
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
    
    log::info!("The metadata is {:#?}", probe_res.format.metadata().skip_to_latest());
    log::info!("Additional metadata is {:#?} and the tracks are {:#?}", 
    probe_res.metadata.get(), probe_res.format.tracks());

    let first_supported_track =  probe_res.format.tracks().iter()
    .find(|t| (t.codec_params.codec == symphonia::core::codecs::CODEC_TYPE_MP3) 
        || (t.codec_params.codec == symphonia::core::codecs::CODEC_TYPE_AAC))
    .ok_or_else(|| {
        log::error!("Please only select radio stations with supported codecs (MP3 or AAC)");
        Err::<&symphonia::core::formats::Track, _>("No Tracks with supported codecs found")
    })
    .expect("At least one track should have a supported codec");

    log::info!("the id of first supported track is {:#?} and the codec param are {:#?}", 
    first_supported_track.id, first_supported_track.codec_params);

    
    let sample_rate = 
        if let Some(sample_rate) = first_supported_track.codec_params.sample_rate {
            log::info!("Found sample rate: {}", sample_rate);
            sample_rate
        }
        else {
            panic!("Could not identify the sample rate")
        };


    /*
        We configured the speaker to use 24 bit-depth audio.
        Once a DMA buffer is full, it will be sent to the speaker for us.
        auto_clear = true means that there will be silence if we have no new data to send.
        Note this acts as our jitter buffer for the audio.
    */ 
    let i2s_config = StdConfig::new(
        i2s::config::Config::default().auto_clear(true)
        .dma_buffer_count(7)
        .frames_per_buffer(DMA_FRAMES_PER_BUFFER),

        i2s::config::StdClkConfig::from_sample_rate_hz(sample_rate)
        .mclk_multiple(i2s::config::MclkMultiple::M384),
        
        i2s::config::StdSlotConfig::philips_slot_default(i2s::config::DataBitWidth::Bits24, 
            i2s::config::SlotMode::Stereo),
            
        i2s::config::StdGpioConfig::default()
        );

    
    // We must drop and remake this driver for each stream,
    // as the reconfig functions are not exposed to Rust from C.
    let mut i2s_driver= 
    I2sDriver::new_std_tx(i2s0, &i2s_config, bclk, dout, mclk.as_mut(), ws)
    .unwrap();

    
    //We want to preload data onto the I2S TX channel at least 1 DMA buffer in size.
    //This helps to ensure smooth playback.
    let mut preload = true; 
    let mut preloaded_bytes= 0;


    //Instantiate a decoder.
    let decoder_options: symphonia::core::codecs::DecoderOptions = symphonia::core::codecs::DecoderOptions::default();
    let mut decoder =              
    symphonia::default::get_codecs()
    .make(&first_supported_track.codec_params, &decoder_options)
    .expect("Making a Decoder for a supported track should not fail");
    

    log::info!("Decoder set up");

    let track_id = first_supported_track.id;
    
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

                            // Get the capacity of the decoded buffer. Note: This is capacity, not length.
                            let duration = decoded_audio.capacity() as u64;
                            
                            // We use i24 as I2S data should be signed.
                            decoded_buf = Some(RawSampleBuffer::<symphonia::core::sample::i24>::new(duration, spec));
                            
                        }

                        // Copy the decoded audio buffer into the sample buffer in an interleaved format.
                        // Note this performs any required conversions.
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
                //If ResetRequired is returned, 
                //then the track list must be re-examined and all decoders re-created
                //(this involves redoing the GET request to get a new response).
                //We can do this simply by returning from this function.
                return;
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

                    //If we could not write all of this buf data into our DMA buffers,
                    // then after enabling transmitting, finish writing this buf data.
                    if new_loaded_bytes < data.len() {

                        //Note this will await if the DMA buffers are full!
                        i2s_driver.write_all_async(&data[new_loaded_bytes..]).await.unwrap();

                    }

                }
                else {
                    i2s_driver.write_all_async(buf.as_bytes()).await.unwrap();
                }

                log::trace!("buffer decoded onto DMA!");
            },
            None => unreachable!(),
        }

    }
       
}


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

            //TODO: maybe adjust this amount if needed. 
            //We sometimes can't handle reading max buf size that we will be given by Symphonia (32768 bytes!),
            //without a lag in audio playback.
            //So we make sure to read at most half that. Then Symphonia will adjust the buffer it gives
            //us to be that size from that point on.

            //This is only needed for a particular mp3 stream it seems!
            //Just for this one https://18063.live.streamtheworld.com/977_CLASSROCK.mp3
            let max_internal = 32768/2;
            if buf.len() > max_internal {
                buf = &mut buf[..max_internal];
            }

            //Note that since the Mutex guarantees this thread to have exclusive access
            //to StreamSourceInner, we do not need to pay the additional performance cost of RefCell
            lock.inner.get_mut().read(buf)
            .inspect(|read_bytes| log::info!("Read {} bytes from stream source!", read_bytes))
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
        
        // Create HTTPS client
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
/// LRCK (Left/Right data alignment clock) and SCLK (Bit clock for synchronisation; This is also sometimes called BCLK)). 
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
        /// 384 for 24 bit audio depth and 256 for 16 bit audio depth (this speaker will be configured
        /// for 24 bit audio depth).
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

            //TODO: maybe need to comment unmute out and try playing with software reset.
            //maybe set all reset bits to 1 with poweron instead of power off.
            //For some reason, the reset procedure does not appear to effect this register.
            //So we make sure the speaker is unmuted.
            speaker_driver.unmute();

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

        pub fn get_current_vol(&self) -> u8 {
            self.vol.0
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
        //C stuff keeps it at the default value for 44.1KHZ and 48Khz (and for all higher sample rates)
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