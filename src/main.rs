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
    http::{self, client::{self, EspHttpConnection}},
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


const STATION_URLS: [&str;7] = 
["https://18063.live.streamtheworld.com/977_CLASSROCK.mp3",
"https://puma.streemlion.com:3130/stream",
"https://ais-sa1.streamon.fm/7000_48k.aac",
"https://media-ice.musicradio.com/SmoothLondonMP3",
"https://broadcast.shoutstream.co.uk:8052/stream",
"https://streamer.radio.co/s52d0fa340/listen",
"https://stream.radiowavenz.com/stream",
];


//Ensures our client does not drop before
//our media_stream (which borrows the client mutably). 
static CLIENT: StaticCell<Client<EspHttpConnection>> = StaticCell::new();

static I2C_GLOBAL: StaticCell<Mutex<I2cDriver<'static>>> = StaticCell::new();


///Our i2s driver uses DMA. This constant specifies how many audio frames we have in one DMA buffer.
///           
///Once a DMA buffer is full, it will be sent to the speaker for us (via an interrupt).
/// 
///interrupt_interval(unit: sec) = DMA_FRAMES_PER_BUFFER / sample_rate 
/// (this is relevant for sending audio with i2s as well).
///The bigger the better for performance ([source]).
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
const RETRIES: usize = 5;

mod led;
mod buttons;
mod wifi;
mod speaker;

#[embassy_executor::main]
async fn main(spawner: Spawner) -> ! {

    //These are some *compile* time checks. 
    const { 
        assert!(DMA_BUFFER_SIZE <= 4092); //This is the max allowed size. See source in DMA_BUFFER_SIZE doc.
        assert!(DMA_BUFFER_SIZE % 3 ==0); //Important for 24 bit-depth audio
        assert!(DMA_FRAMES_PER_BUFFER % 3 ==0); //Important for 24 bit-depth audio

        assert!(!STATION_URLS.is_empty());
    }

    //debug_assertions is by default enabled for non-release builds and disabled for release builds.
    #[cfg(debug_assertions)]
    {
        //Ensure our shared I2C bus later can use just a single baudrate.
        assert_eq!(led::BAUDRATE_FAST, speaker::BAUDRATE);
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

    //Note we share an i2c bus with multiple device drivers (the IO expander for LED control and 
    //the codec to control the speaker later). 
    //Note that since we can use at most 1 device at a time this mutex will never block 
    //(because Embassy here utilizes a single core).
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
    if let (_, false) = led_driver.get_led_states() {
        led_driver.flip_led(led::LEDs::Blue);
    };

    
    //Spawn an async task that continually 
    //blinks the red led to let the user know the program is still running (has not crashed).
    spawner.spawn(blink(led_driver, led_async_timer)).unwrap();

    
    let client = CLIENT.init(client);
    let mut current_station: usize = 0;

    //Initializes the speaker hardware (codec & power amplifier)
    let shared_i2c = MutexDevice::new(i2c);
    let pa_ctrl_pin= peripherals.pins.gpio48;
    let mut speaker_controller = 
    SpeakerDriver::build(shared_i2c, pa_ctrl_pin, Volume::try_from(60).unwrap());

    
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
        unsafe {
            stream(raw_client, current_station, &mut i2s0, &mut bclk, &mut dout, &mut ws, &mut mclk)
        };

        
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
/// Polls the buttons (which are connected to a single pin in a resistor ladder) 
/// for presses and mutes/unmutes, inc/dec volume accordingly.
/// On changing station (pressing the SET or REC button), returns the new station number.
/// 
///### Notes
///* You might want to call [buttons::diagnose][self::buttons::diagnose], 
///  and make sure these values also correspond to your buttons.
/// 
///* Note that ADC2 is also used by the Wifi ([source]), so we use ADC1.
/// 
///[source]: https://docs.espressif.com/projects/esp-idf/en/v5.5.1/esp32s3/api-reference/peripherals/adc_oneshot.html#hardware-limitations.
async fn speaker_control<I2C: I2c>(speaker_controller: &mut SpeakerDriver<I2C>, current_station: usize,
adc1: &mut esp_idf_svc::hal::adc::ADC1, adc_pin: &mut esp_idf_svc::hal::gpio::Gpio5)-> usize {

    const VOL_ADJUST_AMOUNT: u8 = 5;

    let timer_service = EspTaskTimerService::new().unwrap();
    let mut button_async_timer = timer_service.timer_async().unwrap();


    let adc = AdcDriver::new(adc1).unwrap();

    // Configuring pin to analog read, you can regulate the adc input voltage range depending on your need.
    // We use the attenuation of 11db which sets the input voltage range to around 0-3.1V on the esp32-S3.
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
/// ## Safety
/// You must ensure that raw_client is a raw pointer to a Client which will *never* be invalidated.
/// In other words, that it can be safely converted from a `*mut Client` into a `&'static mut Client`.
async unsafe fn stream(raw_client: *mut Client<EspHttpConnection>, current_station: usize, i2s0: &mut i2s::I2S0, 
 bclk: &mut Gpio9, dout: &mut Gpio8, ws: &mut Gpio45, mclk: &mut Option<Gpio16>) {

    let response = connect(raw_client, current_station);

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


///Connect to the internet radio station.
fn connect(raw_client: *mut Client<EspHttpConnection>,  current_station: usize)-> 
client::Response<&'static mut EspHttpConnection> {

    let mut connection_attempts = RETRIES + 1;

    loop {

        if connection_attempts == 0 {
            panic!("Failed to connect to {} with {} retries", STATION_URLS[current_station], RETRIES);
        }

        //SAFTEY:
        //MediaSourceStream demands a 'static lifetime of everything involved in making it.
        //This is fine as we always drop the media stream before the stream function is called again.
        //So there is always ever 1 single mut borrow of Client (which was placed in a static cell)
        //active at any time, and that borrow lasts as long as *actually* needed.
        let client: &'static mut Client<EspHttpConnection> = unsafe {
            &mut *raw_client
        };

        //Send GET request
        /*
            As we did a test get request earlier, we can just use this modify_get_request.
            We needed to make this function in a fork of esp-idf-svc as this functionality was not
            exposed in Rust APIs. The alternative was to drop and recreate the EspHttpConnection
            on each station change which is inefficient.

            The problem with existing Rust APIs is that they don't consider the possibility that the HTTP response
            is an endless stream of data, and try to flush the old response on a new get request.
            So for us, it will block forever.
        */
        client.connection().modify_get_request(STATION_URLS[current_station]).unwrap();
        let request = 
        embedded_svc::http::client::Request::wrap(client.connection());

        log::info!("-> GET {}", STATION_URLS[current_station]);
        
        if let Ok(response) = request.submit() 
        {
            // Process response
            let status = response.status();
            log::info!("Status is: {status}");

            if http::status::OK.contains(&status) {
                return response;
            }
 
        }

        log::warn!("Failed to connect, retrying now");

        connection_attempts -=1;
    };

}


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


mod audio_stream {
    use std::io::Error;
    use symphonia::core::io::ReadOnlySource;
    use esp_idf_svc::http::client::{EspHttpConnection, Response};
    use std::sync::Mutex;

    /// A wrapper around the Response from the GET request.
    /// It implements std::io:Read by calling into the inner read which itself calls 
    /// the embedded::io::Read implementation on the underlying connection.
    /// This allows us to bridge the gap from the HTTPS Response to Symphonia's MediaSource.
    pub struct StreamSource<'a> {
        //We wrap StreamSourceInner in a mutex because we needed the StreamSource
        //to impl Sync as well so we would have
        //impl<R: Read + Send + Sync> MediaSource for ReadOnlySource<R> as needed.
        inner: Mutex<StreamSourceInner<'a>>
    }

    struct StreamSourceInner<'a> {
        inner: Response<&'a mut EspHttpConnection>
    }

    impl std::io::Read for StreamSource<'_> {
        fn read(&mut self, mut buf: &mut [u8]) -> Result<usize, std::io::Error> {
            
            //We sometimes can't handle reading the max buf size 
            //that we will be given by Symphonia (32768 bytes!),
            //without a lag in audio playback.
            //So we make sure to read at most half that.
            let max_internal = 32768/2;
            if buf.len() > max_internal {
                buf = &mut buf[..max_internal];
            }
            
            //Note that get_mut() enables us to avoid paying the performance hit of using a Mutex.
            self
            .inner
            .get_mut()
            .unwrap()
            .inner
            .read(buf)
            .inspect(|read_bytes| log::info!("Read {} bytes from stream source!", read_bytes))
            .map_err(Error::other)
        }
    }

    
    pub fn new_stream<'a> (
        response: Response<&'a mut EspHttpConnection>,
    ) -> ReadOnlySource<StreamSource<'a>> {
        ReadOnlySource::new(StreamSource {
            inner: Mutex::new(StreamSourceInner {
                inner: response,
            }),
        })
    }
    

    ///SAFTEY: 
    /// Note that the Response uniquely accesses the underlying HTTP stream.
    /// 
    /// We needed to implement this because EspHttpConnection has the following field:
    ///raw_client: *mut esp_http_client. 
    unsafe impl<'a> Send for StreamSourceInner<'a> {}
    
}
