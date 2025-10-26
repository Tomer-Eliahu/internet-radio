//! This module is comprised of both the Codec and the Power Amplifier.
//! 
//! ## Codec
//! The board has an ES8311 low-power mono audio codec chip.
//! 
//! - [The data sheet](https://www.lcsc.com/datasheet/C962342.pdf).
//! - [The extended data sheet](https://www.lcdwiki.com/res/PublicFile/ES8311_DS.pdf) 
//!   (this includes register descriptions).
//! - There is a lot more information in the 
//!   [user guide](https://files.waveshare.com/wiki/common/ES8311.user.Guide.pdf).
//! 
//! We can completely configure this device using I2C.
//! 
//! Here is some of the relevant information:
//! 
//! - DAC: 24-bit, 8 to 96 kHz sampling frequency.
//! 
//! - I2S capable of up to 24 bits.
//! 
//! - The ratio between internal DAC clock and LRCK must be equal or greater than 256, 
//!   and this ratio must be integral multiple of sixteen.
//! 
//! - We can increase/decrease volume and even mute using this device.
//! 
//! - The DAC is also capable of DSP/PCM mode serial audio data format instead of I2S. 
//!   With this mode we can use 32-bit depth (but there is no *audible* benefit of using that).
//! 
//! - We can enable the Dynamic Range Control (a.k.a. Dynamic Range Compression) feature of the DAC, 
//!   but since we are using it to listen to music here, we will leave it disabled.
//! 
//! - The DAC has an equalizer, but we are not going to use it.
//! 
//! - ES8311 has an internal power-on reset.


//! ### Implementation Notes
//! Note that we don't rely on being passed MCLK as a reference (that is you can `let mclk = None` for the I2S
//! driver). Instead we take BCLK as the source for the raw_internal_mclk.
//! 
//! As we can then multiply and divide raw_internal_mclk by values we select, by multiplying it by 8
//! we will have DAC internal clock freq = final internal mclk freq = BCLK_freq * 8.
//! 
//! Note: The BCLK freq = sample rate * number of channels * bit depth.
//! Also, when using I2S standard stereo mode, LRCK (word select) has the *same* freq as our sample rate
//! ([see wiki](https://en.wikipedia.org/wiki/I%C2%B2S)).
//! 
//! This means for 16-bit depth audio: 
//!     - DAC internal clock freq = sample_rate * 2 * 16 * 8 = 256 * sample_rate = 256 * LRCK .
//! 
//! And for 24-bit depth audio:
//!     - DAC internal clock freq = sample_rate * 2 * 24 * 8 = 384 * sample_rate = 384 * LRCK .
//! 
//! So this approach sets ideal ratios for each bit depth, and it is completely sample rate agnostic!
//! 
//! Note that we must have internal_DAC_CLOCK_freq. <= 35 MHz (from the user guide page 15) if DVDD is 3.3V 
//! (which is the case for us).
//! 
//! Note that as the max sample rate we expect to encounter is 48000Hz our max internal_DAC_CLOCK_freq
//! will be 384 * 48000 = 18.432 MHz so we are fine.


//! ## Power Amplifier Notes
//! - [PA data sheet](https://www.alldatasheet.com/html-pdf/1131841/ETC1/NS4150/929/8/NS4150.html).
//! 
//! ESP_IO48 is connected to PA_CTRL (from [Korvo schematics]).
//! We use this to power up this power amplifier.
//! PA_CTRL is used to make power consumption more efficient (draw lower power when on standby).
//! So we want to drive this pin high (i.e. on) before use, and drive it low post use 
//! (we have driving it low be a part of [SpeakerDriver]'s drop implementation).
//! 
//! [Korvo schematics]: https://dl.espressif.com/dl/schematics/SCH_ESP32-S3-Korvo-2_V3.1.2_20240116.pdf
//! 
//! ## Additional Resources
//! This [C implementation] was useful as a sanity check (we do deviate from it somewhat).
//! 
//! [C implementation]: https://github.com/espressif/esp-adf/tree/894a483eab7ef4868baa2865e7b16fca28b2b381/components/esp_codec_dev/device/es8311
//! 



use std::time::Duration;

use embedded_hal::i2c::I2c;

use esp_idf_svc::hal::{gpio::{Gpio48, PinDriver}, prelude::*};


///I2C Baudrate Max clock frequency in KHz.
pub const BAUDRATE: KiloHertz = KiloHertz(400);
#[allow(unused)]
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
    pub const I2C_ADDR: u8 = 0b00011000;


    ///Create and initialize the Speaker Driver.
    /// This function initializes the codec and also turns on the power amplifier (PA).
    /// It then sets the volume to be `initial_vol`.
    /// 
    /// Note that SpeakerDriver does not rely on being passed MCLK as a reference 
    /// (that is you can set mclk = None for the I2S driver).
    /// 
    /// Note that the ratio between DAC internal clock and LRCK will be 
    /// 384 for 24 bit depth audio and 256 for 16 bit depth audio (this speaker will be configured
    /// for 24 bit depth audio).
    pub fn build(i2c: I2C, pa_ctrl_pin: Gpio48, initial_vol: Volume) -> Self {

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

            log::info!("SPEAKER IDENTIFIED");

        }

        //Set up raw internal master clock multiplier
        speaker_driver.i2c.write(Self::I2C_ADDR, 
            &[Register::ClockFactors as u8 , 0b00011000]).unwrap();
        
        //Make sure we don't use DAC Equalizer
        speaker_driver.i2c.write(Self::I2C_ADDR, 
            &[Register::DacEqualizer as u8 , Register::DacEqualizer.default()]).unwrap();

        //We power up everything except ADC stuff
        speaker_driver.i2c.write(Self::I2C_ADDR, 
            &[Register::SysPwrMgt as u8 , 0b00110101]).unwrap();
        
        //Turn on relevant clocks
        speaker_driver.i2c.write(Self::I2C_ADDR, 
            &[Register::ClockManager as u8 , 0b10110101]).unwrap();

        //Power up DAC
        speaker_driver.i2c.write(Self::I2C_ADDR, 
            &[Register::SysEnableDac as u8 , 0x1]).unwrap();
        
        //Enable speaker drive
        let write_in = const {Register::SysEnableSpeakerDrive.default() | (1<<4)};
        
        speaker_driver.i2c.write(Self::I2C_ADDR, 
            &[Register::SysEnableSpeakerDrive as u8 , write_in]).unwrap();
        
        //Set the volume to the initial value
        speaker_driver.set_vol(initial_vol);

        //For some reason, the reset procedure does not appear to affect this register.
        //So we make sure the speaker is unmuted.
        speaker_driver.unmute();

        //This is the correct config for pa_ctrl_pin (gpio 48).
        //Unfortunately, there doesn't appear to be a way to set this as the config in a more idiomatic way.
        let raw_config = esp_idf_svc::sys::gpio_config_t { 
            pin_bit_mask: (1u64 << 48), 
            mode: esp_idf_svc::sys::gpio_mode_t_GPIO_MODE_OUTPUT, 
            pull_up_en: esp_idf_svc::sys::gpio_pullup_t_GPIO_PULLUP_DISABLE,
            pull_down_en: esp_idf_svc::sys::gpio_pulldown_t_GPIO_PULLDOWN_DISABLE,
            intr_type: esp_idf_svc::sys::gpio_int_type_t_GPIO_INTR_DISABLE
        };

        unsafe { 
            //Set up the correct config.
            esp_idf_svc::sys::esp!(esp_idf_svc::sys::gpio_config(&raw_config)).unwrap();
        }
        
        log::info!("GPIO 48 should be correctly configured now");

        //Finally, power up the power amplifier.
        speaker_driver.pa_ctrl_pin.set_high().unwrap();

        //The speaker driver is all set!
        speaker_driver
    }



    /// From the [user guide] (page 18): 
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
        
        let mut reg_value: [u8; 1] = [0];

        self.i2c.write_read(Self::I2C_ADDR, 
            &[Register::DacMute as u8], &mut reg_value).unwrap();

        let write_in = reg_value[0] | 0b1110_0000;

        self.i2c.write(Self::I2C_ADDR, 
            &[Register::DacMute as u8, write_in]).unwrap();

    }

    pub fn unmute(&mut self) {

        let mut reg_value: [u8; 1] = [0];

        self.i2c.write_read(Self::I2C_ADDR, 
            &[Register::DacMute as u8], &mut reg_value).unwrap();

        let write_in = reg_value[0] & !(0b1110_0000);

        self.i2c.write(Self::I2C_ADDR, 
            &[Register::DacMute as u8, write_in]).unwrap();

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

        //Power down the PA.
        self.pa_ctrl_pin.set_low().unwrap();

        //reset everything and power down. Minimizes power consumption.
        self.i2c.write(Self::I2C_ADDR, 
            &[Register::Reset as u8, Register::Reset.default()]).unwrap();

    }
}



///Registers (just the ones we need).
/// 
/// **Important:** bits are numbered starting from 0.
#[non_exhaustive]
enum Register {

    ///The Reset register.
    ///Has default value: 0b0001_1111.
    /// 
    ///CSM_ON (bit 7) must be set to 1 to start up state machine in normal mode (i.e. turn on this codec).
    ///See page 18 of the user guide for more info 
    /// (we follow the power up and power down procedure suggested there).
    Reset = 0,

    ///A clock manager register.
    /// Has default value of 0.
    /// 
    /// We want to write 0b10110101 into this register to use BCLK as source for raw_internal_master_clock (before
    /// dividing and multiplying) and turn on the clocks we want (DAC).
    ClockManager = 0x1,

    ///This is another clock manager register. 
    /// Has a default value of 0.
    /// 
    /// We simply use it to set MULT_PRE to all 1's. So we want to write 0b0001_1000 into this.
    /// This means we will multiply the raw_internal_master_clock by 8. 
    /// Read the implementation notes of [speaker][self], to see why we do this.
    ClockFactors = 0x2,
    
    ///Power Management. See page 18 in the user guide for more info.
    /// Has default value: 0b1111_1100.
    SysPwrMgt = 0x0D,

    ///This register enables and disables the DAC. 
    /// It has a default value of 0b0000_0010: DAC powered down 
    /// and internal reference circuits for DAC output disabled.
    /// We want to write 0x1 into this to power up the DAC and enable the reference circuits 
    /// (to increase the accuracy of the conversion).
    SysEnableDac = 0x12,

    ///This register sets whether we use line out drive output or headphone drive output.
    /// Has default value of 0b0100_0000. 
    /// We want to set bit 4 to 1 (has default 0), to enable HP (or speaker) drive output.
    SysEnableSpeakerDrive = 0x13,

    ///Controls DAC mute/unmute.
    ///Write 1 to bits 5 to 7 inclusive to cleanly mute the DAC (according to the user guide page 29).
    /// Has default value of 0.
    DacMute = 0x31,

    ///This register is an 8-bit digital volume control for the DAC. 
    /// It ranges from -95.5dB to +32dB in 0.5dB step resolution.
    /// 
    /// Default value of 0 which is -95.5dB.
    DacVol = 0x32,

    ///Controls the DAC equalizer bypass. 
    /// Has default value of 0b0000_1000.
    /// 
    /// We want to bypass the DAC equalizer.
    /// That means writing this default value into this register.
    /// We do this as the docs are a bit unclear:
    /// They say it has this default value (so equalizer disabled by default), yet in the register
    /// description it seems to say the DAC equalizer is enabled by default.
    DacEqualizer = 0x37,

    ///READ ONLY. Should have a value of 0x83.
    ChipID1 = 0xFD,

    ///READ ONLY. Should have a value of 0x11.
    ChipID2 = 0xFE,


    //----- Currently not needed----------------

    //Serial Digital Port In
    // Bit 6 set to 1 mutes. Set to 0 (default) is unmute.
    // The other default values mean 24 bit I2S, Left channel to DAC.
    // See page 12 of the user guide for more info.
    // While this mute function should work, there is a cleaner way to do this (DacMute register), 
    // which does not result in artifact sounds.
    //SdpIn = 0x09

}

impl Register {

    //We do this instead of impl Default for Register, as that enables us to use a const function here.

    ///Returns the default value of the register.
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
