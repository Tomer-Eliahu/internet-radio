//Be sure to rename this file to just config.rs once you change the info below

use embedded_svc::wifi::AuthMethod;

///Your Wi-Fi network name. Also known as an SSID (Service Set Identifier).
pub const WIFI_NAME: &str = "My_Wifi_Name";

pub const WIFI_PASSWORD: &str = "My_Wifi_Password";

//Pick the value of this enum that applies to your network 
//(typically under your Wi-Fi network details/security on your phone).
pub const AUTH_METHOD: AuthMethod = AuthMethod::WPA2Personal;