use embedded_svc::{
    http::{Method, client::Client as HttpClient},
    wifi::{ClientConfiguration, Configuration},
};

use esp_idf_svc::{
    eventloop::EspSystemEventLoop,
    http::client::{Configuration as HttpConfiguration, EspHttpConnection},
    io::{self, utils::try_read_full},
    nvs::EspDefaultNvsPartition,
    sys::EspError,
    timer::EspTaskTimerService,
    wifi::{AsyncWifi, EspWifi},
};

mod config;

static WIFI: std::sync::Mutex<Option<AsyncWifi<EspWifi<'static>>>> = std::sync::Mutex::new(None);

///Sets up the wifi and returns an HTTPS client.
pub async fn setup_wifi(
    modem: esp_idf_svc::hal::modem::Modem,
) -> Result<HttpClient<EspHttpConnection>, EspError> {
    let sys_loop = EspSystemEventLoop::take()?;
    //It seems like nvs is needed to prevent a crash.
    let nvs = EspDefaultNvsPartition::take()?;
    let timer_service = EspTaskTimerService::new()?;

    let mut wifi: AsyncWifi<EspWifi<'_>> = AsyncWifi::wrap(
        EspWifi::new(modem, sys_loop.clone(), Some(nvs))?,
        sys_loop,
        timer_service,
    )?;

    let wifi_configuration: Configuration = Configuration::Client(ClientConfiguration {
        ssid: config::WIFI_NAME.try_into().unwrap(),
        bssid: None,
        auth_method: config::AUTH_METHOD,
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
    let client = HttpClient::wrap(EspHttpConnection::new(&config)?);

    Ok(client)
}

/// Test sending an HTTP GET request.
pub fn test_get_request(client: &mut HttpClient<EspHttpConnection>) -> Result<(), io::EspIOError> {
    let headers = [("accept", "text/plain")];
    let url = "http://ifconfig.net/";

    // Send request
    let request = client.request(Method::Get, url, &headers)?;
    log::info!("-> GET {url}");
    let mut response = request.submit()?;

    // Process response
    let status = response.status();
    log::info!("<- {status}");

    let mut buf = [0u8; 1024];

    let bytes_read = try_read_full(&mut response, &mut buf).map_err(|e| e.0)?;

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
