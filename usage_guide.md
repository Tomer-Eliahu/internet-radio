# Usage Guide
To run this program on your own board, first install all necessary tools as detailed in the [Rust on ESP book](https://docs.espressif.com/projects/rust/book/) and the [Embedded Rust (std) on Espressif book](https://docs.esp-rs.org/std-training/02_2_software.html) if you have not done so already. Then go to [example_config.rs](src\wifi\example_config.rs) and set your Wi-Fi settings. Be sure to rename this file "config.rs" once you are done.

## Find New Stations
I took stations from [radio.net](https://www.radio.net/), but there are also alternative sources such as
[TuneIn](https://tunein.com/) (and I am sure there are more).

Browse for your wanted station. Then on its webpage, right click -> click 'inspect' -> go to media.
There you should see one or more items (there is more than one due to redirects). The one that grows in size as the station plays will have the final stream URL. Click on that and the station's stream URL will be listed under "Request URL" header.

Here is an example:
<img src="media\Example_Station.png" />


The other field to pay attention to is "Content-Type". Make sure it is either audio/mpeg or audio/aac. Note that audio/aacp is AAC-Plus or HE-AAC (High Efficiency AAC) and is not supported.

## Set the Stations
In [main.rs](src\main.rs), adjust `STATION_URLS`. 

### Preloaded Stations 
One of the nice things about internet radio vs regular radio is that you can listen to stations from all over world! This program comes with the following stations:

| Station Name | Stream URL | Codec |
|---|---|---|
| [70's Rock - HitsRadio](https://www.radio.net/s/hitsradio70srock) (USA) | [link](https://18063.live.streamtheworld.com/977_CLASSROCK.mp3) | MP3 |
| [Classic Rock Legends Radio](https://www.radio.net/s/classicrocklegends) (USA) | [link](https://puma.streemlion.com:3130/stream) | MP3 |
| [CKUA Radio Network](https://www.radio.net/s/ckua) (Canada) | [link](https://ais-sa1.streamon.fm/7000_48k.aac) | AAC |
| [Smooth Radio London](https://www.radio.net/s/smoothradiolondon) | [link](https://media-ice.musicradio.com/SmoothLondonMP3) | MP3 |
| [Timewarp Ireland](https://www.radio.net/s/timewarpireland) | [link](https://broadcast.shoutstream.co.uk:8052/stream) | MP3 |
| [OzInDi Radio Australia](https://www.radio.net/s/ozindiradio) | [link](https://streamer.radio.co/s52d0fa340/listen) | AAC |
| [Radiowave NZ](https://www.radio.net/s/radiowave-nz) | [link](https://stream.radiowavenz.com/stream) | MP3 |
