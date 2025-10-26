# Internet Radio
Transform your [ESP32-S3-Korvo-2 V3.1 board][board] into an internet radio!

* Supports a mix of stations using either MP3 or AAC codec.

* If you want to run or adjust this program, please see the [user guide](user_guide.md).

* Please note that the SET button on the board is used as "station back". Similarly, the REC button is used as "station forward".


## Demo Video

On YouTube:

[<img src="media\Demo_Video_Img.png" />](https://www.youtube.com/watch?v=jrRanWGJ3ZQ)

There is also a 720p copy of this video [under the media folder](https://github.com/Tomer-Eliahu/internet-radio/blob/main/media/Demo_Video_720p.mp4).


## Design Decisions

### Using std over no-std:

We use std Rust (libStd mounted on C-bindings to [ESP-IDF]). Currently, doing this project while going the no-std route is just not feasible. Many of the required libraries lack necessary features (such as TLS features to use HTTPS). 

ESP-IDF, on the other hand, offers a robust networking stack. As the focus of this project is not implementing parts of a networking stack, we just use std. 

### Forking the Symphonia crate

If we don't fork symphonia, the code does not even build. We get the following error:

>error: linking with `ldproxy` failed: exit code: 1
>
>STDERR OUTPUT:
>
>section `.dram0.bss` will not fit in region `dram0_0_seg`
>
>region `dram0_0_seg` overflowed by 373416 bytes

The issue is that symphonia uses *massive* uninitialized statics (by using the LazyStatic crate).
These get placed in internal RAM (comprised out of IRAM and DRAM), which has a size of 512KB.
We do however have 8MB of PSRAM available, so by changing LazyStatic uses to 
```rust 
std::LazyLock::new(|| Box::new(...))
```
We make Rust reserve just 1 `usize` per such static in dram.bss.



[board]: https://docs.espressif.com/projects/esp-adf/en/latest/design-guide/dev-boards/user-guide-esp32-s3-korvo-2.html
[ESP-IDF]: https://github.com/espressif/esp-idf
