# STM32-SimpleScope

STM32 SimpleScope Oscilloscope STM32F103 Blue Pill

## Features

- Two input channels at full capture rate
- SDCard (now disabled by default) (code from Gameinstance)
- Rising, falling, no trigger, singleton, and Spectrum FFT
- Maximum sample rate of 2.67 MSPS (~1.3 MHz scope) - one or two channels
- Up to 250kHz per division (8 divisions)
- Test output (square wave only) Currently 1Khz % 50% (dynamically configurable soon)
- Freeze function
- Fundamental frequency detection
- Scrollable capture buffer

## Upcoming features

- More trigger options
- Y-axis zoom
- Y-axis scrolling
- Broader than 0 - 3.3V support (via analog front end) [You can do so today with a voltage divider]
- Analog frontend planned (BNC)
- May speed up to 5MSPS on single channel at some point

## Parts

- ILI9341 320x240 TFT display (SPI) (Tested with 2.8", but other sizes should work)
  - [TFT With touch](https://www.aliexpress.us/item/3256805747165796.html)
  - [No Touch TFT](https://www.aliexpress.us/item/3256805747231498.html)
- STM32F103C8 a.k.a Blue Pill (You will need an st-link programmer as well)
  - [Example STM32F103](https://www.aliexpress.us/item/3256806535149928.html)
  - [128k Flash STM32F103CBT6](https://www.aliexpress.us/item/3256804732020002.html)
- Rotary Encoder with push/click (debounced)
  - [Knob (half-handle)](https://www.aliexpress.us/item/3256801125707035.html)
  - [Encoder](https://www.aliexpress.us/item/3256805796819763.html)
- [Probes](https://www.aliexpress.us/item/3256803763692666.html)
- [BNC Connectors](https://www.aliexpress.us/item/3256806768918694.html)
- [Breadboard](https://www.aliexpress.us/item/3256805916002941.html)

## Requirements to build

- Arduino 2.x
  - Set Upload method to STLink
  - Install Board: Arduino SAM Boards Cortex-M3 (used to add toolchain)
- Use STM32CubeProgrammer to download compiled code
- Clone below repo to ~/.arduino15/packages/ (Sorry, not sure about Windows path, but ~ should be your home directory)
  - [Required for stm32 support](https://github.com/rogerclarkmelbourne/Arduino_STM32/wiki/Installation)
- Clone below repo to ~/Arduino/libraries/
  - [Optional for SD card support](https://github.com/greiman/SdFat.git)

## Recognition

- Originally forked from [gameinstance](https://www.gameinstance.com/post/80/STM32-Oscilloscope-with-FFT-and-SD-export)
- Huge thanks to Marti Bolivar for https://gist.github.com/mbolivar/1260355 which was a huge help in getting two-channel simultaneous DMA working.
- Other STM32 scopes: They might be good to look at someday:
  - [STM32-O-Scope](https://github.com/pingumacpenguin/STM32-O-Scope)
  - [STM32-Spectrometer](https://github.com/Beherith/STM32_Spectrometer)
