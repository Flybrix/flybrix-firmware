# flybrix-firmware

Flybrix is currently shipping with version 1.5.1 installed.

## Dependencies

* Arduino 1.8.5 from https://www.arduino.cc/en/Main/Software
* Teensyduino 1.41 from https://www.pjrc.com/teensy/td_download.html
* Teensy loader from https://www.pjrc.com/teensy/loader.html
* SDFat 1.0.5 from https://github.com/greiman/SdFat/releases/tag/1.0.5

## Setup

In the Arduino IDE, isntall SdFat from 'Sketch -> Include Library -> Manage Libraries...'

Choose `Tools -> Board: -> Teensy 3.2 / 3.1` to target the correct board, and leave the rest as default ("Serial" and "96 MHz (overclock)").

After that, you are ready to compile and upload to a board connected via USB.

## Tips

If you can't compile because it appears that you're missing libraries,
be sure that you have set up the Arduino IDE to target the correct board ("Teensy 3.2 / 3.1")!

The Arduino IDE defaults to expand tabs with 2 spaces. To change that edit your preferences file.
https://www.arduino.cc/en/Hacking/Preferences -- Change “editor.tabs.size=” to 4 and restart Arduino

If you run into problems, send us an email at support@flybrix.com!
