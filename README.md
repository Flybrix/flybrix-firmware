# flybrix-firmware

Flybrix is at version 1.5.0 and getting better all the time!

## Dependencies

* arduino 1.8.2 from https://www.arduino.cc/en/Main/OldSoftwareReleases
* teensyduino 1.36 from https://www.pjrc.com/teensy/td_download.html
* teensy loader from https://www.pjrc.com/teensy/loader.html
* SDFat 1.0.3 from https://github.com/greiman/SdFat/releases/tag/1.0.3

## Setup

Download the SDFat library's source as a .ZIP file (direct link for download [here](https://github.com/greiman/SdFat/archive/1.0.3.zip))

In the Arduino IDE, go to `Sketch -> Include Library -> Add .ZIP Libraries...` to add it.

Choose `Tools -> Board: -> Teensy 3.2 / 3.1` to target the correct board, and leave the rest as default ("Serial" and "96 MHz (overclock)").

After that, you are ready to compile and upload to a board connected via USB.

## Tips

If you can't compile because it appears that you're missing libraries,
be sure that you have set up the Arduino IDE to target the correct board ("Teensy 3.2 / 3.1")!

The Arduino IDE defaults to expand tabs with 2 spaces. To change that edit your preferences file.
https://www.arduino.cc/en/Hacking/Preferences -- Change “editor.tabs.size=” to 4 and restart arduino

If you run into problems, send us a note!
