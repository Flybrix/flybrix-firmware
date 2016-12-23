# flybrix-firmware

Flybrix is at version 1.4.0 and getting better all the time!

Dependencies:

* arduino 1.6.13 from https://www.arduino.cc/en/Main/OldSoftwareReleases#previous
* teensyduino 1.33 from https://www.pjrc.com/teensy/td_download.html
* teensy loader from https://www.pjrc.com/teensy/loader.html
* SDFat library from https://github.com/greiman/SdFat  -- (IMPORTANT) must be patched to use nonstandard pins (see 'SdSpiTeensy3.cpp.diff')


Tips:

If you can't compile because it appears that you're missing libraries, 
be sure that you have set up the Arduino IDE to target the correct board ("Teensy 3.2 / 3.1")!

The Arduino IDE defaults to expand tabs with 2 spaces. To change that edit your preferences file.
https://www.arduino.cc/en/Hacking/Preferences -- Change “editor.tabs.size=” to 4 and restart arduino

If you run into problems, send us a note!
