# flybrix-firmware

Flybrix is still in alpha, but we're excited to annouce our first open source code release!

Dependencies:

* arduino 1.6.7 from https://www.arduino.cc/en/Main/Software
* teensyduino libraries from https://www.pjrc.com/teensy/teensyduino.html
* teensy loader from https://www.pjrc.com/teensy/loader.html


Tips:

If you can't compile because it appears that you're missing the ADC or i2c_t3 libraries, 
be sure that you have set up the Arduino IDE to target the correct board ("Teensy 3.2 / 3.1")!

The Arduino IDE defaults to expand tabs with 2 spaces. To change that edit your preferences file.
https://www.arduino.cc/en/Hacking/Preferences -- Change “editor.tabs.size=” to 4 and restart arduino

If you run into problems, send us a note!
