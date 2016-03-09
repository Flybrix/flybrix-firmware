/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <version.h/cpp>

    EEPROM based configuration data storage structure

    Nonvolatile parameters are being stored inside an CONFIG structure
    that can be accesed as data union, for easier manipulation as a javascript
    ArrayBuffer object over serial.

*/

#ifndef version_h
#define version_h

#define FIRMWARE_VERSION_A 1
#define FIRMWARE_VERSION_B 1
#define FIRMWARE_VERSION_C 0

#endif
