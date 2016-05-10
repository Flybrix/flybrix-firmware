/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <cardManagement.h/cpp>

    General interaction with the SD card, in the forms of logging or reading.
*/

#include "cardManagement.h"

#include <SdFat.h>
#include <cstdio>
#include "board.h"
#include "debug.h"
#include "version.h"

#ifdef ALPHA
#define SKIP_SD
#endif

namespace {
SdFat sd;

bool openSDHardwarePort() {
#ifdef SKIP_SD
    return false;
#else
    bool opened = sd.begin(board::spi::SD_CARD, SPI_FULL_SPEED);
    if (!opened)
        DebugPrint("Failed to open connection to SD card!");
    return opened;
#endif
}

bool openSD() {
    static bool sd_open{openSDHardwarePort()};
    return sd_open;
}

SdFile file;

bool openFile(const char* base_name) {
    if (!openSD())
        return false;
    char file_dir[64];
    char filename[64];
    // Create the directory /<A>_<B>_<C> if it doesn't exist
    sprintf(file_dir, "%d_%d_%d", FIRMWARE_VERSION_A, FIRMWARE_VERSION_B, FIRMWARE_VERSION_C);
    if (!sd.exists(file_dir) && !sd.mkdir(file_dir)) {
        DebugPrintf("Failed to create directory %s on SD card!", file_dir);
        return false;
    }
    for (int idx = 0; true; ++idx) {
        sprintf(filename, "%s/%s_%d.bin", file_dir, base_name, idx);
        // Look for the first non-existing filename of the format /<A>_<B>_<C>/<base_name>_<idx>.bin
        if (!sd.exists(filename)) {
            bool success = file.open(filename, O_CREAT | O_WRITE);
            if (!success)
                DebugPrintf("Failed to open file %s on SD card!", filename);
            return success;
        }
    }
    return false;
}
}  // namespace

void startupCard() {
    writeToCard(nullptr, 0);
}

void writeToCard(const uint8_t* data, size_t length) {
    static bool openedFile{openFile("st")};
    if (!(openedFile)) {
        if (openSD())
            DebugPrint("Failed to access file on SD card!");
        return;
    }
    if (length > 0)
        file.write(data, length);
}

void commitWriteToCard() {
    file.sync();
}
