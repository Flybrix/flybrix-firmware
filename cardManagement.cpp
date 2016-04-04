/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <cardManagement.h/cpp>

    General interaction with the SD card, in the forms of logging or reading.
*/

#include "cardManagement.h"

#include <SD.h>
#include <cstdio>
#include "board.h"
#include "debug.h"
#include "version.h"

namespace {
bool sd_open{false};

bool openSD() {
    if (sd_open)
        return true;
    sd_open = SD.begin(board::spi::SD_CARD);
    if (sd_open)
        return true;
    DebugPrint("Failed to open connection to SD card!");
    return false;
}
}  // namespace

Logger::Logger(const char* base_name) {
    if (!openSD())
        return;
    char file_dir[64];
    // Create the directory /<A>_<B>_<C> if it doesn't exist
    sprintf(file_dir, "/%d_%d_%d", FIRMWARE_VERSION_A, FIRMWARE_VERSION_B, FIRMWARE_VERSION_C);
    if (!SD.mkdir(file_dir)) {
        DebugPrintf("Failed to create directory %s on SD card!", file_dir);
        return;
    }
    for (int idx = 0; true; ++idx) {
        sprintf(filename, "%s/%s_%d.bin", file_dir, base_name, idx);
        // Look for the first non-existing filename of the format /<A>_<B>_<C>/<base_name>_<idx>.bin
        if (!SD.exists(filename))
            return;
    }
}

void Logger::write(const uint8_t* data, size_t length) {
    if (!openSD())
        return;
    File file{SD.open(filename, FILE_WRITE)};
    if (!file) {
        DebugPrintf("Failed to access file %s on SD card!", filename);
        return;
    }
    for (size_t idx = 0; idx < length; ++idx)
        file.print(char(data[idx]));
    file.close();
}
