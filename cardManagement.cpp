/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "cardManagement.h"

#include <SPI.h>
#include <SdFat.h>
#include <cstdio>
#include "board.h"
#include "debug.h"
#include "loop_stopper.h"
#include "version.h"

namespace sdcard {
namespace {
State card_state{State::Closed};

SdFat sd;

bool openSDHardwarePort() {
#ifdef SKIP_SD
    return false;
#else
    SPI.setMOSI(7);
    SPI.setMISO(8);
    SPI.setSCK(14);
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
}  // namespace

void startup() {
    DebugPrint("Stopping loops for SD card startup!");
    loops::Stopper _stopper;
    openSD();
}

State getState() {
    return card_state;
}

namespace writing {
namespace {

SdBaseFile binFile;

bool isWriting() {
    return card_state == State::WriteStates || card_state == State::WriteCommands;
}

struct SenseCloseIIFE {
    ~SenseCloseIIFE() {
        if (!binFile.isOpen() && isWriting()) {
            card_state = State::Closed;
        }
    };
};

uint32_t bgnBlock, endBlock;
uint8_t* cache;

uint32_t block_number = 0;

// Number of 512B blocks that the file can contain
// The limit can be changed as needed, and currently it's ~128MB
constexpr uint32_t FILE_BLOCK_COUNT = 256000;
constexpr uint32_t ERASE_SIZE = 262144L;
// Number of blocks in the buffer
// Increase this number if there is trouble with buffer overflows caused by
// random delays in the SD card
// This can not fix overflows caused by too high data rates
// This causes the program to take up 0.5kB of RAM per buffer block
#ifdef SKIP_SD
constexpr uint32_t BUFFER_BLOCK_COUNT = 0;
#else
constexpr uint32_t BUFFER_BLOCK_COUNT = 8;
#endif

class WritingBuffer {
   public:
    void write(const uint8_t* data, size_t length);
    bool hasBlock() const;
    uint8_t* popBlock();

   private:
    uint8_t block[BUFFER_BLOCK_COUNT][512];
    uint16_t startBlock{0};
    uint16_t currentBlock{0};
    uint16_t currentPointer{0};
    bool overbuffered{false};
} writingBuffer;

void WritingBuffer::write(const uint8_t* data, size_t length) {
    if (overbuffered)
        return;
    for (size_t i = 0; i < length; ++i) {
        block[currentBlock][currentPointer++] = data[i];
        if (currentPointer < 512)
            continue;
        currentPointer = 0;
        if (++currentBlock >= BUFFER_BLOCK_COUNT)
            currentBlock = 0;
        if (currentBlock == startBlock) {
            overbuffered = true;
            DebugPrint("SD card buffer is full");
            return;
        }
    }
}

bool WritingBuffer::hasBlock() const {
    return overbuffered || (startBlock != currentBlock);
}

uint8_t* WritingBuffer::popBlock() {
    if (!hasBlock())
        return nullptr;
    uint8_t* retval{block[startBlock]};
    if (++startBlock >= BUFFER_BLOCK_COUNT)
        startBlock = 0;
    overbuffered = false;
    return retval;
}

bool openFileHelper(const char* filename) {
    SenseCloseIIFE close_iife;
    binFile.close();
    if (!binFile.createContiguous(sd.vwd(), filename, 512 * FILE_BLOCK_COUNT))
        return false;

    if (!binFile.contiguousRange(&bgnBlock, &endBlock))
        return false;

    cache = (uint8_t*)sd.vol()->cacheClear();
    if (cache == 0)
        return false;

    uint32_t bgnErase = bgnBlock;
    uint32_t endErase;
    while (bgnErase < endBlock) {
        endErase = bgnErase + ERASE_SIZE;
        if (endErase > endBlock)
            endErase = endBlock;
        if (!sd.card()->erase(bgnErase, endErase))
            return false;
        bgnErase = endErase + 1;
    }

    DebugPrint("Starting file write");
    if (!sd.card()->writeStart(bgnBlock, FILE_BLOCK_COUNT))
        return false;

    return true;
}

bool openFile(const char* base_name) {
    SenseCloseIIFE close_iife;
    if (card_state != State::Closed) {
        return false;
    }
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
            bool success = openFileHelper(filename);
            if (success) {
                card_state = sdcard::State::WriteStates;
            } else {
                DebugPrintf("Failed to open file %s on SD card!", filename);
            }

            return success;
        }
    }
    return false;
}

bool locked = false;
}  // namespace

uint32_t bytes_written{0};
uint32_t bytes_sent{0};
uint32_t bytes_read{0};

void printReport(){
    Serial.printf("SD  bytes buffered/sent/received: %8d / %8d / %8d\n", bytes_written, bytes_sent, bytes_read);
}

void open() {
    if (locked)
        return;
    if (!openSD())
        return;
    DebugPrint("Stopping loops for SD card writing::open!");
    loops::Stopper _stopper;
    openFile("st");
}

void write(const uint8_t* data, size_t length) {
    if (!openSD())
        return;
    if (!isWriting())
        return;
    if (block_number == FILE_BLOCK_COUNT)
        return;
    if (length > 0){
        writingBuffer.write(data, length);
        bytes_written += length;
    }
}

void send() {
    if (!openSD())
        return;
    if (!isWriting())
        return;
    if (block_number == FILE_BLOCK_COUNT)
        return;
    if (sd.card()->isBusy())
        return;
    if (!writingBuffer.hasBlock())
        return;
    if (!sd.card()->writeData(writingBuffer.popBlock()))
        DebugPrint("Failed to write data!");
    block_number++;
    bytes_sent += 512;
}

void close() {
    SenseCloseIIFE close_iife;
    if (!isWriting()) {
        return;
    }
    if (locked)
        return;
    if (!openSD())
        return;

    DebugPrint("Stopping loops for SD card writing::close!");
    loops::Stopper _stopper;

    if (!sd.card()->writeStop()) {
        DebugPrint("Write stop failed");
        return;
    }
    if (block_number != FILE_BLOCK_COUNT) {
        if (!binFile.truncate(512L * block_number)) {
            DebugPrint("Truncating failed");
            return;
        }
    }
    block_number = 0;
    binFile.close();
    DebugPrint("File closing successful");
}

void setLock(bool enable) {
    locked = enable;
}

bool isLocked() {
    return locked;
}
}  // namespace writing

namespace reading {
namespace {
File read_file;

bool isReading() {
    return card_state == State::ReadCommands;
}

struct SenseCloseIIFE {
    ~SenseCloseIIFE() {
        if (!read_file.isOpen() && isReading()) {
            card_state = State::Closed;
        }
    };
};
}  // namespace

void open() {
    if (!openSD()) {
        return;
    }
    if (read_file) {
        return;
    }
    
    DebugPrint("Stopping loops for SD card reading::open!");
    loops::Stopper _stopper;
    
    read_file = sd.open("commands.bin");
    if (!read_file) {
        return;
    }
    card_state = State::ReadCommands;
}

void close() {
    SenseCloseIIFE close_iife;
    if (!isReading()) {
        return;
    }
    if (!openSD()) {
        return;
    }
    DebugPrint("Stopping loops for SD card reading::close!");
    loops::Stopper _stopper;
    
    read_file.close();
    DebugPrint("File closing successful");
}

bool hasMore() {
    return read_file.available();
}

char read() {
    return read_file.read();
}
}  // namespace reading

}  // namespace sdcard
