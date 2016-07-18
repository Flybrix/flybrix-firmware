/*
    *  Flybrix Flight Controller -- Copyright 2015 Flying Selfie Inc.
    *
    *  License and other details available at: http://www.flybrix.com/firmware

    <serialFork.h/cpp>

    Interface for forking serial communication channels.
*/

#include "serialFork.h"
#include <Arduino.h>
#include "board.h"

namespace {
struct USBComm {
    USBComm() {
        Serial.begin(9600);  // USB is always 12 Mbit/sec
    }

    bool read() {
        while (Serial.available()) {
            data_input.AppendToBuffer(Serial.read());
            if (data_input.IsDone())
                return true;
        }
        return false;
    }

    void write(uint8_t* data, size_t length) {
        Serial.write(data, length);
    }

    CobsReaderBuffer& buffer() {
        return data_input;
    }

   private:
    CobsReaderBuffer data_input;
};

USBComm usb_comm;

#ifndef ALPHA
struct Bluetooth {
    Bluetooth() {
        pinMode(board::bluetooth::RESET, OUTPUT);
        digitalWrite(board::bluetooth::RESET, HIGH);
        Serial1.begin(57600);
    }

    bool read() {
        while (Serial1.available()) {
            data_input.AppendToBuffer(Serial1.read());
            if (data_input.IsDone())
                return true;
        }
        return false;
    }

    void write(uint8_t* data, size_t length) {
        data_output.push(data, length);
    }

    CobsReaderBuffer& buffer() {
        return data_input;
    }

    void flush() {
        size_t l{data_output.hasData()};
        if (!l)
            return;
        Serial1.write(data_output.pop(), l);
    }

   private:
    struct BluetoothBuffer {
        size_t hasData() {
            size_t increment{(readerPointer <= writerPointer) ? (writerPointer - readerPointer) : (bufferSize - readerPointer)};
            if (increment > bufferChunk)
                return bufferChunk;
            return increment;
        }

        bool canFit(size_t size) {
            if (readerPointer <= writerPointer)
                return bufferSize - (writerPointer - readerPointer) > size;
            else
                return readerPointer - writerPointer > size;
        }

        const uint8_t* pop() {
            size_t outputPointer{readerPointer};
            readerPointer += hasData();
            if (readerPointer >= bufferSize)
                readerPointer -= bufferSize;
            return data + outputPointer;
        }

        void push(uint8_t* input_data, size_t length) {
            if (!canFit(length))  // The buffer is too full
                return;
            size_t lengthSubtraction{0};
            if (bufferSize - writerPointer < length)
                lengthSubtraction = length - (bufferSize - writerPointer);
            length -= lengthSubtraction;
            for (size_t pos = 0; pos < length; ++pos)
                data[writerPointer++] = input_data[pos];
            if (writerPointer == bufferSize) {
                writerPointer = 0;
                push(input_data + length, lengthSubtraction);
            }
        }

       private:
        static constexpr size_t bufferCount{40};
        static constexpr size_t bufferChunk{20};
        static constexpr size_t bufferSize{bufferCount * bufferChunk};
        uint8_t data[bufferSize];
        size_t writerPointer{0};
        size_t readerPointer{0};
    };

    size_t writerPosition{0};
    BluetoothBuffer data_output;
    CobsReaderBuffer data_input;
};

Bluetooth bluetooth;
#endif
}


void setBluetoothUart()
{
  pinMode(30, OUTPUT);
  pinMode(28, OUTPUT);
  digitalWriteFast(30,LOW); //set AT mode
  digitalWriteFast(28,LOW); //reset BMD
  delay(10);
  digitalWriteFast(28,HIGH); //reset BMD complete, now in AT mode
  uint8_t data[18];
  data[0] = 'a';
  data[1] = 't';
  data[2] = '$';
  data[3] = 'u';
  data[4] = 'e';
  data[5] = 'n';
  data[6] = ' ';
  data[7] = '0';
  data[8] = '1';
  data[9] = '\n';
  bluetooth.write(data,10);
  
  data[3] = 'n';
  data[4] = 'a';
  data[5] = 'm';
  data[6] = 'e';
  data[7] = ' ';
  data[8] = 'F';
  data[9] = 'L';
  data[10] = 'Y';
  data[11] = 'B';
  data[12] = 'R';
  data[13] = 'I';
  data[14] = 'X';
  data[15] = '\n';
  bluetooth.write(data,16);
  
  delay(10);
  digitalWriteFast(30,HIGH);
  digitalWriteFast(28,LOW); //reset BMD
  delay(10);
  digitalWriteFast(28,HIGH); //reset BMD complete, now not in AT mode
}

CobsReaderBuffer* readSerial() {
    if (usb_comm.read())
        return &usb_comm.buffer();
#ifndef ALPHA
    if (bluetooth.read())
        return &bluetooth.buffer();
#endif
    return nullptr;
}

void writeSerial(uint8_t* data, size_t length) {
    usb_comm.write(data, length);
#ifndef ALPHA
    bluetooth.write(data, length);
#endif
}

void flushSerial() {
#ifndef ALPHA
    bluetooth.flush();
#endif
}
