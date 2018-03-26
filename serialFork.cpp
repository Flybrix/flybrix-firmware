/*
    *  Flybrix Flight Controller -- Copyright 2018 Flying Selfie Inc. d/b/a Flybrix
    *
    *  http://www.flybrix.com
*/

#include "serialFork.h"
#include <Arduino.h>
#include "board.h"
#include "devicename.h"
#include "usbModeSelector.h"
#include "debug.h"
#include "utility/clock.h"

class ChannelBuffer {
  public:
    ChannelBuffer(size_t _bufferCount, size_t _bufferChunk) {
        bufferCount = _bufferCount;
        bufferChunk = _bufferChunk;
        bufferSize = bufferCount * bufferChunk;
        data = (uint8_t*) malloc(bufferSize);
    };
    ~ChannelBuffer() {
        free(data);
    };

    size_t hasData() {
        size_t increment{(readerPointer <= writerPointer) ? (writerPointer - readerPointer) : (bufferSize - readerPointer)};
        if (increment > bufferChunk)
            return bufferChunk;
        return increment;
    };

    bool canFit(size_t size) {
        if (readerPointer <= writerPointer)
            return bufferSize - (writerPointer - readerPointer) > size;
        else
            return readerPointer - writerPointer > size;
    };

    const uint8_t* pop() {
        size_t outputPointer{readerPointer};
        readerPointer += hasData();
        if (readerPointer >= bufferSize)
            readerPointer -= bufferSize;
        return data + outputPointer;
    };

    void push(uint8_t* input_data, size_t length) {
        if (!canFit(length))  // The buffer is too full
            return;
        size_t lengthSubtraction{0};
        if (bufferSize - writerPointer < length)
            lengthSubtraction = length - (bufferSize - writerPointer);
        length -= lengthSubtraction;
        for (size_t pos = 0; pos < length; ++pos) {
            data[writerPointer++] = input_data[pos];
        }
        if (writerPointer == bufferSize) {
            writerPointer = 0;
            push(input_data + length, lengthSubtraction);
        }
    };
  private:
    size_t bufferCount;
    size_t bufferChunk;
    size_t bufferSize;
    uint8_t *data;
    size_t writerPointer{0};
    size_t readerPointer{0};
};

class Channel{
  public:
    Channel(size_t bufferCount, size_t bufferChunk) {
        data_output = new ChannelBuffer(bufferCount, bufferChunk);
    };

    virtual ~Channel() {};

    virtual uint8_t _serial_available() = 0;
    virtual uint8_t _serial_read() = 0;
    virtual uint8_t _serial_write(const uint8_t *data, size_t length) = 0;
    virtual void _serial_flush() = 0;

    bool get() {
        bool did_work{false};
        while (_serial_available() && !await_read) { // we can't seem to keep up with incoming data...
            bytes_read++;
            data_input.AppendToBuffer(_serial_read());
            await_read = data_input.IsDone();
            did_work = true;
        }
        return did_work;
    }

    bool send() {
        size_t length{data_output->hasData()};
        if (!length) {
            return false;
        }
        const uint8_t *data = data_output->pop();
        _serial_write(data, length);
        //_serial_flush();
        bytes_sent += length;
        return true;
    }

    CobsReaderBuffer* read() {
        await_read = false;
        if (data_input.IsDone()){
            return &data_input;
        }
        return nullptr;
    }

    void write(uint8_t* data, size_t length) {
        bytes_written += length;
        data_output->push(data, length);
    }

    void printStats(){
        Serial.printf("bytes buffered/sent/received: %8d / %8d / %8d", bytes_written, bytes_sent, bytes_read);
    }

  private:
    ChannelBuffer *data_output;
    CobsReaderBuffer data_input;

    uint32_t bytes_written{0};
    uint32_t bytes_read{0};
    uint32_t bytes_sent{0};

    bool await_read{false};
};

class USBComm : public Channel {
  public:

    USBComm() : Channel(20,64) {
        Serial.begin(9600);  // USB is always 12 Mbit/sec
    };

    uint8_t _serial_available(){
        return Serial.available();
    };
    uint8_t _serial_read(){
        return Serial.read();
    };
    uint8_t _serial_write(const uint8_t *data, size_t length){
        return Serial.write(data, length);
    };
    void _serial_flush(){
        return Serial.flush();
    };
};

USBComm usb_comm;

class Bluetooth : public Channel {
  public:
    Bluetooth() : Channel(100,20) {
        // PIN 12 of teensy is BMD (P0.13)
        // PIN 30 of teensy is BMD (PO.14) AT Mode
        // PIN 28 of teensy is BMD RST
        // 20 - CTS P0.05
        // 0 - Rx P0.06
        // 6 - RTS P0.05
        // 1 - Tx
        Serial1.setTX(1); //change to board::bluetooth:PIN eventually
        Serial1.setRX(0);
        Serial1.begin(57600);
        //Serial1.attachRts(6);
        //Serial1.attachCts(20);
        Serial1.clear();

        pinMode(board::bluetooth::RESET, OUTPUT);
        digitalWrite(board::bluetooth::RESET, HIGH);
        pinMode(board::bluetooth::MODE, OUTPUT);
        digitalWriteFast(board::bluetooth::MODE, LOW);   // set AT mode

        digitalWriteFast(board::bluetooth::RESET, LOW);  // reset BMD
        delay(100);
        digitalWriteFast(board::bluetooth::RESET, HIGH);  // reset BMD complete, now in AT mode
        start_time = ClockTime::now(); //we need to wait about 2500msec total
    };

    uint8_t _serial_available(){
        return Serial1.available();
    };
    uint8_t _serial_read(){
        return Serial1.read();
    };
    uint8_t _serial_write(const uint8_t *data, size_t length){
        return Serial1.write(data, length);
    };
    void _serial_flush(){
        return Serial1.flush();
    };
    void setBluetoothUart(const DeviceName& name);

  private:
    ClockTime start_time{ClockTime::zero()}; //used in setup
};

Bluetooth bluetooth;

void flushATmodeResponse() {
    // We can't ignore responses provided by AT mode
    delay(100);
    while (!Serial1.available()) {
    }
    while (Serial1.available()) {
        char c = Serial1.read();
        //Serial.write(c); //OK or ERR
        if (c == '\n'){
            break;
        }
    }
}

void Bluetooth::setBluetoothUart(const DeviceName& name) {

    uint32_t waited = (ClockTime::now() - start_time)/1000;
    if (waited < 2500) {
        DebugPrintf("Delaying %d msec for Rigado AT mode.",  2500-waited);
        delay(2500-waited); // time needed initialization of AT mode
    }
    else {
        DebugPrintf("Waited %d msec for Rigado AT mode.", waited);
    }

    Serial1.print("at$name ");
    Serial1.print(name.value);
    Serial1.print("\n");
    flushATmodeResponse();

    Serial1.print("at$btxpwr 04\n"); //enable +4dBm beacon
    flushATmodeResponse();

    Serial1.print("at$ctxpwr 04\n"); //enable +4dBm connectable
    flushATmodeResponse();

    Serial1.print("at$uen 01\n"); //enable pass-through UART
    flushATmodeResponse();

    Serial1.print("at$ubr 57600\n"); //set pass-through UART baud rate
    flushATmodeResponse();

    Serial1.print("at$ufc 00\n"); //disable flow control (req'd over 57k)
    flushATmodeResponse();

    /*
    Serial1.print("at$ubr 115200\n"); //set pass-through UART baud rate
    flushATmodeResponse();

    Serial1.print("at$ufc 01\n"); //enable flow control (req'd over 57k)
    flushATmodeResponse();

    Serial1.end();
    Serial1.setTX(1);
    Serial1.setRX(0);
    Serial1.begin(115200);
    Serial1.attachRts(6); //change to board::bluetooth:RTS eventually
    Serial1.attachCts(20);
    */

    digitalWriteFast(board::bluetooth::MODE, HIGH);
    digitalWriteFast(board::bluetooth::RESET, LOW);  // reset BMD
    delay(100);
    digitalWriteFast(board::bluetooth::RESET, HIGH);  // reset BMD complete, now not in AT mode
}

void setBluetoothUart(const DeviceName& name) {
    bluetooth.setBluetoothUart(name);
}

void writeSerial(uint8_t* data, size_t length) {
    //this only puts the data into the buffers; it doesn't send!
    if (usb_mode::get() == usb_mode::BLUETOOTH_MIRROR) {
        usb_comm.write(data, length);
    }
    bluetooth.write(data, length);
}

bool usb_sendData(){
    return usb_comm.send();
}

bool usb_getData(){
    return usb_comm.get();
}

CobsReaderBuffer* usb_readData(){
    return usb_comm.read();
}

bool bluetooth_sendData(){
    return bluetooth.send();
}

bool bluetooth_getData(){
    return bluetooth.get();
}

CobsReaderBuffer* bluetooth_readData(){
    return bluetooth.read();
}

void printSerialReport() {
    Serial.print("USB "); usb_comm.printStats(); Serial.println();
    Serial.print("BT  "); bluetooth.printStats(); Serial.println();
}
