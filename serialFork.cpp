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

namespace {
struct USBComm {
    USBComm() {
        Serial.begin(9600);  // USB is always 12 Mbit/sec
    }

    bool read() {
        while (Serial.available()) {
            bytes_received++;
            data_input.AppendToBuffer(Serial.read());
            if (data_input.IsDone())
                return true;
        }
        return false;
    }

    void write(uint8_t* data, size_t length) {
        bytes_sent += length;
        Serial.write(data, length);
    }

    CobsReaderBuffer& buffer() {
        return data_input;
    }
    
    void printStats(){
        Serial.printf("USB bytes sent/received: %8d / %8d \n", bytes_sent, bytes_received);
    }
    
   private:
    CobsReaderBuffer data_input;
    uint32_t bytes_sent{0};
    uint32_t bytes_received{0};  
};

USBComm usb_comm;

struct Bluetooth {
    Bluetooth() {
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
        
        pinMode(board::bluetooth::RESET, OUTPUT);
        digitalWrite(board::bluetooth::RESET, HIGH);
        pinMode(board::bluetooth::MODE, OUTPUT);
        digitalWriteFast(board::bluetooth::MODE, LOW);   // set AT mode
        
        digitalWriteFast(board::bluetooth::RESET, LOW);  // reset BMD
        delay(100);
        digitalWriteFast(board::bluetooth::RESET, HIGH);  // reset BMD complete, now in AT mode
        start_time = micros(); //we need to wait about 2500msec total
    }

    void setBluetoothUart(const DeviceName& name);

    bool read() {
        size_t length{0};
        while (Serial1.available()) {
            bytes_received++;
            length++;
            char c = Serial1.read();

            /*
            Serial.write((c<0x10) ? "READ: 0x0" : "READ: 0x");
            Serial.print(c, HEX);
            Serial.println();
            */
            
            data_input.AppendToBuffer(c);
            if (data_input.IsDone()) {
                //Serial.write("{+] READ: ");
                //Serial.println(length, DEC);
                return true;
            }
        }
        /*
        if (length>0) {
            Serial.write("[-] READ: ");
            Serial.println(length, DEC);
        }
        */
        return false;
    }

    void write(uint8_t* data, size_t length) {
        bytes_sent += length;
        data_output.push(data, length);

        /*
        Serial.write("BYTES BUFFERED: ");
        Serial.println(length, DEC);
        for (size_t i = 0; i < length; i++) {
            uint8_t c = data[i++];
            Serial.write((c<0x10) ? "BUFFERED: 0x0" : "BUFFERED: 0x");
            Serial.print(c, HEX);
            Serial.println();
        }
        */
    }

    void printStats(){
        Serial.printf(" BT bytes sent/received: %8d / %8d \n", bytes_sent, bytes_received);
    }

    CobsReaderBuffer& buffer() {
        return data_input;
    }

    bool flush() {
        size_t length{data_output.hasData()};
        if (!length) {
            return false;
        }
        const uint8_t *data = data_output.pop();
        Serial1.write(data, length); //put only 20 bytes into FIFO!
        /*
        for (; length < 20; length++) {
            Serial1.write(0); //zero pad
        }
        */
        Serial1.flush(); //force it to send
        /*
        Serial.write("BYTES SENT: ");
        Serial.println(length, DEC);
        for (size_t i = 0; i < length; i++) {
            uint8_t c = data[i++];
            Serial.write((c<0x10) ? "SENT: 0x0" : "SENT: 0x");
            Serial.print(c, HEX);
            Serial.println();
        }
        */
        return true;
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
            for (size_t pos = 0; pos < length; ++pos) {
                data[writerPointer++] = input_data[pos];
            }
            if (writerPointer == bufferSize) {
                writerPointer = 0;
                push(input_data + length, lengthSubtraction);
            }
        }

       private:
        static constexpr size_t bufferCount{100};
        static constexpr size_t bufferChunk{20};
        static constexpr size_t bufferSize{bufferCount * bufferChunk};
        uint8_t data[bufferSize];
        size_t writerPointer{0};
        size_t readerPointer{0};
    };

    size_t writerPosition{0};
    BluetoothBuffer data_output;
    uint32_t start_time{0};
    CobsReaderBuffer data_input;
    uint32_t bytes_sent{0};
    uint32_t bytes_received{0};    
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

    uint32_t waited = (micros() - start_time)/1000;
    if (waited < 2500) {
        Serial.printf("\nFinishing delay for AT mode. Waiting %d msec.\n", 2500-waited);
        delay(2500-waited); // time needed initialization of AT mode
    }
    else {
        Serial.printf("\nAlready waited %d msec. AT mode should be ready.\n", waited);
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
    
    Serial1.print("at$ufc 00\n"); //enable flow control (req'd over 57k)
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
}  // namespace

void setBluetoothUart(const DeviceName& name) {
    bluetooth.setBluetoothUart(name);
}

CobsReaderBuffer* readSerial() {
    if (usb_mode::get() == usb_mode::BLUETOOTH_MIRROR) {
        if (usb_comm.read())
            return &usb_comm.buffer();
    }
    if (bluetooth.read())
        return &bluetooth.buffer();
    return nullptr;
}

void writeSerial(uint8_t* data, size_t length) {
    if (usb_mode::get() == usb_mode::BLUETOOTH_MIRROR) {
        usb_comm.write(data, length);
    }
    bluetooth.write(data, length);
}

void writeSerialDebug(uint8_t* data, size_t length) {
    usb_comm.write(data, length);
    // if we're connected, mirror to bluetooth?
    // bluetooth.write(data, length);
}

bool sendBluetoothUART() {
    return bluetooth.flush();
}

void printSerialReport() {
    usb_comm.printStats();
    bluetooth.printStats();
}
