// BY8X01-16P-Arduino SoftwareSerial Example
// In this example, we utilize the software serial library for chips that do not have a
// hardware serial line. If one uncomments the line below inside the main header file (or
// defines it via custom build flag), software serial mode for the library will be enabled.
// You may refer to https://forum.arduino.cc/index.php?topic=602603.0 on how to define
// custom build flags manually via modifying platform[.local].txt.
//
// In BY8X01-16P.h:
// // Uncomment or -D this define to enable use of the SoftwareSerial library.
// #define BY8X0116P_ENABLE_SOFTWARE_SERIAL        // https://www.arduino.cc/en/Reference/softwareSerial
//
// Alternatively, in platform[.local].txt:
// build.extra_flags=-DBY8X0116P_ENABLE_SOFTWARE_SERIAL

#include "BY8X01-16P.h"

const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial swSerial(rxPin, txPin);  // SoftwareSerial using RX pin D2 and TX pin D3

BY8X0116P audioController(swSerial);    // Library using SoftwareSerial @9600bps, and default disabled busy pin hookup

void setup() {
    swSerial.begin(audioController.getSerialBaud()); // Begin SoftwareSerial

    pinMode(rxPin, INPUT);              // Must manually set pin modes for RX/TX pins (SoftwareSerial bug)
    pinMode(txPin, OUTPUT);

    audioController.init();             // Initializes module

    audioController.play();             // Starts playback of loaded tracks
}

void loop() {
}
