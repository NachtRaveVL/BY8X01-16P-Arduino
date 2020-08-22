// BY8X01-16P-Arduino SoftwareSerial Example
// In this example, we utilize the software serial library for chips that do not have a
// hardware serial line. If one uncomments the line below inside the main header file (or
// defines it via custom build flag), software serial mode for the library will be enabled.
//
// In BY8X01-16P.h:
// // Uncomment this define to enable use of the SoftwareSerial library.
// #define BY8X0116P_ENABLE_SOFTWARE_SERIAL

#include "BY8X01-16P.h"

const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial swSerial(rxPin, txPin);  // SoftwareSerial using RX pin D2 and TX pin D3

BY8X0116P audioController(swSerial);    // Library using SoftwareSerial @9600bps, and default disabled busy pin hookup

void setup() {
    // Library will begin SoftwareSerial, so we don't need to begin anything

    pinMode(rxPin, INPUT);              // Must manually set pin modes for RX/TX pins (SoftwareSerial bug)
    pinMode(txPin, OUTPUT);

    audioController.init();             // Initializes module, also begins SoftwareSerial

    audioController.play();             // Starts playback of loaded tracks
}

void loop() {
}
