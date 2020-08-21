// BY8X01-16P-Arduino SoftwareSerial Example
// In this example, we use SoftwareSerial to replicate a hardware serial line.

#include "BY8X01-16P.h"
#include "SoftwareSerial.h"

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
