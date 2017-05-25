// BY8X01-16P-Arduino SoftwareSerial Example
// In this example, we use SoftwareSerial to replicate a hardware serial line.

```Arduino
#include "BY8X01-16P.h"
#include "SoftwareSerial.h"

const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial swSerial(rxPin, txPin); // SoftwareSerial using RX pin D2 and TX pin D3

BY8X0116P audioController(swSerial); // Library using SoftwareSerial and no busy pin hookup

void setup() {
    Serial.begin(115200);

    pinMode(rxPin, INPUT);          // Must manually setup pin modes for RX/TX pins
    pinMode(txPin, OUTPUT);

    swSerial.begin(9600);           // swSerial must be started first - only supported UART baud rate is 9600

    audioController.init();         // Initializes module

    audioController.play();         // Starts playback of loaded tracks
}

void loop() {
}
