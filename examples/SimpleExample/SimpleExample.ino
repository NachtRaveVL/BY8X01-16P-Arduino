// BY8X01-16P-Arduino Simple Example

#include "BY8X01-16P.h"

BY8X0116P audioController;          // Library using default Serial1 UART and no busy pin hookup

void setup() {
    Serial1.begin(9600);            // Serial1 must be started first - only supported UART baud rate is 9600

    audioController.init();         // Initializes module

    audioController.setVolume(20);  // Sets player volume to 20 (out of 30 max)

    audioController.play();         // Starts playback of loaded tracks
}

void loop() {
}
