// BY8X01-16P-Arduino Simple Example

#include "BY8X01-16P.h"

BY8X0116P audioController;              // Library using default disabled busy pin hookup, and default Serial1 @9600bps

void setup() {
    Serial.begin(115200);               // Begin Serial and Serial1 interfaces
    Serial1.begin(audioController.getSerialBaud(),
                  audioController.getSerialMode());

    audioController.init();             // Initializes module

    audioController.setVolume(20);      // Sets player volume to 20 (out of 30 max)

    audioController.play();             // Starts playback of loaded tracks
}

void loop() {
}
