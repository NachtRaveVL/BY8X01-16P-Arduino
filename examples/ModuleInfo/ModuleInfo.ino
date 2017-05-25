// BY8X01-16P-Arduino Module Info
// If one uncomments the BY8X0116P_ENABLE_DEBUG_OUTPUT define in the libraries main
// header file (thus enabling debug output) the printModuleInfo() method becomes
// available, which will display information about the module itself, including
// initalized states, register values, current settings, etc. All calls being made will
// display internal debug information about the structure of the call itself.

// Uncomment this define to enable debug output.
#define BY8X0116P_ENABLE_DEBUG_OUTPUT       1

#include "BY8X01-16P.h"

BY8X0116P audioController;

void setup() {
    Serial.begin(115200);

    audioController.printModuleInfo();
}

void loop() {
}
