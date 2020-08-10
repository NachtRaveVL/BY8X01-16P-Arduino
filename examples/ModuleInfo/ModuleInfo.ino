// BY8X01-16P-Arduino Module Info
// In this example, we enable debug output support. If one uncomments the line below
// inside the main header file (or defines it via custom build flag), debug output
// support will be enabled and the printModuleInfo() method will become available.
// Calling this method will display information about the module itself, including
// initalized states, register values, current settings, etc. Additionally, all
// library calls being made will display internal debug information about the
// structure of the call itself.
//
// In BY8X01-16P.h:
// // Uncomment this define to enable debug output.
// #define BY8X0116P_ENABLE_DEBUG_OUTPUT       1

#include "BY8X01-16P.h"

BY8X0116P audioController;

void setup() {
    Serial.begin(115200);

    audioController.printModuleInfo();
}

void loop() {
}
