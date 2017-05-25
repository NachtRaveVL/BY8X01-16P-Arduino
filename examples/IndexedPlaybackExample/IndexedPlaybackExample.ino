// BY8X01-16P-Arduino Indexed Playback Example
// In this example, folders are named "00" through "99" and files inside them are named
// "001.mp3" (or .wav) through "255.mp3" (or .wav). The folder and file index passed into
// playFolderFileIndex will play that specific file. Note that combination play here is
// not supported so waiting between track plays is required. Having a busy pin connected
// here will allow for tighter timing control between file playbacks.

#include "BY8X01-16P.h"

const byte busyPin = 22;
BY8X0116P audioController(Serial1, busyPin); // Library using Serial1 UART and busy pin input D22

void setup() {
    Serial.begin(115200);

    Serial1.begin(9600);            // Serial1 must be started first - only supported UART baud rate is 9600

    audioController.init();         // Initializes module

    audioController.playFolderFileIndex(0, 1); // Plays "00\001.mp3"

    int numTracks = getNumberOfTracksInCurrentFolder(); // Gets number of tracks in current folder
    Serial.println(numTracks);      // Should display number of tracks in the "00" folder

    char buffer[12];
    audioController.getCurrentTrackFilename(buffer); // Gets current filename (in 8.3 format) and places it into buffer

    Serial.println(buffer);         // Should display "001.mp3"

    audioController.waitBusy();     // Blocking call that waits until all songs have completed

    // Plays all the remaining songs in the entire folder, printing out their file name upon playback
    for (int i = 2; i <= numTracks; ++i) {
        audioController.playFolderFileIndex(0, i);

        audioController.getCurrentTrackFilename(buffer);
        Serial.println(buffer);

        audioController.waitBusy();
    }

    Serial.println("All done!");
}

void loop() {
}
