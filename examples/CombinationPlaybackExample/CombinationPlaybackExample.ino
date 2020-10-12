// BY8X01-16P-Arduino Combination Playback Example
// In this example, files are loaded onto the MicroSD card and queued using the special
// playFileIndex method, which allows up to 10 songs to be queued for playback. Index is
// prescribed by the FAT file system, and is generally in the order that the files were
// copied to the flash drive, but not guaranteed. Indexing runs across all files in every
// subfolder. A file sorter software program (such as "DriveSort" or "FAT32 Sorter")
// should be used if specific file index order for playback is required.

#include "BY8X01-16P.h"

BY8X0116P audioController;              // Library using default disabled busy pin hookup, and default Serial1 @9600bps

void setup() {
    Serial.begin(115200);               // Begin Serial and Serial1 interfaces
    Serial1.begin(audioController.getSerialBaud(),
                  audioController.getSerialMode());

    audioController.init();             // Initializes module

    audioController.setEqualizerProfile(BY8X0116P_EqualizerProfile_Rock); // Sets player equalizer profile to Rock

    audioController.playFileIndex(0); // Queues first file on MicroSD card for playback
    audioController.playFileIndex(1); // Queues second file on MicroSD card for playback
    audioController.playFileIndex(2); // Queues third file on MicroSD card for playback
    audioController.playFileIndex(3); // Queues fourth file on MicroSD card for playback
    audioController.playFileIndex(4); // Queues fifth file on MicroSD card for playback

    audioController.waitPlaybackFinished(); // Blocking call that waits until all songs have completed

    Serial.println("All done!");
}

void loop() {
}
