# BY8X01-16P-Arduino
Arduino Library for the BY8001-16P/BY8301-16P Audio Module.

**BY8X01-16P-Arduino v1.0.6**

Library to control a BY8001-16P or BY83001-16P audio module from an Arduino board.  
Licensed under the non-restrictive MIT license.

Created by NachtRaveVL, August 1st, 2016.

This library allows communication with boards running a BY8001-16P or BY8301-16P audio module. It supports the full feature set of the BY8X01-16P chipset such as queued combination playback, indexed folder/file playback, loop playback mode, equalizer profile, spot insertion play, etc.

Dependencies include Scheduler if on a ARM/ARMD architecture (Due, Zero, etc.), but usage can be disabled via library setup defines.

## Library Setup

There are several defines inside of the library's header file that allows for more fine-tuned control.

In BY8X01-16P.h:
```Arduino
// Uncomment this define to disable usage of the Scheduler library on SAM/SAMD architecures.
//#define BY8X0116P_DISABLE_SCHEDULER         1   // https://github.com/arduino-libraries/Scheduler

// Uncomment this define to enable debouncing of the input line on isBusy() calls.
//#define BY8X0116P_ENABLE_DEBOUNCING         1

// Uncomment this define to enable debug output.
//#define BY8X0116P_ENABLE_DEBUG_OUTPUT       1
```

## Hookup Instructions

Make sure to flip RX/TX lines when plugging into device from MCU. If running a 5v Arduino board, put a 1k Ohm resistor between the MCU's TX and device's RX pin (not required if on a 3.3v device). Also, remove A, B, and C resistors on device (factory default is a resistor on A and C, while B is left open), which puts the device into the recommended 1-1-1 mode used for MCU serial control. Busy pin returns a 2.8v signal when playback is active (just enough for 5v boards to register as logic level HIGH), and is optional for library usage.

## Example Usage

Below are several examples of library usage.

### Simple Example
```Arduino
#include "BY8X01-16P.h"

BY8X0116P audioController;          // Library using default Serial1 UART and no busy pin hookup

void setup() {
    Serial1.begin(9600);            // Serial1 must be started first - only supported UART baud rate is 9600

    audioController.init();         // Initializes module

    audioController.setVolume(20);  // Sets player volume to 20 (out of 30 max)

    audioController.play();         // Starts playback of loaded tracks
}

```

### Combination Playback Example

In this example, files are loaded onto the MicroSD and queued using the special playFileIndex method, which allows up to 10 songs to be queued for playback. Index is prescribed by the FAT file system, and is generally in the order that the files were copied to the flash drive, but not guaranteed. Indexing runs across all files in every subfolder. A file sorter software program (such as "DriveSort" or "FAT32 Sorter") should be used if specific file index order for playback is required.

```Arduino
#include "BY8X01-16P.h"

BY8X0116P audioController;          // Library using default Serial1 UART and no busy pin hookup

void setup() {
    Serial.begin(115200);

    Serial1.begin(9600);            // Serial1 must be started first - only supported UART baud rate is 9600

    audioController.init();         // Initializes module

    audioController.setEqualizerProfile(BY8X0116P_EqualizerProfile_Rock); // Sets player equalizer profile to Rock

    audioController.playFileIndex(0); // Queues first file on MicroSD card for playback
    audioController.playFileIndex(1); // Queues second file on MicroSD card for playback
    audioController.playFileIndex(2); // Queues third file on MicroSD card for playback
    audioController.playFileIndex(3); // Queues fourth file on MicroSD card for playback
    audioController.playFileIndex(4); // Queues fifth file on MicroSD card for playback

    audioController.waitPlaybackFinished(); // Blocking call that waits until all songs have completed

    Serial.println("All done!");
}

```

### Indexed Playback Example

In this example, folders are named "00" through "99" and files inside them are named "001.mp3" (or .wav) through "255.mp3" (or .wav). The folder and file index passed into playFolderFileIndex will play that specific file. Note that combination play here is not supported so waiting between track plays is required. Having a busy pin connected here will allow for tighter timing control between file playbacks.

```Arduino
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

```

### SoftwareSerial Example

In this example, we use SoftwareSerial to replicate a hardware serial line.

```Arduino
#include "BY8X01-16P.h"
#include "SoftwareSerial.h"

const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial swSerial(rxPin, txPin); // SoftwareSerial using RX pin D2 and TX pin D3

BY8X0116P audioController(swSerial); // Library using SoftwareSerial and no busy pin hookup

void setup() {
    pinMode(rxPin, INPUT);          // Must manually setup pin modes for RX/TX pins
    pinMode(txPin, OUTPUT);

    swSerial.begin(9600);           // swSerial must be started first - only supported UART baud rate is 9600

    audioController.init();         // Initializes module

    audioController.play();         // Starts playback of loaded tracks
}

```

## Module Info

If one uncomments the BY8X0116P_ENABLE_DEBUG_OUTPUT define in the libraries main header file (thus enabling debug output) the printModuleInfo() method becomes available, which will display information about the module itself, including initalized states, register values, current settings, etc. All calls being made will display internal debug information about the structure of the call itself. An example of this output is shown here:

In BY8X01-16P.h:
```Arduino
// Uncomment this define to enable debug output.
#define BY8X0116P_ENABLE_DEBUG_OUTPUT       1
```

In main sketch:
```Arduino
BY8X0116P audioController;

void setup() {
    // ...

    audioController.printModuleInfo();
}
```

In serial monitor:
```
 ~~~ BY8X0116P Module Info ~~~

Busy Pin:
D22 (active-high)

State:
  Standing By: false, Resetting: false, Card Inserted: true

Timers:
  Current Time: 26 ms, Last Request: 0 ms, Last Clean: 0 ms

Firmware Version:
BY8X0116P::getFirmwareVersion
  BY8X0116P::writeRequest Cmd: 0x14, Chk: 0x17
  BY8X0116P::readResponse respData[2]: 20
fw20

Total Number Of Tracks:
BY8X0116P::getTotalNumberOfTracks
  BY8X0116P::writeRequest Cmd: 0x18, Chk: 0x1B
  BY8X0116P::readResponse respData[4]: 0001
  BY8X0116P::writeRequest Cmd: 0x15, Chk: 0x16
  BY8X0116P::readResponse respData[4]: 009c
156

Current Track File Index:
BY8X0116P::getCurrentTrackFileIndex
  BY8X0116P::writeRequest Cmd: 0x18, Chk: 0x1B
  BY8X0116P::readResponse respData[4]: 0001
  BY8X0116P::writeRequest Cmd: 0x19, Chk: 0x1A
  BY8X0116P::readResponse respData[4]: 004c
76

Current Track Filename:
BY8X0116P::getCurrentTrackFilename
  BY8X0116P::writeRequest Cmd: 0x1E, Chk: 0x1D
  BY8X0116P::readResponse respData[11]: 021     MP3
021.MP3

Current Track Elapsed Time:
BY8X0116P::getCurrentTrackElapsedTime
  BY8X0116P::writeRequest Cmd: 0x1C, Chk: 0x1F
  BY8X0116P::readResponse respData[4]: 0000
0

Current Track Total Time:
BY8X0116P::getCurrentTrackTotalTime
  BY8X0116P::writeRequest Cmd: 0x1D, Chk: 0x1E
  BY8X0116P::readResponse respData[4]: 0000
0

Playback Status:
BY8X0116P::getPlaybackStatus
  BY8X0116P::writeRequest Cmd: 0x10, Chk: 0x13
  BY8X0116P::readResponse respData[4]: 0000
0: BY8X0116P_PlaybackStatus_Stopped

Loop Playback Mode:
BY8X0116P::getLoopPlaybackMode
  BY8X0116P::writeRequest Cmd: 0x13, Chk: 0x10
  BY8X0116P::readResponse respData[4]: 0004
4: BY8X0116P_LoopPlaybackMode_Disabled

Equalizer Profile:
BY8X0116P::getEqualizerProfile
  BY8X0116P::writeRequest Cmd: 0x12, Chk: 0x11
  BY8X0116P::readResponse respData[4]: 0000
0: BY8X0116P_EqualizerProfile_None

Playback Device:
BY8X0116P::getPlaybackDevice
  BY8X0116P::writeRequest Cmd: 0x18, Chk: 0x1B
  BY8X0116P::readResponse respData[4]: 0001
1: BY8X0116P_PlaybackDevice_MicroSD
```
