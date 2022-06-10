# BY8X01-16P-Arduino
Arduino Library for the BY8001-16P/BY8301-16P Audio Module.

**BY8X01-16P-Arduino v1.1.0**

Library to control a BY8001-16P or BY83001-16P audio module from an Arduino board.  
Licensed under the non-restrictive MIT license.

Created by NachtRaveVL, August 1st, 2016.

This library allows communication with boards running a BY8001-16P or BY8301-16P audio module. It supports the full feature set of the BY8X01-16P chipset such as queued combination playback, indexed folder/file playback, loop playback mode, equalizer profile, spot insertion play, etc.

Made primarily for Arduino microcontrollers, but should work with PlatformIO, ESP32/8266, Teensy, and others - although one might experience turbulence until the bug reports get ironed out. Unknown architectures must ensure `BUFFER_LENGTH` (or `I2C_BUFFER_LENGTH`) and `WIRE_INTERFACES_COUNT` are properly defined.

Dependencies include: Scheduler (SAM/SAMD only, disableable), TaskScheduler (alternate to Scheduler, disableable), and CoopTask (alternate to TaskScheduler, optional).

The datasheet for the IC is available at <https://forum.arduino.cc/index.php?action=dlattach;topic=306442.0;attach=129563>.

## Library Setup

### Installation

The easiest way to install this library is to utilize the Arduino IDE library manager, or through a package manager such as PlatformIO. Otherwise, simply download this library and extract its files into a `BY8X01-16P-Arduino` folder in your Arduino custom libraries folder, typically found in your `[My ]Documents\Arduino\libraries` folder (Windows), or `~/Documents/Arduino/libraries/` folder (Linux/OSX).

### Header Defines

There are several defines inside of the library's main header file that allow for more fine-tuned control of the library. You may edit and uncomment these lines directly, or supply them via custom build flags. While editing the main header file isn't ideal, it is often easiest. Note that editing the library's main header file directly will affect all projects compiled on your system using those modified library files.

Alternatively, you may also refer to <https://forum.arduino.cc/index.php?topic=602603.0> on how to define custom build flags manually via modifying the platform[.local].txt file. Note that editing such directly will affect all other projects compiled on your system using those modified platform framework files.

From BY8X01-16P.h:
```Arduino
// Uncomment or -D this define to enable usage of the SoftwareSerial Arduino library.
//#define BY8X0116P_ENABLE_SOFTWARE_SERIAL          // https://www.arduino.cc/en/Reference/softwareSerial

// Uncomment or -D this define to completely disable usage of any multitasking commands, such as yield().
//#define BY8X0116P_DISABLE_MULTITASKING

// Uncomment or -D this define to disable usage of the Scheduler library, for SAM/SAMD architechtures.
//#define BY8X0116P_DISABLE_SCHEDULER               // https://github.com/arduino-libraries/Scheduler

// Uncomment or -D this define to disable usage of the TaskScheduler library, in place of Scheduler.
//#define BY8X0116P_DISABLE_TASKSCHEDULER           // https://github.com/arkhipenko/TaskScheduler

// Uncomment or -D this define to enable usage of the CoopTask library, in place of TaskScheduler and Scheduler.
//#define BY8X0116P_ENABLE_COOPTASK                 // https://github.com/dok-net/CoopTask

// Uncomment or -D this define to enable debouncing of the input line on isBusy() calls.
//#define BY8X0116P_ENABLE_DEBOUNCING

// Uncomment or -D this define to enable debug output.
//#define BY8X0116P_ENABLE_DEBUG_OUTPUT
```

### Library Initialization

There are several initialization mode settings exposed through this library that are used for more fine-tuned control.

#### Class Instantiation

The library's class object must first be instantiated, commonly at the top of the sketch where pin setups are defined (or exposed through some other mechanism), which makes a call to the library's class constructor. The constructor allows one to set the busy pin, busy pin active-on mode, and Serial class instance. The default constructor values of the library, if left unspecified, is busy pin `DISABLED`, busy pin active-on mode `HIGH`, and Serial class instance `Serial1` @`9600`bps using mode `SERIAL_8N1`. _In the case that Serial1 cannot be readily detected, then the Serial class instance must be explicitly provided._

From BY8X01-16P.h, in class BY8X0116P, when in hardware serial mode:
```Arduino
    // Library constructor. Typically called during class instantiation, before setup().
    // May skip usage of busy pin, but isBusy() will always respond false if so. May also
    // set usage of busy pin as being either active-high or active-low.
    // Boards with more than one serial line (e.g. Due/Mega/etc.) can supply a different
    // Serial instance, such as Serial1 (using RX1/TX1), Serial2 (using RX2/TX2), etc.
    // The only supported baud rate is 9600bps using mode SERIAL_8N1.
    BY8X0116P(byte busyPin = DISABLED, byte busyActiveOn = HIGH, HardwareSerial& serial = Serial1);

    // Convenience constructor for custom Serial instance. See main constructor.
    // Becomes standard library constuctor in case Serial1 isn't readily detected.
    BY8X0116P(HardwareSerial& serial, byte busyPin = DISABLED, byte busyActiveOn = HIGH);
```

From BY8X01-16P.h, in class BY8X0116P, when in sofware serial mode (see examples for sample usage):
```Arduino
    // Library constructor. Typically called during class instantiation, before setup().
    // May skip usage of busy pin, but isBusy() will always respond false if so. May also
    // set usage of busy pin as being either active-high or active-low.
    // The only supported baud rate is 9600bps using mode SERIAL_8N1.
    BY8X0116P(SoftwareSerial& serial, byte busyPin = DISABLED, byte busyActiveOn = HIGH);
```

#### Device Initialization

Additionally, a call is expected to be provided to the library class object's `init()` method, commonly called inside of the sketch's `setup()` function.

From BY8X01-16P.h, in class BY8X0116P:
```Arduino
    // Initializes module. Typically called in setup().
    void init();
```

## Hookup Callouts

### Serial UART

* Make sure to flip RX/TX lines when hooking into module from microcontroller.
* If running a 5v microcontroller, place a 1kΩ resistor between the microcontroller's TX pin and module's RX pin.
  * Alternatively, instead use a bi-directional logic level converter from 5v to 3.3v.
* Remove A, B, and C resistors on module (factory default is a resistor on A and C, while B is left open).
  * This puts the device into the recommended 1-1-1 mode used for microcontroller serial control.
* Busy pin is optional to utilize but returns a 2.8v signal when playback is active (just enough for 5v boards to register as logic level HIGH).

## Example Usage

Below are several examples of library usage.

### Simple Example

```Arduino
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

```

### Combination Playback Example

In this example, files are loaded onto the MicroSD card and queued using the special `playFileIndex()` method, which allows up to 10 songs to be queued for playback.

Index is prescribed by the FAT file system, and is generally in the order that the files were copied to the flash drive, but not guaranteed. Indexing runs across all files in every subfolder. A file sorter software program (such as "DriveSort" or "FAT32 Sorter") should be used if specific file index order for playback is required.

```Arduino
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

```

### Indexed Playback Example

In this example, we use an indexed naming strategy to play files in folders via integer indexes.

Folders on the MicroSD card should be named "00" through "99" and audio files inside them named "001.mp3" (or .wav) through "255.mp3" (or .wav). The folder and file index passed into `playFolderFileIndex()` will play that specific audio file. Note that combination play here is not supported so waiting between track plays is required. Having a busy pin connected here will allow for tighter timing control between file playbacks.

```Arduino
#include "BY8X01-16P.h"

const byte busyPin = 22;
BY8X0116P audioController(busyPin);     // Library using busy pin input D22, and default Serial1 @9600bps

void setup() {
    Serial.begin(115200);               // Begin Serial and Serial1 interfaces
    Serial1.begin(audioController.getSerialBaud(),
                  audioController.getSerialMode());

    audioController.init();             // Initializes module

    audioController.playFolderFileIndex(0, 1); // Plays "00/001.mp3"

    int numTracks = audioController.getNumberOfTracksInCurrentFolder(); // Gets number of tracks in current folder
    Serial.println(numTracks);          // Should display number of tracks in the "00" folder

    char buffer[12];
    audioController.getCurrentTrackFilename(buffer); // Gets current filename (in 8.3 format) and places it into buffer

    Serial.println(buffer);             // Should display "001.mp3"

    audioController.waitBusy();         // Blocking call that waits until all songs have completed

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

```

### SoftwareSerial Example

In this example, we utilize the software serial library for chips that do not have a hardware serial line.

If one uncomments the line below inside the main header file (or defines it via custom build flag), software serial mode for the library will be enabled. Additionally, if this serial class instance is instantiated before `setup()` then you will have to manually set the pin modes for the RX and TX pins due to a library bug (in which it attempts to do so in its class constructor, before the Arduino system has initialized). Lastly note that, while in software serial mode, the serial baud rate returned by the library (via `getSerialBaud()`) is only an upper bound and may not represent the actual serial baud rate achieved.

In BY8X01-16P.h:
```Arduino
// Uncomment or -D this define to enable usage of the SoftwareSerial library.
#define BY8X0116P_ENABLE_SOFTWARE_SERIAL        // https://www.arduino.cc/en/Reference/softwareSerial
```  
Alternatively, in platform[.local].txt:
```Arduino
build.extra_flags=-DBY8X0116P_ENABLE_SOFTWARE_SERIAL
```

In main sketch:
```Arduino
#include "BY8X01-16P.h"

const byte rxPin = 2;
const byte txPin = 3;
SoftwareSerial swSerial(rxPin, txPin);  // SoftwareSerial using RX pin D2 and TX pin D3

BY8X0116P audioController(swSerial);    // Library using SoftwareSerial @9600bps, and default disabled busy pin hookup

void setup() {
    swSerial.begin(audioController.getSerialBaud()); // Begin SoftwareSerial

    pinMode(rxPin, INPUT);              // Must manually set pin modes for RX/TX pins (SoftwareSerial bug)
    pinMode(txPin, OUTPUT);

    audioController.init();             // Initializes module

    audioController.play();             // Starts playback of loaded tracks
}

void loop() {
}

```

## Module Info

In this example, we enable debug output support to print out module diagnostic information.

If one uncomments the line below inside the main header file (or defines it via custom build flag), debug output support will be enabled and the `printModuleInfo()` method will become available. Calling this method will display information about the module itself, including initalized states, register values, current settings, etc. Additionally, all library calls being made will display internal debug information about the structure of the call itself. An example of this output is shown below.

In BY8X01-16P.h:
```Arduino
// Uncomment or -D this define to enable debug output.
#define BY8X0116P_ENABLE_DEBUG_OUTPUT
```  
Alternatively, in platform[.local].txt:
```Arduino
build.extra_flags=-DBY8X0116P_ENABLE_DEBUG_OUTPUT
```

In main sketch:
```Arduino
#include "BY8X01-16P.h"

BY8X0116P audioController;              // Library using default disabled busy pin hookup, and default Serial1 @9600bps

void setup() {
    Serial.begin(115200);               // Begin Serial and Serial1 interfaces
    Serial1.begin(audioController.getSerialBaud(),
                  audioController.getSerialMode());

    audioController.init();             // Initializes module

    audioController.printModuleInfo();  // Prints module diagnostic information
}

void loop() {
}

```

In serial monitor:
```
 ~~~ BY8X0116P Module Info ~~~

Busy Pin: <disabled>

Serial Instance: 1: Serial1
Serial Baud: 9600Hz

State:
  Standing By: false, Resetting: false, Card Inserted: true

Timers:
  Current Time: 17 ms, Last Request: 0 ms, Last Clean: 0 ms

Firmware Version:
BY8X0116P::getFirmwareVersion
  BY8X0116P::writeRequest Cmd: 0x14, Chk: 0x17
  BY8X0116P::readResponse respData[2]: 20
fw20

Total Number Of Tracks:
BY8X0116P::getTotalNumberOfTracks
  BY8X0116P::writeRequest Cmd: 0x18, Chk: 0x1B
  BY8X0116P::readResponse respData[4]: 00ff
  BY8X0116P::writeRequest Cmd: 0x15, Chk: 0x16
  BY8X0116P::readResponse respData[4]: 0000
0

Current Track File Index:
BY8X0116P::getCurrentTrackFileIndex
  BY8X0116P::writeRequest Cmd: 0x18, Chk: 0x1B
  BY8X0116P::readResponse respData[4]: 00ff
  BY8X0116P::writeRequest Cmd: 0x19, Chk: 0x1A
  BY8X0116P::readResponse respData[4]: 0000
0

Current Track Filename:
BY8X0116P::getCurrentTrackFilename
  BY8X0116P::writeRequest Cmd: 0x1E, Chk: 0x1D
  BY8X0116P::readResponse respData[0]: 


Current Track Elapsed Time:
BY8X0116P::getCurrentTrackElapsedTime
  BY8X0116P::writeRequest Cmd: 0x1C, Chk: 0x1F
  BY8X0116P::readResponse respData[4]: ffff
65535

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
  BY8X0116P::readResponse respData[4]: 00ff
255: 

```
