/*  Arduino Library for the BY8001-16P/BY8301-16P Audio Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>

    Permission is hereby granted, free of charge, to any person
    obtaining a copy of this software and associated documentation
    files (the "Software"), to deal in the Software without
    restriction, including without limitation the rights to use,
    copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the
    Software is furnished to do so, subject to the following
    conditions:

    This permission notice shall be included in all copies or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.

    BY8X01-16P-Arduino - Version 1.0.8
*/

#ifndef BY8X0116P_H
#define BY8X0116P_H

// Library Setup

// NOTE: While editing the main header file isn't ideal, it is often the easiest given
// the Arduino IDE's limited custom build flag support. Editing this header file directly
// will affect all projects compiled on your system using these library files.

// Uncomment or -D this define to enable use of the SoftwareSerial library.
//#define BY8X0116P_ENABLE_SOFTWARE_SERIAL        // https://www.arduino.cc/en/Reference/softwareSerial

// Uncomment or -D this define to disable usage of the Scheduler library on SAM/SAMD architecures.
//#define BY8X0116P_DISABLE_SCHEDULER             // https://github.com/arduino-libraries/Scheduler

// Uncomment or -D this define to enable debouncing of the input line on isBusy() calls.
//#define BY8X0116P_ENABLE_DEBOUNCING

// Uncomment or -D this define to enable debug output.
//#define BY8X0116P_ENABLE_DEBUG_OUTPUT

// Hookup Callouts
// -PLEASE READ-
// Make sure to flip RX/TX lines when plugging into module from microcontroller. If
// running a 5v microcontroller, put a 1kΩ resistor on the line between the 
// microcontroller's TX and module's RX pins. Remove A, B, and C resistors on module
// (factory default is a resistor on A and C, while B is left open). This puts the device
// into the recommended 1-1-1 mode used for microcontroller serial control. Busy pin is
// optional to utilize but returns a 2.8v signal when playback is active (just enough
// for 5v boards to register as logic level HIGH).

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#if !defined(BY8X0116P_DISABLE_SCHEDULER) && (defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD))
#include "Scheduler.h"
#define BY8X0116P_USE_SCHEDULER
#endif

#ifdef BY8X0116P_ENABLE_SOFTWARE_SERIAL
#include <SoftwareSerial.h>
#define BY8X0116P_USE_SOFTWARE_SERIAL
#endif

#if defined(HAVE_HWSERIAL1) || defined(PIN_SERIAL1_RX) || defined(SERIAL_PORT_HARDWARE1) || defined(UART1)
#define BY8X0116P_HAS_SERIAL1
#endif

#ifndef ENABLED
#define ENABLED                         0x1                 // Enabled define (convenience)
#endif
#ifndef DISABLED
#define DISABLED                        0x0                 // Disabled define (convenience)
#endif


enum BY8X0116P_PlaybackStatus {
    BY8X0116P_PlaybackStatus_Stopped,           // Playback status stopped
    BY8X0116P_PlaybackStatus_Playing,           // Playback status playing
    BY8X0116P_PlaybackStatus_Paused,            // Playback status paused
    BY8X0116P_PlaybackStatus_FastForwarding,    // Playback status fast-forwarding
    BY8X0116P_PlaybackStatus_FastRewinding,     // Playback status fast-rewinding

    BY8X0116P_PlaybackStatus_Count,             // Internal use only
    BY8X0116P_PlaybackStatus_Undefined = -1     // Internal use only
};

enum BY8X0116P_LoopPlaybackMode {
    BY8X0116P_LoopPlaybackMode_All,             // Loop playback mode all
    BY8X0116P_LoopPlaybackMode_Folder,          // Loop playback mode folder
    BY8X0116P_LoopPlaybackMode_Single,          // Loop playback mode single
    BY8X0116P_LoopPlaybackMode_Random,          // Loop playback mode random
    BY8X0116P_LoopPlaybackMode_Disabled,        // Loop playback mode disabled

    BY8X0116P_LoopPlaybackMode_Count,           // Internal use only
    BY8X0116P_LoopPlaybackMode_Undefined = -1   // Internal use only
};

enum BY8X0116P_EqualizerProfile {
    BY8X0116P_EqualizerProfile_None,            // Equalizer profile none
    BY8X0116P_EqualizerProfile_Pop,             // Equalizer profile pop
    BY8X0116P_EqualizerProfile_Rock,            // Equalizer profile rock
    BY8X0116P_EqualizerProfile_Jazz,            // Equalizer profile jazz
    BY8X0116P_EqualizerProfile_Classic,         // Equalizer profile classic
    BY8X0116P_EqualizerProfile_Bass,            // Equalizer profile bass

    BY8X0116P_EqualizerProfile_Count,           // Internal use only
    BY8X0116P_EqualizerProfile_Undefined = -1   // Internal use only
};

enum BY8X0116P_PlaybackDevice {
    BY8X0116P_PlaybackDevice_USB,               // Playback device USB
    BY8X0116P_PlaybackDevice_MicroSD,           // Playback device MicroSD

    BY8X0116P_PlaybackDevice_Count,             // Internal use only
    BY8X0116P_PlaybackDevice_Undefined = -1     // Internal use only
};


class BY8X0116P {
public:
#ifndef BY8X0116P_USE_SOFTWARE_SERIAL

#ifdef BY8X0116P_HAS_SERIAL1
    // Library constructor. Typically called during class instantiation, before setup().
    // May skip usage of busy pin, but isBusy() will always respond false if so. May also
    // set usage of busy pin as being either active-high or active-low.
    // Boards with more than one serial line (e.g. Due/Mega/etc.) can supply a different
    // Serial instance, such as Serial1 (using RX1/TX1), Serial2 (using RX2/TX2), etc.
    // The only supported baud rate is 9600bps using mode SERIAL_8N1.
    BY8X0116P(byte busyPin = DISABLED, byte busyActiveOn = HIGH, HardwareSerial& serial = Serial1);
#endif // /ifdef BY8X0116P_HAS_SERIAL1

    // Convenience constructor for custom Serial instance. See main constructor.
    // Becomes standard library constuctor in case Serial1 isn't readily detected.
    BY8X0116P(HardwareSerial& serial, byte busyPin = DISABLED, byte busyActiveOn = HIGH);

#else

    // Library constructor. Typically called during class instantiation, before setup().
    // May skip usage of busy pin, but isBusy() will always respond false if so. May also
    // set usage of busy pin as being either active-high or active-low.
    // The only supported baud rate is 9600bps using mode SERIAL_8N1.
    BY8X0116P(SoftwareSerial& serial, byte busyPin = DISABLED, byte busyActiveOn = HIGH);

#endif // /ifndef BY8X0116P_USE_SOFTWARE_SERIAL

    // Initializes module. Typically called in setup().
    void init();

    // Mode accessors
    byte getBusyPin();
    byte getBusyActiveOn();
    uint32_t getSerialBaud();
    uint16_t getSerialMode();

    // Playback control
    void play();
    void pause();
    void stop(bool blocking = false);

    // Indexed track play
    // playFileIndex supports combination play, in which multiple calls will queue up to
    // 10 songs. Index is prescribed by the FAT file system, and is generally in the
    // order that the files were copied to the flash drive, but not guaranteed. Indexing
    // runs across all files in every subfolder. A file sorter software program (such as
    // "DriveSort" or "FAT32 Sorter") should be used if specific file index order for
    // playback is required.
    void playFileIndex(uint16_t fileIndex); // fileIndex 1-65535

    // playFolderFileIndex requires that folders be named "00" through "99" and the files
    // inside of them be named "001" through "255". This function does not support combination
    // play, and will adjust the internal current track index.
    void playFolderFileIndex(byte folderIndex, byte fileIndex); // folderIndex 0-99, fileIndex 1-255

    // Track fast-f/r control
    void fastForward();
    void fastRewind();

    // Track/folder control
    void nextTrack();
    void previousTrack();
    void nextFolder();
    void previousFolder();

    // Volume control (volume range: 0-30, retains setting in eeprom)
    void increaseVolume();      // +1
    void decreaseVolume();      // -1
    void setVolume(int volume);
    int getVolume();

    // Playback status
    BY8X0116P_PlaybackStatus getPlaybackStatus();

    // Loop playback mode
    void setLoopPlaybackMode(BY8X0116P_LoopPlaybackMode pbLoopMode);
    BY8X0116P_LoopPlaybackMode getLoopPlaybackMode();

    // Equalizer profile (retains setting in eeprom)
    void setEqualizerProfile(BY8X0116P_EqualizerProfile eqProfile);
    BY8X0116P_EqualizerProfile getEqualizerProfile();

    // Total number of tracks
    uint16_t getTotalNumberOfTracks();
    uint16_t getTotalNumberOfTracks(BY8X0116P_PlaybackDevice device);

    // Number of tracks in current folder (unavailable on firmware fw0001)
    uint16_t getNumberOfTracksInCurrentFolder();

    // Current track file index (unavailable on firmware fw0001)
    uint16_t getCurrentTrackFileIndex();
    uint16_t getCurrentTrackFileIndex(BY8X0116P_PlaybackDevice device);

    // Current track elapsed/total time
    uint16_t getCurrentTrackElapsedTime();
    uint16_t getCurrentTrackTotalTime();

    // Playback device control
    void setPlaybackDevice(BY8X0116P_PlaybackDevice device);
    BY8X0116P_PlaybackDevice getPlaybackDevice();

    // Spot insertion play (USB playback device only) - pauses current track and resumes it after playing spot track
    void spotPlayFileIndex(uint16_t fileIndex); // fileIndex 1-65535, see notes for playFileIndex
    void spotPlayFolderFileIndex(byte folderIndex, byte fileIndex); // folderIndex 0-99, fileIndex 1-255, see notes for playFolderFileIndex

    // Gets current track's filename in short 8.3 format
    void getCurrentTrackFilename(char *buffer, int maxLength = 12); // maxLength must at least be 12, recommended 13

    // Gets module's firmware number in format "fwXX" or "fwXXXX" (format depends on hw revision)
    void getFirmwareVersion(char *buffer, int maxLength = 6); // maxLength must at least be 6, recommended 7

    // Busy line signal (true when device is playing anything)
    bool isBusy(); // Requires busyPin connection
    // Blocking call that waits until busy signal disables (timeout in ms, 0 = no timeout)
    bool waitBusy(int timeout = 0); // Requires busyPin connection

    // Playback active status (true when playback status is not stopped)
    bool isPlaybackActive();
    // Blocking call that waits until playback has finished (timeout in ms, 0 = no timeout)
    bool waitPlaybackFinished(int timeout = 0);

    // Standby mode toggle (when in standby mode, module has 10mA current draw)
    void toggleStandBy(bool blocking = true);
    bool isStandingBy();

    // Device reset control
    void reset(bool blocking = true); // Blocking call is highly recommended here
    bool isResetting();

    // MicroSD card insertion (once removed, attempts a standby double call to clear flag)
    bool isCardInserted();

    // A response line cleanup routine that can be setup as a scheduled task, to keep state control updated
    void cleanupRoutine();

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    int getSerialInterfaceNumber();
    void printModuleInfo();
#endif

protected:
    byte _busyPin;                                          // Busy pin to use for playback tracking (default: DISABLED)
    byte _busyActiveOn;                                     // Busy pin is active on HIGH or LOW (default: HIGH)
#ifndef BY8X0116P_USE_SOFTWARE_SERIAL
    HardwareSerial* _serial;                                // Serial class instance (unowned) (default: Serial1)
#else
    SoftwareSerial* _serial;                                // Serial class instance (unowned)
#endif
    int8_t _isBlockingRspLn;                                // Tracks if response line should be blocked by other routines
    bool _isCleaningRspLn;                                  // Tracks if code is already inside of clean response line routine
    bool _isStandingBy;                                     // Tracks if device is in standby mode
    bool _isResetting;                                      // Tracks if device is reseting
    bool _isCardInserted;                                   // Tracks if MicroSD card is inserted
    unsigned long _lastReqTime;                             // Timestamp of last request made
    unsigned long _lastClnTime;                             // Timestamp of last cleanup call

    bool _isBusy();
    bool _waitBusy(int timeout = 0);
    bool _isPlaybackActive();
    bool _waitPlaybackFinished(int timeout = 0);

    void sendCommand(byte cmdID);
    void sendCommand(byte cmdID, byte param);
    void sendCommand(byte cmdID, uint16_t param);
    void sendCommand(byte cmdID, byte param1, byte param2);

    uint16_t receiveCommand(byte cmdID);
    int receiveCommand(byte cmdID, char *respData, int respLength, int maxLength);

    void writeRequest(byte *rqstData, bool cleanRspLn = false);
    int readResponse(char *respData, int respLength, int maxLength);

    void waitRequest();
    bool waitResponse();

    bool cleanResponse();
    void waitClean(int timeout = 0);
};

#endif // /ifndef BY8X0116P_H
