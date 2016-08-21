/*  Arduino Library for the BY8001-16P/BY8301-16P Audio Module.
    Copyright (c) 2016 NachtRaveVL      <nachtravevl@gmail.com>

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

    BY8X01-16P-Arduino - Version 1.0.6
*/

#ifndef BY8X0116P_H
#define BY8X0116P_H

// Library Setup

// Uncomment this define to disable usage of the Scheduler library on SAM/SAMD architecures.
//#define BY8X0116P_DISABLE_SCHEDULER         1   // https://github.com/arduino-libraries/Scheduler

// Uncomment this define to enable debouncing of the input line on isBusy() calls.
//#define BY8X0116P_ENABLE_DEBOUNCING         1

// Uncomment this define to enable debug output.
//#define BY8X0116P_ENABLE_DEBUG_OUTPUT       1

// Hookup Instructions
// Make sure to flip RX/TX lines when plugging into device from MCU. If running a 5v
// Arduino board, put a 1k Ohm resistor between the MCU's TX and device's RX pin (not
// required if on a 3.3v device). Also, remove A, B, and C resistors on device (factory
// default is a resistor on A and C, while B is left open), which puts the device into
// the recommended 1-1-1 mode used for MCU serial control. Busy pin returns a 2.8v signal
// when playback is active (just enough for 5v boards to register as logic level HIGH),
// and is optional for library usage.

#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif
#include <Stream.h>

typedef enum {
    BY8X0116P_PlaybackStatus_Stopped,
    BY8X0116P_PlaybackStatus_Playing,
    BY8X0116P_PlaybackStatus_Paused,
    BY8X0116P_PlaybackStatus_FastForwarding,
    BY8X0116P_PlaybackStatus_FastRewinding,

    BY8X0116P_PlaybackStatus_Count
} BY8X0116P_PlaybackStatus;

typedef enum {
    BY8X0116P_LoopPlaybackMode_All,
    BY8X0116P_LoopPlaybackMode_Folder,
    BY8X0116P_LoopPlaybackMode_Single,
    BY8X0116P_LoopPlaybackMode_Random,
    BY8X0116P_LoopPlaybackMode_Disabled,

    BY8X0116P_LoopPlaybackMode_Count
} BY8X0116P_LoopPlaybackMode;

typedef enum {
    BY8X0116P_EqualizerProfile_None,
    BY8X0116P_EqualizerProfile_Pop,
    BY8X0116P_EqualizerProfile_Rock,
    BY8X0116P_EqualizerProfile_Jazz,
    BY8X0116P_EqualizerProfile_Classic,
    BY8X0116P_EqualizerProfile_Bass,

    BY8X0116P_EqualizerProfile_Count
} BY8X0116P_EqualizerProfile;

typedef enum {
    BY8X0116P_PlaybackDevice_USB,
    BY8X0116P_PlaybackDevice_MicroSD,

    BY8X0116P_PlaybackDevice_Count
} BY8X0116P_PlaybackDevice;

class BY8X0116P {
public:
    // May use any instance of Stream for serial communication, including SoftwareSerial,
    // HardwareSerial, etc. The only supported baud rate is 9600 and mode SERIAL_8N1. May
    // skip usage of busy pin, but isBusy() will always respond false if so. May also set
    // usage of busy pin being either active-high or active-low.
#ifdef HAVE_HWSERIAL1
    BY8X0116P(Stream& stream = Serial1, byte busyPin = 0, byte busyActiveOn = HIGH);
#else
    BY8X0116P(Stream& stream, byte busyPin = 0, byte busyActiveOn = HIGH);
#endif

    // Called in setup()
    void init();

    byte getBusyPin();
    byte getBusyActiveOn();

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
    void printModuleInfo();
#endif

private:
    Stream *_stream;            // Stream/Serial class to use (default: Serial1, if available)
    byte _busyPin;              // Busy pin to use for playback tracking (default: disabled/0)
    byte _busyActiveOn;         // Busy pin is active on HIGH or LOW (default: HIGH)
    int8_t _isBlockingRspLn;    // Tracks if response line should be blocked by other routines
    bool _isCleaningRspLn;      // Tracks if code is already inside of clean response line routine
    bool _isStandingBy;         // Tracks if device is in standby mode
    bool _isResetting;          // Tracks if device is reseting
    bool _isCardInserted;       // Tracks if MicroSD card is inserted
    unsigned long _lastReqTime; // Timestamp of last request made
    unsigned long _lastClnTime; // Timestamp of last cleanup call

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

#endif
