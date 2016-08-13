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

    BY8X01-16P-Arduino - Version 1.0.3
*/

#include "BY8X01-16P.h"
#if (defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)) && !defined(BY8X0116P_DISABLE_SCHEDULER)
#include "Scheduler.h"
#define BY8X0116P_USE_SCHEDULER         1
#endif

#define BY8X0116P_CMD_PLAY              0x01
#define BY8X0116P_CMD_PAUSE             0x02
#define BY8X0116P_CMD_NEXT_TRACK        0x03
#define BY8X0116P_CMD_PREV_TRACK        0x04
#define BY8X0116P_CMD_INC_VOLUME        0x05
#define BY8X0116P_CMD_DEC_VOLUME        0x06
#define BY8X0116P_CMD_TGL_STANDBY       0x07 // Toggles between standby/normal, when in standby 10mA current draw
#define BY8X0116P_CMD_RESET             0x09
#define BY8X0116P_CMD_FAST_FORWARD      0x0A
#define BY8X0116P_CMD_FAST_REWIND       0x0B
#define BY8X0116P_CMD_STOP              0x0E
#define BY8X0116P_CMD_SET_VOLUME        0x31 // level 0-30 (power failure retains setting)
#define BY8X0116P_CMD_SET_EQ_PROFILE    0x32 // profile 0-5 (none, Pop, Rock, Jazz, Classic, Bass) (power failure retains setting)
#define BY8X0116P_CMD_SET_LOOP_MODE     0x33 // mode 0-4 (all, folder, single, random, disabled) (power failure restores to 4)
#define BY8X0116P_CMD_SWITCH_FOLDER     0x34 // folder 0-1 (previous, next)
#define BY8X0116P_CMD_SWITCH_DEVICE     0x35 // device 0-1 (U-Disk, TF card)
#define BY8X0116P_CMD_PLAY_INDEX        0x41 // index 1-65535 (combination play 10 track queue)
#define BY8X0116P_CMD_PLAY_FOLDER       0x42 // folder 0-99, index 1-255
#define BY8X0116P_CMD_SPOT_PB_INDEX     0x43 // index 1-65535 (Interrupts current play to play another, then resume play) (TF device not supported)
#define BY8X0116P_CMD_SPOT_PB_FOLDER    0x44 // folder 0-99, index 1-255 (Interrupts current play to play another, then resume play) (TF device not supported)

#define BY8X0116P_QRY_PB_STATUS         0x10 // 0-4 (stopped / playback / paused/suspended / fast-fwd / rewind)
#define BY8X0116P_QRY_VOLUME            0x11 // 0-30
#define BY8X0116P_QRY_EQ_PROFILE        0x12 // 0-5 (none, Pop, Rock, Jazz, Classic, Bass)
#define BY8X0116P_QRY_LOOP_MODE         0x13 // 0-4 (all, folder, single, random, disabled)
#define BY8X0116P_QRY_FIRMWARE_VER      0x14 // 0-65535
#define BY8X0116P_QRY_NUM_TRACKS_TFC    0x15 // 0-65535 (total number of track files on TF card)
#define BY8X0116P_QRY_NUM_TRACKS_USB    0x16 // 0-65535 (total number of track files on USB flash drive)
#define BY8X0116P_QRY_PLAYBACK_DEVICE   0x18 // 0-1 (U-Disk, SD)
#define BY8X0116P_QRY_CURR_TRACK_TFC    0x19 // 0-65535 
#define BY8X0116P_QRY_CURR_TRACK_USB    0x1A // 0-65535
#define BY8X0116P_QRY_ELAPSED_PB_TIME   0x1C // 0-65535 (in seconds)
#define BY8X0116P_QRY_TOTAL_PB_TIME     0x1D // 0-65535 (in seconds)
#define BY8X0116P_QRY_CURR_FILENAME     0x1E // YYYYYYYYZZZ
#define BY8X0116P_QRY_NUM_TRACKS_FOLDER 0x1F // 0-65535

#define BY8X0116P_READ_DELAY            130 // Delay for receive operations, in ms
#define BY8X0116P_CLEAN_DELAY           2500 // Delay between cleanup routine, in ms
#define BY8X0116P_RESET_TIMEOUT         2800 // Timeout before another reset command made, in ms
#define BY8X0116P_GEN_CMD_TIMEOUT       5000 // Timeout for commands to be processed
#define BY8X0116P_BUSY_DEBOUNCE_TIME    20 // Time to spend debouncing busy input line

BY8X0116P::BY8X0116P(Stream& stream, byte busyPin, byte busyActiveOn) {
    _stream = &stream;
    _busyPin = busyPin;
    _busyActiveOn = busyActiveOn;
    _isBlockingRspLn = 0;
    _isCleaningRspLn = false;
    _isStandingBy = false;
    _isResetting = false;
    _isCardInserted = true;
    _lastReqTime = 0;
    _lastClnTime = 0;
}

void BY8X0116P::init() {
    if (_busyPin) {
        pinMode(_busyPin, INPUT);
        
        // Enables pull-up resistor if busy signal is active-low, otherwise disables
        digitalWrite(_busyPin, _busyActiveOn == HIGH ? LOW : HIGH);
    }
}

void BY8X0116P::play() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::play");
#endif

    sendCommand(BY8X0116P_CMD_PLAY);
}

void BY8X0116P::pause() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::pause");
#endif

    sendCommand(BY8X0116P_CMD_PAUSE);
}

void BY8X0116P::stop(bool blocking) {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::stop blocking: ");
    Serial.println(blocking);
#endif

    if (!blocking) {
        sendCommand(BY8X0116P_CMD_STOP);
    }
    else {
        ++_isBlockingRspLn;

        sendCommand(BY8X0116P_CMD_STOP);
        waitRequest();

        if (_busyPin)
            _waitBusySignal(BY8X0116P_GEN_CMD_TIMEOUT);
        else
            _waitPlaybackFinished(BY8X0116P_GEN_CMD_TIMEOUT);

        --_isBlockingRspLn;
    }
}

void BY8X0116P::playFileIndex(word fileIndex) {
    fileIndex = (word)constrain(fileIndex, 0x0001, 0xFFFF);

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::playFileIndex fileIndex: ");
    Serial.println(fileIndex);
#endif

    sendCommand(BY8X0116P_CMD_PLAY_INDEX, fileIndex);
}

void BY8X0116P::playFolderFileIndex(byte folderIndex, byte fileIndex) {
    folderIndex = (byte)constrain(folderIndex, 0, 99);
    fileIndex = (byte)constrain(fileIndex, 0x01, 0xFF);

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::playFolderFileIndex folderIndex: ");
    Serial.print(folderIndex);
    Serial.print(", fileIndex: ");
    Serial.println(fileIndex);
#endif

    sendCommand(BY8X0116P_CMD_PLAY_FOLDER, folderIndex, fileIndex);
}

void BY8X0116P::fastForward() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::fastForward");
#endif

    sendCommand(BY8X0116P_CMD_FAST_FORWARD);
}

void BY8X0116P::fastRewind() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::fastRewind");
#endif

    sendCommand(BY8X0116P_CMD_FAST_REWIND);
}

void BY8X0116P::nextTrack() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::nextTrack");
#endif

    sendCommand(BY8X0116P_CMD_NEXT_TRACK);
}

void BY8X0116P::previousTrack() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::previousTrack");
#endif

    sendCommand(BY8X0116P_CMD_PREV_TRACK);
}

void BY8X0116P::nextFolder() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::nextFolder");
#endif

    sendCommand(BY8X0116P_CMD_SWITCH_FOLDER, (byte)0x01);
}

void BY8X0116P::previousFolder() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::previousFolder");
#endif

    sendCommand(BY8X0116P_CMD_SWITCH_FOLDER, (byte)0x00);
}

void BY8X0116P::increaseVolume() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::increaseVolume");
#endif

    sendCommand(BY8X0116P_CMD_INC_VOLUME);
}

void BY8X0116P::decreaseVolume() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::decreaseVolume");
#endif

    sendCommand(BY8X0116P_CMD_DEC_VOLUME);
}

void BY8X0116P::setVolume(int volume) {
    volume = constrain(volume, 0, 30);

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::setVolume volume: ");
    Serial.println(volume);
#endif

    sendCommand(BY8X0116P_CMD_SET_VOLUME, (byte)volume);
}

int BY8X0116P::getVolume() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getVolume");
#endif

    return receiveCommand(BY8X0116P_QRY_VOLUME); // Returns "HHHH\r\nOK"
}

BY8X0116P_PlaybackStatus BY8X0116P::getPlaybackStatus() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getPlaybackStatus");
#endif

    return (BY8X0116P_PlaybackStatus)receiveCommand(BY8X0116P_QRY_PB_STATUS); // Returns "HHHH\r\nOK"
}

void BY8X0116P::setLoopPlaybackMode(BY8X0116P_LoopPlaybackMode pbLoopMode) {
    pbLoopMode = (BY8X0116P_LoopPlaybackMode)constrain((int)pbLoopMode, 0, (int)BY8X0116P_LoopPlaybackMode_Count - 1);

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::setLoopPlaybackMode pbLoopMode: ");
    Serial.println(pbLoopMode);
#endif

    sendCommand(BY8X0116P_CMD_SET_LOOP_MODE, (byte)pbLoopMode);
}

BY8X0116P_LoopPlaybackMode BY8X0116P::getLoopPlaybackMode() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getLoopPlaybackMode");
#endif

    return (BY8X0116P_LoopPlaybackMode)receiveCommand(BY8X0116P_QRY_LOOP_MODE); // Returns "HHHH\r\nOK"
}

void BY8X0116P::setEqualizerProfile(BY8X0116P_EqualizerProfile eqProfile) {
    eqProfile = (BY8X0116P_EqualizerProfile)constrain((int)eqProfile, 0, (int)BY8X0116P_EqualizerProfile_Count - 1);

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::setEqualizerProfile eqProfile: ");
    Serial.println(eqProfile);
#endif

    sendCommand(BY8X0116P_CMD_SET_EQ_PROFILE, (byte)eqProfile);
}

BY8X0116P_EqualizerProfile BY8X0116P::getEqualizerProfile() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getEqualizerProfile");
#endif

    return (BY8X0116P_EqualizerProfile)receiveCommand(BY8X0116P_QRY_EQ_PROFILE); // Returns "HHHH\r\nOK"
}

word BY8X0116P::getTotalNumberOfTracks() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getTotalNumberOfTracks");
#endif

    switch ((BY8X0116P_PlaybackDevice)receiveCommand(BY8X0116P_QRY_PLAYBACK_DEVICE)) {
        case BY8X0116P_PlaybackDevice_USB:
            return receiveCommand(BY8X0116P_QRY_NUM_TRACKS_USB); // Returns "HHHH\r\nOK"
        default:
            return receiveCommand(BY8X0116P_QRY_NUM_TRACKS_TFC); // Returns "HHHH\r\nOK"
    }
}

word BY8X0116P::getTotalNumberOfTracks(BY8X0116P_PlaybackDevice device) {
    device = (BY8X0116P_PlaybackDevice)constrain((int)device, 0, (int)BY8X0116P_PlaybackDevice_Count - 1);

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::getTotalNumberOfTracks device: ");
    Serial.println(device);
#endif

    switch (device) {
        case BY8X0116P_PlaybackDevice_USB:
            return receiveCommand(BY8X0116P_QRY_NUM_TRACKS_USB); // Returns "HHHH\r\nOK"
        default:
            return receiveCommand(BY8X0116P_QRY_NUM_TRACKS_TFC); // Returns "HHHH\r\nOK"
    }
}

word BY8X0116P::getNumberOfTracksInCurrentFolder() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getNumberOfTracksInCurrentFolder");
#endif

    return receiveCommand(BY8X0116P_QRY_NUM_TRACKS_FOLDER); // Returns "OKHHHH\r\n"
}

word BY8X0116P::getCurrentTrackFileIndex() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getCurrentTrackFileIndex");
#endif

    switch ((BY8X0116P_PlaybackDevice)receiveCommand(BY8X0116P_QRY_PLAYBACK_DEVICE)) {
        case BY8X0116P_PlaybackDevice_USB:
            return receiveCommand(BY8X0116P_QRY_CURR_TRACK_USB); // Returns "HHHH\r\nOK"
        default:
            return receiveCommand(BY8X0116P_QRY_CURR_TRACK_TFC); // Returns "HHHH\r\nOK"
    }
}

word BY8X0116P::getCurrentTrackFileIndex(BY8X0116P_PlaybackDevice device) {
    device = (BY8X0116P_PlaybackDevice)constrain((int)device, 0, (int)BY8X0116P_PlaybackDevice_Count - 1);

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::getCurrentTrackFileIndex device: ");
    Serial.println(device);
#endif

    switch (device) {
        case BY8X0116P_PlaybackDevice_USB:
            return receiveCommand(BY8X0116P_QRY_CURR_TRACK_USB); // Returns "HHHH\r\nOK"
        default:
            return receiveCommand(BY8X0116P_QRY_CURR_TRACK_TFC); // Returns "HHHH\r\nOK"
    }
}

word BY8X0116P::getCurrentTrackElapsedTime() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getCurrentTrackElapsedTime");
#endif

    return receiveCommand(BY8X0116P_QRY_ELAPSED_PB_TIME); // Returns "OKHHHH\r\n"
}

word BY8X0116P::getCurrentTrackTotalTime() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getCurrentTrackTotalTime");
#endif

    return receiveCommand(BY8X0116P_QRY_TOTAL_PB_TIME); // Returns "OKHHHH\r\n"
}

void BY8X0116P::setPlaybackDevice(BY8X0116P_PlaybackDevice device) {
    device = (BY8X0116P_PlaybackDevice)constrain((int)device, 0, (int)BY8X0116P_PlaybackDevice_Count - 1);

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::setPlaybackDevice device: ");
    Serial.println(device);
#endif

    sendCommand(BY8X0116P_CMD_SWITCH_DEVICE, (byte)device);
}

BY8X0116P_PlaybackDevice BY8X0116P::getPlaybackDevice() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getPlaybackDevice");
#endif

    return (BY8X0116P_PlaybackDevice)receiveCommand(BY8X0116P_QRY_PLAYBACK_DEVICE); // Returns "HHHH\r\nOK"
}

void BY8X0116P::spotPlayFileIndex(word fileIndex) {
    fileIndex = (word)constrain(fileIndex, 0x0001, 0xFFFF);

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::spotPlayFileIndex fileIndex: ");
    Serial.println(fileIndex);
#endif

    sendCommand(BY8X0116P_CMD_SPOT_PB_INDEX, fileIndex);
}

void BY8X0116P::spotPlayFolderFileIndex(byte folderIndex, byte fileIndex) {
    folderIndex = (byte)constrain(folderIndex, 0, 99);
    fileIndex = (byte)constrain(fileIndex, 0x01, 0xFF);

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::spotPlayFolderFileIndex folderIndex: ");
    Serial.print(folderIndex);
    Serial.print(", fileIndex: ");
    Serial.println(fileIndex);
#endif

    sendCommand(BY8X0116P_CMD_SPOT_PB_FOLDER, folderIndex, fileIndex);
}

void BY8X0116P::getCurrentTrackFilename(char *buffer, int maxLength) {
    if (!buffer || maxLength < 12) return;

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getCurrentTrackFilename");
#endif

    char innerBuffer[11];
    int bytesRead = receiveCommand(BY8X0116P_QRY_CURR_FILENAME, 15, innerBuffer, 11); // Returns "OKXXXXXXXXYYY\r\n" or "XXXXXXXXYYY\r\nOK"

    for (int i = 0; i < bytesRead && maxLength-- > 0; ++i) {
        *buffer++ = innerBuffer[i];

        if (i == 7 && maxLength-- > 0)
            *buffer++ = '.';
    }

    if (maxLength-- > 0)
        *buffer = '\0';
}

void BY8X0116P::getFirmwareVersion(char *buffer, int maxLength) {
    if (!buffer || maxLength < 6) return;

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getFirmwareVersion");
#endif

    char innerBuffer[4];
    int bytesRead = receiveCommand(BY8X0116P_QRY_FIRMWARE_VER, 8, innerBuffer, 4); // Returns "VVVV\r\nOK" or "VV\r\nOK"

    if (maxLength-- > 0)
        *buffer++ = 'f';
    if (maxLength-- > 0)
        *buffer++ = 'w';

    for (int i = 0; i < bytesRead && maxLength-- > 0; ++i)
        *buffer++ = innerBuffer[i];

    if (maxLength-- > 0)
        *buffer = '\0';
}

bool BY8X0116P::isBusy() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::isBusy");
#endif

    return _isBusy();
}

bool debouncedDigitalRead(byte pin, bool activeOn, int sampleTime, int sampleRate) {
    unsigned long endTime = millis() + (unsigned long)sampleTime;
    int activeCount = 0;
    int inactiveCount = 0;

    sampleRate = 1000 / sampleRate;
    
    while (millis() < endTime) {
        if (digitalRead(pin) == activeOn)
            ++activeCount;
        else
            ++inactiveCount;

        delayMicroseconds(sampleRate);
    }

    return activeCount >= inactiveCount;
}

bool BY8X0116P::_isBusy() {
    if (_busyPin) {
        waitRequest();

        return debouncedDigitalRead(_busyPin, _busyActiveOn, BY8X0116P_BUSY_DEBOUNCE_TIME
#if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
            , 4
#endif
            );
    }

    return false;
}

bool BY8X0116P::waitBusySignal(int timeout) {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::waitBusySignal timeout: ");
    Serial.println(timeout);
#endif

    return _waitBusySignal();
}

bool BY8X0116P::_waitBusySignal(int timeout) {
    if (!_isBusy()) return false;

    unsigned long endTime = millis() + (unsigned long)timeout;

    while (_isBusy() && (timeout <= 0 || millis() < endTime)) {
#ifdef BY8X0116P_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif
    }

    return _isBusy();
}

bool BY8X0116P::isPlaybackActive() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::isPlaybackActive");
#endif

    return _isPlaybackActive();
}

bool BY8X0116P::_isPlaybackActive() {
    return (BY8X0116P_PlaybackStatus)receiveCommand(BY8X0116P_QRY_PB_STATUS) != BY8X0116P_PlaybackStatus_Stopped;
}

bool BY8X0116P::waitPlaybackFinished(int timeout) {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::waitPlaybackFinished timeout: ");
    Serial.println(timeout);
#endif

    return _waitPlaybackFinished(timeout);
}

bool BY8X0116P::_waitPlaybackFinished(int timeout) {
    if (!_isPlaybackActive()) return false;

    unsigned long endTime = millis() + (unsigned long)timeout;

    while (_isPlaybackActive() && (timeout <= 0 || millis() < endTime)) {
#ifdef BY8X0116P_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif
    }

    return _isPlaybackActive();
}

void BY8X0116P::toggleStandBy(bool blocking) {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::toggleStandBy isStandingBy: ");
    Serial.println(_isStandingBy);
#endif

    if (!blocking) {
        // Returns "OKIDLE" going into standby
        // Returns "MP3OKOK"/"MP3OKNO FILE" (TFC inserted/not-inserted) coming out of standby
        sendCommand(BY8X0116P_CMD_TGL_STANDBY);
    }
    else {
        ++_isBlockingRspLn;

        sendCommand(BY8X0116P_CMD_TGL_STANDBY);
        waitClean(BY8X0116P_GEN_CMD_TIMEOUT);

        --_isBlockingRspLn;
    }
}

bool BY8X0116P::isStandingBy() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::isStandingBy");
#endif
    
    if (millis() < _lastReqTime + BY8X0116P_READ_DELAY + BY8X0116P_READ_DELAY || _stream->available() > 4)
        cleanResponse();

    return _isStandingBy;
}

void BY8X0116P::reset(bool blocking) {
    if (_isResetting) return;

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("BY8X0116P::reset blocking: ");
    Serial.println(blocking);
#endif

    if (!blocking) {
        // Returns "OKEEPROM ONLINEkey_SWITCH:07\r\nOK" or <nothing> going into reset
        // Returns "MP3OK" or "OKMP3"/"MP3NO FILE" (TFC inserted/not-inserted) coming out of reset
        sendCommand(BY8X0116P_CMD_RESET);
    }
    else {
        ++_isBlockingRspLn;

        // Sometimes reset commands don't get a "MP3" response back and must call reset multiple times in order for device to come back online
        do {
            sendCommand(BY8X0116P_CMD_RESET);

            unsigned long endTime = millis() + (unsigned long)BY8X0116P_RESET_TIMEOUT;

            while (_isResetting && millis() < endTime) {
#ifdef BY8X0116P_USE_SCHEDULER
                Scheduler.yield();
#else
                delay(1);
#endif
                cleanResponse();
            }
        } while (_isResetting);

        --_isBlockingRspLn;
    }
}

bool BY8X0116P::isResetting() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::isResetting");
#endif

    if (_stream->available() > 2)
        cleanResponse();

    return _isResetting;
}

bool BY8X0116P::isCardInserted() {
#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.println("BY8X0116P::isCardInserted");
#endif

    if (!_isCardInserted) {
        ++_isBlockingRspLn;

        sendCommand(BY8X0116P_CMD_TGL_STANDBY);
        waitClean(BY8X0116P_GEN_CMD_TIMEOUT);

        sendCommand(BY8X0116P_CMD_TGL_STANDBY);
        waitClean(BY8X0116P_GEN_CMD_TIMEOUT);

        --_isBlockingRspLn;

        return _isCardInserted;
    }
    else {
        if (_stream->available() > 4)
            cleanResponse();

        return _isCardInserted;
    }
}

void BY8X0116P::cleanupRoutine() {
    if (!_isBlockingRspLn && !_isCleaningRspLn && _stream->available() && millis() >= _lastClnTime + BY8X0116P_CLEAN_DELAY) {
#ifdef BY8X0116P_DEBUG_OUTPUT
        Serial.println("BY8X0116P::cleanupRoutine");
#endif

        cleanResponse();
    }
}

void BY8X0116P::sendCommand(byte cmdID) {
    byte cmdBuffer[] = { 0x7E, 0x03, cmdID, 0x00, 0xEF };
    writeRequest(cmdBuffer);
}

void BY8X0116P::sendCommand(byte cmdID, byte param) {
    byte cmdBuffer[] = { 0x7E, 0x04, cmdID, param, 0x00, 0xEF };
    writeRequest(cmdBuffer);
}

void BY8X0116P::sendCommand(byte cmdID, word param) {
    byte cmdBuffer[] = { 0x7E, 0x05, cmdID, highByte(param), lowByte(param), 0x00, 0xEF };
    writeRequest(cmdBuffer);
}

void BY8X0116P::sendCommand(byte cmdID, byte param1, byte param2) {
    byte cmdBuffer[] = { 0x7E, 0x05, cmdID, param1, param2, 0x00, 0xEF };
    writeRequest(cmdBuffer);
}

void BY8X0116P::writeRequest(byte *cmdBuffer, bool cleanRspLn) {
    byte length = cmdBuffer[1];
    byte checkCode = length;

    for (int i = 2; i < length; ++i)
        checkCode = checkCode ^ cmdBuffer[i];
    cmdBuffer[length] = checkCode;

    ++_isBlockingRspLn;

    if (cleanRspLn || _stream->available() > 8)
        cleanResponse();
    else // Allow time for last command to be properly processed
        waitRequest();

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("  BY8X0116P::writeRequest Cmd: 0x");
    Serial.print(cmdBuffer[2], HEX);
    for (int i = 3; i < length; ++i) {
        Serial.print("-0x");
        Serial.print(cmdBuffer[i], HEX);
    }
    Serial.print(", Chk: 0x");
    Serial.println(cmdBuffer[length], HEX);
#endif

    if (cmdBuffer[2] == BY8X0116P_CMD_RESET)
        _isResetting = true;

    _stream->write(cmdBuffer, length + 2);
    _lastReqTime = millis();

    --_isBlockingRspLn;
}

word BY8X0116P::receiveCommand(byte cmdID) {
    ++_isBlockingRspLn;

    byte cmdBuffer[] = { 0x7E, 0x03, cmdID, 0x00, 0xEF };
    writeRequest(cmdBuffer, true);

    char respBuffer[5];
    readResponse(respBuffer, 8, 5);

    word retVal = strtol(respBuffer, NULL, 16);

    --_isBlockingRspLn;

    return retVal;
}

int BY8X0116P::receiveCommand(byte cmdID, int expectedLength, char *respBuffer, int maxLength) {
    ++_isBlockingRspLn;

    byte cmdBuffer[] = { 0x7E, 0x03, cmdID, 0x00, 0xEF };
    writeRequest(cmdBuffer, true);

    int retVal = readResponse(respBuffer, expectedLength, maxLength);

    --_isBlockingRspLn;

    return retVal;
}

int BY8X0116P::readResponse(char *respBuffer, int expectedLength, int maxLength) {
    int bytesRead = 0;

    if (waitResponse()) {
        int lastChar = -1;

        while (_stream->available() && expectedLength > 0) {
            char currChar = _stream->read(); --expectedLength;

            if (lastChar != -1) {
                if ((lastChar == 'O' && currChar == 'K') ||
                    (lastChar == '\r' && currChar == '\n')) {
                    lastChar = -1;
                    continue;
                }

                if (maxLength-- > 0)
                    respBuffer[bytesRead++] = lastChar;
            }

            lastChar = currChar;
        }

        if (lastChar != -1 && maxLength-- > 0)
            respBuffer[bytesRead++] = lastChar;
    }

    if (maxLength-- > 0)
        respBuffer[bytesRead] == '\0';

#ifdef BY8X0116P_DEBUG_OUTPUT
    Serial.print("  BY8X0116P::readResponse Rsp: ");
    for (int i = 0; i < bytesRead; ++i) {
        if (respBuffer[i] >= ' ')
            Serial.print(respBuffer[i]);
        else {
            Serial.print("\\x");
            Serial.print(respBuffer[i], HEX);
        }
    }
    Serial.println("");
#endif
    
    return bytesRead;
}

void delayTimeout(int timeout) {
    unsigned long endTime = millis() + timeout;

    while (millis() < endTime) {
#ifdef BY8X0116P_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif
    }
}

void BY8X0116P::waitRequest() {
    unsigned long endTime = _lastReqTime + BY8X0116P_READ_DELAY;

    while (millis() < endTime) {
#ifdef BY8X0116P_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif
    }
}

bool BY8X0116P::waitResponse() {
    int available = _stream->available();
    unsigned long endTime = max(millis(), _lastReqTime + BY8X0116P_READ_DELAY) + BY8X0116P_READ_DELAY;

    while (millis() < endTime) {
#ifdef BY8X0116P_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif
        if (available != _stream->available()) {
            available = _stream->available();
            endTime = max(millis(), _lastReqTime + BY8X0116P_READ_DELAY) + BY8X0116P_READ_DELAY;
        }
    }

    return available;
}

bool BY8X0116P::cleanResponse() {
    _isCleaningRspLn = true;

    if (waitResponse()) {
        char respBuffer[33];
        int respLength = 0;

        _lastClnTime = millis();
        
        while (_stream->available() && respLength < 32)
            respBuffer[respLength++] = _stream->read();

        respBuffer[respLength] = '\0';

        if (respLength) {
#ifdef BY8X0116P_DEBUG_OUTPUT
            Serial.print("  BY8X0116P::cleanResponse Line: ");
            for (int i = 0; i < respLength; ++i) {
                if (respBuffer[i] >= ' ')
                    Serial.print(respBuffer[i]);
                else {
                    Serial.print("\\x");
                    Serial.print((int)respBuffer[i], HEX);
                }
            }
            Serial.println("");
#endif

            char *respScan = &respBuffer[0];

            while (*respScan) {
                // Standby responses
                if (strncmp(respScan, "OKIDLE", 6) == 0) {
                    respScan += 6;
                    _isStandingBy = true;
                }
                else if (strncmp(respScan, "MP3OKNO FILE", 12) == 0) {
                    respScan += 12;
                    _isStandingBy = false;
                    _isCardInserted = false;
                }
                else if (strncmp(respScan, "MP3OKOK", 7) == 0) {
                    respScan += 7;
                    _isStandingBy = false;
                    _isCardInserted = true;
                }
                // Reset responses
                else if (strncmp(respScan, "MP3", 3) == 0) {
                    respScan += 3;
                    _isResetting = false;
                }
                // TFC responses
                else if (strncmp(respScan, "NO FILE", 7) == 0) {
                    respScan += 7;
                    _isCardInserted = false;
                }
                // N/A responses
                else if (strncmp(respScan, "EEPROM ONLINE", 13) == 0 || strncmp(respScan, "key_SWITCH:07", 13) == 0) {
                    respScan += 13;
                }
                else if (strncmp(respScan, "key_ABC SWITCH:07", 17) == 0) {
                    respScan += 17;
                }
                else if (strncmp(respScan, "STOP", 4) == 0) {
                    respScan += 4;
                }
                else if (strncmp(respScan, "OK", 2) == 0 || strncmp(respScan, "\r\n", 2) == 0) {
                    respScan += 2;
                }
                else {
                    ++respScan;
                }
            }

            _isCleaningRspLn = false;
            return true;
        }
    }

    _isCleaningRspLn = false;
    return false;
}

void BY8X0116P::waitClean(int timeout) {
    unsigned long endTime = millis() + timeout;

    while (!cleanResponse() && (timeout <= 0 || millis() < endTime));
}
