/*  Arduino Library for the BY8001-16P/BY8301-16P Audio Module.
    Copyright (C) 2016-2020 NachtRaveVL     <nachtravevl@gmail.com>
    BY8X01-16P Main
*/

#include "BY8X01-16P.h"

#define BY8X0116P_CMD_PLAY              (byte)0x01
#define BY8X0116P_CMD_PAUSE             (byte)0x02
#define BY8X0116P_CMD_NEXT_TRACK        (byte)0x03
#define BY8X0116P_CMD_PREV_TRACK        (byte)0x04
#define BY8X0116P_CMD_INC_VOLUME        (byte)0x05
#define BY8X0116P_CMD_DEC_VOLUME        (byte)0x06
#define BY8X0116P_CMD_TGL_STANDBY       (byte)0x07          // toggles between standby/normal, when in standby 10mA current draw
#define BY8X0116P_CMD_RESET             (byte)0x09
#define BY8X0116P_CMD_FAST_FORWARD      (byte)0x0A
#define BY8X0116P_CMD_FAST_REWIND       (byte)0x0B
#define BY8X0116P_CMD_STOP              (byte)0x0E
#define BY8X0116P_CMD_SET_VOLUME        (byte)0x31          // level 0-30 (power failure retains setting)
#define BY8X0116P_CMD_SET_EQ_PROFILE    (byte)0x32          // profile 0-5 (none, Pop, Rock, Jazz, Classic, Bass) (power failure retains setting)
#define BY8X0116P_CMD_SET_LOOP_MODE     (byte)0x33          // mode 0-4 (all, folder, single, random, disabled) (power failure restores to 4)
#define BY8X0116P_CMD_SWITCH_FOLDER     (byte)0x34          // folder 0-1 (previous, next)
#define BY8X0116P_CMD_SWITCH_DEVICE     (byte)0x35          // device 0-1 (U-Disk, TF card)
#define BY8X0116P_CMD_PLAY_INDEX        (byte)0x41          // index 1-65535 (combination play 10 track queue)
#define BY8X0116P_CMD_PLAY_FOLDER       (byte)0x42          // folder 0-99, index 1-255
#define BY8X0116P_CMD_SPOT_PB_INDEX     (byte)0x43          // index 1-65535 (Interrupts current play to play another, then resume play) (TF device not supported)
#define BY8X0116P_CMD_SPOT_PB_FOLDER    (byte)0x44          // folder 0-99, index 1-255 (Interrupts current play to play another, then resume play) (TF device not supported)

#define BY8X0116P_QRY_PB_STATUS         (byte)0x10          // 0-4 (stopped / playback / paused/suspended / fast-fwd / rewind)
#define BY8X0116P_QRY_VOLUME            (byte)0x11          // 0-30
#define BY8X0116P_QRY_EQ_PROFILE        (byte)0x12          // 0-5 (none, Pop, Rock, Jazz, Classic, Bass)
#define BY8X0116P_QRY_LOOP_MODE         (byte)0x13          // 0-4 (all, folder, single, random, disabled)
#define BY8X0116P_QRY_FIRMWARE_VER      (byte)0x14          // 0-65535
#define BY8X0116P_QRY_NUM_TRACKS_TFC    (byte)0x15          // 0-65535 (total number of track files on TF card)
#define BY8X0116P_QRY_NUM_TRACKS_USB    (byte)0x16          // 0-65535 (total number of track files on USB flash drive)
#define BY8X0116P_QRY_PLAYBACK_DEVICE   (byte)0x18          // 0-1 (U-Disk, SD)
#define BY8X0116P_QRY_CURR_TRACK_TFC    (byte)0x19          // 0-65535 
#define BY8X0116P_QRY_CURR_TRACK_USB    (byte)0x1A          // 0-65535
#define BY8X0116P_QRY_ELAPSED_PB_TIME   (byte)0x1C          // 0-65535 (in seconds)
#define BY8X0116P_QRY_TOTAL_PB_TIME     (byte)0x1D          // 0-65535 (in seconds)
#define BY8X0116P_QRY_CURR_FILENAME     (byte)0x1E          // YYYYYYYYZZZ
#define BY8X0116P_QRY_NUM_TRACKS_FOLDER (byte)0x1F          // 0-65535

#define BY8X0116P_SERIAL_BAUD           9600                // Serial baud rate, in bps
#define BY8X0116P_SERIAL_MODE           SERIAL_8N1          // Serial mode
#define BY8X0116P_READ_DELAY            130                 // Delay for receive operations, in ms
#define BY8X0116P_CLEAN_DELAY           2500                // Delay between cleanup routine, in ms
#define BY8X0116P_RESET_TIMEOUT         2800                // Timeout before another reset command made, in ms
#define BY8X0116P_GEN_CMD_TIMEOUT       5000                // Timeout for commands to be processed, in ms
#define BY8X0116P_BUSY_DEBOUNCE_TIME    20                  // Time to spend debouncing busy input line, in ms

#ifndef BY8X0116P_USE_SOFTWARE_SERIAL

#ifdef BY8X0116P_HAS_SERIAL1

BY8X0116P::BY8X0116P(byte busyPin, byte busyActiveOn, HardwareSerial& serial)
    : _busyPin(busyPin), _busyActiveOn(busyActiveOn),
      _serial(&serial),
      _isBlockingRspLn(0), _isCleaningRspLn(false),
      _isStandingBy(false), _isResetting(false), _isCardInserted(true),
      _lastReqTime(0), _lastClnTime(0)
 { }

 #endif // /ifdef BY8X0116P_HAS_SERIAL1

BY8X0116P::BY8X0116P(HardwareSerial& serial, byte busyPin, byte busyActiveOn)
    : _busyPin(busyPin), _busyActiveOn(busyActiveOn),
      _serial(&serial),
      _isBlockingRspLn(0), _isCleaningRspLn(false),
      _isStandingBy(false), _isResetting(false), _isCardInserted(true),
      _lastReqTime(0), _lastClnTime(0)
{ }

#else

BY8X0116P::BY8X0116P(SoftwareSerial& serial, byte busyPin, byte busyActiveOn)
    : _busyPin(busyPin), _busyActiveOn(busyActiveOn),
      _serial(&serial),
      _isBlockingRspLn(0), _isCleaningRspLn(false),
      _isStandingBy(false), _isResetting(false), _isCardInserted(true),
      _lastReqTime(0), _lastClnTime(0)
{ }

#endif // /ifndef BY8X0116P_USE_SOFTWARE_SERIAL

void BY8X0116P::init() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("BY8X0116P::init busyPin: ");
    if (_busyPin != DISABLED) {
        Serial.print(_busyPin);
        Serial.print(_busyActiveOn ? " <active-high>" : " <active-low>");
    }
    else
        Serial.print("<disabled>");
    Serial.print(", serialUART#: ");
    Serial.print(getSerialInterfaceNumber());
    Serial.print(", serialBaud: ");
    Serial.print(getSerialBaud()); Serial.print("bps");
    Serial.println("");
#endif

    if (_busyPin != DISABLED) {
        pinMode(_busyPin, INPUT);

        // Enables pull-up resistor if busy signal is active-low, otherwise disables
        digitalWrite(_busyPin, _busyActiveOn == HIGH ? LOW : HIGH);
    }
}

byte BY8X0116P::getBusyPin() {
    return _busyPin;
}

byte BY8X0116P::getBusyActiveOn() {
    return _busyActiveOn;
}

uint32_t BY8X0116P::getSerialBaud() {
    return BY8X0116P_SERIAL_BAUD;
}

uint16_t BY8X0116P::getSerialMode() {
    return BY8X0116P_SERIAL_MODE;
}

void BY8X0116P::play() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::play");
#endif

    sendCommand(BY8X0116P_CMD_PLAY);
}

void BY8X0116P::pause() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::pause");
#endif

    sendCommand(BY8X0116P_CMD_PAUSE);
}

void BY8X0116P::stop(bool blocking) {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
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

        if (_busyPin != DISABLED)
            _waitBusy(BY8X0116P_GEN_CMD_TIMEOUT);
        else
            _waitPlaybackFinished(BY8X0116P_GEN_CMD_TIMEOUT);

        --_isBlockingRspLn;
    }
}

void BY8X0116P::playFileIndex(uint16_t fileIndex) {
    fileIndex = (uint16_t)min(max(fileIndex, 0x0001), 0xFFFF);

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("BY8X0116P::playFileIndex fileIndex: ");
    Serial.println(fileIndex);
#endif

    sendCommand(BY8X0116P_CMD_PLAY_INDEX, fileIndex);
}

void BY8X0116P::playFolderFileIndex(byte folderIndex, byte fileIndex) {
    folderIndex = (byte)min(max(folderIndex, 0), 99);
    fileIndex = (byte)min(max(fileIndex, 0x01), 0xFF);

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("BY8X0116P::playFolderFileIndex folderIndex: ");
    Serial.print(folderIndex);
    Serial.print(", fileIndex: ");
    Serial.println(fileIndex);
#endif

    sendCommand(BY8X0116P_CMD_PLAY_FOLDER, folderIndex, fileIndex);
}

void BY8X0116P::fastForward() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::fastForward");
#endif

    sendCommand(BY8X0116P_CMD_FAST_FORWARD);
}

void BY8X0116P::fastRewind() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::fastRewind");
#endif

    sendCommand(BY8X0116P_CMD_FAST_REWIND);
}

void BY8X0116P::nextTrack() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::nextTrack");
#endif

    sendCommand(BY8X0116P_CMD_NEXT_TRACK);
}

void BY8X0116P::previousTrack() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::previousTrack");
#endif

    sendCommand(BY8X0116P_CMD_PREV_TRACK);
}

void BY8X0116P::nextFolder() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::nextFolder");
#endif

    sendCommand(BY8X0116P_CMD_SWITCH_FOLDER, (byte)0x01);
}

void BY8X0116P::previousFolder() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::previousFolder");
#endif

    sendCommand(BY8X0116P_CMD_SWITCH_FOLDER, (byte)0x00);
}

void BY8X0116P::increaseVolume() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::increaseVolume");
#endif

    sendCommand(BY8X0116P_CMD_INC_VOLUME);
}

void BY8X0116P::decreaseVolume() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::decreaseVolume");
#endif

    sendCommand(BY8X0116P_CMD_DEC_VOLUME);
}

void BY8X0116P::setVolume(int volume) {
    volume = min(max(volume, 0), 30);

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("BY8X0116P::setVolume volume: ");
    Serial.println(volume);
#endif

    sendCommand(BY8X0116P_CMD_SET_VOLUME, (byte)volume);
}

int BY8X0116P::getVolume() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getVolume");
#endif

    return receiveCommand(BY8X0116P_QRY_VOLUME); // Returns "HHHH\r\nOK"
}

BY8X0116P_PlaybackStatus BY8X0116P::getPlaybackStatus() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getPlaybackStatus");
#endif

    return (BY8X0116P_PlaybackStatus)receiveCommand(BY8X0116P_QRY_PB_STATUS); // Returns "HHHH\r\nOK"
}

void BY8X0116P::setLoopPlaybackMode(BY8X0116P_LoopPlaybackMode pbLoopMode) {
    pbLoopMode = (BY8X0116P_LoopPlaybackMode)min(max((int)pbLoopMode, 0), (int)BY8X0116P_LoopPlaybackMode_Count - 1);

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("BY8X0116P::setLoopPlaybackMode pbLoopMode: ");
    Serial.println(pbLoopMode);
#endif

    sendCommand(BY8X0116P_CMD_SET_LOOP_MODE, (byte)pbLoopMode);
}

BY8X0116P_LoopPlaybackMode BY8X0116P::getLoopPlaybackMode() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getLoopPlaybackMode");
#endif

    return (BY8X0116P_LoopPlaybackMode)receiveCommand(BY8X0116P_QRY_LOOP_MODE); // Returns "HHHH\r\nOK"
}

void BY8X0116P::setEqualizerProfile(BY8X0116P_EqualizerProfile eqProfile) {
    eqProfile = (BY8X0116P_EqualizerProfile)min(max((int)eqProfile, 0), (int)BY8X0116P_EqualizerProfile_Count - 1);

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("BY8X0116P::setEqualizerProfile eqProfile: ");
    Serial.println(eqProfile);
#endif

    sendCommand(BY8X0116P_CMD_SET_EQ_PROFILE, (byte)eqProfile);
}

BY8X0116P_EqualizerProfile BY8X0116P::getEqualizerProfile() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getEqualizerProfile");
#endif

    return (BY8X0116P_EqualizerProfile)receiveCommand(BY8X0116P_QRY_EQ_PROFILE); // Returns "HHHH\r\nOK"
}

uint16_t BY8X0116P::getTotalNumberOfTracks() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getTotalNumberOfTracks");
#endif

    switch ((BY8X0116P_PlaybackDevice)receiveCommand(BY8X0116P_QRY_PLAYBACK_DEVICE)) {
        case BY8X0116P_PlaybackDevice_USB:
            return receiveCommand(BY8X0116P_QRY_NUM_TRACKS_USB); // Returns "HHHH\r\nOK"
        default:
            return receiveCommand(BY8X0116P_QRY_NUM_TRACKS_TFC); // Returns "HHHH\r\nOK"
    }
}

uint16_t BY8X0116P::getTotalNumberOfTracks(BY8X0116P_PlaybackDevice device) {
    device = (BY8X0116P_PlaybackDevice)min(max((int)device, 0), (int)BY8X0116P_PlaybackDevice_Count - 1);

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
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

uint16_t BY8X0116P::getNumberOfTracksInCurrentFolder() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getNumberOfTracksInCurrentFolder");
#endif

    return receiveCommand(BY8X0116P_QRY_NUM_TRACKS_FOLDER); // Returns "OKHHHH\r\n"
}

uint16_t BY8X0116P::getCurrentTrackFileIndex() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getCurrentTrackFileIndex");
#endif

    switch ((BY8X0116P_PlaybackDevice)receiveCommand(BY8X0116P_QRY_PLAYBACK_DEVICE)) {
        case BY8X0116P_PlaybackDevice_USB:
            return receiveCommand(BY8X0116P_QRY_CURR_TRACK_USB); // Returns "HHHH\r\nOK"
        default:
            return receiveCommand(BY8X0116P_QRY_CURR_TRACK_TFC); // Returns "HHHH\r\nOK"
    }
}

uint16_t BY8X0116P::getCurrentTrackFileIndex(BY8X0116P_PlaybackDevice device) {
    device = (BY8X0116P_PlaybackDevice)min(max((int)device, 0), (int)BY8X0116P_PlaybackDevice_Count - 1);

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
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

uint16_t BY8X0116P::getCurrentTrackElapsedTime() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getCurrentTrackElapsedTime");
#endif

    return receiveCommand(BY8X0116P_QRY_ELAPSED_PB_TIME); // Returns "OKHHHH\r\n"
}

uint16_t BY8X0116P::getCurrentTrackTotalTime() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getCurrentTrackTotalTime");
#endif

    return receiveCommand(BY8X0116P_QRY_TOTAL_PB_TIME); // Returns "OKHHHH\r\n"
}

void BY8X0116P::setPlaybackDevice(BY8X0116P_PlaybackDevice device) {
    device = (BY8X0116P_PlaybackDevice)min(max((int)device, 0), (int)BY8X0116P_PlaybackDevice_Count - 1);

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("BY8X0116P::setPlaybackDevice device: ");
    Serial.println(device);
#endif

    sendCommand(BY8X0116P_CMD_SWITCH_DEVICE, (byte)device);
}

BY8X0116P_PlaybackDevice BY8X0116P::getPlaybackDevice() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getPlaybackDevice");
#endif

    return (BY8X0116P_PlaybackDevice)receiveCommand(BY8X0116P_QRY_PLAYBACK_DEVICE); // Returns "HHHH\r\nOK"
}

void BY8X0116P::spotPlayFileIndex(uint16_t fileIndex) {
    fileIndex = (uint16_t)min(max(fileIndex, 0x0001), 0xFFFF);

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("BY8X0116P::spotPlayFileIndex fileIndex: ");
    Serial.println(fileIndex);
#endif

    sendCommand(BY8X0116P_CMD_SPOT_PB_INDEX, fileIndex);
}

void BY8X0116P::spotPlayFolderFileIndex(byte folderIndex, byte fileIndex) {
    folderIndex = (byte)min(max(folderIndex, 0), 99);
    fileIndex = (byte)min(max(fileIndex, 0x01), 0xFF);

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("BY8X0116P::spotPlayFolderFileIndex folderIndex: ");
    Serial.print(folderIndex);
    Serial.print(", fileIndex: ");
    Serial.println(fileIndex);
#endif

    sendCommand(BY8X0116P_CMD_SPOT_PB_FOLDER, folderIndex, fileIndex);
}

void BY8X0116P::getCurrentTrackFilename(char *buffer, int maxLength) {
    if (!buffer || maxLength < 12) return;

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getCurrentTrackFilename");
#endif

    char innerBuffer[11];
    int bytesRead = receiveCommand(BY8X0116P_QRY_CURR_FILENAME, innerBuffer, 15, 11); // Returns "OKXXXXXXXXYYY\r\n" or "XXXXXXXXYYY\r\nOK"

    for (int i = 7; i >= 0 && innerBuffer[i] == ' '; --i)
        innerBuffer[i] = '\0';

    for (int i = 0; i < bytesRead && maxLength > 0; ++i) {
        if (innerBuffer[i] && maxLength-- > 0)
            *buffer++ = innerBuffer[i];

        if (i == 7 && maxLength-- > 0)
            *buffer++ = '.';
    }

    if (maxLength-- > 0)
        *buffer = '\0';
}

void BY8X0116P::getFirmwareVersion(char *buffer, int maxLength) {
    if (!buffer || maxLength < 6) return;

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::getFirmwareVersion");
#endif

    char innerBuffer[4];
    int bytesRead = receiveCommand(BY8X0116P_QRY_FIRMWARE_VER, innerBuffer, 8, 4); // Returns "VVVV\r\nOK" or "VV\r\nOK"

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
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::isBusy");
#endif

    return _isBusy();
}

#ifdef BY8X0116P_ENABLE_DEBOUNCING

static bool debouncedDigitalRead(byte pin, byte activeOn, int sampleTime, int sampleRate) {
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

#endif

bool BY8X0116P::_isBusy() {
    if (_busyPin != DISABLED) {
        waitRequest();

#ifndef BY8X0116P_ENABLE_DEBOUNCING
        return digitalRead(_busyPin) == _busyActiveOn;
#else
        return debouncedDigitalRead(_busyPin, _busyActiveOn, BY8X0116P_BUSY_DEBOUNCE_TIME,
#if defined(ARDUINO_ARCH_SAM) || defined(ARDUINO_ARCH_SAMD)
            4);
#else
            1);
#endif
#endif
    }

    return false;
}

bool BY8X0116P::waitBusy(int timeout) {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("BY8X0116P::waitBusy timeout: ");
    Serial.println(timeout);
#endif

    return _waitBusy();
}

bool BY8X0116P::_waitBusy(int timeout) {
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
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::isPlaybackActive");
#endif

    return _isPlaybackActive();
}

bool BY8X0116P::_isPlaybackActive() {
    return (BY8X0116P_PlaybackStatus)receiveCommand(BY8X0116P_QRY_PB_STATUS) != BY8X0116P_PlaybackStatus_Stopped;
}

bool BY8X0116P::waitPlaybackFinished(int timeout) {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
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
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
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
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::isStandingBy");
#endif

    if (millis() < _lastReqTime + BY8X0116P_READ_DELAY + BY8X0116P_READ_DELAY || _serial->available() > 4)
        cleanResponse();

    return _isStandingBy;
}

void BY8X0116P::reset(bool blocking) {
    if (_isResetting) return;

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
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
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.println("BY8X0116P::isResetting");
#endif

    if (_serial->available() > 2)
        cleanResponse();

    return _isResetting;
}

bool BY8X0116P::isCardInserted() {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
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
        if (_serial->available() > 4)
            cleanResponse();

        return _isCardInserted;
    }
}

void BY8X0116P::cleanupRoutine() {
    if (!_isBlockingRspLn && !_isCleaningRspLn && _serial->available() && millis() >= _lastClnTime + BY8X0116P_CLEAN_DELAY) {
#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
        Serial.println("BY8X0116P::cleanupRoutine");
#endif

        cleanResponse();
    }
}

void BY8X0116P::sendCommand(byte cmdID) {
    byte cmdData[] = { 0x7E, 0x03, cmdID, 0x00, 0xEF };
    writeRequest(cmdData);
}

void BY8X0116P::sendCommand(byte cmdID, byte param) {
    byte cmdData[] = { 0x7E, 0x04, cmdID, param, 0x00, 0xEF };
    writeRequest(cmdData);
}

void BY8X0116P::sendCommand(byte cmdID, uint16_t param) {
    byte cmdData[] = { 0x7E, 0x05, cmdID, highByte(param), lowByte(param), 0x00, 0xEF };
    writeRequest(cmdData);
}

void BY8X0116P::sendCommand(byte cmdID, byte param1, byte param2) {
    byte cmdData[] = { 0x7E, 0x05, cmdID, param1, param2, 0x00, 0xEF };
    writeRequest(cmdData);
}

uint16_t BY8X0116P::receiveCommand(byte cmdID) {
    ++_isBlockingRspLn;

    byte cmdData[] = { 0x7E, 0x03, cmdID, 0x00, 0xEF };
    writeRequest(cmdData, true);

    char respData[5];
    readResponse(respData, 8, 5);

    uint16_t retVal = strtol(respData, NULL, 16);

    --_isBlockingRspLn;

    return retVal;
}

int BY8X0116P::receiveCommand(byte cmdID, char *respData, int respLength, int maxLength) {
    ++_isBlockingRspLn;

    byte cmdData[] = { 0x7E, 0x03, cmdID, 0x00, 0xEF };
    writeRequest(cmdData, true);

    int retLen = readResponse(respData, respLength, maxLength);

    --_isBlockingRspLn;

    return retLen;
}

void BY8X0116P::writeRequest(byte *rqstData, bool cleanRspLn) {
    byte length = rqstData[1];
    byte checkCode = length;

    for (int i = 2; i < length; ++i)
        checkCode = checkCode ^ rqstData[i];
    rqstData[length] = checkCode;

    ++_isBlockingRspLn;

    if (cleanRspLn || _serial->available() > 8)
        cleanResponse();
    else // Allow time for last command to be properly processed
        waitRequest();

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("  BY8X0116P::writeRequest Cmd: 0x");
    Serial.print(rqstData[2], HEX);
    if (length - 3 > 0) {
        Serial.print(", Prm: ");
        for (int i = 3; i < length; ++i) {
            Serial.print(i > 3 ? "-0x" : "0x");
            Serial.print(rqstData[i], HEX);
        }
    }
    Serial.print(", Chk: 0x");
    Serial.println(rqstData[length], HEX);
#endif

    if (rqstData[2] == BY8X0116P_CMD_RESET) // Safer to capture state transition here
        _isResetting = true;

    _serial->write(rqstData, length + 2);
    _lastReqTime = millis();

    --_isBlockingRspLn;
}

int BY8X0116P::readResponse(char *respData, int respLength, int maxLength) {
    int bytesRead = 0;

    if (waitResponse()) {
        int lastChar = -1;

        while (_serial->available() && respLength > 0) {
            char currChar = _serial->read(); --respLength;

            if (lastChar != -1) {
                if ((lastChar == 'O' && currChar == 'K') ||
                    (lastChar == '\r' && currChar == '\n')) {
                    lastChar = -1;
                    continue;
                }

                if (maxLength-- > 0)
                    respData[bytesRead++] = lastChar;
            }

            lastChar = currChar;
        }

        if (lastChar != -1 && maxLength-- > 0)
            respData[bytesRead++] = lastChar;
    }

    if (maxLength-- > 0)
        respData[bytesRead] = '\0';

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
    Serial.print("  BY8X0116P::readResponse respData[");
    Serial.print(bytesRead);
    Serial.print("]: ");
    for (int i = 0; i < bytesRead; ++i) {
        if (respData[i] >= ' ')
            Serial.print(respData[i]);
        else {
            Serial.print("\\x");
            Serial.print(respData[i], HEX);
        }
    }
    Serial.println("");
#endif

    return bytesRead;
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
    int available = _serial->available();
    unsigned long endTime = max(millis(), _lastReqTime + BY8X0116P_READ_DELAY) + BY8X0116P_READ_DELAY;

    while (millis() < endTime) {
#ifdef BY8X0116P_USE_SCHEDULER
        Scheduler.yield();
#else
        delay(1);
#endif
        if (available != _serial->available()) {
            available = _serial->available();
            endTime = max(millis(), _lastReqTime + BY8X0116P_READ_DELAY) + BY8X0116P_READ_DELAY;
        }
    }

    return available;
}

bool BY8X0116P::cleanResponse() {
    _isCleaningRspLn = true;

    if (waitResponse()) {
        char respData[33];
        char *respLine = &respData[0];
        int respLength = _serial->available();

        if (respLength > 0) {
            _lastClnTime = millis();

            if (respLength > 32)
                respLine = (char *)malloc(respLength + 1);

            for (int i = 0; i < respLength; ++i)
                respLine[i] = _serial->read();
            respLine[respLength] = '\0';

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
            Serial.print("  BY8X0116P::cleanResponse Line: ");
            for (int i = 0; i < respLength; ++i) {
                if (respLine[i] >= ' ')
                    Serial.print(respLine[i]);
                else {
                    Serial.print("\\x");
                    Serial.print(respLine[i], HEX);
                }
            }
            Serial.println("");
#endif

            char *respScan = respLine;
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

            if (respLine != &respData[0])
                free(respLine);

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

#ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT

int BY8X0116P::getSerialInterfaceNumber() {
#ifndef BY8X0116P_USE_SOFTWARE_SERIAL
#if defined(HWSERIAL0) || defined(HAVE_HWSERIAL0)
    if (_serial == &Serial) return 0;
#endif
#if defined(HWSERIAL1) || defined(HAVE_HWSERIAL1)
    if (_serial == &Serial1) return 1;
#endif
#if defined(HWSERIAL2) || defined(HAVE_HWSERIAL2)
    if (_serial == &Serial2) return 2;
#endif
#if defined(HWSERIAL3) || defined(HAVE_HWSERIAL3)
    if (_serial == &Serial3) return 3;
#endif
#endif // /ifndef BY8X0116P_USE_SOFTWARE_SERIAL
    return -1;
}

static const char *textForSerialInterfaceNumber(int serialNum) {
#ifndef BY8X0116P_USE_SOFTWARE_SERIAL
    switch (serialNum) {
        case 0: return "Serial";
        case 1: return "Serial1";
        case 2: return "Serial2";
        case 3: return "Serial3";
        default: return "<CustomSerial>";
    }
#else
    return "SoftwareSerial";
#endif // /ifndef BY8X0116P_USE_SOFTWARE_SERIAL
}

void BY8X0116P::printModuleInfo() {
    char buffer[16];

    Serial.println(""); Serial.println(" ~~~ BY8X0116P Module Info ~~~");

    Serial.println(""); Serial.print("Busy Pin: ");
    if (_busyPin != DISABLED) {
        Serial.print("D"); Serial.print(_busyPin);
        Serial.println(_busyActiveOn ? " (active-high)" : " (active-low)");
    }
    else
        Serial.println("<disabled>");

    Serial.println(""); Serial.print("Serial Instance: ");
    Serial.print(getSerialInterfaceNumber()); Serial.print(": ");
    Serial.println(textForSerialInterfaceNumber(getSerialInterfaceNumber()));
    Serial.print("Serial Baud: ");
    Serial.print(getSerialBaud()); Serial.println("Hz");

    Serial.println(""); Serial.println("State:");
    Serial.print("  Standing By: ");
    Serial.print(_isStandingBy ? "true" : "false");
    Serial.print(", Resetting: ");
    Serial.print(_isResetting ? "true" : "false");
    Serial.print(", Card Inserted: ");
    Serial.println(_isCardInserted ? "true" : "false");

    Serial.println(""); Serial.println("Timers:");
    Serial.print("  Current Time: ");
    Serial.print(millis());
    Serial.print(" ms, Last Request: ");
    Serial.print(_lastReqTime);
    Serial.print(" ms, Last Clean: ");
    Serial.print(_lastClnTime);
    Serial.println(" ms");

    Serial.println(""); Serial.println("Firmware Version:");
    getFirmwareVersion(buffer, 16);
    Serial.println(buffer);

    Serial.println(""); Serial.println("Total Number Of Tracks:");
    Serial.println(getTotalNumberOfTracks());

    Serial.println(""); Serial.println("Current Track File Index:");
    Serial.println(getCurrentTrackFileIndex());

    Serial.println(""); Serial.println("Current Track Filename:");
    getCurrentTrackFilename(buffer, 16);
    Serial.println(buffer);

    Serial.println(""); Serial.println("Current Track Elapsed Time:");
    Serial.println(getCurrentTrackElapsedTime());

    Serial.println(""); Serial.println("Current Track Total Time:");
    Serial.println(getCurrentTrackTotalTime());

    Serial.println(""); Serial.println("Playback Status:");
    BY8X0116P_PlaybackStatus playbackStatus = getPlaybackStatus();
    Serial.print(playbackStatus); Serial.print(": ");
    switch (playbackStatus) {
        case BY8X0116P_PlaybackStatus_Stopped:
            Serial.println("BY8X0116P_PlaybackStatus_Stopped"); break;
        case BY8X0116P_PlaybackStatus_Playing:
            Serial.println("BY8X0116P_PlaybackStatus_Playing"); break;
        case BY8X0116P_PlaybackStatus_Paused:
            Serial.println("BY8X0116P_PlaybackStatus_Paused"); break;
        case BY8X0116P_PlaybackStatus_FastForwarding:
            Serial.println("BY8X0116P_PlaybackStatus_FastForwarding"); break;
        case BY8X0116P_PlaybackStatus_FastRewinding:
            Serial.println("BY8X0116P_PlaybackStatus_FastRewinding"); break;
        case BY8X0116P_PlaybackStatus_Count:
        case BY8X0116P_PlaybackStatus_Undefined:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Loop Playback Mode:");
    BY8X0116P_LoopPlaybackMode playbackMode = getLoopPlaybackMode();
    Serial.print(playbackMode); Serial.print(": ");
    switch (playbackMode) {
        case BY8X0116P_LoopPlaybackMode_All:
            Serial.println("BY8X0116P_LoopPlaybackMode_All"); break;
        case BY8X0116P_LoopPlaybackMode_Folder:
            Serial.println("BY8X0116P_LoopPlaybackMode_Folder"); break;
        case BY8X0116P_LoopPlaybackMode_Single:
            Serial.println("BY8X0116P_LoopPlaybackMode_Single"); break;
        case BY8X0116P_LoopPlaybackMode_Random:
            Serial.println("BY8X0116P_LoopPlaybackMode_Random"); break;
        case BY8X0116P_LoopPlaybackMode_Disabled:
            Serial.println("BY8X0116P_LoopPlaybackMode_Disabled"); break;
        case BY8X0116P_LoopPlaybackMode_Count:
        case BY8X0116P_LoopPlaybackMode_Undefined:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Equalizer Profile:");
    BY8X0116P_EqualizerProfile eqProfile = getEqualizerProfile();
    Serial.print(eqProfile); Serial.print(": ");
    switch (eqProfile) {
        case BY8X0116P_EqualizerProfile_None:
            Serial.println("BY8X0116P_EqualizerProfile_None"); break;
        case BY8X0116P_EqualizerProfile_Pop:
            Serial.println("BY8X0116P_EqualizerProfile_Pop"); break;
        case BY8X0116P_EqualizerProfile_Rock:
            Serial.println("BY8X0116P_EqualizerProfile_Rock"); break;
        case BY8X0116P_EqualizerProfile_Jazz:
            Serial.println("BY8X0116P_EqualizerProfile_Jazz"); break;
        case BY8X0116P_EqualizerProfile_Classic:
            Serial.println("BY8X0116P_EqualizerProfile_Classic"); break;
        case BY8X0116P_EqualizerProfile_Bass:
            Serial.println("BY8X0116P_EqualizerProfile_Bass"); break;
        case BY8X0116P_EqualizerProfile_Count:
        case BY8X0116P_EqualizerProfile_Undefined:
            Serial.println(""); break;
    }

    Serial.println(""); Serial.println("Playback Device:");
    BY8X0116P_PlaybackDevice pbDevice = getPlaybackDevice();
    Serial.print(pbDevice); Serial.print(": ");
    switch (pbDevice) {
        case BY8X0116P_PlaybackDevice_USB:
            Serial.println("BY8X0116P_PlaybackDevice_USB"); break;
        case BY8X0116P_PlaybackDevice_MicroSD:
            Serial.println("BY8X0116P_PlaybackDevice_MicroSD"); break;
        case BY8X0116P_PlaybackDevice_Count:
        case BY8X0116P_PlaybackDevice_Undefined:
            Serial.println(""); break;
    }
}

#endif // /ifdef BY8X0116P_ENABLE_DEBUG_OUTPUT
