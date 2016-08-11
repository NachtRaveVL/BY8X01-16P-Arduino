# BY8X01-16P-Arduino
Arduino Library for the BY8001-16P/BY8301-16P Audio Module.

**BY8X01-16P-Arduino v1.0**

Library to control a BY8001-16P or BY83001-16P audio module from an Arduino board.  
Licensed under the non-restrictive MIT license.

Created by NachtRaveVL, August 1st, 2016.

## Library Setup

There are several defines inside of the library's header file that allows for more fine-tuned control.

```Arduino
// Uncomment this define to disable usage of Scheduler.h on SAM/SAMD architecures.
//#define BY8X0116P_DISABLE_SCHEDULER     1

// Uncomment this define to enable debug output
//#define BY8X0116P_DEBUG_OUTPUT          1
```

## Hookup Instructions

Make sure to flip RX/TX lines when plugging into device from MCU. If running a 5v Arduino board, put a 1k Ohm resistor between the MCU's TX and device's RX pin (not required if on a 3.3v device). Also, remove A, B, and C resistors on device (factory default is a resistor on A and C, while B is left open), which puts the device into the recommended 1-1-1 mode used for MCU serial control. Busy pin returns a 2.8v signal when playback is active, and is optional for library usage.
