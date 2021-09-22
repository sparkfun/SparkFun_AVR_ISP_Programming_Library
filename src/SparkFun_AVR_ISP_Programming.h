/*
  Functions to facilitate programming of AVR ISP processors.

  Some of the content is based on Nick Gammon's arduino_sketches:
  https://github.com/nickgammon/arduino_sketches
  predominantly Atmega_Hex_Uploader:
  https://github.com/nickgammon/arduino_sketches/tree/master/Atmega_Hex_Uploader
  Thank you Nick.

  https://github.com/sparkfun/SparkFun_AVR_ISP_Programming_Library

  Development environment specifics:
  Arduino IDE 1.8.13

  SparkFun code, firmware, and software is released under the MIT License(http://opensource.org/licenses/MIT).
  The MIT License (MIT)
  Copyright (c) 2021 SparkFun Electronics
  Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
  associated documentation files (the "Software"), to deal in the Software without restriction,
  including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
  and/or sell copies of the Software, and to permit persons to whom the Software is furnished to
  do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all copies or substantial
  portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
  NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
  WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#ifndef SPARKFUN_AVR_ISP_LIBRARY_H
#define SPARKFUN_AVR_ISP_LIBRARY_H

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <SPI.h>
#include <SD.h> // Needed for SD read/write and File

//Fast GPIO code taken from: CapacitiveSense.h v.04
//  https://github.com/PaulStoffregen/CapacitiveSensor
//  http://www.pjrc.com/teensy/td_libs_CapacitiveSensor.html
//  http://playground.arduino.cc/Main/CapacitiveSensor
//  Copyright (c) 2008 Paul Bagder  All rights reserved.

// Direct I/O through registers and bitmask (from OneWire library)

#if defined(__AVR__)
#define PIN_TO_BASEREG(pin)             (portInputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask), (*((base)+2)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))

#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK66FX1M0__) || defined(__MK64FX512__)
#define PIN_TO_BASEREG(pin)             (portOutputRegister(pin))
#define PIN_TO_BITMASK(pin)             (1)
#define IO_REG_TYPE uint8_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (*((base)+512))
#define DIRECT_MODE_INPUT(base, mask)   (*((base)+640) = 0)
#define DIRECT_MODE_OUTPUT(base, mask)  (*((base)+640) = 1)
#define DIRECT_WRITE_LOW(base, mask)    (*((base)+256) = 1)
#define DIRECT_WRITE_HIGH(base, mask)   (*((base)+128) = 1)

#elif defined(__MKL26Z64__)
#define PIN_TO_BASEREG(pin)             (portOutputRegister(pin))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint8_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         ((*((base)+16) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   (*((base)+20) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  (*((base)+20) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    (*((base)+8) = (mask))
#define DIRECT_WRITE_HIGH(base, mask)   (*((base)+4) = (mask))

#elif defined(__SAM3X8E__)
#define PIN_TO_BASEREG(pin)             (&(digitalPinToPort(pin)->PIO_PER))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*((base)+15)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+5)) = (mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+4)) = (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+13)) = (mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+12)) = (mask))

#elif defined(__PIC32MX__)
#define PIN_TO_BASEREG(pin)             (portModeRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*(base+4)) & (mask)) ? 1 : 0)  //PORTX + 0x10
#define DIRECT_MODE_INPUT(base, mask)   ((*(base+2)) = (mask))            //TRISXSET + 0x08
#define DIRECT_MODE_OUTPUT(base, mask)  ((*(base+1)) = (mask))            //TRISXCLR + 0x04
#define DIRECT_WRITE_LOW(base, mask)    ((*(base+8+1)) = (mask))          //LATXCLR  + 0x24
#define DIRECT_WRITE_HIGH(base, mask)   ((*(base+8+2)) = (mask))          //LATXSET + 0x28

#elif defined(ARDUINO_ARCH_ESP8266)
#define PIN_TO_BASEREG(pin)             (portOutputRegister(digitalPinToPort(pin)))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*(base+6)) & (mask)) ? 1 : 0)    //GPIO_IN_ADDRESS
#define DIRECT_MODE_INPUT(base, mask)   ((*(base+5)) = (mask))              //GPIO_ENABLE_W1TC_ADDRESS
#define DIRECT_MODE_OUTPUT(base, mask)  ((*(base+4)) = (mask))              //GPIO_ENABLE_W1TS_ADDRESS
#define DIRECT_WRITE_LOW(base, mask)    ((*(base+2)) = (mask))              //GPIO_OUT_W1TC_ADDRESS
#define DIRECT_WRITE_HIGH(base, mask)   ((*(base+1)) = (mask))              //GPIO_OUT_W1TS_ADDRESS

#elif defined(__SAMD21G18A__)
// runs extremely slow/unreliable on Arduino Zero - help wanted....
#define PIN_TO_BASEREG(pin)             portModeRegister(digitalPinToPort(pin))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, mask)         (((*((base)+8)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) = (mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+2)) = (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+5)) = (mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+6)) = (mask))

#elif defined(RBL_NRF51822)
#define PIN_TO_BASEREG(pin)             (0)
#define PIN_TO_BITMASK(pin)             (pin)
#define IO_REG_TYPE uint32_t
#define IO_REG_ASM
#define DIRECT_READ(base, pin)          nrf_gpio_pin_read(pin)
#define DIRECT_WRITE_LOW(base, pin)     nrf_gpio_pin_clear(pin)
#define DIRECT_WRITE_HIGH(base, pin)    nrf_gpio_pin_set(pin)
#define DIRECT_MODE_INPUT(base, pin)    nrf_gpio_cfg_input(pin, NRF_GPIO_PIN_NOPULL)
#define DIRECT_MODE_OUTPUT(base, pin)   nrf_gpio_cfg_output(pin)

#else
#define PIN_TO_BASEREG(pin)             (0)
#define PIN_TO_BITMASK(pin)             (pin)
#define DIRECT_READ(base, pin)          digitalRead(pin)
#define DIRECT_WRITE_LOW(base, pin)     digitalWrite(pin, LOW)
#define DIRECT_WRITE_HIGH(base, pin)    digitalWrite(pin, HIGH)
#define DIRECT_MODE_INPUT(base, pin)    pinMode(pin, INPUT)
#define DIRECT_MODE_OUTPUT(base, pin)   pinMode(pin, OUTPUT)

#endif // /Fast pin code

// meaning of the byte positions in fuses
enum {
  lowFuse,
  highFuse,
  extFuse,
  lockByte,
  calibrationByte
};

// actions to take
enum {
  checkFile,
  verifyFlash,
  writeToFlash,
};

// types of record in .hex file
enum {
  hexDataRecord,  // 00
  hexEndOfFile,   // 01
  hexExtendedSegmentAddressRecord, // 02
  hexStartSegmentAddressRecord,  // 03
  hexExtendedLinearAddressRecord, // 04
  hexStartLinearAddressRecord // 05
};

// structure to hold signature and other relevant data about each chip
typedef struct {
  byte sig [3];                // chip signature
  char desc [14];              // fixed array size keeps chip names in PROGMEM
  unsigned long flashSize;     // how big the flash is (bytes)
  unsigned int baseBootSize;   // base bootloader size (others are multiples of 2/4/8)
  unsigned long pageSize;      // flash programming page size (bytes)
  byte fuseWithBootloaderSize; // ie. one of: lowFuse, highFuse, extFuse
  bool timedWrites;            // true if pollUntilReady won't work by polling the chip
} signatureType;

const unsigned long kb = 1024;
const byte NO_FUSE = 0xFF;

// see Atmega datasheets
const signatureType signatures [] PROGMEM =
{
  //     signature        description   flash size   bootloader  flash  fuse     timed
  //                                                     size    page    to      writes
  //                                                             size   change

  // Attiny43U
  { { 0x1E, 0x92, 0x0C }, "ATtiny43U",  4 * kb,           0,   64,   NO_FUSE,  false  },

  // Attiny841 family
  { { 0x1E, 0x92, 0x15 }, "ATtiny441",  4 * kb,           0,   16,   NO_FUSE,  false  },
  { { 0x1E, 0x93, 0x15 }, "ATtiny841",  8 * kb,           0,   16,   NO_FUSE,  false  },

  // Attiny84 family
  { { 0x1E, 0x91, 0x0B }, "ATtiny24",   2 * kb,           0,   32,   NO_FUSE,  false  },
  { { 0x1E, 0x92, 0x07 }, "ATtiny44",   4 * kb,           0,   64,   NO_FUSE,  false  },
  { { 0x1E, 0x93, 0x0C }, "ATtiny84",   8 * kb,           0,   64,   NO_FUSE,  false  },

  // Attiny85 family
  { { 0x1E, 0x91, 0x08 }, "ATtiny25",   2 * kb,           0,   32,   NO_FUSE,  false  },
  { { 0x1E, 0x92, 0x06 }, "ATtiny45",   4 * kb,           0,   64,   NO_FUSE,  false  },
  { { 0x1E, 0x93, 0x0B }, "ATtiny85",   8 * kb,           0,   64,   NO_FUSE,  false  },

  // Atmega328 family
  { { 0x1E, 0x92, 0x0A }, "ATmega48PA",   4 * kb,         0,    64,  NO_FUSE,  false  },
  { { 0x1E, 0x93, 0x0F }, "ATmega88PA",   8 * kb,       256,   128,  extFuse,  false },
  { { 0x1E, 0x94, 0x0B }, "ATmega168PA", 16 * kb,       256,   128,  extFuse,  false },
  { { 0x1E, 0x94, 0x06 }, "ATmega168V",  16 * kb,       256,   128,  extFuse,  false },
  { { 0x1E, 0x95, 0x0F }, "ATmega328P",  32 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x16 }, "ATmega328PB", 32 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x14 }, "ATmega328",   32 * kb,       512,   128,  highFuse, false },

  // Atmega644 family
  { { 0x1E, 0x94, 0x0A }, "ATmega164P",   16 * kb,      256,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x08 }, "ATmega324P",   32 * kb,      512,   128,  highFuse, false },
  { { 0x1E, 0x96, 0x0A }, "ATmega644P",   64 * kb,   1 * kb,   256,  highFuse, false },

  // Atmega2560 family
  { { 0x1E, 0x96, 0x08 }, "ATmega640",    64 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x97, 0x03 }, "ATmega1280",  128 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x97, 0x04 }, "ATmega1281",  128 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x98, 0x01 }, "ATmega2560",  256 * kb,   1 * kb,   256,  highFuse, false },

  { { 0x1E, 0x98, 0x02 }, "ATmega2561",  256 * kb,   1 * kb,   256,  highFuse, false },

  // AT90USB family
  { { 0x1E, 0x93, 0x82 }, "At90USB82",    8 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x94, 0x82 }, "At90USB162",  16 * kb,       512,   128,  highFuse, false },

  // Atmega32U2 family
  { { 0x1E, 0x93, 0x89 }, "ATmega8U2",    8 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x94, 0x89 }, "ATmega16U2",  16 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x8A }, "ATmega32U2",  32 * kb,       512,   128,  highFuse, false },

  // Atmega32U4 family -  (datasheet is wrong about flash page size being 128 words)
  { { 0x1E, 0x94, 0x88 }, "ATmega16U4",  16 * kb,       512,   128,  highFuse, false },
  { { 0x1E, 0x95, 0x87 }, "ATmega32U4",  32 * kb,       512,   128,  highFuse, false },

  // ATmega1284P family
  { { 0x1E, 0x97, 0x05 }, "ATmega1284P", 128 * kb,   1 * kb,   256,  highFuse, false },
  { { 0x1E, 0x97, 0x06 }, "ATmega1284",  128 * kb,   1 * kb,   256,  highFuse, false },

  // ATtiny4313 family
  { { 0x1E, 0x91, 0x0A }, "ATtiny2313A",   2 * kb,        0,    32,  NO_FUSE,  false   },
  { { 0x1E, 0x92, 0x0D }, "ATtiny4313",    4 * kb,        0,    64,  NO_FUSE,  false   },

  // ATtiny13 family
  { { 0x1E, 0x90, 0x07 }, "ATtiny13A",     1 * kb,        0,    32,  NO_FUSE,  false  },

  // Atmega8A family
  { { 0x1E, 0x93, 0x07 }, "ATmega8A",      8 * kb,      256,    64,  highFuse, true },

  // ATmega64rfr2 family
  { { 0x1E, 0xA6, 0x02 }, "ATmega64rfr2",  256 * kb, 1 * kb,   256,  highFuse, false },
  { { 0x1E, 0xA7, 0x02 }, "ATmega128rfr2", 256 * kb, 1 * kb,   256,  highFuse, false },
  { { 0x1E, 0xA8, 0x02 }, "ATmega256rfr2", 256 * kb, 1 * kb,   256,  highFuse, false },

};  // end of signatures

/*
  Functions needed for SPI (ICSP) progamming
  Author: Nick Gammon

  From: https://github.com/nickgammon/arduino_sketches/blob/master/Atmega_Hex_Uploader/ICSP_Utils.ino
*/

// programming commands to send via SPI to the chip
enum {
  progamEnable = 0xAC,

  // writes are preceded by progamEnable
  chipErase = 0x80,
  writeLockByte = 0xE0,
  writeLowFuseByte = 0xA0,
  writeHighFuseByte = 0xA8,
  writeExtendedFuseByte = 0xA4,

  pollReady = 0xF0,

  programAcknowledge = 0x53,

  readSignatureByte = 0x30,
  readCalibrationByte = 0x38,

  readLowFuseByte = 0x50,       readLowFuseByteArg2 = 0x00,
  readExtendedFuseByte = 0x50,  readExtendedFuseByteArg2 = 0x08,
  readHighFuseByte = 0x58,      readHighFuseByteArg2 = 0x08,
  readLockByte = 0x58,          readLockByteArg2 = 0x00,

  readProgramMemory = 0x20,
  writeProgramMemory = 0x4C,
  loadExtendedAddressByte = 0x4D,
  loadProgramMemory = 0x40,

};  // end of enum

// which program instruction writes which fuse
const byte fuseCommands [4] = { writeLowFuseByte, writeHighFuseByte, writeExtendedFuseByte, writeLockByte };

class SFE_AVR_ISP
{

  public:

    byte fuses [5]; // copy of fuses/lock bytes found for this processor

    // copy of current signature entry for matching processor
    signatureType currentSignature;

    //Constructor
    SFE_AVR_ISP(uint8_t ispCIPO, uint8_t ispCOPI, uint8_t ispSCK, uint8_t ispRST);

    //Begin the library
    bool begin(uint8_t microSDCS = 255, uint8_t switchEnableSpi = 255); //Initialize the library

    //Sets the local file name variable for the firmware file you want
    //to load from the SD card
    //Call this before calling programTarget()
    void setFileName(char *name);

    //Sets the local target name for the AVR IC you want to program. ie: "ATtiny84"
    //If your target name does not match the name associated with the found
    //signature then an error is raised
    //Call this before calling programTarget()
    void setTargetName(char *name);

    //Sets the local low fuse byte.
    //Set this before calling programTarget()
    void setLowFuse(byte fuse);

    //Sets the local high fuse byte.
    //Set this before calling programTarget()
    void setHighFuse(byte fuse);

    //Sets the local extended fuse byte.
    //The user can call this if they want to change the extended fuse during programTarget()
    void setExtendedFuse(byte fuse);

    //This is the main programming function
    //Power up the SD card, mount it, open the firmware file
    //Set fuses, reset target (so new fuses can take effect)
    //Erase the target, program the firmware file onto target, verify firmware
    //Returns true if everything was successful
    bool programTarget();

    //Call this function to set the clock speed (bit-bang delay) during the programming cycle
    //The default clock delay is 6 microseconds (6us high, 6us low = approx. 87kHz)
    void setProgrammingClockSpeed(uint32_t speed);

    // Functions needed for SPI (ICSP) progamming
    // Author: Nick Gammon
    // From: https://github.com/nickgammon/arduino_sketches/blob/master/Atmega_Hex_Uploader/ICSP_Utils.ino
    bool startProgramming();
    void stopProgramming();
    bool getSignature();
    void readSignature (byte sig [3]);
    void getFuseBytes();
    byte readFuse (const byte which);
    void writeFuse(byte newValue, byte whichFuse);
    byte readFlash (unsigned long addr);
    void writeFlash (unsigned long addr, const byte data);
    void eraseMemory();
    void writeData(const unsigned long addr, const byte * pData, const int length);
    void verifyData(const unsigned long addr, const byte * pData, const int length);

    // Enable debug messages using the chosen Serial port (Stream)
    // Boards like the RedBoard Turbo use SerialUSB (not Serial).
    // But other boards like the SAMD51 Thing Plus use Serial (not SerialUSB).
    // These lines let the code compile cleanly on as many SAMD boards as possible.
    #if defined(ARDUINO_ARCH_SAMD)  // Is this a SAMD board?
    #if defined(USB_VID)            // Is the USB Vendor ID defined?
    #if (USB_VID == 0x1B4F)         // Is this a SparkFun board?
    #if !defined(ARDUINO_SAMD51_THING_PLUS) & !defined(ARDUINO_SAMD51_MICROMOD) // If it is not a SAMD51 Thing Plus or SAMD51 MicroMod
    void enableDebugging(Stream &debugPort = SerialUSB); //Given a port to print to, enable debug messages.
    #else
    void enableDebugging(Stream &debugPort = Serial); //Given a port to print to, enable debug messages.
    #endif
    #else
    void enableDebugging(Stream &debugPort = Serial); //Given a port to print to, enable debug messages.
    #endif
    #else
    void enableDebugging(Stream &debugPort = Serial); //Given a port to print to, enable debug messages.
    #endif
    #else
    void enableDebugging(Stream &debugPort = Serial); //Given a port to print to, enable debug messages.
    #endif

    void disableDebugging(void);      //Turn off debug statements

  private:

    char firmwareFileName[13] = {'\0'}; //Firmware file to read. Limited file name length to 8.3 format
    bool firmwareFileNameDefined = false; //Make sure user sets the filename before calling programTarget()
    char targetDevice[14] = {'\0'}; //Name of the chip we want to program
    bool targetDeviceDefined = false; //Make sure user sets the target device before calling programTarget()
    byte newLowFuseValue;
    bool lowFuseDefined = false; //Make sure user sets the fuse before calling programTarget()
    byte newHighFuseValue;
    bool highFuseDefined = false; //Make sure user sets the fuse before calling programTarget()
    byte newExtendedFuseValue;
    bool extendedFuseDefined = false; //Optional: flag if the user wants to change the extended fuse in programTarget()

    //Control the speed of bit bang transfers
    //Don't go faster than 125kHz when ATtiny is set to 1MHz internal
    //bitBangDelayAmount = 6; //Original - 92us per call = 87kHz
    //bitBangDelayAmount = 4; //60 us per call = 133kHz
    //bitBangDelayAmount = 2; //28 us per call = 286kHz
    //bitBangDelayAmount = 0; //16 us per call = 500kHz
    byte bitBangDelayus = 6; // Default bit-bang delay for programming. Change with setProgrammingClockSpeed
    const byte slowBitBangDelayus = 10; // Bit-bang delay for writing the fuses
    // The delay used by BB_SPITransfer. Changing this to an parameter/argument would involve way too much hacking!
    byte bitBangDelayAmount = slowBitBangDelayus;

    const byte ENTER_PROGRAMMING_ATTEMPTS = 50;

    #define NUMITEMS(arg) ((unsigned int) (sizeof (arg) / sizeof (arg [0]))) // number of items in an array

    byte lastAddressMSB = 0;

    unsigned long pagemask;
    unsigned long oldPage;

    unsigned int errors; // count errors

    const unsigned long NO_PAGE = 0xFFFFFFFF;

    unsigned long lowestAddress;
    unsigned long highestAddress;
    unsigned long bytesWritten;

    bool gotEndOfFile;
    unsigned long extendedAddress;
    unsigned int lineCount;

    // Define the ISP pins as const uint8_t to make the DIRECT pin support as fast as possible
    const uint8_t ISP_CIPO; // The pin connected to the target CIPO pin
    const uint8_t ISP_COPI; // The pin connected to the target COPI pin
    const uint8_t ISP_SCK; // The pin connected to the target SCK pin
    const uint8_t ISP_RST; // The pin connected to the target RST pin

    uint8_t SWITCH_ENABLE_SPI; // The pin which enables the SD card SPI buffer
    uint8_t MICROSD_CS; // The pin connected to the SD card chip select

    //To get to the SD card we may need to provide it power by enabling an SPI switch/buffer
    //And begin SD with the SD CS pin
    bool mountSDCard();

    // Functions needed for SPI (ICSP) progamming
    // Author: Nick Gammon
    // From: https://github.com/nickgammon/arduino_sketches/blob/master/Atmega_Hex_Uploader/ICSP_Utils.ino
    byte program(const byte b1, const byte b2 = 0, const byte b3 = 0, const byte b4 = 0);
    void clearPage();
    void pollUntilReady();
    void commitPage(unsigned long addr, bool showMessage = false);
    bool hexConv(const char * (& pStr), byte &b);
    bool processLine(const char * pLine, const byte action);
    bool readHexFile(File firmwareFile, const byte action);
    byte BB_SPITransfer(byte c);

    Stream *_debugSerial;           //The stream to send debug messages to if enabled
    boolean _printDebug = false;    //Flag to print the serial commands we are sending to the Serial port for debug

    #define debugPrint1( var ) {if (_printDebug) _debugSerial->print( var );}
    #define debugPrintln1( var ) {if (_printDebug) _debugSerial->println( var );}
    #define debugPrint2( var1, var2 ) {if (_printDebug) _debugSerial->print( var1, var2 );}
    #define debugPrintln2( var1, var2 ) {if (_printDebug) _debugSerial->println( var1, var2 );}
    #define debugPrintf1( var ) {if (_printDebug) _debugSerial->printf( var );}
    #define debugPrintf2( var1, var2 ) {if (_printDebug) _debugSerial->printf( var1, var2 );}
    #define debugPrintf3( var1, var2, var3 ) {if (_printDebug) _debugSerial->printf( var1, var2, var3 );}
    #define debugPrintf4( var1, var2, var3, var4 ) {if (_printDebug) _debugSerial->printf( var1, var2, var3, var4 );}
    #define debugPrintf5( var1, var2, var3, var4, var5 ) {if (_printDebug) _debugSerial->printf( var1, var2, var3, var4, var5 );}
    // gfvalvo's flash string helper code: https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809
    void debugPrint(const char *);
    void debugPrint(const __FlashStringHelper *);
    void debugPrintln(const char *);
    void debugPrintln(const __FlashStringHelper *);
    void doDebugPrint(char (*)(const char *), const char *, bool newLine = false);

};

#endif
