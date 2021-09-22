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

#include "SparkFun_AVR_ISP_Programming.h"

//Constructor - define the pins as const uint8_t to make the DIRECT pin support as fast as possible
SFE_AVR_ISP::SFE_AVR_ISP(uint8_t ispCIPO, uint8_t ispCOPI, uint8_t ispSCK, uint8_t ispRST)
             : ISP_CIPO(ispCIPO), ISP_COPI(ispCOPI), ISP_SCK(ispSCK), ISP_RST(ispRST)
{
}

//Initialize the library
bool SFE_AVR_ISP::begin(uint8_t microSDCS, uint8_t switchEnableSpi)
{
  // Grab the pins
  SWITCH_ENABLE_SPI = switchEnableSpi; // Defaults to 255
  MICROSD_CS = microSDCS; // Defaults to 255

  // Initialize the pins
  if (SWITCH_ENABLE_SPI < 255)
  {
    pinMode(SWITCH_ENABLE_SPI, OUTPUT);
    digitalWrite(SWITCH_ENABLE_SPI, LOW); // Disable the SD card SPI buffer
  }

  if (MICROSD_CS < 255)
  {
    pinMode(MICROSD_CS, OUTPUT);
    digitalWrite(MICROSD_CS, HIGH); // Disable the SD card CS
  }

  pinMode(ISP_RST, OUTPUT);
  digitalWrite(ISP_RST, HIGH); //Do not reset ATtiny

  //Bit bang pins for AVR ISP programming
  pinMode(ISP_CIPO, INPUT);
  pinMode(ISP_COPI, OUTPUT);
  digitalWrite(ISP_COPI, LOW);

  return (true);
}

//This is the main programming function
//Power up the SD card, mount it, open the firmware file
//Set fuses, reset target (so new fuses can take effect)
//Erase the target, program the firmware file onto target, verify firmware
//Returns true if everything was successful
bool SFE_AVR_ISP::programTarget()
{
  //Verify that user has set all the required bits before running
  if ((firmwareFileNameDefined == false) || (strlen(firmwareFileName) == 0))
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: Firmware filename not set"));
    return (false);
  }

  if ((targetDeviceDefined == false) || (strlen(targetDevice) == 0))
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: Target device not set"));
    return (false);
  }

  if (lowFuseDefined == false)
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: Low fuse not defined"));
    return (false);
  }

  if (highFuseDefined == false)
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: High fuse not defined"));
    return (false);
  }

  debugPrintln(F("SFE_AVR_ISP::programTarget: Begin Microcontroller programming"));

  if (mountSDCard() == false) //Start communication with SD card. This also starts SPI.
    return (false);

  File firmwareFile = SD.open(firmwareFileName); //Firmware file to read. Limited file name length to 8.3 format
  if (!firmwareFile)
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: Firmware file failed to open"));
    return (false);
  }
  debugPrintln(F("SFE_AVR_ISP::programTarget: Firmware file opened"));

  //Begin Atmel ISP programming
  bitBangDelayAmount = slowBitBangDelayus; //Go slow
  startProgramming();
  if (getSignature() == false)
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: Microcontroller signature not found"));
    return (false);
  }

  if (strcmp(currentSignature.desc, targetDevice) != 0)
  {
    debugPrint(F("SFE_AVR_ISP::programTarget: "));
    debugPrint1(targetDevice);
    debugPrintln(F(" not found"));
    return (false);
  }
  debugPrint(F("SFE_AVR_ISP::programTarget: Target correctly identified as "));
  debugPrintln1(currentSignature.desc);

  unsigned long startTime = millis();

  writeFuse(newLowFuseValue, lowFuse);
  writeFuse(newHighFuseValue, highFuse);
  if (newExtendedFuseValue)
    writeFuse(newExtendedFuseValue, extFuse);

  //Check fuses were set correctly
  if (readFuse(lowFuse) != newLowFuseValue)
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: Low fuse programming failed"));
    stopProgramming();
    return (false);
  }
  if (readFuse(highFuse) != newHighFuseValue)
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: High fuse programming failed"));
    stopProgramming();
    return (false);
  }
  if (newExtendedFuseValue && readFuse(extFuse) != newExtendedFuseValue)
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: Extended fuse programming failed"));
    stopProgramming();
    return (false);
  }
  debugPrintln(F("SFE_AVR_ISP::programTarget: Fuses set"));

  stopProgramming(); //Once we change the fuses, we need to hang up, and
  startProgramming(); //start ISP over again. This will reset IC and cause IC to run at new fuses.

  bitBangDelayAmount = bitBangDelayus; //Go fast

  //Write contents of firmware file to target
  if (readHexFile(firmwareFile, writeToFlash) == false)
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: Firmware write failed"));
    return (false);
  }

  firmwareFile.seek(0); //Reset read location to beginning of firmware file

  //Verify
  if (readHexFile(firmwareFile, verifyFlash) == false)
  {
    debugPrintln(F("SFE_AVR_ISP::programTarget: Verification failed"));
    return (false);
  }

  unsigned long stopTime = millis();

  stopProgramming(); //Release target from reset

  debugPrint(F("SFE_AVR_ISP::programTarget: Finished in: "));
  debugPrintln2( (stopTime - startTime) / 1000.0, 4);

  debugPrintln(F("SFE_AVR_ISP::programTarget: Firmware successfully written"));

  if (SWITCH_ENABLE_SPI < 255)
    digitalWrite(SWITCH_ENABLE_SPI, LOW); //Disable SPI

  return (true);
}

//Sets the local file name variable for the firmware file you want
//to load from the SD card
void SFE_AVR_ISP::setFileName(char *name)
{
  strcpy(firmwareFileName, name);
  firmwareFileNameDefined = true;
}

//Sets the local target name for the AVR IC you want to program. ie: "ATtiny84"
//If your target name does not match the name associated with the found
//signature then an error is raised
void SFE_AVR_ISP::setTargetName(char *name)
{
  strcpy(targetDevice, name);
  targetDeviceDefined = true;
}

//Sets the local low fuse byte.
//Set this before calling programTarget()
void SFE_AVR_ISP::setLowFuse(byte fuse)
{
  newLowFuseValue = fuse;
  lowFuseDefined = true;
}

//Sets the local high fuse byte.
//Set this before calling programTarget()
void SFE_AVR_ISP::setHighFuse(byte fuse)
{
  newHighFuseValue = fuse;
  highFuseDefined = true;
}

//Sets the local extended fuse byte.
//Optional: the user can call this to change the extended fuse in programTarget()
void SFE_AVR_ISP::setExtendedFuse(byte fuse)
{
  newExtendedFuseValue = fuse;
  extendedFuseDefined = true;
}

//The max programming speed is 1/8 of the target's oscillator speed
//If target is internal 1MHz, this is 125kHz
//If target is internal 8MHz, this is 1MHz
//Remember that the oscillator speed is controlled by the fuses
//The user needs to ensure that the speed selected here is appropriate
//(Because we are bit banging, the fastest we can do is ~500kHz)
void SFE_AVR_ISP::setProgrammingClockSpeed(uint32_t speed)
{
  debugPrint(F("SFE_AVR_ISP::setProgrammingClockSpeed: selected clock speed is "));
  debugPrint1(speed);
  debugPrintln(F(" Hz"));
  if (speed >= 500000) // Convert speed to delay. Do it long hand to avoid rounding errors
    bitBangDelayus = 0;
  else if (speed >= 333333)
    bitBangDelayus = 1;
  else if (speed >= 250000)
    bitBangDelayus = 2;
  else if (speed >= 166666)
    bitBangDelayus = 3;
  else if (speed >= 125000)
    bitBangDelayus = 4;
  else if (speed >= 100000)
    bitBangDelayus = 5;
  else if (speed >= 83333)
    bitBangDelayus = 6;
  else if (speed >= 71249)
    bitBangDelayus = 7;
  else if (speed >= 62500)
    bitBangDelayus = 8;
  else if (speed >= 55555)
    bitBangDelayus = 9;
  else //if (speed >= 50000)
    bitBangDelayus = 10;
  debugPrint(F("SFE_AVR_ISP::setProgrammingClockSpeed: setting the bit-bang delay to "));
  debugPrint1(bitBangDelayus);
  debugPrintln(F(" microseconds"));
}

//To get to the SD card we need to provide it power, enable the SPI switch
//And begin SD with the SD CS pin
//Finally, we open the firmware file to deal with
//Note: The firmware file name is limited to a length of 8 chars with 3 char extension (no long file names)
bool SFE_AVR_ISP::mountSDCard()
{
  if (MICROSD_CS == 255)
  {
    debugPrintln(F("SFE_AVR_ISP::mountSDCard: MICROSD_CS not defined!"));
    return (false);
  }

  if (SWITCH_ENABLE_SPI < 255)
    digitalWrite(SWITCH_ENABLE_SPI, HIGH); //Enable SPI

  //Mount SD card
  if (!SD.begin(MICROSD_CS)) {
    debugPrintln(F("SFE_AVR_ISP::mountSDCard: SD initialization failed!"));
    if (SWITCH_ENABLE_SPI < 255)
      digitalWrite(SWITCH_ENABLE_SPI, LOW); //Enable SPI
    return (false);
  }
  return (true);
}

/*
  Functions needed for SPI (ICSP) progamming
  Author: Nick Gammon

  From: https://github.com/nickgammon/arduino_sketches/blob/master/Atmega_Hex_Uploader/ICSP_Utils.ino
*/

// execute one programming instruction ... b1 is command, b2, b3, b4 are arguments
//  processor may return a result on the 4th transfer, this is returned.
byte SFE_AVR_ISP::program(const byte b1, const byte b2, const byte b3, const byte b4)
{
  noInterrupts();

  BB_SPITransfer(b1);
  BB_SPITransfer(b2);
  BB_SPITransfer(b3);
  byte response = BB_SPITransfer(b4);

  interrupts();

  return (response);
}

// clear entire temporary page to 0xFF in case we don't write to all of it
void SFE_AVR_ISP::clearPage()
{
  unsigned int len = currentSignature.pageSize;
  for (unsigned int i = 0; i < len; i++)
    writeFlash(i, 0xFF);
}  // end of clearPage

// read a byte from flash memory
byte SFE_AVR_ISP::readFlash (unsigned long addr)
{
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB)
  {
    program (loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }  // end if different MSB

  return program (readProgramMemory | high, highByte (addr), lowByte (addr));
} // end of readFlash

// write a byte to the flash memory buffer (ready for committing)
void SFE_AVR_ISP::writeFlash (unsigned long addr, const byte data)
{
  byte high = (addr & 1) ? 0x08 : 0;  // set if high byte wanted
  addr >>= 1;  // turn into word address
  program(loadProgramMemory | high, 0, lowByte(addr), data);
} // end of writeFlash

byte SFE_AVR_ISP::readFuse (const byte which)
{
  switch (which)
  {
    case lowFuse:         return program (readLowFuseByte, readLowFuseByteArg2);
    case highFuse:        return program (readHighFuseByte, readHighFuseByteArg2);
    case extFuse:         return program (readExtendedFuseByte, readExtendedFuseByteArg2);
    case lockByte:        return program (readLockByte, readLockByteArg2);
    case calibrationByte: return program (readCalibrationByte);
  }  // end of switch

  return 0;
}  // end of readFuse

// write specified value to specified fuse/lock byte
void SFE_AVR_ISP::writeFuse(byte newValue, byte whichFuse)
{
  if (newValue == 0)
    return;  // ignore

  program(progamEnable, fuseCommands[whichFuse], 0, newValue);
  pollUntilReady();
}  // end of writeFuse

void SFE_AVR_ISP::readSignature (byte sig [3])
{
  for (byte i = 0; i < 3; i++)
    sig [i] = program (readSignatureByte, 0, i);

  // make sure extended address is zero to match lastAddressMSB variable
  program (loadExtendedAddressByte, 0, 0);
  lastAddressMSB = 0;

}  // end of readSignature

// poll the target device until it is ready to be programmed
void SFE_AVR_ISP::pollUntilReady()
{
  if (currentSignature.timedWrites)
    delay (10);  // at least 2 x WD_FLASH which is 4.5 mS
  else
    while ((program(pollReady) & 1) == 1) ; // wait till ready

}  // end of pollUntilReady

// commit page to flash memory
void SFE_AVR_ISP::commitPage(unsigned long addr, bool showMessage)
{
  if (showMessage)
  {
    debugPrint(F("SFE_AVR_ISP::commitPage: Committing page starting at 0x"));
    debugPrintln2(addr, HEX);
  }
  else
  {
    debugPrint(".");
    //showProgress ();
  }

  addr >>= 1;  // turn into word address

  // set the extended (most significant) address byte if necessary
  byte MSB = (addr >> 16) & 0xFF;
  if (MSB != lastAddressMSB)
  {
    program(loadExtendedAddressByte, 0, MSB);
    lastAddressMSB = MSB;
  }  // end if different MSB

  program(writeProgramMemory, highByte(addr), lowByte(addr));
  pollUntilReady();

  clearPage(); // clear ready for next page full
}  // end of commitPage

void SFE_AVR_ISP::eraseMemory()
{
  program(progamEnable, chipErase);   // erase it
  delay(20);  // for Atmega8
  pollUntilReady();
  clearPage();  // clear temporary page
}  // end of eraseMemory

// put chip into programming mode
//Returns true if target correctly answered to start of programming over SPI
bool SFE_AVR_ISP::startProgramming()
{
  byte confirm;

  debugPrintln(F("SFE_AVR_ISP::startProgramming: Entering Programming Mode..."));

  pinMode(ISP_RST, OUTPUT);

  digitalWrite(ISP_SCK, LOW);
  pinMode(ISP_SCK, OUTPUT);
  pinMode(ISP_COPI, OUTPUT);
  pinMode(ISP_CIPO, INPUT);

  //  digitalWrite(ISP_RST, HIGH);  // ensure SS stays high for now
  //  SPI.begin();
  //  SPI.setClockDivider(SPI_CLOCK_DIV64);
  //  pinMode(TARGET_SCK, OUTPUT);

  unsigned int timeout = 0;

  // we are in sync if we get back programAcknowledge on the third byte
  do
  {
    // We failed so delay a bit and try again
    delay(10); //Originally 100

    noInterrupts();

    digitalWrite(ISP_SCK, LOW); // ensure SCK low

    // then pulse reset, see page 309 of datasheet
    digitalWrite(ISP_RST, HIGH);
    delayMicroseconds(10);  // pulse for at least 2 clock cycles
    digitalWrite(ISP_RST, LOW);

    interrupts();

    delay(25);  // wait at least 20 mS
    noInterrupts();

    //    SPI.transfer(progamEnable);
    //    SPI.transfer(programAcknowledge);
    //    confirm = SPI.transfer(0);
    //    SPI.transfer(0);

    BB_SPITransfer(progamEnable);
    BB_SPITransfer(programAcknowledge);
    confirm = BB_SPITransfer(0);
    BB_SPITransfer(0);

    interrupts();

    if (confirm != programAcknowledge)
    {
      debugPrint(F("."));
      if (timeout++ >= ENTER_PROGRAMMING_ATTEMPTS)
      {
        debugPrintln(F(""));
        debugPrintln(F("SFE_AVR_ISP::startProgramming: Failed to enter programming mode. Double-check wiring!"));
        return (false);
      }  // end of too many attempts
    }  // end of not entered programming mode


  } while (confirm != programAcknowledge);

  return (true);
}  // end of startProgramming

void SFE_AVR_ISP::stopProgramming()
{
  digitalWrite(ISP_RST, LOW);
  pinMode(ISP_RST, INPUT);

  //  SPI.end();

  // turn off pull-ups, if any
  digitalWrite(ISP_SCK, LOW);
  digitalWrite(ISP_COPI, LOW);
  digitalWrite(ISP_CIPO, LOW);

  // set everything back to inputs
  pinMode(ISP_SCK, INPUT);
  pinMode(ISP_COPI, INPUT);
  pinMode(ISP_CIPO, INPUT);

  debugPrintln(F("SFE_AVR_ISP::stopProgramming: Programming mode off."));

} // end of stopProgramming

// write data to temporary buffer, ready for committing
void SFE_AVR_ISP::writeData(const unsigned long addr, const byte * pData, const int length)
{
  // write each byte
  for (int i = 0 ; i < length ; i++)
  {
    unsigned long thisPage = (addr + i) & pagemask;

    // page changed? commit old one
    if (thisPage != oldPage && oldPage != NO_PAGE)
      commitPage(oldPage);

    oldPage = thisPage; // now this is the current page

    writeFlash(addr + i, pData[i]); // put byte into work buffer
  }

}  // end of writeData

//Given an address and an array of data, check if it matches what is located
//in flash
void SFE_AVR_ISP::verifyData(const unsigned long addr, const byte * pData, const int length)
{
  // check each byte
  for (int i = 0 ; i < length ; i++)
  {
    unsigned long thisPage = (addr + i) & pagemask;

    // page changed? show progress
    if (thisPage != oldPage && oldPage != NO_PAGE)
      debugPrint("#");

    // now this is the current page
    oldPage = thisPage;

    byte found = readFlash(addr + i);
    byte expected = pData[i];
    if (found != expected)
    {
      if (errors <= 2)
      {
        debugPrint(F("SFE_AVR_ISP::verifyData: Verification error at address "));
        debugPrint2(addr + i, HEX);
        debugPrint(F(". Got: 0x"));
        debugPrint2(found, HEX);
        debugPrint(F(" Expected: 0x"));
        debugPrintln2(expected, HEX);
      }  // end of haven't shown 100 errors yet
      errors++;
    }  // end if error
  }  // end of for

}  // end of verifyData

void SFE_AVR_ISP::getFuseBytes()
{
  fuses[lowFuse] = readFuse(lowFuse);
  fuses[highFuse] = readFuse(highFuse);
  fuses[extFuse] = readFuse(extFuse);
  fuses[lockByte] = readFuse(lockByte);
  fuses[calibrationByte] = readFuse(calibrationByte);

  debugPrint(F("SFE_AVR_ISP::getFuseBytes: LFuse = 0x"));
  if (fuses[lowFuse] < 0x10) debugPrint(F("0"));
  debugPrintln2(fuses[lowFuse], HEX);
  debugPrint(F("HFuse = 0x"));
  if (fuses[highFuse] < 0x10) debugPrint(F("0"));
  debugPrintln2(fuses[highFuse], HEX);
  debugPrint(F("EFuse = 0x"));
  if (fuses[extFuse] < 0x10) debugPrint(F("0"));
  debugPrintln2(fuses[extFuse], HEX);
  debugPrint(F("Lock byte = 0x"));
  if (fuses[lockByte] < 0x10) debugPrint(F("0"));
  debugPrintln2(fuses[lockByte], HEX);
  debugPrint(F("Clock calibration = 0x"));
  if (fuses[calibrationByte] < 0x10) debugPrint(F("0"));
  debugPrintln2(fuses[calibrationByte], HEX);
}  // end of getFuseBytes

// convert two hex characters into a byte
//    returns true if error, false if OK
bool SFE_AVR_ISP::hexConv(const char * (& pStr), byte &b)
{
  if (!isxdigit (pStr [0]) || !isxdigit (pStr [1]))
  {
    debugPrint(F("SFE_AVR_ISP::hexConv: Invalid hex digits: "));
    debugPrint1(pStr [0]);
    debugPrintln1(pStr [1]);
    return true;
  } // end not hex

  b = *pStr++ - '0';
  if (b > 9)
    b -= 7;

  // high-order nybble
  b <<= 4;

  byte b1 = *pStr++ - '0';
  if (b1 > 9)
    b1 -= 7;

  b |= b1;

  return false;  // OK
}  // end of hexConv

//Reads the three byte signature from target and loads the
//global currentSignature structure with description, flash size, etc of this target.
//Returns true if a signature is found
bool SFE_AVR_ISP::getSignature()
{
  byte sig[3];
  readSignature(sig); //Reads the three byte signature from target

  debugPrint (F("SFE_AVR_ISP::getSignature: Signature = 0x"));
  for (byte i = 0 ; i < sizeof(sig) ; i++)
  {
    if (sig[i] < 0x10)
      debugPrint(F("0"));
    debugPrint2(sig[i], HEX);
  }
  debugPrintln(F(""));

  //Search for this signature from the table
  //If found, load currentSignature with info
  for (unsigned int j = 0 ; j < NUMITEMS(signatures) ; j++)
  {
    memcpy_P(&currentSignature, &signatures[j], sizeof(currentSignature));

    //Does this record match the signature we found?
    if (memcmp(sig, currentSignature.sig, sizeof(sig)) == 0)
      return (true);
  }

  memcpy(&currentSignature.sig, &sig, sizeof(sig)); // Report the actual read signature
  strcpy(currentSignature.desc, "UNKNOWN"); // Set the chip description to UNKNOWN

  return (false);
}  // end of getSignature

//Given a line from a HEX file, split text into bytes
//Check CRC. If passes, and action is write, then writes line to flash
//Returns true on error
bool SFE_AVR_ISP::processLine(const char * pLine, const byte action)
{
  if (*pLine++ != ':')
  {
    debugPrintln(F("SFE_AVR_ISP::processLine: Line does not start with ':' character."));
    return true;  // error
  }

  const int maxHexData = 40;
  byte hexBuffer[maxHexData];
  int bytesInLine = 0;

  //if (action == checkFile)
  //if (lineCount++ % 40 == 0)
  //showProgress();

  // convert entire line from ASCII into binary
  while (isxdigit(*pLine))
  {
    // can't fit?
    if (bytesInLine >= maxHexData)
    {
      debugPrintln(F("SFE_AVR_ISP::processLine: Line too long to process."));
      return true;
    } // end if too long

    if (hexConv(pLine, hexBuffer[bytesInLine++]))
      return true;
  }  // end of while

  if (bytesInLine < 5)
  {
    debugPrintln(F("SFE_AVR_ISP::processLine: Line too short."));
    return true;
  }

  // sumcheck it
  byte sumCheck = 0;
  for (int i = 0 ; i < (bytesInLine - 1) ; i++)
    sumCheck += hexBuffer[i];

  // 2's complement
  sumCheck = ~sumCheck + 1;

  // check sumcheck
  if (sumCheck != hexBuffer[bytesInLine - 1])
  {
    debugPrint(F("SFE_AVR_ISP::processLine: Sumcheck error. Expected: 0x"));
    debugPrint2(sumCheck, HEX);
    debugPrint(F(", got: "));
    debugPrintln2(hexBuffer[bytesInLine - 1], HEX);
    return true;
  }

  // length of data (eg. how much to write to memory)
  byte len = hexBuffer[0];

  // the data length should be the number of bytes, less
  //   length / address (2) / transaction type / sumcheck
  if (len != (bytesInLine - 5))
  {
    debugPrint(F("SFE_AVR_ISP::processLine: Line not expected length. Expected "));
    debugPrint2(len, DEC);
    debugPrint(F(" bytes, got "));
    debugPrint2(bytesInLine - 5, DEC);
    debugPrintln(F(" bytes."));
    return true;
  }

  // two bytes of address
  unsigned long addrH = hexBuffer[1];
  unsigned long addrL = hexBuffer[2];

  unsigned long addr = addrL | (addrH << 8);

  byte recType = hexBuffer[3];

  switch (recType)
  {
    // stuff to be written to memory
    case hexDataRecord:
      lowestAddress  = min(lowestAddress, addr + extendedAddress);
      highestAddress = max(lowestAddress, addr + extendedAddress + len - 1);
      bytesWritten += len;

      switch (action)
      {
        case checkFile:  // nothing much to do, we do the checks anyway
          break;

        case verifyFlash:
          verifyData(addr + extendedAddress, &hexBuffer[4], len);
          break;

        case writeToFlash:
          writeData(addr + extendedAddress, &hexBuffer[4], len); //Push these bytes into buffer on AVR
          break;
      } // end of switch on action
      break;

    // end of data
    case hexEndOfFile:
      gotEndOfFile = true;
      break;

    // we are setting the high-order byte of the address
    case hexExtendedSegmentAddressRecord:
      extendedAddress = ((unsigned long) hexBuffer[4]) << 12;
      break;

    // ignore these, who cares?
    case hexStartSegmentAddressRecord:
    case hexExtendedLinearAddressRecord:
    case hexStartLinearAddressRecord:
      break;

    default:
      debugPrint(F("SFE_AVR_ISP::processLine: Cannot handle record type: "));
      debugPrintln2(recType, DEC);
      return true;
  }  // end of switch on recType

  return false;
} // end of processLine

//Deal with a hex file: either write a given file, or read to the given file,
//or verify a given file
//Returns false if an error occurs
bool SFE_AVR_ISP::readHexFile(File firmwareFile, const byte action)
{
  const int maxLine = 80;
  char buffer[maxLine];
  int lineNumber = 0;

  gotEndOfFile = false;
  extendedAddress = 0;
  errors = 0;
  lowestAddress = 0xFFFFFFFF;
  highestAddress = 0;
  bytesWritten = 0;

  pagemask = ~(currentSignature.pageSize - 1);
  oldPage = NO_PAGE;

  switch (action)
  {
    case checkFile:
      debugPrintln(F("SFE_AVR_ISP::readHexFile: Checking file ..."));
      break;

    case verifyFlash:
      debugPrint(F("SFE_AVR_ISP::readHexFile: Verifying flash"));
      break;

    case writeToFlash:
      debugPrintln(F("SFE_AVR_ISP::readHexFile: Erasing chip..."));
      eraseMemory();
      debugPrint(F("SFE_AVR_ISP::readHexFile: Writing flash"));
      break;
  } // end of switch

  //Read the next set of bytes from file into our temp array
  while (firmwareFile.available())
  {
    //Read the next line from the file
    int count = 0;
    while (count < 64)
    {
      byte incoming = firmwareFile.read();

      if (incoming == '\r' || incoming == '\n')
      {
        if (firmwareFile.peek() == '\r' || firmwareFile.peek() == '\n')
        {
          firmwareFile.read(); //Dump it
        }
        break; //We've reached the end of the line
      }
      buffer[count++] = incoming;
    }

    if (count == maxLine)
    {
      debugPrint(F("SFE_AVR_ISP::readHexFile: Line "));
      debugPrint1(lineNumber);
      debugPrint(F(" too long."));
      return false; //Error
    }  // end of fail (line too long?)

    buffer[count] = '\0'; //Terminate the buffer

    lineNumber++;

    // ignore empty lines
    if (count > 1)
    {
      if (processLine(buffer, action))
      {
        debugPrint(F("SFE_AVR_ISP::readHexFile: Error in line "));
        debugPrintln1(lineNumber);
        return false; //Error
      }
    }
  }    // end of while each line

  if (!gotEndOfFile)
  {
    debugPrintln (F("SFE_AVR_ISP::readHexFile: Did not get 'end of file' record."));
    return false; //Error
  }

  switch (action)
  {
    case writeToFlash:
      // commit final page
      if (oldPage != NO_PAGE)
        commitPage(oldPage);
      debugPrintln(F(""));   // finish line of dots
      debugPrintln(F("SFE_AVR_ISP::readHexFile: Write Finished"));
      break;

    case verifyFlash:
      debugPrintln(F(""));   // finish line of dots
      if (errors == 0)
        debugPrintln(F("SFE_AVR_ISP::readHexFile: No errors found."));
      else
      {
        debugPrint(F("SFE_AVR_ISP::readHexFile: "));
        debugPrint2(errors, DEC);
        debugPrintln(F(" verification error(s)."));
        return false; //Error
      }  // end if
      break;

    case checkFile:
      debugPrintln(F("")); // finish line of dots
      debugPrint(F("SFE_AVR_ISP::readHexFile: Lowest address  = 0x"));
      debugPrintln2(lowestAddress, HEX);
      debugPrint(F("SFE_AVR_ISP::readHexFile: Highest address = 0x"));
      debugPrintln2(highestAddress, HEX);
      debugPrint(F("SFE_AVR_ISP::readHexFile: Bytes to write  = "));
      debugPrintln2(bytesWritten, DEC);
      break;
  }  // end of switch

  return true; //Success
}  // end of readHexFile

// Bit Banged SPI transfer
byte SFE_AVR_ISP::BB_SPITransfer(byte c)
{
  for (byte bit = 0 ; bit < 8 ; bit++)
  {
    // write MOSI on falling edge of previous clock
    if (c & 0x80)
      //digitalWrite(ISP_COPI, HIGH);
      //ISP_COPI_PORT |= bit(ISP_COPI_BIT);
      DIRECT_WRITE_HIGH(PIN_TO_BASEREG(ISP_COPI), PIN_TO_BITMASK(ISP_COPI));
    else
      //digitalWrite(ISP_COPI, LOW);
      //ISP_COPI_PORT &= ~bit(ISP_COPI_BIT);
      DIRECT_WRITE_LOW(PIN_TO_BASEREG(ISP_COPI), PIN_TO_BITMASK(ISP_COPI));

    c <<= 1;

    // read MISO
    //if(digitalRead(ISP_CIPO) == HIGH) c |= 1;
    //c |= (ISP_CIPO_PORT & bit(ISP_CIPO_BIT)) != 0;
    c |= DIRECT_READ(PIN_TO_BASEREG(ISP_CIPO), PIN_TO_BITMASK(ISP_CIPO)) != 0;

    // clock high
    //digitalWrite(ISP_SCK, HIGH);
    //ISP_SCK_PORT |= bit(ISP_SCK_BIT);
    DIRECT_WRITE_HIGH(PIN_TO_BASEREG(ISP_SCK), PIN_TO_BITMASK(ISP_SCK));

    // delay between rise and fall of clock
    delayMicroseconds(bitBangDelayAmount);

    // clock low
    //digitalWrite(ISP_SCK, LOW);
    //ISP_SCK_PORT &= ~bit(ISP_SCK_BIT);
    DIRECT_WRITE_LOW(PIN_TO_BASEREG(ISP_SCK), PIN_TO_BITMASK(ISP_SCK));

    // delay between rise and fall of clock
    delayMicroseconds(bitBangDelayAmount);
  }

  return c;
}  // end of BB_SPITransfer


//Enable or disable helpful debug messages
void SFE_AVR_ISP::enableDebugging(Stream &debugPort)
{
  _debugSerial = &debugPort; //Grab which port the user wants us to use for debugging
  _printDebug = true; //Should we print the commands we send? Good for debugging
}
void SFE_AVR_ISP::disableDebugging(void)
{
  _printDebug = false; //Turn off extra print statements
}

// gfvalvo's flash string helper code: https://forum.arduino.cc/index.php?topic=533118.msg3634809#msg3634809

void SFE_AVR_ISP::debugPrint(const char *line)
{
  doDebugPrint([](const char *ptr) {return *ptr;}, line);
}

void SFE_AVR_ISP::debugPrint(const __FlashStringHelper *line)
{
  doDebugPrint([](const char *ptr) {return (char) pgm_read_byte_near(ptr);}, (const char*) line);
}

void SFE_AVR_ISP::debugPrintln(const char *line)
{
  doDebugPrint([](const char *ptr) {return *ptr;}, line, true);
}

void SFE_AVR_ISP::debugPrintln(const __FlashStringHelper *line)
{
  doDebugPrint([](const char *ptr) {return (char) pgm_read_byte_near(ptr);}, (const char*) line, true);
}

void SFE_AVR_ISP::doDebugPrint(char (*funct)(const char *), const char *string, bool newLine)
{
  if (_printDebug)
  {
    char ch;

    while ((ch = funct(string++)))
    {
      _debugSerial->print(ch);
    }

    if (newLine)
    {
      _debugSerial->print(F("\r\n"));
    }
  }
}