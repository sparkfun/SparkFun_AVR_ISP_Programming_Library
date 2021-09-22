/*
  AVR ISP Programming Example
  By: Paul Clark
  SparkFun Electronics
  Date: September 21st 2021
  License: MIT. Please see the license file for more information
  but you can basically do whatever you want with this code.

  Feel like supporting open source hardware?
  Buy a board from SparkFun!
  SparkFun ISP Pogo Adapter: https://www.sparkfun.com/products/11591
  SparkFun RedBoard Plus: https://www.sparkfun.com/products/18158

  This example reads the fuse bytes from the AVR device through the ISP pins.

  The ISP connection is made via standard GPIO pins and the data is written
  and read using bit-banging. You will need the dedicated SPI pins to read
  data from SD card in the later examples, so we recommend avoiding using
  the true SPI pins in this example (usually D11-D13).

  Hardware Connections:
  Attach RedBoard to computer using a USB cable.
  Connect the ISP pogo pin adapter to pins on your RedBoard:
    MISO/D12 -> D3
    SCK/D12  -> D4
    RST/D10  -> D5
    V+       -> 3.3V (or 5V but **only if** you want to program a 5V chip using 5V SPI programming signals)
    MOSI/D11 -> D6
    GND      -> GND
    
  Upload the code and open the Serial Monitor at 115200 baud.
*/

#include "SparkFun_AVR_ISP_Programming.h" // Click here to get the library: http://librarymanager/All#SparkFun_AVR_ISP_Programming

// These are the four GPIO pins that will be used for bit-banged ISP SPI. Change these if required.
const uint8_t ispCIPOpin = 3; // Use D3 to control the ISP CIPO pin
const uint8_t ispSCKpin  = 4; // Use D4 to control the ISP SCK pin
const uint8_t ispRSTpin  = 5; // Use D5 to control the ISP RST pin
const uint8_t ispCOPIpin = 6; // Use D6 to control the ISP COPI pin

SFE_AVR_ISP myISP(ispCIPOpin, ispCOPIpin, ispSCKpin, ispRSTpin); // Tell the library which pins to use

void setup()
{
  Serial.begin(115200);
  Serial.println(F("SparkFun AVR ISP Programming Example"));

  //myISP.enableDebugging(); // Uncomment this line to enable lots of helpful debug messages on Serial

  myISP.begin();

  myISP.startProgramming(); // Go into programming mode
  myISP.getFuseBytes(); // Attempt to read the fuse bytes from the ISP device
  myISP.stopProgramming(); // Exit programming mode

  Serial.print(F("Extended Fuse:    "));
  printDecHexBin(myISP.fuses[extFuse]);
  
  Serial.print(F("High Fuse:        "));
  printDecHexBin(myISP.fuses[highFuse]);
  
  Serial.print(F("Low Fuse:         "));
  printDecHexBin(myISP.fuses[lowFuse]);
  
  Serial.print(F("Lock Byte:        "));
  printDecHexBin(myISP.fuses[lockByte]);
  
  Serial.print(F("Calibration Byte: "));
  printDecHexBin(myISP.fuses[calibrationByte]);
}

void loop()
{
  // Nothing to do here
}

// Pretty-print the value in decimal, hex and binary
void printDecHexBin(uint32_t val)
{
  // Decimal
  if (val < 100) Serial.print(F(" "));
  if (val < 10) Serial.print(F(" "));
  Serial.print(val);

  // Hex
  Serial.print(F(" 0x"));
  if (val < 0x10) Serial.print(F("0"));
  Serial.print(val, HEX);
  
  // Binary
  Serial.print(F(" 0b"));
  for (int bitPos = 8; bitPos > 0; bitPos--) // For each bit
  {
    if ((val & (1 << (bitPos - 1))) > 0) // Check if bit is set. Print '1' if it is
      Serial.print(F("1"));
    else
      Serial.print(F("0")); // Else print '0'
  }
  Serial.println();
}
