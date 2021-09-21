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
  microSD Breakout: 

  This example performs a complete erase-program-verify cycle using a hex file from SD card.

  Hardware Connections:
  
  Attach RedBoard to computer using a USB cable.
  
  Connect the ISP pogo pin adapter to pins on your RedBoard:
    MISO/D12 -> D3
    SCK/D12  -> D4
    RST/D10  -> D5
    V+       -> 3.3V (or 5V but **only if** you want to program a 5V chip using 5V SPI programming signals)
    MOSI/D11 -> D6
    GND      -> GND

  Connect the SD breakout to your RedBoard:
    VCC            -> 3.3V (Note! microSD cards are not 5V compliant! You must use a level-shifter for 5V!)
    DAT3/!CS / CS  -> D10
    DAT0/SDO / DO  -> D12 (CIPO)
    CLK/SCK  / SCK -> D13 (SCK)
    CMD/SDI  / DI  -> D11 (COPI)
    GND            -> GND
    
  Upload the code and open the Serial Monitor at 115200 baud.
*/

#include "SparkFun_AVR_ISP_Programming.h" // Click here to get the library: http://librarymanager/All#SparkFun_AVR_ISP_Programming

SFE_AVR_ISP myISP;

const uint8_t ispCIPOpin = 3; // Use D3 to control the ISP CIPO pin
const uint8_t ispSCKpin  = 4; // Use D4 to control the ISP SCK pin
const uint8_t ispRSTpin  = 5; // Use D5 to control the ISP RST pin
const uint8_t ispCOPIpin = 6; // Use D6 to control the ISP COPI pin

const uint8_t microSDCSpin = 10; // Use D10 for the microSD Chip Select

const char firmwareFileName[] = "smol_Power_Board_AAA_ATtiny43U.ino.hex"; // Tell the library the name of the firmware Hex file

const char targetName[] = "ATtiny43U"; // Tell the library the device we are expecting to program

const byte newLowFuse  = 0x42; // Tell the library what we want the low fuse byte to be set to
const byte newHighFuse = 0xDF; // Tell the library what we want the high fuse byte to be set to

void setup()
{
  Serial.begin(115200);
  Serial.println(F("SparkFun AVR ISP Programming Example"));

  SPI.begin(); // Needed for SPI communication with the microSD card

  //myISP.enableDebugging(); // Uncomment this line to enable lots of helpful debug messages on Serial

  myISP.begin(ispCIPOpin, ispCOPIpin, ispSCKpin, ispRSTpin, microSDCSpin); // Tell the library which pins to use

  // Tell the library the name of the firmware file
  myISP.setFileName(firmwareFileName);

  // Tell the library which device we are expecting to program
  myISP.setTargetName(targetName);

  // Tell the library what we want the low fuse byte set to
  // (Note: setLowFuse does not actually program the fuse byte. programTarget does that.)
  myISP.setLowFuse(newLowFuse); 

  // Tell the library what we want the high fuse byte set to
  // (Note: setHighFuse does not actually program the fuse byte. programTarget does that.)
  myISP.setHighFuse(newHighFuse); 

  // Program the target device
  if (myISP.programTarget())
    Serial.print(F("Hey, it worked!"));
  else
    Serial.println(F("Programming failed! Please see the debug messages for more information."));
}

void loop()
{
  // Nothing to do here
}
