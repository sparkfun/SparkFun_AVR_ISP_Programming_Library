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
  microSD Breakout:  https://www.sparkfun.com/products/544
  microSD Breakout - Level Shifting:  https://www.sparkfun.com/products/13743

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
  Note! SD cards are not 5V compliant! You must use a level-shifter for 5V: https://www.sparkfun.com/products/13743
    VCC            -> 3.3V (See note above)
    DAT3/CS  / CS  -> D10
    DAT0/SDO / DO  -> SPI CIPO (D12)
    CLK/SCK  / SCK -> SPI SCK (D13)
    CMD/SDI  / DI  -> SPI COPI (D11)
    GND            -> GND
    
  Upload the code and open the Serial Monitor at 115200 baud.
*/

#include "SparkFun_AVR_ISP_Programming.h" // Click here to get the library: http://librarymanager/All#SparkFun_AVR_ISP_Programming

// These are the four GPIO pins that will be used for bit-banged ISP SPI. Change these if required.
const uint8_t ispCIPOpin = 3; // Use D3 to control the ISP CIPO pin
const uint8_t ispSCKpin  = 4; // Use D4 to control the ISP SCK pin
const uint8_t ispRSTpin  = 5; // Use D5 to control the ISP RST pin
const uint8_t ispCOPIpin = 6; // Use D6 to control the ISP COPI pin

// This is the GPIO pin which will be used as the microSD Chip Select. Change this if required.
const uint8_t microSDCSpin = 10; // Use D10 for the microSD Chip Select

SFE_AVR_ISP myISP(ispCIPOpin, ispCOPIpin, ispSCKpin, ispRSTpin); // Tell the library which pins to use

void setup()
{
  Serial.begin(115200);
  Serial.println(F("SparkFun AVR ISP Programming Example"));

  SPI.begin(); // Needed for SPI communication with the microSD card

  myISP.enableDebugging(); // Uncomment this line to enable lots of helpful debug messages on Serial

  myISP.begin(microSDCSpin); // Tell the library which chip select pin to use

  // Tell the library the name of the firmware file
  // Note: the filename must be in 8.3 format
  char fileName[] = "smolAAA1.hex";
  myISP.setFileName(fileName);

  // Tell the library which device we are expecting to program
  char targetName[] = "ATtiny43U";
  myISP.setTargetName(targetName);

  // Tell the library what we want the low fuse byte set to
  // (Note: setLowFuse does not actually program the fuse byte. programTarget does that.)
  myISP.setLowFuse(0x62); 

  // Tell the library what we want the high fuse byte set to
  // (Note: setHighFuse does not actually program the fuse byte. programTarget does that.)
  myISP.setHighFuse(0xDF); 

  // Optional: tell the library what we want the extended fuse byte set to
  // (Note: setExtendedFuse does not actually program the fuse byte. programTarget does that.)
  myISP.setExtendedFuse(0xFF);

  //Tell the library what clock speed to use during programming
  //The max programming speed is 1/8 of the target's oscillator speed
  //If target is internal 1MHz, this is 125kHz
  //If target is internal 8MHz, this is 1MHz
  //The fastest the bit-bang code can go is 500kHz. Selecting a speed higher than 500kHz will deliver 500kHz.
  myISP.setProgrammingClockSpeed(125000); //Set the clock speed to 125kHz

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
