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

  This example reads the signature bytes from the AVR device through the ISP pins.

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
  myISP.getSignature(); // Attempt to read the signature from the ISP device
  myISP.stopProgramming(); // Exit programming mode

  Serial.print(F("Chip signature is: 0x"));
  if (myISP.currentSignature.sig[0] < 0x10) Serial.print(F("0"));
  Serial.print(myISP.currentSignature.sig[0], HEX);
  Serial.print(F(" 0x"));
  if (myISP.currentSignature.sig[1] < 0x10) Serial.print(F("0"));
  Serial.print(myISP.currentSignature.sig[1], HEX);
  Serial.print(F(" 0x"));
  if (myISP.currentSignature.sig[2] < 0x10) Serial.print(F("0"));
  Serial.println(myISP.currentSignature.sig[2], HEX);

  Serial.print(F("The chip type is: "));
  Serial.print(myISP.currentSignature.desc);
}

void loop()
{
  // Nothing to do here
}
