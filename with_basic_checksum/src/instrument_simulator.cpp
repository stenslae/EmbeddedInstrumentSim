/* instrument_simulator.cpp
Author:  Tyler Holliday, Connor Parrott, Andrew Johnson, Emma Stensland
Date:    23 December 2024
-----------
Description
-----------
This script runs an instrument simulation used in the development of the REAL cubesat. */

/********************
Includes
*********************/
#include "instrument_driver.h"
#include "instrument_simulator.h"

/********************
Global Constants
*********************/
// Packet structure
const int baud_rate = 115200;  // baud rate

/**********************************************************************************************************************
* Function      : void setup()
* Description   : Initializes and sets up the Teensy
* Arguments     : none
**********************************************************************************************************************/
void setup() {
  // Setup serial connection
  Serial2.begin(baud_rate, SERIAL_8O1); // Data = 8 bits, Parity = odd parity, Stop bits = 1
}

/**********************************************************************************************************************
* Function      : void loop()
* Description   : Main loop of the script
* Arguments     : none
**********************************************************************************************************************/
void loop() {
  // Continually check for data input
  getData();
}
