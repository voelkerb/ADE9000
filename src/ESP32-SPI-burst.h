/***************************************************
 Library implementing a simple ringbuffer.
 Functions like available, write and read are implemented

 Feel free to use the code as it is.

 Gregor Richter, Benjamin Völker (voelkerb@me.com)
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

// Unfortunately the "RX-Buffer-Full-interrupt" of the ESP-SPI does not work as
// expected, therefore the SPI bus must be accessed close to the hardware.

#ifndef ESP32_SPI_BURST_H_
#define ESP32_SPI_BURST_H_

#include <SPI.h>

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif



#endif  // ESP32_SPI_BURST_H_
