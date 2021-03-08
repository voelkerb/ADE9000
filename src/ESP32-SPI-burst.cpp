/***************************************************
 SPI burst read helper

 Feel free to use the code as it is.

 Gregor Richter, Benjamin Völker (voelkerb@me.com)
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/


#include <SPI.h>
#include <driver/spi_common.h>
#include <soc/spi_struct.h>
#include <esp32-hal-spi.h>

#include "ESP32-SPI-burst.h"


struct spi_struct_t {
    spi_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};


