/***************************************************
 ESP32-S3 example for the whatnick ADE9000 breakout.

 Leave the PM0/PM1 jumpers open and wire the pads to GPIO6/GPIO7 so the
 MCU selects the power mode.
 ****************************************************/

#include "ADE9000.h"
#include "ADE9000_REGISTER.h"

#define ADE_SPI_BUS FSPI
#define ADE_RESET_PIN 4
#define ADE_DREADY_PIN 5
#define ADE_PM0_PIN 6
#define ADE_PM1_PIN 7

#define ADE_SCK 39
#define ADE_CS 40
#define ADE_MOSI 41
#define ADE_MISO 42

ADE9000 ade9k(
    ADE_RESET_PIN,
    ADE_DREADY_PIN,
    ADE_PM1_PIN,
    ADE_SPI_BUS);

void setup() {
  Serial.begin(115200);
  delay(1500);

  ade9k.setPowerModePins(ADE_PM0_PIN, ADE_PM1_PIN);
  ade9k.setPowerMode(ADE9000_PSM0_NORMAL);
  ade9k.setSPIFrequency(10000000);
  ade9k.initSPI(ADE_SCK, ADE_MISO, ADE_MOSI, ADE_CS);

  Serial.printf("ADE9000 VERSION before init: 0x%04lX\n",
                static_cast<unsigned long>(
                    ade9k.readRegister(ADDR_VERSION, 16)));

  if (!ade9k.init()) {
    Serial.println("ADE9000 initialization failed");
    return;
  }

  Serial.printf("ADE9000 initialized, RUN: 0x%04lX\n",
                static_cast<unsigned long>(
                    ade9k.readRegister(ADDR_RUN, 16)));
}

void loop() {
  delay(1000);
  Serial.printf("ADE9000 VERSION: 0x%04lX\n",
                static_cast<unsigned long>(
                    ade9k.readRegister(ADDR_VERSION, 16)));
}
