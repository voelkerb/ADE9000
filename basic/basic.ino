/***************************************************
 Example file for using the ADE9000 library.
 
 License: Creative Common V1. 

 Benjamin Voelker, voelkerb@me.com
 Embedded Systems Engineer
 ****************************************************/

#include "ADE9000.h"

// Pins for ADE9000 SPI Connection
#define ADE_SPI_BUS VSPI
#define ADE_RESET_PIN   2
// NOTE: Sharing the same pin as ERROR LED; Obacht  
#define ADE_PM1_PIN   4
// // Event Pin/Data Ready
#define ADE_DREADY_PIN    34

// Used SPI pins
#define ADE_SCK 12
#define ADE_MISO 13
#define ADE_CS 14
#define ADE_MOSI 15

// ADE9000 Object
ADE9000 ade9k(ADE_RESET_PIN, ADE_DREADY_PIN, ADE_PM1_PIN, ADE_SPI_BUS);

float values[4] = {0.0};

void setup() {
  Serial.begin(115200);

  ade9k.initSPI(ADE_SCK, ADE_MISO, ADE_MOSI, ADE_CS);
  bool success = ade9k.init();
  if (!success) Serial.println("ADE9k Init Failed");
}

void loop() {
  delay(1000);
  ade9k.readActivePower(&values[0]);
  Serial.printf("Active Power - L1: %.2fW\tL2: %.2fW\tL3: %.2fW\t\n", values[0], values[1], values[3]);
  ade9k.readCurrentRMS(&values[0]);
  Serial.printf("RMS Current - L1: %.2fA\tL2: %.2fA\tL3: %.2fA\t\n", values[0], values[1], values[3]);
  ade9k.readVoltageRMS(&values[0]);
  Serial.printf("RMS Voltage - L1: %.2fV\tL2: %.2fV\tL3: %.2fV\t\n", values[0], values[1], values[3]);
}
