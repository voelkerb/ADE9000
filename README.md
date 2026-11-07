# ADE9000
Library to interface with the Power Monitor Chips ADE9000 by Analog Devices.\
As these chips are meant to work with 230V, be sure to know what you are doing! Keep safe! This library just helps you to interface with the chips and gives some basic stuff on top like calibration.

## ESP32-S3 breakout wiring

The [`esp32_s3_breakout`](examples/esp32_s3_breakout/esp32_s3_breakout.ino)
example is configured for the whatnick ADE9000 breakout:

| ADE9000 header | ESP32-S3 GPIO |
|---|---:|
| SCLK | 39 |
| SS | 40 |
| MOSI | 41 |
| MISO | 42 |
| RESET | 4 |
| CF4_DREADY | 5 |
| PM0 | 6 |
| PM1 | 7 |
| GND | GND |

The MCU drives PM0 and PM1 (set the mode with `setPowerMode()`; `init()` selects
normal mode PSM0 before reset), so leave the board's PM0/PM1 jumpers open and
wire the pads to GPIO6/GPIO7. Alternatively, bridge both to `0` on the breakout
and skip `setPowerModePins()`. GPIO4-7 are ordinary GPIOs on ESP32-S3 and avoid
the USB pins, strapping pins, and flash/PSRAM pins.

## SPI speed

The default SPI clock is a conservative 10 MHz. Change it with
`ade9k.setSPIFrequency(hz)` (before or after `initSPI()`) or compile with
`-DADE9000_SPI_DEFAULT_HZ=...`. Above 15 MHz the library compensates the
one-bit MISO shift, but wiring must be short. The portable path uses only the
public Arduino `SPIClass` API (ESP32, ESP32-S3, Arduino-ESP32 2.x and 3.x). On the
classic ESP32 with Arduino-ESP32 2.x, `-DADE9000_USE_DIRECT_SPI_REGS` enables the
original register-level asynchronous burst transfer for the highest throughput;
it is not available on the S3 or Arduino-ESP32 3.x.

```C++
#include "ADE9000.h"
...
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

```