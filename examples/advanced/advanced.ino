/***************************************************
 Example file for using the ADE9000 library.
 
 License: Creative Common V1. 

 Benjamin Voelker, voelkerb@me.com
 Embedded Systems Engineer
 ****************************************************/

#include "ADE9000.h"

void IRAM_ATTR isr_adc_ready();

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

enum class Measures{VI, VI_L1, VI_L2, VI_L3, VI_RMS, PQ};

Measures measures = Measures::VI;         // The measures to send
uint32_t samplingRate = 8000;
uint32_t internalSamplingRate = 8000;
volatile unsigned long totalSamples = 0;
uint8_t leavoutSamples = 0;
bool sampling = false;

// ADE9000 Object
ADE9000 ade9k(ADE_RESET_PIN, ADE_DREADY_PIN, ADE_PM1_PIN, ADE_SPI_BUS);


void setup() {
  Serial.begin(115200);

  ade9k.initSPI(ADE_SCK, ADE_MISO, ADE_MOSI, ADE_CS);
  bool success = ade9k.init(&isr_adc_ready);
  if (!success) Serial.println("ADE9k Init Failed");


  startSampling();
}

float value[4] = {0.0};
void loop() {
  delay(1000);
}


void sopSampling() {
  ade9k.stopSampling();
  sampling = false;
}

void startSampling() {
  totalSamples = 0;
  if (samplingRate <= 8000) internalSamplingRate = 8000;
  else internalSamplingRate = 32000;
  int leavout = internalSamplingRate/samplingRate;
  leavoutSamples = leavout;
  sampling = true;
  ade9k.startSampling(internalSamplingRate);
}

/****************************************************
 * Sampling interrupt, A new ADC reading is available
 * Sample is read and put into queue
 * NOTE: Function must be small and quick
 ****************************************************/
// xQueueHandle xQueue = xQueueCreate(2000, sizeof(CurrentADC));
xQueueHandle xQueue = xQueueCreate(1000, sizeof(bool));
bool volatile IRAM_timeout = false;
TaskHandle_t xHandle = NULL;
volatile uint8_t cntLeavoutSamples = 0;
portMUX_TYPE intterruptSamplesMux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR isr_adc_ready() {
  // Skip this interrup if we want less samples
  cntLeavoutSamples++;
  if (cntLeavoutSamples < leavoutSamples) return;
  cntLeavoutSamples = 0;

  BaseType_t xHigherPriorityTaskWoken;
  bool success = false;
  BaseType_t xStatus = xQueueSendToBackFromISR( xQueue, &success, &xHigherPriorityTaskWoken );

  // check whether sending is ok or not
  if( xStatus == pdPASS ) {
  } else {
    IRAM_timeout = true;
  }

  xTaskResumeFromISR( xHandle );
}

/****************************************************
 * RTOS task for handling a new sample
 ****************************************************/
bool QueueTimeout = false;
uint32_t lastOutput = millis();
float data[7] = {0.0};
void sample_timer_task( void * parameter) {
  vTaskSuspend( NULL );  // ISR wakes up the task

  // We can only get all data from adc, if we want a single phase, skip the rest
  uint8_t offset = 0;
  if (measures == Measures::VI) offset = 0;
  else if (measures == Measures::VI_L1) offset = 0;
  else if (measures == Measures::VI_L2) offset = 2;
  else if (measures == Measures::VI_L3) offset = 4;

  CurrentADC adc;
  bool success = false;
  while(sampling){

    BaseType_t xStatus = xQueueReceive( xQueue, &success, portMAX_DELAY);

    if(xStatus == pdPASS) {
      if(IRAM_timeout) {
        Serial.println("Lost interrupt!");
        IRAM_timeout = false;
      }

      if(QueueTimeout) {
        Serial.println("Queue timeout!");
        QueueTimeout = false;
      }

      // No matter what, we need to remove the sample from the fifo     
      ade9k.readRawMeasurement(&adc);
      ade9k.convertRawMeasurement(&adc, &data[0]);

      totalSamples++;

      if (millis()-lastOutput > 1000) {
        lastOutput = millis();
        Serial.printf("V - L1: %.2fV\tL1: %.2fV\tL2: %.2fV\tL3: %.2fV\n", data[0], data[2], data[4]);
        Serial.printf("I - L1: %.2fmA\tL1: %.2fmA\tL2: %.2fmA\tL3: %.2fmA\tN: %.2fmA\n", data[1], data[3], data[5], data[6]);
      }
    } else {
      QueueTimeout = true;
    }
    if(!uxQueueMessagesWaiting(xQueue)) {
      vTaskSuspend( NULL );  // release computing time, ISR wakes up the task.
    }
  }
  vTaskDelete( NULL );
}
