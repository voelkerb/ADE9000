/***************************************************
 Library to interface with the ADE9000 Power Meter

 Feel free to use the code as it is.

 Gregor Richter, Benjamin Völker (voelkerb@me.com)
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#include <SPI.h>
#include <driver/spi_common.h>
#include <soc/spi_struct.h>

// #include <esp32/rom/crc.h> // crc.h des ESP32 liefter falsche Ergebnisse
#include "CRC-16-CCITT.h"  

#include "esp32-hal-spi.h"

#include "ADE9000.h"
#include "ADE9000_REGISTER.h"

#include "esp32-hal-spi.h"

#include "RTOS-ESP32-helper.h"


struct spi_struct_t {
    spi_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
};


// _____________________________________________________________________________
ADE9000::ADE9000(uint8_t reset_pin, uint8_t dready_pin, uint8_t pm1_pin, uint8_t spi_bus): 
      _PM1_PIN(pm1_pin), _DREADY_PIN(dready_pin), _RESET_PIN(reset_pin) {
  _spi = new SPIClass(spi_bus);
  _burst_en = false;
  _interruptHandler = NULL;
  _interruptHandler2= NULL;
  _samplingrate = -1;
  _spi_inited = false;

  // Init arrays with zeros
  for (uint8_t i = 0; i < 31; i++) {
    _burst_read_tx_array[i] = 0;
    _burst_read_rx_array[i] = 0;
  }

  gainI = STANDARD_CURRENT_GAIN;
  gainV = STANDARD_VOLTAGE_GAIN;

  calcConstants();

  for (uint8_t i = 0; i < 6; i++) _calibration[i] = 1.0;
}

// _____________________________________________________________________________
ADE9000::~ADE9000() {
  delete _spi;
}

void ADE9000::setCalibration(float *values) {
  for (int i = 0; i < 6; i++) _calibration[i] = values[i];
}

void ADE9000::calcConstants() {
  double _adcToI, _adcToV, _adcToIRMS, _adcToVRMS, _adcToPower;

  if (_samplingrate == 8000) {
    _adcToI = (float)((double)FULL_SCALE_VOLTAGE_I/(double)ADE9000_8K_FULL_SCALE_CODES)*1000.0; // 1000 for mA
    _adcToV = (float)((double)FULL_SCALE_VOLTAGE_V/(double)ADE9000_8K_FULL_SCALE_CODES); 
  } else {
    _adcToI = (float)((double)FULL_SCALE_VOLTAGE_I/(double)ADE9000_32K_FULL_SCALE_CODES)*1000.0; // 1000 for mA
    _adcToV = (float)((double)FULL_SCALE_VOLTAGE_V/(double)ADE9000_32K_FULL_SCALE_CODES); 
  }
  _adcToIRMS = (float)((double)FULL_SCALE_VOLTAGE_I/(double)ADE9000_RMS_FULL_SCALE_CODES*(double)sqrt(2.0))*1000.0; // 1000 for mA
  _adcToVRMS = (float)((double)FULL_SCALE_VOLTAGE_V/(double)ADE9000_RMS_FULL_SCALE_CODES*(double)sqrt(2.0)); 
  _adcToPower = (float)((double)FULL_SCALE_VOLTAGE_V*FULL_SCALE_VOLTAGE_I/((double)ADE9000_WATT_FULL_SCALE_CODES*2.0)); 
  
  // Serial.printf("_adcToI: %.10f\n", _adcToI);
  // Serial.printf("_adcToV: %.10f\n", _adcToV);
  // Serial.printf("_adcToIRMS: %.10f\n", _adcToIRMS);
  // Serial.printf("_adcToVRMS: %.10f\n", _adcToVRMS);
  // Serial.printf("_adcToPower: %.10f\n", _adcToPower);

  _adcToI = (double)_adcToI/(double)gainI;
  _adcToV = (double)_adcToV/(double)gainV;
  _adcToIRMS = (double)_adcToIRMS/(double)gainI/2.0; // TODO: where does /2.0 come from?
  _adcToVRMS = (double)_adcToVRMS/(double)gainV/2.0;
  _adcToPower = (double)_adcToPower/(double)(gainI*gainV);


  _adcToI_L1 = _adcToI*(double)_calibration[1];
  _adcToI_L2 = _adcToI*(double)_calibration[3];
  _adcToI_L3 = _adcToI*(double)_calibration[5];
  _adcToV_L1 = _adcToV*(double)_calibration[0];
  _adcToV_L2 = _adcToV*(double)_calibration[2];
  _adcToV_L3 = _adcToV*(double)_calibration[4];
  _adcToIRMS_L1 = _adcToIRMS*(double)_calibration[1];
  _adcToIRMS_L2 = _adcToIRMS*(double)_calibration[3];
  _adcToIRMS_L3 = _adcToIRMS*(double)_calibration[5];
  _adcToVRMS_L1 = _adcToVRMS*(double)_calibration[0];
  _adcToVRMS_L2 = _adcToVRMS*(double)_calibration[2];
  _adcToVRMS_L3 = _adcToVRMS*(double)_calibration[4];
  _adcToPower_L1 = _adcToPower*(double)_calibration[0]*(double)_calibration[1];
  _adcToPower_L2 = _adcToPower*(double)_calibration[2]*(double)_calibration[3];
  _adcToPower_L3 = _adcToPower*(double)_calibration[4]*(double)_calibration[5];
}


// _____________________________________________________________________________
bool ADE9000::init(void (*handleInterrupt)(void*)) {
  _interruptHandler2 = handleInterrupt;
  return init();
}

// _____________________________________________________________________________
bool ADE9000::init(void (*handleInterrupt)(void)){
  // Set member
  _interruptHandler = handleInterrupt;
  return init();
}

// _____________________________________________________________________________
bool ADE9000::init() {

  for (uint8_t i = 0; i < 6; i++) _calibration[i] = 1.0;
  calcConstants();
  // Power Mode
  // pinMode(PM0_PIN, OUTPUT);
  // pinMode(PM1_PIN, OUTPUT); // TODO: IS this standard 0? Pulldown instead?
  pinMode(_RESET_PIN, OUTPUT);
  pinMode(_PM1_PIN, OUTPUT);
  digitalWrite(_PM1_PIN, LOW);

  // Reset the ADE9000
  reset();

  // init the SPI interface
  if (!_spi_inited) initSPI();

  // Enable interrupt pin
  pinMode(_DREADY_PIN, INPUT_PULLUP);

  delay(10);

  _write_16(ADDR_PGA_GAIN,ADE9000_PGA_GAIN);     
  _write_32(ADDR_CONFIG0,ADE9000_CONFIG0); 
  _write_16(ADDR_CONFIG1,ADE9000_CONFIG1);
  _write_16(ADDR_CONFIG2,ADE9000_CONFIG2);
  _write_16(ADDR_CONFIG3,ADE9000_CONFIG3);
  _write_16(ADDR_ACCMODE,ADE9000_ACCMODE);
  _write_16(ADDR_TEMP_CFG,ADE9000_TEMP_CFG);
  _write_16(ADDR_ZX_LP_SEL,ADE9000_ZX_LP_SEL);
  _write_32(ADDR_MASK0,ADE9000_MASK0);
  _write_32(ADDR_MASK1,ADE9000_MASK1);
  _write_32(ADDR_EVENT_MASK,ADE9000_EVENT_MASK);
  _write_16(ADDR_WFB_CFG,ADE9000_WFB_CFG);
  _write_32(ADDR_VLEVEL,ADE9000_VLEVEL);
  _write_32(ADDR_DICOEFF,ADE9000_DICOEFF);
  _write_16(ADDR_EGY_TIME,ADE9000_EGY_TIME);
  _write_16(ADDR_EP_CFG,ADE9000_EP_CFG);		//Energy accumulation ON
  _write_16(ADDR_RUN,ADE9000_RUN_ON);		//DSP ON

  _write_16(ADDR_PGA_GAIN, 0x15ff); // Gain is 2 for voltage and 4 for current

  return true;
}


// _____________________________________________________________________________
void ADE9000::reset(void){
  // Remove interrupt
  detachInterrupt(digitalPinToInterrupt(_DREADY_PIN));
  // Reset sequence
  digitalWrite(_RESET_PIN, LOW);
  delay(50);
  digitalWrite(_RESET_PIN, HIGH);
  delay(1000);
}


// _____________________________________________________________________________
void ADE9000::initSPI(int8_t sck, int8_t miso, int8_t mosi, int8_t cs) {
  _spi_inited = true;
  _spi->setDataMode(SPI_MODE0);
  // _spi->setFrequency(24000000); // TODO: can we get faster?
  _spi->setFrequency(24000000); // TODO: can we get faster?

  _CS = cs;
  _spi->begin(sck, miso, mosi, _CS);
  _spi->setHwCs(true);
  
	// digitalWrite(_CS, HIGH);

}



void ADE9000::startSampling(int samplingrate) {
  _samplingrate = samplingrate;
  // Set gain
  _write_16(ADDR_PGA_GAIN, 0x15ff); // Gain is 2 for voltage and 4 for current
  // TODO: make dependent of samplingrate
  if (samplingrate <= 8000) {
    _burst_read_tx_array[0] = addr8k >> 8;    // addr [15 - 8]
    _burst_read_tx_array[1] = addr8k | 0x8;   // addr [7 - 4], read-bit[3]
    // DREADY = Sinc4 + IIR LPF output at 8 kSPS.
    _write_16(ADDR_WFB_CFG, 0x0200);
    // _write_16(0x4A0,0x0300);
  } else {
    _burst_read_tx_array[0] = addr32k >> 8;    // addr [15 - 8]
    _burst_read_tx_array[1] = addr32k | 0x8;   // addr [7 - 4], read-bit[3]
    // DREADY = Sinc4 + IIR LPF output at 32 kSPS.
    _write_16(ADDR_WFB_CFG, 0x0000);
  }

  // select which function to output on:Here DREADY
  _write_16(ADDR_CONFIG1, 0x000C);
  // enable spi burst read
  _burst_read_en(true);


  // TODO:
  if (_interruptHandler != nullptr) {
    attachInterrupt(digitalPinToInterrupt(_DREADY_PIN), _interruptHandler, RISING);
  } else if (_interruptHandler2 != nullptr) {
    Serial.printf("Init pin interrupt ESP Style\n");
    const gpio_num_t int_pin = GPIO_NUM_34;
    // Configure interrupt for low level
    gpio_config_t config = {
      .pin_bit_mask = 1ull << int_pin,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&config));
    esp_err_t err;
    
    err = gpio_set_intr_type(int_pin, GPIO_INTR_POSEDGE);
    if (err != ESP_OK) {
      Serial.printf("set intr type failed with error 0x%x \r\n", err);
    }


    err = gpio_isr_handler_add(int_pin, _interruptHandler2, (void *) nullptr);
    if (err != ESP_OK) {
      Serial.printf("handler add failed with error 0x%x \r\n", err);
    }
  }
}

void ADE9000::stopSampling() {
  // Remove interrupt
  if (_interruptHandler != nullptr) {
    detachInterrupt(_DREADY_PIN);
  } else if (_interruptHandler2 != nullptr) {
    const gpio_num_t int_pin = GPIO_NUM_34;
    gpio_isr_handler_remove(int_pin);
  }
  _burst_read_en(false);
}

void IRAM_ATTR ADE9000::leavoutMeasurement() {
  _burst_write_SPI(_burst_read_tx_array, 31);  // is this correct here?
  _burst_read_SPI_Buffer(_burst_read_rx_array, 31);
}

void IRAM_ATTR ADE9000::readRawMeasurement(CurrentADC *adc) {
  // where to read from
  _burst_write_SPI(_burst_read_tx_array, 31);  // is this correct here?
  _burst_read_SPI_Buffer(_burst_read_rx_array, 31);
  _dataBitShift(_burst_read_rx_array, 31);
  // _burst_write_SPI(_burst_read_tx_array, 31);  // is this correct here?
  for (uint8_t i = 0; i < 7; i++) {
    adc->adc[i]  = _burst_read_rx_array[ 2+i*4 ] << 24;
    adc->adc[i] |= _burst_read_rx_array[ 3+i*4 ] << 16;
    adc->adc[i] |= _burst_read_rx_array[ 4+i*4 ] << 8;
    adc->adc[i] |= _burst_read_rx_array[ 5+i*4 ];
  }
}

void ADE9000::convertRawMeasurement(CurrentADC *adc, float *values) {
  // Until here 20us passed
  // Converting takes around 1us
  values[1] = (float)(_adcToI_L1*(float)(int32_t)adc->adc[0]);
  values[3] = (float)(_adcToI_L2*(float)(int32_t)adc->adc[2]);
  values[5] = (float)(_adcToI_L3*(float)(int32_t)adc->adc[4]);
  values[6] = (float)(_adcToI_L1*(float)(int32_t)adc->adc[6]);

  values[0] = (float)(_adcToV_L1*(float)(int32_t)adc->adc[1]);
  values[2] = (float)(_adcToV_L2*(float)(int32_t)adc->adc[3]);
  values[4] = (float)(_adcToV_L3*(float)(int32_t)adc->adc[5]);
}

void IRAM_ATTR ADE9000::readMeasurement(float *values) {
  // where to read from
  // _burst_write_SPI(_burst_read_tx_array, 31);  // is this correct here?
  _burst_read_SPI_Buffer(_burst_read_rx_array, 31);
  _dataBitShift(_burst_read_rx_array, 31);
  _burst_write_SPI(_burst_read_tx_array, 31);  // is this correct here?
  CurrentADC adc;
  for (uint8_t i = 0; i < 7; i++) {
    adc.adc[i]  = _burst_read_rx_array[ 2+i*4 ] << 24;
    adc.adc[i] |= _burst_read_rx_array[ 3+i*4 ] << 16;
    adc.adc[i] |= _burst_read_rx_array[ 4+i*4 ] << 8;
    adc.adc[i] |= _burst_read_rx_array[ 5+i*4 ];
  }
  // Until here 20us passed
  // Converting takes around 1us
  convertRawMeasurement(&adc, values);
  // 8kHz is 125us, 32kHz is ca. 30us
}

// _____________________________________________________________________________
void ADE9000::_dataBitShift(uint8_t *data, uint16_t size) {
  for (uint16_t i = 0; i < size - 1; i++){
    data[i] = data[i] << 1;
    data[i] |= data[i+1] >> 7;
  }
  data[size-1] = data[size-1] << 1;
}


// _____________________________________________________________________________
uint16_t ADE9000::_read_16(uint16_t addr) {
  // wegen Performancegründen ist es so kompliziert
  uint8_t rx_array[7];
  uint8_t tx_array[7] = { 0 };

  addr = addr << 4;

  tx_array[0] = addr >> 8;    // addr [15 - 8]
  tx_array[1] = addr | 0x8;   // addr [7 - 4], read-bit[3]

  _spi->transferBytes( tx_array, rx_array, 7);

  _dataBitShift(rx_array, 7);

  uint8_t rx_data[2];
  rx_data[0] = rx_array[2];
  rx_data[1] = rx_array[3];


  uint16_t rx_crc = (rx_array[4] << 8) | rx_array[5];

  // CRC check
  if (!_burst_en) {
    if ( rx_crc != CRCCCITT(rx_data, 2, 0xffff, 0)) {
      Serial.println("\nCRC ERROR!");
      Serial.print("RX  : 0x");
      Serial.println(rx_crc , HEX);
      Serial.print("calc: 0x");
      Serial.println(CRCCCITT(rx_data, 2, 0xffff, 0), HEX);
    }
  }
  return (rx_array[2] << 8) | rx_array[3];
}


uint32_t ADE9000::SPI_Read_32(uint16_t Address)
{
	uint16_t temp_address;
	uint16_t temp_highpacket;
	uint16_t temp_lowpacket;
	uint32_t returnData;
	
	
	temp_address = (((Address << 4) & 0xFFF0)+8);

	// digitalWrite(_CS, LOW);
	_spi->transfer16(temp_address);

	temp_highpacket = _spi->transfer16(0);
  temp_lowpacket = _spi->transfer16(0);	
	
	// digitalWrite(_CS, HIGH);
	returnData = temp_highpacket << 16;
	returnData = returnData + temp_lowpacket;
	
	return returnData;

}

// _____________________________________________________________________________
uint32_t ADE9000::_read_32(uint16_t addr) {
  // wegen Performancegründen ist es so kompliziert
  uint8_t rx_array[9];
  uint8_t tx_array[9] = { 0 };

  addr = addr << 4;

  tx_array[0] = addr >> 8;    // addr [15 - 8]
  tx_array[1] = addr | 0x8;   // addr [7 - 4], read-bit[3]

  _spi->transferBytes( tx_array, rx_array, 9);

  _dataBitShift(rx_array, 9);

  uint8_t rx_data[4];
  rx_data[0] = rx_array[2];
  rx_data[1] = rx_array[3];
  rx_data[2] = rx_array[4];
  rx_data[3] = rx_array[5];

  uint16_t rx_crc = (rx_array[6] << 8) | rx_array[7];

  // CRC check
  if (!_burst_en) {
    if ( rx_crc != CRCCCITT(rx_data, 4, 0xffff, 0)) {
      Serial.println("\nCRC ERROR!");
      // Serial.print("RX  : 0x");
      // Serial.println(rx_crc , HEX);
      // Serial.print("calc: 0x");
      // Serial.println(CRCCCITT(rx_data, 4, 0xffff, 0), HEX);
    }
  }
  uint32_t data = rx_array[2] << 24;
  data |= rx_array[3] << 16;
  data |= rx_array[4] << 8;
  data |= rx_array[5];

  return data;
}


// _____________________________________________________________________________
void ADE9000::_write_16(uint16_t addr, uint16_t data){
  // wegen Performancegründen ist es so kompliziert
  uint8_t rx_array[4];
  uint8_t tx_array[4] = { 0 };

  uint16_t caddr = addr << 4;

  tx_array[0] = caddr >> 8;    // addr [15 - 8]
  tx_array[1] = caddr & ~(0x8);   // addr [7 - 4], read-bit[3]

  tx_array[2] = data >> 8;
  tx_array[3] = data;

  _spi->transferBytes( tx_array, rx_array, 4);

  // verify write operation
  uint16_t read_data = _read_16(addr);
  if(read_data != data) {
    Serial.println("Error");
    Serial.print("write: 0x");
    Serial.println(data , HEX);
    Serial.print("read:  0x");
    Serial.println(read_data, HEX);
  }
}


// _____________________________________________________________________________
void ADE9000::_write_32(uint16_t addr, uint32_t data){
  // wegen Performancegründen ist es so kompliziert
  uint8_t rx_array[6];
  uint8_t tx_array[6] = { 0 };

  uint16_t caddr = addr << 4;

  tx_array[0] = caddr >> 8;    // addr [15 - 8]
  tx_array[1] = caddr | 0x8;   // addr [7 - 4], read-bit[3]

  tx_array[2] = data >> 24;
  tx_array[3] = data >> 16;
  tx_array[4] = data >> 8;
  tx_array[5] = data;

  _spi->transferBytes( tx_array, rx_array, 6);

  // verify write operation
  uint32_t read_data = _read_32(addr);
  if(read_data != data) {
    Serial.println("Error");
    Serial.print("write: 0x");
    Serial.println(data , HEX);
    Serial.print("read:  0x");
    Serial.println(read_data, HEX);
  }
}


// _____________________________________________________________________________
void ADE9000::_burst_read_en(bool enable){
  uint16_t buffer = _read_16(CONFIG1);
  if(enable){
    _burst_en = true;
    _write_16(CONFIG1, buffer | (1 << CONFIG1_BURST_EN) );
  } else {
    _write_16(CONFIG1, buffer & ~(1 << CONFIG1_BURST_EN) );
    _burst_en = false;
  }
}


// _____________________________________________________________________________
void ADE9000::_burst_read(uint16_t addr, uint32_t *data, uint16_t size){
    // wegen Performancegründen ist es so kompliziert
  uint8_t rx_array[ 2 + (size*4) ];
  uint8_t tx_array[ 2 + (size*4) ] = { 0 };

  if (!_burst_en) {
    _burst_read_en(true);
  }

  tx_array[0] = addr >> 8;    // addr [15 - 8]
  tx_array[1] = addr | 0x8;   // addr [7 - 4], read-bit[3]

  _spi->transferBytes( tx_array, rx_array, 2 + size*4);

  for (uint8_t i = 0; i < size; i++){
    data[i] = rx_array[ 2+i*4 ] << 24;
    data[i] |= rx_array[ 3+i*4 ] << 16;
    data[i] |= rx_array[ 4+i*4 ] << 8;
    data[i] |= rx_array[ 5+i*4 ];
  }
}


// _____________________________________________________________________________
void ADE9000::_burst_write_SPI(uint8_t *tx_data, uint8_t bytes) {
  if(!_spi->bus()) {
    return;
  }
  int i;

  if(bytes > 64) {
    bytes = 64;
  }

  uint32_t words = (bytes + 3) / 4;//16 max

  uint32_t wordsBuf[16] = {0,};
  uint8_t * bytesBuf = (uint8_t *) wordsBuf;

  memcpy(bytesBuf, tx_data, bytes);//copy data to buffer

  _spi->bus()->dev->mosi_dlen.usr_mosi_dbitlen = ((bytes * 8) - 1);
  _spi->bus()->dev->miso_dlen.usr_miso_dbitlen = ((bytes * 8) - 1);

  for(i=0; i<words; i++) {
    _spi->bus()->dev->data_buf[i] = wordsBuf[i];    //copy buffer to spi fifo
  }

  _spi->bus()->dev->cmd.usr = 1;
}

// _____________________________________________________________________________
void ADE9000::_burst_read_SPI_Buffer(uint8_t *rx_data, uint8_t bytes) {
  if(!_spi->bus()) {
    return;
  }
  int i;

  if(bytes > 64) {
    bytes = 64;
  }

  uint32_t words = (bytes + 3) / 4;//16 max

  uint32_t wordsBuf[16] = {0,};
  uint8_t * bytesBuf = (uint8_t *) wordsBuf;


  // FIXME dont use "bytes", use (spi->dev->miso_dlen.usr_miso_dbitlen+1) / 8

  while(_spi->bus()->dev->cmd.usr);

  for(i=0; i<words; i++) {
      wordsBuf[i] = _spi->bus()->dev->data_buf[i];//copy spi fifo to buffer
  }
  memcpy(rx_data, bytesBuf, bytes);//copy buffer to output
}



// _____________________________________________________________________________
void ADE9000::readActivePowerRegs(ActivePowerRegs *data) {
	data->ActivePowerReg_A = int32_t (_read_32(ADDR_AWATT));
	data->ActivePowerReg_B = int32_t (_read_32(ADDR_BWATT));
	data->ActivePowerReg_C = int32_t (_read_32(ADDR_CWATT));
}

// _____________________________________________________________________________
void ADE9000::convertActivePowerRegs(ActivePowerRegs *data, float *values) {
  values[0] = _adcToPower_L1*(float)data->ActivePowerReg_A;
  values[1] = _adcToPower_L2*(float)data->ActivePowerReg_B;
  values[2] = _adcToPower_L3*(float)data->ActivePowerReg_C;
}

// _____________________________________________________________________________
void ADE9000::readActivePower(float *values) {
  ActivePowerRegs data;
  readActivePowerRegs(&data);
  convertActivePowerRegs(&data, values);
}



// _____________________________________________________________________________
void ADE9000::readReactivePowerRegs(ReactivePowerRegs *data) {
	data->ReactivePowerReg_A = int32_t (_read_32(ADDR_AVAR));
	data->ReactivePowerReg_B = int32_t (_read_32(ADDR_BVAR));
	data->ReactivePowerReg_C = int32_t (_read_32(ADDR_CVAR));	
}

// _____________________________________________________________________________
void ADE9000::convertReactivePowerRegs(ReactivePowerRegs *data, float *values) {
  values[0] = _adcToPower_L1*(float)data->ReactivePowerReg_A;
  values[1] = _adcToPower_L2*(float)data->ReactivePowerReg_B;
  values[2] = _adcToPower_L3*(float)data->ReactivePowerReg_C;
}

// _____________________________________________________________________________
void ADE9000::readReactivePower(float *values) {
  ReactivePowerRegs data;
  readReactivePowerRegs(&data);
  convertReactivePowerRegs(&data, values);
}


// _____________________________________________________________________________
void ADE9000::readApparentPowerRegs(ApparentPowerRegs *data) {
	data->ApparentPowerReg_A = int32_t (_read_32(ADDR_AVA));
	data->ApparentPowerReg_B = int32_t (_read_32(ADDR_BVA));
	data->ApparentPowerReg_C = int32_t (_read_32(ADDR_CVA));	
}

// _____________________________________________________________________________
void ADE9000::convertApparentPowerRegs(ApparentPowerRegs *data, float *values) {
  values[0] = _adcToPower_L1*(float)data->ApparentPowerReg_A;
  values[1] = _adcToPower_L2*(float)data->ApparentPowerReg_B;
  values[2] = _adcToPower_L3*(float)data->ApparentPowerReg_C;
}

// _____________________________________________________________________________
void ADE9000::readApparentPower(float *values) {
  ApparentPowerRegs data;
  readApparentPowerRegs(&data);
  convertApparentPowerRegs(&data, values);

}


// _____________________________________________________________________________
void ADE9000::readVoltageRMSRegs(VoltageRMSRegs *data) {
	data->VoltageRMSReg_A = int32_t (_read_32(ADDR_AVRMS));
	data->VoltageRMSReg_B = int32_t (_read_32(ADDR_BVRMS));
	data->VoltageRMSReg_C = int32_t (_read_32(ADDR_CVRMS));	
}

// _____________________________________________________________________________
void ADE9000::convertVoltageRMSRegs(VoltageRMSRegs *data, float *values) {
  values[0] = _adcToVRMS_L1*(float)data->VoltageRMSReg_A;
  values[1] = _adcToVRMS_L2*(float)data->VoltageRMSReg_B;
  values[2] = _adcToVRMS_L3*(float)data->VoltageRMSReg_C;
}

// _____________________________________________________________________________
void ADE9000::readVoltageRMS(float *values) {
  VoltageRMSRegs data;
  readVoltageRMSRegs(&data);
  convertVoltageRMSRegs(&data, values);
}


// _____________________________________________________________________________
void ADE9000::readCurrentRMSRegs(CurrentRMSRegs *data) {
	data->CurrentRMSReg_A = int32_t (_read_32(ADDR_AIRMS));
	data->CurrentRMSReg_B = int32_t (_read_32(ADDR_BIRMS));
	data->CurrentRMSReg_C = int32_t (_read_32(ADDR_CIRMS));
	data->CurrentRMSReg_N = int32_t (_read_32(ADDR_NIRMS));
}

// _____________________________________________________________________________
void ADE9000::convertCurrentRMSRegs(CurrentRMSRegs *data, float *values) {
  values[0] = _adcToIRMS_L1*(float)data->CurrentRMSReg_A;
  values[1] = _adcToIRMS_L2*(float)data->CurrentRMSReg_B;
  values[2] = _adcToIRMS_L3*(float)data->CurrentRMSReg_C;
  // TODO: this is nasty....
  values[3] = _adcToIRMS_L1*(float)data->CurrentRMSReg_N;
}

// _____________________________________________________________________________
void ADE9000::readCurrentRMS(float *values) {
  CurrentRMSRegs data;
  readCurrentRMSRegs(&data);
  convertCurrentRMSRegs(&data, values);
}


// _____________________________________________________________________________
void ADE9000::readFundActivePowerRegs(FundActivePowerRegs *data) {
	data->FundActivePowerReg_A = int32_t (_read_32(ADDR_AFWATT));
	data->FundActivePowerReg_B = int32_t (_read_32(ADDR_BFWATT));
	data->FundActivePowerReg_C = int32_t (_read_32(ADDR_CFWATT));	
}

// _____________________________________________________________________________
void ADE9000::convertFundActivePowerRegs(FundActivePowerRegs *data, float *values) {
  values[0] = _adcToPower_L1*(float)data->FundActivePowerReg_A;
  values[1] = _adcToPower_L2*(float)data->FundActivePowerReg_B;
  values[2] = _adcToPower_L3*(float)data->FundActivePowerReg_C;
}

// _____________________________________________________________________________
void ADE9000::readFundActivePower(float *values) {
  FundActivePowerRegs data;
  readFundActivePowerRegs(&data);
  convertFundActivePowerRegs(&data, values);
}


// _____________________________________________________________________________
void ADE9000::readFundReactivePowerRegs(FundReactivePowerRegs *data) {
	data->FundReactivePowerReg_A = int32_t (_read_32(ADDR_AFVAR));
	data->FundReactivePowerReg_B = int32_t (_read_32(ADDR_BFVAR));
	data->FundReactivePowerReg_C = int32_t (_read_32(ADDR_CFVAR));	
}

// _____________________________________________________________________________
void ADE9000::convertFundReactivePowerRegs(FundReactivePowerRegs *data, float *values) {
  values[0] = _adcToPower_L1*(float)data->FundReactivePowerReg_A;
  values[1] = _adcToPower_L2*(float)data->FundReactivePowerReg_B;
  values[2] = _adcToPower_L3*(float)data->FundReactivePowerReg_C;
}

// _____________________________________________________________________________
void ADE9000::readFundReactivePower(float *values) {
  FundReactivePowerRegs data;
  readFundReactivePowerRegs(&data);
  convertFundReactivePowerRegs(&data, values);
}


// _____________________________________________________________________________
void ADE9000::readFundApparentPowerRegs(FundApparentPowerRegs *data) {
	data->FundApparentPowerReg_A = int32_t (_read_32(ADDR_AFVA));
	data->FundApparentPowerReg_B = int32_t (_read_32(ADDR_BFVA));
	data->FundApparentPowerReg_C = int32_t (_read_32(ADDR_CFVA));	
}

// _____________________________________________________________________________
void ADE9000::convertFundApparentPowerRegs(FundApparentPowerRegs *data, float *values) {
  values[0] = _adcToPower_L1*(float)data->FundApparentPowerReg_A;
  values[1] = _adcToPower_L2*(float)data->FundApparentPowerReg_B;
  values[2] = _adcToPower_L3*(float)data->FundApparentPowerReg_C;
}

// _____________________________________________________________________________
void ADE9000::readFundApparentPower(float *values) {
  FundApparentPowerRegs data;
  readFundApparentPowerRegs(&data);
  convertFundApparentPowerRegs(&data, values);
}


// _____________________________________________________________________________
void ADE9000::readFundVoltageRMSRegs(FundVoltageRMSRegs *data) {
	data->FundVoltageRMSReg_A = int32_t (_read_32(ADDR_AVFRMS));
	data->FundVoltageRMSReg_B = int32_t (_read_32(ADDR_BVFRMS));
	data->FundVoltageRMSReg_C = int32_t (_read_32(ADDR_CVFRMS));	
}

// _____________________________________________________________________________
void ADE9000::convertFundVoltageRMSRegs(FundVoltageRMSRegs *data, float *values) {
  values[0] = _adcToVRMS_L1*(float)data->FundVoltageRMSReg_A;
  values[1] = _adcToVRMS_L2*(float)data->FundVoltageRMSReg_B;
  values[2] = _adcToVRMS_L3*(float)data->FundVoltageRMSReg_C;
}

// _____________________________________________________________________________
void ADE9000::readFundVoltageRMS(float *values) {
  FundVoltageRMSRegs data;
  readFundVoltageRMSRegs(&data);
  convertFundVoltageRMSRegs(&data, values);
}

// _____________________________________________________________________________
void ADE9000::readFundCurrentRMSRegs(FundCurrentRMSRegs *data) {
	data->FundCurrentRMSReg_A = int32_t (_read_32(ADDR_AIFRMS));
	data->FundCurrentRMSReg_B = int32_t (_read_32(ADDR_BIFRMS));
	data->FundCurrentRMSReg_C = int32_t (_read_32(ADDR_CIFRMS));	
}

// _____________________________________________________________________________
void ADE9000::convertFundCurrentRMSRegs(FundCurrentRMSRegs *data, float *values) {
  values[0] = _adcToIRMS_L1*(float)data->FundCurrentRMSReg_A;
  values[1] = _adcToIRMS_L2*(float)data->FundCurrentRMSReg_B;
  values[2] = _adcToIRMS_L3*(float)data->FundCurrentRMSReg_C;
}

// _____________________________________________________________________________
void ADE9000::readFundCurrentRMS(float *values) {
  FundCurrentRMSRegs data;
  readFundCurrentRMSRegs(&data);
  convertFundCurrentRMSRegs(&data, values);
}

float energyA, energyB, energyC = 0.0;

// _____________________________________________________________________________
void ADE9000::readActiveEnergy(float *values) {
  uint32_t status0 = (uint32_t)_read_32(ADDR_STATUS0);

  uint32_t high = uint32_t (_read_32(ADDR_AWATTHR_HI));
  uint32_t low = uint32_t (_read_32(ADDR_AWATTHR_LO));
                           

  uint64_t test = (uint64_t)((uint64_t)(high & 0x1fffffff) << 32);
  test = test | ((uint64_t)(uint64_t)(low & 0x00001fff) << 19);
  test = test >> 19;


  uint32_t frame[2];
  frame[0] = (test & 0x00000000ffffffff);
  frame[1] = (test & 0xffffffff00000000) >> 32;

  // To adc/second
  test = test/8000;
  // To Watt/second
  float watt = test*_adcToPower_L1;
  // To Watt/hour
  watt = watt*3600.0;
  // To kWatt/hour
  watt = watt/1000.0;
}

char regStr[70] = {'\0'};
char * ADE9000::registerToStr(uint32_t reg) {
  uint8_t frame[4];
  frame[0] = (reg & 0x000000ff);
  frame[1] = (reg & 0x0000ff00) >> 8;
  frame[2] = (reg & 0x00ff0000) >> 16;
  frame[3] = (reg & 0xff000000) >> 24;

  sprintf(regStr, " ");
  int idx = 0;
  for (int8_t i = 3; i >= 0; i--) {
    idx += sprintf(&regStr[idx], "%i ", (int)((frame[i] & 0x80) > 0));
    idx += sprintf(&regStr[idx], "%i ", (int)((frame[i] & 0x40) > 0));
    idx += sprintf(&regStr[idx], "%i ", (int)((frame[i] & 0x20) > 0));
    idx += sprintf(&regStr[idx], "%i ", (int)((frame[i] & 0x10) > 0));
    idx += sprintf(&regStr[idx], "%i ", (int)((frame[i] & 0x08) > 0));
    idx += sprintf(&regStr[idx], "%i ", (int)((frame[i] & 0x04) > 0));
    idx += sprintf(&regStr[idx], "%i ", (int)((frame[i] & 0x02) > 0));
    idx += sprintf(&regStr[idx], "%i ", (int)((frame[i] & 0x01) > 0));
  }
  return &regStr[0];
}

char ADE9000::_hex(uint8_t i) {
  i = i & 0xf;
  if(i <= 9) {
    return i + '0';
  } else if (i <= 0xf) {
    return i - 10 + 'A';
  }
  return '!';
}

void ADE9000::_print_hex(uint32_t data) {
  Serial.print(" ");
  char str[9];
  str[8] = 0;

  str[7] = _hex(data);
  data = data >> 4;
  str[6] = _hex(data);
  data = data >> 4;
  str[5] = _hex(data);
  data = data >> 4;
  str[4] = _hex(data);
  data = data >> 4;
  str[3] = _hex(data);
  data = data >> 4;
  str[2] = _hex(data);
  data = data >> 4;
  str[1] = _hex(data);
  data = data >> 4;
  str[0] = _hex(data);
  Serial.print(str);
}

