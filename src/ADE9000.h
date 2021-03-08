/***************************************************
 Library to interface with the ADE9000 Power Meter

 Feel free to use the code as it is.

 Gregor Richter, Benjamin Völker (voelkerb@me.com)
 Embedded Systems
 University of Freiburg, Institute of Informatik
 ****************************************************/

#ifndef ADE9000_H_
#define ADE9000_H_

#include <SPI.h>

// Struct of an ADC reading
typedef struct{
  int32_t adc[7] = { 0 };
} CurrentADC;


/****************************************************************************************************************
 Structures and Global Variables
****************************************************************************************************************/

  struct ActivePowerRegs
 {
	int32_t ActivePowerReg_A;
	int32_t ActivePowerReg_B;
	int32_t ActivePowerReg_C;
 };
 
   struct ReactivePowerRegs
 {
	int32_t ReactivePowerReg_A;
	int32_t ReactivePowerReg_B;
	int32_t ReactivePowerReg_C;
 };
 
  struct ApparentPowerRegs
 {
	int32_t ApparentPowerReg_A;
	int32_t ApparentPowerReg_B;
	int32_t ApparentPowerReg_C;
 };

  struct VoltageRMSRegs
 {
	int32_t VoltageRMSReg_A;
	int32_t VoltageRMSReg_B;
	int32_t VoltageRMSReg_C;
 };

  struct CurrentRMSRegs
 {
	int32_t CurrentRMSReg_A;
	int32_t CurrentRMSReg_B;
	int32_t CurrentRMSReg_C;
	int32_t CurrentRMSReg_N;
 };

  struct FundActivePowerRegs
 {
	int32_t FundActivePowerReg_A;
	int32_t FundActivePowerReg_B;
	int32_t FundActivePowerReg_C;
 }; 
 
   struct FundReactivePowerRegs
 {
	int32_t FundReactivePowerReg_A;
	int32_t FundReactivePowerReg_B;
	int32_t FundReactivePowerReg_C;
 }; 
 
   struct FundApparentPowerRegs
 {
	int32_t FundApparentPowerReg_A;
	int32_t FundApparentPowerReg_B;
	int32_t FundApparentPowerReg_C;
 }; 
 
    struct FundVoltageRMSRegs
 {
	int32_t FundVoltageRMSReg_A;
	int32_t FundVoltageRMSReg_B;
	int32_t FundVoltageRMSReg_C;
 }; 
 
    struct FundCurrentRMSRegs
 {
	int32_t FundCurrentRMSReg_A;
	int32_t FundCurrentRMSReg_B;
	int32_t FundCurrentRMSReg_C;
 };



/*------Structure definition for STATUS0 register ADE9000--------------*/
/*  Row:     0                                                            */
/*  Address: 0x402                                                        */
/*  Name:    STATUS0[31:0]                                            */
/*  Read(R) Write(W) Latch(L): RW                                         */
/*  Default: 0x00                                                      */
typedef union {
  struct {
    unsigned EGYRDY : 1;
    unsigned REVAPA : 1;
    unsigned REVAPB : 1;
    unsigned REVAPC : 1;
    unsigned REVRPA : 1;
    unsigned REVRPB : 1;
    unsigned REVRPC : 1;
    unsigned REVPSUM1: 1;
    unsigned REVPSUM2: 1;
    unsigned REVPSUM3: 1;
    unsigned REVPSUM4: 1;
    unsigned CF1: 1;
    unsigned CF2: 1;
    unsigned CF3: 1;
    unsigned CF4: 1;
    unsigned DREADY: 1;
    unsigned WFB_TRIG_IRQ: 1;
    unsigned PAGE_FULL: 1;
    unsigned PWRRDY: 1;
    unsigned RMSONERDY: 1;
    unsigned RMS1012RDY: 1;
    unsigned THD_PF_RDY: 1;
    unsigned WFB_TRIG: 1;
    unsigned COH_WFB_FULL: 1;
    unsigned MISMTCH: 1;
    unsigned TEMP_RDY: 1;
  };
  struct {
    unsigned LSW: 16;
    unsigned MSW: 16;
  };
} STATUS0_Reg;


class ADE9000 {
  
  /****************************************************************************************************************
   Definitions
  ****************************************************************************************************************/
  /*Configuration registers*/
  #define ADE9000_PGA_GAIN 0x15ff    	    /*PGA@0x0000. Gain of all channels=1*/
  #define ADE9000_CONFIG0 0x00000000		/*Integrator disabled*/
  #define ADE9000_CONFIG1	0x0002			/*CF3/ZX pin outputs Zero crossing */
  #define ADE9000_CONFIG2	0x0C00			/*Default High pass corner frequency of 1.25Hz*/
  #define ADE9000_CONFIG3	0x0000			/*Peak and overcurrent detection disabled*/
  #define ADE9000_ACCMODE 0x0000			/*60Hz operation, 3P4W Wye configuration, signed accumulation*/
                      /*Clear bit 8 i.e. ACCMODE=0x00xx for 50Hz operation*/
                      /*ACCMODE=0x0x9x for 3Wire delta when phase B is used as reference*/	
  #define ADE9000_TEMP_CFG 0x000C			/*Temperature sensor enabled*/
  #define ADE9000_ZX_LP_SEL 0x001E		/*Line period and zero crossing obtained from combined signals VA,VB and VC*/	
  #define ADE9000_MASK0 0x00000001		/*Enable EGYRDY interrupt*/				
  #define ADE9000_MASK1 0x00000000		/*MASK1 interrupts disabled*/
  #define ADE9000_EVENT_MASK 0x00000000	/*Events disabled */
  #define ADE9000_VLEVEL	0x0022EA28		/*Assuming Vnom=1/2 of full scale. */
                      /*Refer Technical reference manual for detailed calculations.*/
  #define ADE9000_DICOEFF 0x00000000 		/* Set DICOEFF= 0xFFFFE000 when integrator is enabled*/

  /*Constant Definitions***/
  #define ADE90xx_FDSP 8000   			/*ADE9000 FDSP: 8000sps, ADE9078 FDSP: 4000sps*/
  #define ADE9000_RUN_ON 0x0001			/*DSP ON*/
  /*Energy Accumulation Settings*/
  #define ADE9000_EP_CFG 0x0011			/*Enable energy accumulation, accumulate samples at 8ksps*/
                      /*latch energy accumulation after EGYRDY*/
                      /*If accumulation is changed to half line cycle mode, change EGY_TIME*/
  #define ADE9000_EGY_TIME 0x1F3F 				/*Accumulate 8000 samples*/

  /*Waveform buffer Settings*/
  #define ADE9000_WFB_CFG 0x1000			/*Neutral current samples enabled, Resampled data enabled*/
                      /*Burst all channels*/
  /*Full scale Codes referred from Datasheet.Respective digital codes are produced when ADC inputs are at full scale. Donot Change. */
  #define ADE9000_RMS_FULL_SCALE_CODES  52702092
  #define ADE9000_WATT_FULL_SCALE_CODES 20694066
  #define ADE9000_RESAMPLED_FULL_SCALE_CODES  18196
  #define ADE9000_8K_FULL_SCALE_CODES  74532013
  #define ADE9000_32K_FULL_SCALE_CODES  67107786
  
  #define FULL_SCALE_VOLTAGE_V 994.7f   // volt
  #define FULL_SCALE_VOLTAGE_I 127.065f // Ampere
  #define STANDARD_VOLTAGE_GAIN 2.0 
  #define STANDARD_CURRENT_GAIN 4.0 
  
  /*
  #define ADC_TO_V_8k 0.000013345943038
  #define ADC_TO_V_32k 0.000014822423139
  #define ADC_TO_mA_8k 0.001704837892947
  #define ADC_TO_mA_32k 0.001893446462382
  #define ADC_TO_URMS 0.000026691885979
  #define ADC_TO_IRMS 0.003409675773458
  #define ADC_TO_P 0.003053811549167
  */

  public:
    // Constructor with reset dready or cf4 pin and spi bus (VSPI or HSPI)
    ADE9000(uint8_t reset_pin, uint8_t dready_pin, uint8_t pm1_pin, uint8_t spi_bus = VSPI);  

    // Destructor
    ~ADE9000(void);


    void calcConstants();

    // Call this function to init the SPI connection. Pass SPI pins if you have reconfigured them
    void initSPI(int8_t sck = -1, int8_t miso = -1, int8_t mosi = -1, int8_t cs = -1);
    
    // Configures Interrupt, SPI and reset ADE9000, config it with defaults
    // NOTE: If you have reconfigured the SPI interface, you have to call initSPI before!
    // USE IRAM_ATTR for the passed function
    // Example: void IRAM_ATTR ISR_FUNC_NAME(void) { ... }
    // call: init( ISR_FUNC_NAME );
    bool init(void (*interruptHandler)(void));
    bool init(void (*interruptHandler)(void*));
    // Init without interrupt function
    bool init();

    void setCalibration(float *values);
    // Reset the ADE9000
    void reset(void);

    // Start sampling the ADE9000
    void startSampling(int frequency);

    // Stop sampling
    void stopSampling();

    // Read all measurements and store them in the passed values array
    // NOTE: Make sure values is an array with 7 floats
    void readMeasurement(float *values);
    
    void leavoutMeasurement();

    // Read all measurements in raw format and store them in the passed adc struct
    // NOTE: Make sure that passed adc is a CurrentADC struct
    void readRawMeasurement(CurrentADC *adc);

    // Convert the raw adc measurement into floats
    // NOTE: Make sure values is an array with 7 floats
    void convertRawMeasurement(CurrentADC *adc, float *values);

    void readActivePowerRegs(ActivePowerRegs *data);
    void convertActivePowerRegs(ActivePowerRegs *data, float *values);
    void readActivePower(float *values);

    void readReactivePowerRegs(ReactivePowerRegs *data);
    void convertReactivePowerRegs(ReactivePowerRegs *data, float *values);
    void readReactivePower(float *values);

    void readApparentPowerRegs(ApparentPowerRegs *data);
    void convertApparentPowerRegs(ApparentPowerRegs *data, float *values);
    void readApparentPower(float *values);

    void readVoltageRMSRegs(VoltageRMSRegs *data);
    void convertVoltageRMSRegs(VoltageRMSRegs *data, float *values);
    void readVoltageRMS(float *values);

    void readCurrentRMSRegs(CurrentRMSRegs *data);
    void convertCurrentRMSRegs(CurrentRMSRegs *data, float *values);
    void readCurrentRMS(float *values);

    void readFundActivePowerRegs(FundActivePowerRegs *data);
    void convertFundActivePowerRegs(FundActivePowerRegs *data, float *values);
    void readFundActivePower(float *values);

    void readFundReactivePowerRegs(FundReactivePowerRegs *data);
    void convertFundReactivePowerRegs(FundReactivePowerRegs *data, float *values);
    void readFundReactivePower(float *values);

    void readFundApparentPowerRegs(FundApparentPowerRegs *data);
    void convertFundApparentPowerRegs(FundApparentPowerRegs *data, float *values);
    void readFundApparentPower(float *values);

    void readFundVoltageRMSRegs(FundVoltageRMSRegs *data);
    void convertFundVoltageRMSRegs(FundVoltageRMSRegs *data, float *values);
    void readFundVoltageRMS(float *values);

    void readFundCurrentRMSRegs(FundCurrentRMSRegs *data);
    void convertFundCurrentRMSRegs(FundCurrentRMSRegs *data, float *values);
    void readFundCurrentRMS(float *values);


    // _____________________________________________________________________________
    void readActiveEnergy(float *values);
    
    uint8_t gainI; // Current gain of all Channel A B C
    uint8_t gainV;


  private:

    float _calibration[6];
    float _adcToI_L1, _adcToI_L2, _adcToI_L3;
    float _adcToV_L1, _adcToV_L2, _adcToV_L3;
    float _adcToIRMS_L1,_adcToIRMS_L2,_adcToIRMS_L3;
    float _adcToVRMS_L1,_adcToVRMS_L2,_adcToVRMS_L3;
    float _adcToPower_L1,_adcToPower_L2,_adcToPower_L3;

    void _dataBitShift(uint8_t *data, uint16_t size);

    uint16_t _read_16(uint16_t addr);

    uint32_t _read_32(uint16_t addr);

    uint32_t SPI_Read_32(uint16_t Address);

    void _write_16(uint16_t addr, uint16_t data);

    void _write_32(uint16_t addr, uint32_t data);

    void _burst_read_en(bool enable);

    void _burst_read(uint16_t addr, uint32_t *data, uint16_t size);

    // Unfortunately the "RX-Buffer-Full-interrupt" of the ESP-SPI does not work
    // as expected, therefore the SPI bus must be accessed close to the hardware:
    // write Data on SPI out
    void _burst_write_SPI(uint8_t *tx_data, uint8_t bytes);

    // read RX Buffer. Returns "False" if the SPI is still busy.
    void _burst_read_SPI_Buffer(uint8_t *rx_data, uint8_t bytes);


  char * registerToStr(uint32_t reg);

    // For debugging print hexadecimal
    char _hex(uint8_t i);
    void _print_hex(uint32_t data);

    // Class members
    SPIClass *_spi;
    bool _spi_inited;
    
    const uint8_t _PM1_PIN;
    const uint8_t _DREADY_PIN;
    const uint8_t _RESET_PIN;
    int8_t _CS;

    int _samplingrate;
    void (*_interruptHandler)(void);
    void (*_interruptHandler2)(void*);
    bool _burst_en;


    uint8_t _burst_read_tx_array [31];
    uint8_t _burst_read_rx_array [31];

    const uint16_t addr32k = 0x5000;  // 32k Register @ addr 0x500(0)
    const uint16_t addr8k = 0x5100;  // 8k Register @ addr 0x510(0)
};

#endif  // ADE9000_H_
