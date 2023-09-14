#ifndef MAX326_H_
#define MAX326_H_

#include <stdbool.h>

#include "i2c.h"
#include "gpio.h"

#include "hal_utils.h"

#define MAX326_MAX_BUFSIZE			1000    // bytes
#define MAX326_TIM_RES				100		// milliseconds

#define MAX326_DISABLE              false
#define MAX326_ENABLE               true

#define MAX326_MODE_ONE            0x01
#define MAX326_MODE_TWO            0x02

#define WRITE_FIFO_INBYTE  			0x04


#define APP_MODE               		0x00
#define BOOTLOADER_MODE        		0x08
#define NO_WRITE               		0x00

#define CONFIGURATION_REGISTER 		0x0A
#define PULSE_MASK             		0xFC
#define READ_PULSE_MASK        		0x03
#define SAMP_MASK              		0xE3
#define READ_SAMP_MASK         		0x1C
#define ADC_MASK               		0x9F
#define READ_ADC_MASK          		0x60

#define MAX326_MAXFAST_ARRSIZE        	6  // Number of bytes....
#define MAX326_MAXFAST_EXT_ARRSIZE	11
#define MAX326_MAX30101_LED_ARRAY          12 // 4 values of 24 bit (3 byte) LED values
#define MAX326_MAXFAST_SENS_ARRSIZE	18
#define MAX326_MAXFAST_SENS_EXT_ARRSIZE	23

#define SET_FORMAT             		0x00
#define READ_FORMAT            		0x01 // Index Byte under Family Byte: READ_OUTPUT_MODE (0x11)
#define WRITE_SET_THRESHOLD    		0x01 //Index Byte for WRITE_INPUT(0x14)
#define WRITE_EXTERNAL_TO_FIFO 		0x00

#define MAX326_I2C_WADDR			0xAA
#define MAX326_I2C_RADDR			0xAB


// The family register bytes are the larger umbrella for all the Index and
// Write Bytes listed below. You can not reference a nestled byte without first
// referencing it's larger category: Family Register Byte.
#define MAX326_FB_HUBSTATUS				0x00
#define MAX326_FB_S_DEVMODE				0x01
#define MAX326_FB_R_DEVMODE				0x02
#define MAX326_FB_OUTMODE  				0x10
#define MAX326_FB_R_OUTMODE				0x11
#define MAX326_FB_R_DATA_OUT				0x12
#define MAX326_FB_R_DATA_IN				0x13
#define MAX326_FB_W_INPUT					0x14
#define MAX326_FB_W_REGISTER				0x40
#define MAX326_FB_R_REGISTER				0x41
#define MAX326_FB_R_ATTRS_AFE				0x42
#define MAX326_FB_DUMP_REGISTERS			0x43
#define MAX326_FB_EN_SENSOR				0x44
#define MAX326_FB_R_SENSOR_MODE			0x45
#define MAX326_FB_CHANGE_ALGCONF			0x50
#define MAX326_FB_R_ALGOCONF				0x51
#define MAX326_FB_EN_ALGORITHM			0x52
#define MAX326_FB_BL_FLASH 				0x80
#define MAX326_FB_BL_INFO					0x81
#define MAX326_FB_IDENTITY				0xFF

// Index Bytes (FAMILY)
// (R_DATA_OUT)
#define MAX326_IB_NSAMPLES				0x00
#define MAX326_IB_R_DATA					0x01
// (R_DATA_IN)
#define MAX326_IB_SAMPLESIZE				0x00
#define MAX326_IB_R_DATAIN				0x01
#define MAX326_IB_R_SENSDATA 				0x02	// For external accelerometer
#define MAX326_IB_R_NSAMPLES_IN			0x03	// For external accelerometer
#define MAX326_IB_R_NSAMPLES_SENS			0x04
// (W_REGISTER)
#define MAX326_IB_W_MAX30101  			0x03
#define MAX326_IB_W_ACCEL					0x04
// (R_REGISTER)
#define MAX326_IB_READ_MAX30101  			0x03
#define MAX326_IB_READ_ACCEL				0x04
// (R_ATTRS_AFE)
#define MAX326_IB_RETR_AFE_MAX30101  		0x03
#define MAX326_IB_RETR_AFE_ACCEL
// (DUMP_REGISTERS)
#define MAX326_IB_DUMP_REG_MAX30101  		0x03
#define MAX326_IB_DUMP_REG_ACCEL
// (EN_SENSOR)
#define MAX326_IB_EN_MAX30101  			0x03
#define MAX326_IB_EN_ACCEL
// (R_SENSOR_MODE)
#define MAX326_IB_R_EN_MAX30101 			0x03
#define MAX326_IB_R_EN_ACCEL
// (CHANGE_ALGCONF)
#define MAX326_IB_S_TARG_PERC  			0x00
#define MAX326_IB_S_STEP_SIZE  			0x00
#define MAX326_IB_S_SENSITIVITY  			0x00
#define MAX326_IB_S_AVG_SAMPLES  			0x00
#define MAX326_IB_S_POX_COEF  			0x02
#define MAX326_IB_BPT_CONF  				0x04
// (R_ALGCONF)
#define MAX326_IB_R_AGC_PERC  			0x00
#define MAX326_IB_R_AGC_STEP_SIZE  		0x00
#define MAX326_IR_R_AGC_SENSITIVITY  		0x00
#define MAX326_IR_R_AGC_NSAMPLES  		0x00
#define MAX326_IR_R_MAX_FAST_COEF  		0x02
// (EN_ALGORITHM)
#define MAX326_IB_EN_AGC_ALGO  			0x00
#define MAX326_IB_EN_WHRM_ALGO  			0x02
// (BL_FLASH)
#define MAX326_IB_S_INIT_VECTOR_BYTES 	0x00
#define MAX326_IB_S_AUTH_BYTES			0x01
#define MAX326_IB_S_NUM_PAGES				0x02
#define MAX326_IB_ERASE_FLASH				0x03
#define MAX326_IB_SEND_PAGE_VALUE			0x04
// (BL_INFO)
#define MAX326_IB_BL_VERS					0x00
#define MAX326_IB_PAGE_SIZE				0x01
// (IDENTITY)
#define MAX326_IB_R_MCU_TYPE				0x00
#define MAX326_IB_R_SENS_HUB_VERS			0x03
#define MAX326_IB_R_ALGO_VERS				0x07

// === WRITE BYTES (FAMILY, INDEX) ===
// DEVMODE (S_DEVMODE, 0x00)
#define MAX326_WB_EXIT_BL					0x00
#define MAX326_WB_EXIT_RST				0x02
#define MAX326_WB_ENTER_BL				0x08
// OUTPUT (OUTPUT_MODE, SET_FORMAT)
#define MAX326_WB_PAUSE					0x00
#define MAX326_WB_SENSDATA				0x01
#define MAX326_WB_ALGDATA					0x02
#define MAX326_WB_SENS_ALG				0x03
#define MAX326_WB_PAUSE_TWO				0x04
#define MAX326_WB_SENS_COUNTER			0x05
#define MAX326_WB_ALG_COUNTER				0x06
#define MAX326_WB_SENS_ALG_COUNTER		0x07
// ALG_AGC (CHANGE_ALGCONF, S_TARG_PERC)
#define MAX326_WB_AGC_GAIN_ID  			0x00
#define MAX326_WB_AGC_STEP_SIZE_ID		0x01
#define MAX326_WB_AGC_SENSITIVITY_ID		0x02
#define MAX326_WB_AGC_NUM_SAMPID			0x03
#define MAX326_WB_MAXMFAST_COEFID			0x0B
// ALG_BPT (CHANGE_ALGCONF, S_TAG_PERC)
#define MAX326_WB_BPT_MEDICATION  		0x00
#define MAX326_WB_SYSTOLIC_VALUE			0x01
#define MAX326_WB_DIASTOLIC_VALUE			0x02
#define MAX326_WB_BPT_CALIB_DATA			0x03
#define MAX326_WB_AGC_SPO2_COEFS  			0x0B
#define MAX326_WB_PATIENT_RESTING			0x05
// ALG_ACG (R_ALGCONF, R_ALG - AGC)
#define MAX326_WB_R_AGC_PERC_ID  			0x00
#define MAX326_WB_R_AGC_STEP_SIZE_ID		0x01
#define MAX326_WB_R_AGC_SENSITIVITY_ID	0x02
#define MAX326_WB_R_AGC_NSAMPLES_ID		0x03
#define MAX326_WB_R_MAXFAST_COEF_ID  		0x0B

#define MAX326_TIMEOUT(cnt, timeout) ({\
	uint32_t ticks = (timeout) / MAX326_TIM_RES;\
	(cnt) % ticks == 0;\
})

#define str(x) #x
#define xstr(x) str(x)

// Status Bytes are communicated back after every I-squared-C transmission and
// are indicators of success or failure of the previous transmission.
typedef enum MAX326_Status {
	MAX326_SB_SUCCESS 					= 0x00,
	MAX326_SB_ILLEGAL_FBCB,
	MAX326_SB_NOTIMPL_FUNC,
	MAX326_SB_ERR_NUM_FB,
	MAX326_SB_ILLEGAL_CONF,
	MAX326_SB_ERR_MODE,
	MAX326_SB_ERR_GENERAL 				= 0x80,
	MAX326_SB_ERR_CHECKSUM,
	MAX326_SB_ERR_AUTH,
	MAX326_SB_ERR_APP,
	MAX326_SB_INCORR_PARAM 				= 0xEE,
	MAX326_SB_DEV_BUSY 					= 0xFE,
	MAX326_SB_ERR_UNKNOWN 				= 0xFF

} MAX326_Status;

//Returns 0x00 for the
// MAX32625 and 0x01 for the MAX32660/MAX32664.
typedef enum MAX326_MCUType {
	MAX326_MCU_25							= 0x00,
	MAX326_MCU_60_64
} MAX326_MCUType;


typedef enum MAX326_MeasureStatus {
	MAX326_MS_SUCCESS = 0x00,
	MAX326_MS_NOTREADY,
	MAX326_MS_OBJECT,
	MAX326_MS_FINGER

} MAX326_MeasureStatus;

typedef enum MAX326_Pipeline {
	MAX326_PL_NONE = 0x00,
	MAX326_PL_INIT,
	MAX326_PL_CONFBPM,
	MAX326_PL_MEASURE,

} MAX326_Pipeline;

typedef enum MAX326_Operation {
	MAX326_OP_NONE = 0x00,
	MAX326_OP_DELAY,
	MAX326_OP_SET_OUTPUT_MODE,
	MAX326_OP_SET_FIFO_THRES,
	MAX326_OP_AGC_ALGO_CONTROL,
	MAX326_OP_MAX30101_CONTROL,
	MAX326_OP_MAXFAST_CONTROL,
	MAX326_OP_READ_ALGO_SAMPLES,
	MAX326_OP_READ_SHVERSION

} MAX326_Operation;

typedef enum MAX326_State {
	MAX326_ST_UNINIT = 0x00,
	MAX326_ST_IDLE,
	MAX326_ST_BPM_READY,
	MAX326_ST_OPERATION
} MAX326_State;

typedef enum MAX326_BytesFormat {
	MAX326_BF_8 = 0x00,
	MAX326_BF_16,
	MAX326_BF_32
} MAX326_BytesFormat;

typedef enum MAX326_TransType {
	MAX326_TT_READ = 0x00,
	MAX326_TT_WRITE,
	MAX326_TT_ENABLE
} MAX326_TransType;

typedef enum MAX326_Interrupt {
	MAX326_IT_NONE = 0x00,
	MAX326_IT_INIT,
	MAX326_IT_CONFBPM,
	MAX326_IT_OUTPUT
} MAX326_Interrupt;

typedef enum MAX326_MedicationFlag {
	MAX326_MF_NOMED = 0x00,
	MAX326_MF_ISMED = 0x01
} MAX326_MedicationFlag;

typedef enum MAX326_RestingFlag {
	MAX326_RF_NOREST = 0x00,
	MAX326_RF_ISREST = 0x01
} MAX326_RestingFlag;

typedef struct MAX326_Output {

	uint32_t IRLed;
	uint32_t RedLed;
	uint16_t HeartRate; // LSB = 0.1bpm
	uint8_t Confidence; // 0-100% LSB = 1%
	uint16_t Oxygen; // 0-100% LSB = 1%
	MAX326_MeasureStatus Status; // 0: Success, 1: Not Ready, 2: Object Detectected, 3: Finger Detected
	float RValue;      // -- Algorithm Mode 2 vv
	int8_t ExtStatus;   // --
	uint8_t ReserveOne;  // --
	uint8_t ReserveTwo; // -- Algorithm Mode 2 ^^

} MAX326_Output;

typedef struct MAX326_Version {
	// 3 bytes total
	uint8_t Major;
	uint8_t Minor;
	uint8_t Revision;

} MAX326_Version;

typedef struct MAX326_SensorAttr {

	uint8_t ByteWord;
	uint8_t AvailRegisters;

} MAX326_SensorAttr;

typedef struct MAX326_SPO2Coef {
	int32_t A;
	int32_t B;
	int32_t C;
} MAX326_SPO2Coef;

typedef struct MAX326_Transaction {
	MAX326_TransType Type;

	uint8_t FamilyByte;
	uint8_t IndexByte;
	uint8_t WriteByte;

	uint8_t* DataBytes8;
	uint16_t* DataBytes16;
	int32_t* DataBytes32;

	bool HasWriteByte;
	MAX326_BytesFormat DataBytesFormat;
	size_t DataBytesNum;

} MAX326_Transaction;

typedef struct MAX326_InitStruct {
	I2C_HandleTypeDef* I2CHandle;
	bool I2CUseDMA;
	GPIO_Line* ResetLine;
	GPIO_Line* MFIOLine;
	IRQn_Type IRQLine;			/*!< must be an EXTI line, control up to developer */
	uint8_t I2CSetupAddress;
	uint8_t I2CReadAddress;
	uint8_t I2CWriteAddress;

} MAX326_InitStruct;

typedef struct MAX326_Handle {

	MAX326_InitStruct Init;
	MAX326_Version Version;
	MAX326_Output Output;
	bool IsOutputReady;

	MAX326_Status Status;
	MAX326_Interrupt Interrupt;

	uint8_t I2CTxBuffer[MAX326_MAX_BUFSIZE];
	size_t I2CTxBufferSize;
	uint8_t I2CRxBuffer[MAX326_MAX_BUFSIZE];
	size_t I2CRxBufferSize;

	MAX326_State CurrState;
	MAX326_Pipeline CurrPipeline;
	MAX326_Operation CurrOperation;
	MAX326_Transaction CurrTransaction;

	uint32_t Counter;

	// raw data
	uint8_t BpmArr[MAX326_MAXFAST_EXT_ARRSIZE];
	uint8_t SensArr[MAX326_MAX30101_LED_ARRAY];
	uint8_t BpmSenArr[MAX326_MAXFAST_SENS_EXT_ARRSIZE];

	uint32_t WriteCoefArr[3];

	uint8_t UserSelectedMode;
	uint8_t SampleRate;

	void (*I2CTransmit)(struct MAX326_Handle* handle);
	void (*I2CReceive)(struct MAX326_Handle* handle);
} MAX326_Handle;

// Constructor ----------
MAX326_Handle* MAX326_Create(MAX326_InitStruct init);

// Callbacks ------------
void MAX326_TimerCallback(MAX326_Handle* handle);
void MAX326_I2CTxCallback(MAX326_Handle* handle);
void MAX326_I2CRxCallback(MAX326_Handle* handle);

// Functions ------------

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function initializes the sensor. To place the MAX32664 into
// application mode, the MFIO pin must be pulled HIGH while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in application mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x00 which is the byte indicating
// which mode the IC is in.
void MAX326_Init(MAX326_Handle *handle);

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function puts the MAX32664 into bootloader mode. To place the MAX32664 into
// bootloader mode, the MFIO pin must be pulled LOW while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in bootloader mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x08 which is the byte indicating
// that the board is in bootloader mode.
MAX326_Status MAX326_BeginBootloader(MAX326_Handle *handle);

// Family Byte: HUB_STATUS (0x00), Index Byte: 0x00, No Write Byte.
// The following function checks the status of the FIFO.
MAX326_Status MAX326_ReadSensorHubStatus(MAX326_Handle *handle);

// Family Byte: SET_DEVICE_MODE (0x01), Index Byte: 0x01, Write Byte: 0x00
// The following function is an alternate way to set the mode of the of
// MAX32664. It can take three parameters: Enter and Exit Bootloader Mode, as
// well as reset.
// INCOMPLETE
MAX326_Status MAX326_SetOperatingMode(MAX326_Handle *handle, uint8_t);

// This function sets very basic settings to get sensor and biometric data.
// The biometric data includes data about heartrate, the confidence
// level, SpO2 levels, and whether the sensor has detected a finger or not.
void MAX326_ConfigBpm(MAX326_Handle *handle, uint8_t);

// This function sets very basic settings to get LED count values from the MAX30101.
// Sensor data includes 24 bit LED values for the three LED channels: Red, IR,
// and Green.
MAX326_Status MAX326_ConfigSensor(MAX326_Handle *handle);

// This function sets very basic settings to get sensor and biometric data.
// Sensor data includes 24 bit LED values for the two LED channels: Red and IR.
// The biometric data includes data about heartrate, the confidence
// level, SpO2 levels, and whether the sensor has detected a finger or not.
// Of note, the number of samples is set to one.
MAX326_Status MAX326_ConfigSensorBpm(MAX326_Handle *handle, uint8_t);

// This function takes the 8 bytes from the FIFO buffer related to the wrist
// heart rate algortihm: heart rate (uint16_t), confidence (uint8_t) , SpO2 (uint16_t),
// and the finger detected status (uint8_t). Note that the the algorithm is stated as
// "wrist" though the sensor only works with the finger. The data is loaded
// into the whrmFifo and returned.
void MAX326_ReadBpm(MAX326_Handle *handle);

// This function takes 9 bytes of LED values from the MAX30101 associated with
// the RED, IR, and GREEN LEDs. In addition it gets the 8 bytes from the FIFO buffer
// related to the wrist heart rate algortihm: heart rate (uint16_t), confidence (uint8_t),
// SpO2 (uint16_t), and the finger detected status (uint8_t). Note that the the algorithm
// is stated as "wrist" though the sensor only works with the finger. The data is loaded
// into the whrmFifo and returned.
MAX326_Output MAX326_ReadSensor(MAX326_Handle *handle);

// This function takes the information of both the LED value and the biometric
// data from the MAX32664's FIFO. In essence it combines the two functions
// above into a single function call.
MAX326_Output MAX326_ReadSensorBpm(MAX326_Handle *handle);

// This function modifies the pulse width of the MAX30101 LEDs. All of the LEDs
// are modified to the same width. This will affect the number of samples that
// can be collected and will also affect the ADC resolution.
// MAX326_Width(us) - Resolution -  Sample Rate
// Default: 69us - 15 resolution - 50 samples per second.
//  69us     -    15      -   <= 3200 (fastest - least resolution)
//  118us    -    16      -   <= 1600
//  215us    -    17      -   <= 1600
//  411us    -    18      -   <= 1000 (slowest - highest resolution)
uint8_t MAX326_SetPulseWidth(MAX326_Handle *handle, uint16_t);

// This function reads the CONFIGURATION_REGISTER (0x0A), bits [1:0] from the
// MAX30101 Sensor. It returns one of the four settings in microseconds.
uint16_t MAX326_ReadPulseWidth(MAX326_Handle *handle);

// This function changes the sample rate of the MAX30101 sensor. The sample
// rate is affected by the set pulse width of the MAX30101 LEDs.
// Default: 69us - 15 resolution - 50 samples per second.
// MAX326_Width(us) - Resolution -  Sample Rate
//  69us     -    15      -   <= 3200 (fastest - least resolution)
//  118us    -    16      -   <= 1600
//  215us    -    17      -   <= 1600
//  411us    -    18      -   <= 1000 (slowest - highest resolution)
//  Samples Options:
//  50, 100, 200, 400, 800, 1000, 1600, 3200
uint8_t MAX326_SetSampleRate(MAX326_Handle *handle, uint16_t);

// This function reads the CONFIGURATION_REGISTER (0x0A), bits [4:2] from the
// MAX30101 Sensor. It returns one of the 8 possible sample rates.
uint16_t MAX326_ReadSampleRate(MAX326_Handle *handle);

// MAX30101 Register: CONFIGURATION_REGISTER (0x0A), bits [6:5]
// This functions sets the dynamic range of the MAX30101's ADC. The function
// accepts the higher range as a parameter.
// Default Range: 7.81pA - 2048nA
// Possible Ranges:
// 7.81pA  - 2048nA
// 15.63pA - 4096nA
// 32.25pA - 8192nA
// 62.5pA  - 16384nA
uint8_t MAX326_SetAdcRange(MAX326_Handle *handle, uint16_t);

// MAX30101 Register: CONFIGURATION_REGISTER (0x0A), bits [6:5]
// This function returns the set ADC range of the MAX30101 sensor.
uint16_t MAX326_ReadAdcRange(MAX326_Handle *handle);

// Family Byte: IDENTITY (0x01), Index Byte: READ_MCU_TYPE, Write Byte: NONE
// The following function returns a byte that signifies the microcontoller that
// is in communcation with your host microcontroller. Returns 0x00 for the
// MAX32625 and 0x01 for the MAX32660/MAX32664.
MAX326_MCUType MAX326_GetMcuType(MAX326_Handle *handle);

// Family Byte: BOOTLOADER_INFO (0x80), Index Byte: BOOTLOADER_VERS (0x00)
// This function checks the version number of the bootloader on the chip and
// returns a four bytes: Major version Byte, Minor version Byte, Space Byte,
// and the Revision Byte.
int32_t MAX326_GetBootloaderInf(MAX326_Handle *handle);

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_MAX30101 (0x03), Write
// Byte: senSwitch (parameter - 0x00 or 0x01).
// This function enables the MAX30101.
void MAX326_Max30101Control(MAX326_Handle *handle, bool);

// Family Byte: READ_SENSOR_MODE (0x45), Index Byte: READ_ENABLE_MAX30101 (0x03)
// This function checks if the MAX30101 is enabled or not.
uint8_t MAX326_ReadMAX30101State(MAX326_Handle *handle);

// Family Byte: ENABLE_SENSOR (0x44), Index Byte: ENABLE_ACCELEROMETER (0x04), Write
// Byte: accelSwitch (parameter - 0x00 or 0x01).
// This function enables the ACCELEROMETER.
MAX326_Status MAX326_AccelControl(MAX326_Handle *handle, uint8_t);

// Family Byte: OUTPUT_MODE (0x10), Index Byte: SET_FORMAT (0x00),
// Write Byte : outputType (Parameter values in OUTPUT_MODE_WRITE_BYTE)
void MAX326_SetOutputMode(MAX326_Handle *handle, uint8_t);

// Family Byte: OUTPUT_MODE, Index Byte: WRITE_SET_THRESHOLD, Write byte: intThres
// (parameter - value betwen 0 and 0xFF).
// This function changes the threshold for the FIFO interrupt bit/pin. The
// interrupt pin is the MFIO pin which is set to INPUT after IC initialization
// (begin).
void MAX326_SetFifoThreshold(MAX326_Handle *handle, uint8_t);

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: NUM_SAMPLES (0x00), Write
// Byte: NONE
// This function returns the number of samples available in the FIFO.
uint8_t MAX326_NumSamplesOutFifo(MAX326_Handle *handle);

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x00), Write
// Byte: NONE
// This function returns the data in the FIFO.
uint8_t* MAX326_GetDataOutFifo(MAX326_Handle *handle, uint8_t data[]);

// Family Byte: READ_DATA_OUTPUT (0x12), Index Byte: READ_DATA (0x00), Write
// Byte: NONE
// This function adds support for the acceleromter that is NOT included on
// SparkFun's product, The Family Registery of 0x13 and 0x14 is skipped for now.
MAX326_Status MAX326_NumSamplesExternalSensor(MAX326_Handle *handle);

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_MAX30101 (0x03), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the MAX30101 sensor and returns a boolean indicating a successful or
// non-successful write.
MAX326_Status MAX326_WriteRegisterMAX30101(MAX326_Handle *handle, uint8_t, uint8_t);

// Family Byte: WRITE_REGISTER (0x40), Index Byte: WRITE_ACCELEROMETER (0x04), Write Bytes:
// Register Address and Register Value
// This function writes the given register value at the given register address
// for the Accelerometer and returns a boolean indicating a successful or
// non-successful write.
MAX326_Status MAX326_WriteRegisterAccel(MAX326_Handle *handle, uint8_t, uint8_t);

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte:
// Register Address
// This function reads the given register address for the MAX30101 Sensor and
// returns the values at that register.
uint8_t MAX326_ReadRegisterMAX30101(MAX326_Handle *handle, uint8_t);

// Family Byte: READ_REGISTER (0x41), Index Byte: READ_MAX30101 (0x03), Write Byte:
// Register Address
// This function reads the given register address for the MAX30101 Sensor and
// returns the values at that register.
uint8_t MAX326_ReadRegisterAccel(MAX326_Handle *handle, uint8_t);

// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte: RETRIEVE_AFE_MAX30101/ (0x03)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// MAX30101 sensor. It returns the number of bytes in a word for the sensor
// and the number of registers available.
MAX326_SensorAttr MAX326_GetAfeAttributesMAX30101(MAX326_Handle *handle);

// Family Byte: READ_ATTRIBUTES_AFE (0x42), Index Byte:
// RETRIEVE_AFE_ACCELEROMETER (0x04)
// This function retrieves the attributes of the AFE (Analog Front End) of the
// Accelerometer. It returns the number of bytes in a word for the sensor
// and the number of registers available.
MAX326_SensorAttr MAX326_GetAfeAttributesAccelerometer(MAX326_Handle *handle);

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_MAX30101 (0x03)
// This function returns all registers and register values sequentially of the
// MAX30101 sensor: register zero and register value zero to register n and
// register value n. There are 36 registers in this case.
MAX326_Status MAX326_DumpRegisterMAX30101(MAX326_Handle *handle,
		uint8_t regArray[]);

// Family Byte: DUMP_REGISTERS (0x43), Index Byte: DUMP_REGISTER_ACCELEROMETER (0x04)
// This function returns all registers and register values sequentially of the
// Accelerometer: register zero and register value zero to register n and
// register value n.
MAX326_Status MAX326_DumpRegisterAccelerometer(MAX326_Handle *handle, uint8_t,
		uint8_t regArray[]);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_TARG_PERC (0x00), Write Byte: AGC_GAIN_ID (0x00)
// This function sets the target percentage of the full-scale ADC range that
// the automatic gain control algorithm uses. It takes a paramater of zero to
// 100 percent.
MAX326_Status MAX326_SetAlgoRange(MAX326_Handle *handle, uint8_t);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_STEP_SIZE (0x00), Write Byte: AGC_STEP_SIZE_ID (0x01)
// This function changes the step size toward the target for the AGC algorithm.
// It takes a paramater of zero to 100 percent.
MAX326_Status MAX326_SetAlgoStepSize(MAX326_Handle *handle, uint8_t);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_SENSITIVITY (0x00), Write Byte: AGC_SENSITIVITY_ID (0x02)
// This function changes the sensitivity of the AGC algorithm.
MAX326_Status MAX326_SetAlgoSensitivity(MAX326_Handle *handle, uint8_t);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_AVG_SAMPLES (0x00), Write Byte: AGC_NUM_SAMP_ID (0x03)
// This function changes the number of samples that are averaged.
// It takes a paramater of zero to 255.
MAX326_Status MAX326_SetAlgoSamples(MAX326_Handle *handle, uint8_t);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte:
// SET_PULSE_OX_COEF (0x02), Write Byte: MAXIMFAST_COEF_ID (0x0B)
// This function takes three values that are used as the Sp02 coefficients.
MAX326_Status MAX326_SetMaximFastCoef(MAX326_Handle *handle, int32_t, int32_t,
		int32_t);

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_PERCENTAGE (0x00), Write Byte: READ_AGC_PERC_ID (0x00)
// This function reads and returns the currently set target percentage
// of the full-scale ADC range that the Automatic Gain Control algorithm is using.
uint8_t MAX326_ReadAlgoRange(MAX326_Handle *handle);

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_STEP_SIZE (0x00), Write Byte: READ_AGC_STEP_SIZE_ID (0x01)
// This function returns the step size toward the target for the AGC algorithm.
// It returns a value between zero and 100 percent.
uint8_t MAX326_ReadAlgoStepSize(MAX326_Handle *handle);

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_SENSITIVITY_ID (0x00), Write Byte: READ_AGC_SENSITIVITY_ID (0x02)
// This function returns the sensitivity (percentage) of the automatic gain control.
uint8_t MAX326_ReadAlgoSensitivity(MAX326_Handle *handle);

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_AGC_NUM_SAMPLES (0x00), Write Byte: READ_AGC_NUM_SAMPlES_ID (0x03)
// This function changes the number of samples that are averaged.
// It takes a paramater of zero to 255.
void MAX326_ReadAlgoSamples(MAX326_Handle *handle);

// Family Byte: READ_ALGORITHM_CONFIG (0x51), Index Byte:
// READ_MAX_FAST_COEF (0x02), Write Byte: READ_MAX_FAST_COEF_ID (0x0B)
// This function reads the maximum age for the wrist heart rate monitor
// (WHRM) algorithm. It returns three uint32_t integers that are
// multiplied by 100,000.
// INCOMPLETE
uint8_t MAX326_ReadMaximFastCoef(MAX326_Handle *handle, int32_t coefArr[3]);

// Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
// ENABLE_AGC_ALGO (0x00)
// This function enables (one) or disables (zero) the automatic gain control algorithm.
void MAX326_AgcAlgoControl(MAX326_Handle *handle, bool);

// Family Byte: ENABLE_ALGORITHM (0x52), Index Byte:
// ENABLE_WHRM_ALGO (0x02)
// This function enables (one) or disables (zero) the wrist heart rate monitor
// algorithm.
void MAX326_MaximFastAlgoControl(MAX326_Handle *handle, uint8_t);

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: SET_NUM_PAGES (0x02),
// Write Bytes: 0x00 - Number of pages at byte 0x44 from .msbl file.
void MAX326_SetNumPages(MAX326_Handle *handle, uint8_t);

// Family Byte: BOOTLOADER_FLASH (0x80), Index Byte: ERASE_FLASH (0x03)
// Returns true on successful communication.
void MAX326_EraseFlash(MAX326_Handle *handle);

// Family Byte: BOOTLOADER_INFO (0x81), Index Byte: BOOTLOADER_VERS (0x00)
void MAX326_ReadBootloaderVers(MAX326_Handle *handle);

// Family Byte: BOOTLOADER_INFO (0x81), Index Byte: PAGE_SIZE (0x01)
// Family Byte: IDENTITY (0xFF), Index Byte: READ_SENSOR_HUB_VERS (0x03)
void MAX326_ReadSensorHubVersion(MAX326_Handle *handle);

// Family Byte: IDENTITY (0xFF), Index Byte: READ_ALGO_VERS (0x07)
void MAX326_ReadAlgorithmVersion(MAX326_Handle *handle);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_MEDICATION (0x00)
void MAX326_WritePatientBPMedication(MAX326_Handle *handle, MAX326_MedicationFlag);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_MEDICATION (0x00)
void MAX326_ReadPatientBPMedication(MAX326_Handle *handle);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: DIASTOLIC_VALUE (0x02)
void MAX326_WriteDiastolicVals(MAX326_Handle *handle, uint8_t, uint8_t, uint8_t);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: DIASTOLIC_VALUE (0x02)
void MAX326_ReadDiastolicVals(MAX326_Handle *handle);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: SYSTOLIC_VALUE (0x01)
void MAX326_WriteSystolicVals(MAX326_Handle *handle, uint8_t, uint8_t, uint8_t);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: SYSTOLIC_VALUE (0x01)
void MAX326_ReadSystolicVals(MAX326_Handle *handle);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_CALIB_DATA (0x03)
void MAX326_WriteBPTAlgoData(MAX326_Handle *handle, uint8_t bptCalibData[]);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: BPT_CALIB_DATA (0x03)
void MAX326_ReadBPTAlgoData(MAX326_Handle *handle);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: PATIENT_RESTING (0x05)
void MAX326_WritePatientResting(MAX326_Handle *handle, MAX326_RestingFlag);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: PATIENT_RESTING (0x05)
void MAX326_ReadPatientResting(MAX326_Handle *handle);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: MAX326_WB_AGC_SPO2_COEFS (0x0B)
void MAX326_WriteSP02AlgoCoef(MAX326_Handle *handle, MAX326_SPO2Coef);

// Family Byte: CHANGE_ALGORITHM_CONFIG (0x50), Index Byte: BPT_CONFIG (0x04),
// Write Byte: MAX326_WB_AGC_SPO2_COEFS (0x0B)
void MAX326_ReadSP02AlgoCoef(MAX326_Handle *handle);

#endif
