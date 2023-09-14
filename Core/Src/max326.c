#include "usart.h"

#include <stdlib.h>
#include <assert.h>
#include <max326.h>

// ============= Private methods =================================
bool __init_valid(MAX326_InitStruct init) {
	if (init.I2CHandle == NULL)
		return false;
	if (init.MFIOLine == NULL || init.ResetLine == NULL)
		return false;
	return true;
}

void __i2c_transmit_it(MAX326_Handle* handle) {
	USART_PRINT("\r\nI2C Tx started");
	HAL_I2C_Master_Transmit_IT(handle->Init.I2CHandle, handle->Init.I2CWriteAddress, handle->I2CTxBuffer, handle->I2CTxBufferSize);
}

void __i2c_receive_it(MAX326_Handle* handle) {
	HAL_I2C_Master_Receive_IT(handle->Init.I2CHandle, handle->Init.I2CReadAddress, handle->I2CRxBuffer, handle->I2CRxBufferSize);
}

void __i2c_transmit_dma(MAX326_Handle* handle) {
	HAL_I2C_Master_Transmit_DMA(handle->Init.I2CHandle, handle->Init.I2CWriteAddress, handle->I2CTxBuffer, handle->I2CTxBufferSize);
}

void __i2c_receive_dma(MAX326_Handle* handle) {
	HAL_I2C_Master_Receive_DMA(handle->Init.I2CHandle, handle->Init.I2CReadAddress, handle->I2CRxBuffer, handle->I2CRxBufferSize);
}

MAX326_Transaction __make_trans(MAX326_TransType type, uint8_t fam, uint8_t idx) {
	MAX326_Transaction trans = {0};
	trans.Type = type;
	trans.FamilyByte = fam;
	trans.IndexByte = idx;
	trans.HasWriteByte = false;
	trans.DataBytesNum = 0;
	// with HasWriteByte=false, WriteByte is not considered
	// with DataBytesNum=0, DataBytesFormat and DataBytesXX fields are not considered
	return trans;
}

void __add_write_byte(MAX326_Transaction* trans, uint8_t wrt) {
	trans->HasWriteByte = true;
	trans->WriteByte = wrt;
}

void __add_data_bytes(MAX326_Transaction* trans, MAX326_BytesFormat fmt, void* data, size_t size) {
	trans->DataBytesFormat = fmt;
	trans->DataBytesNum = size;

	if (fmt == MAX326_BF_32)
		trans->DataBytes32 = (int32_t*)data;
	else if (fmt == MAX326_BF_16)
		trans->DataBytes16 = (uint16_t*)data;
	else // MAX326_BF_8
		trans->DataBytes8 = (uint8_t*)data;
}

// =============== Transaction Callback =============================
void __trans_clbk(MAX326_Handle* handle) {
	// first of all, record obtained status byte
	USART_PRINT("\r\nI2C Rx completed");
	handle->Status = handle->I2CRxBuffer[0];

	USART_PRINT("\r\n[DEBUG] Terminated (%u, %u) with status %u", handle->CurrPipeline, handle->CurrOperation, handle->Status);

	MAX326_Transaction trans = handle->CurrTransaction;

	if (trans.DataBytesNum > 0 && trans.Type == MAX326_TT_READ) {
		if (trans.DataBytesFormat == MAX326_BF_32 && trans.DataBytes32 != NULL) {
			// always index + 1 because first byte is status byte
			for (size_t i = 0; i < trans.DataBytesNum; i++) {
				trans.DataBytes32[i] = (handle->I2CRxBuffer[4 * i + 1] << 24);
				trans.DataBytes32[i] |= (handle->I2CRxBuffer[4 * i + 2] << 16);
				trans.DataBytes32[i] |= (handle->I2CRxBuffer[4 * i + 3] << 8);
				trans.DataBytes32[i] |= handle->I2CRxBuffer[4 * i + 4];
			}
		}
		else if (trans.DataBytesFormat == MAX326_BF_16 && trans.DataBytes16 != NULL) {
			for (size_t i = 0; i < trans.DataBytesNum; i++) {
				trans.DataBytes16[i] |= (handle->I2CRxBuffer[2 * i + 1] << 8);
				trans.DataBytes16[i] |= handle->I2CRxBuffer[2 * i + 2];
			}
		}
		else if (trans.DataBytesFormat == MAX326_BF_8 && trans.DataBytes8 != NULL) {
			for (size_t i = 0; i < trans.DataBytesNum; i++) {
				trans.DataBytes8[i] = handle->I2CRxBuffer[i + 1];
			}
		}

	}
}

void __trans_exec(MAX326_Handle *handle, MAX326_Transaction trans) {

	handle->I2CTxBuffer[0] = trans.FamilyByte;
	handle->I2CTxBuffer[1] = trans.IndexByte;
	handle->I2CTxBufferSize = 2;

	if (trans.HasWriteByte) {
		handle->I2CTxBuffer[handle->I2CTxBufferSize] = trans.WriteByte;
		handle->I2CTxBufferSize += 1;
	}

	if (trans.DataBytesNum > 0 && trans.Type == MAX326_TT_WRITE) {
		size_t base = handle->I2CTxBufferSize;

		if (trans.DataBytesFormat == MAX326_BF_32 && trans.DataBytes32 != NULL) {
			for (size_t i = base; i < trans.DataBytesNum; i++) {
				handle->I2CTxBuffer[4 * i] = (trans.DataBytes32[i] >> 24);
				handle->I2CTxBuffer[4 * i + 1] = (trans.DataBytes32[i] >> 16);
				handle->I2CTxBuffer[4 * i + 2] = (trans.DataBytes32[i] >> 8);
				handle->I2CTxBuffer[4 * i + 3] = trans.DataBytes32[i];
			}
			handle->I2CTxBufferSize += 4 * trans.DataBytesNum;
		}
		else if (trans.DataBytesFormat == MAX326_BF_16 && trans.DataBytes16 != NULL) {
			for (size_t i = base; i < trans.DataBytesNum; i++) {
				handle->I2CTxBuffer[2 * i] = (trans.DataBytes16[i] >> 8);
				handle->I2CTxBuffer[2 * i + 1] = trans.DataBytes16[i];
			}
			handle->I2CTxBufferSize += 2 * trans.DataBytesNum;
		}
		else if (trans.DataBytesFormat == MAX326_BF_8 && trans.DataBytes8 != NULL) {
			for (size_t i = base; i < trans.DataBytesNum; i++) {
				handle->I2CTxBuffer[i] = trans.DataBytes8[i];
			}
			handle->I2CTxBufferSize += trans.DataBytesNum;
		}
	}

	handle->CurrTransaction = trans;
	handle->I2CTransmit(handle);

	USART_PRINT("\r\n[DEBUG] Started (%u, %u)", handle->CurrPipeline, handle->CurrOperation);
}

// ---- Common Transactions -----
void __read_byte(MAX326_Handle* handle, uint8_t fam, uint8_t idx) {
	MAX326_Transaction trans = __make_trans(MAX326_TT_READ, fam, idx);
	__trans_exec(handle, trans);
}

void __read_byte_w(MAX326_Handle* handle, uint8_t fam, uint8_t idx, uint8_t wrt) {
	MAX326_Transaction trans = __make_trans(MAX326_TT_READ, fam, idx);
	__add_write_byte(&trans, wrt);
	__trans_exec(handle, trans);
}

void __write_byte(MAX326_Handle* handle, uint8_t fam, uint8_t idx, uint8_t wrt) {
	MAX326_Transaction trans = __make_trans(MAX326_TT_WRITE, fam, idx);
	__add_write_byte(&trans, wrt);
	__trans_exec(handle, trans);
}

void __set_enable(MAX326_Handle* handle, uint8_t fam, uint8_t idx, uint8_t en) {
	MAX326_Transaction trans = __make_trans(MAX326_TT_ENABLE, fam, idx);
	__add_write_byte(&trans, en);
	__trans_exec(handle, trans);
}

// ========= I2C Callbacks =================
void MAX326_I2CTxCallback(MAX326_Handle* handle) {
	USART_PRINT("\r\nI2C Tx completed");
	handle->I2CReceive(handle);
}

void MAX326_I2CRxCallback(MAX326_Handle* handle) {
	// handle transaction completed
	__trans_clbk(handle);
	// from now on, for READ transactions, DataBytesXX fields are available

	switch (handle->CurrPipeline) {
	case MAX326_PL_INIT:
		MAX326_Init(handle);
		break;
	case MAX326_PL_CONFBPM:
		MAX326_ConfigBpm(handle, 0);
		break;
	case MAX326_PL_MEASURE:
		MAX326_ReadBpm(handle);
	default:
		break;
	}
}

// ========= Timer Callback ===========================
void MAX326_TimerCallback(MAX326_Handle* handle) {

	handle->Counter += 1;

	if (handle->CurrPipeline == MAX326_PL_CONFBPM && handle->CurrOperation == MAX326_OP_DELAY) {
		if (MAX326_TIMEOUT(handle->Counter, 1000)) {
			handle->CurrOperation = MAX326_OP_NONE;
			handle->CurrPipeline = MAX326_PL_NONE;
			handle->CurrState = MAX326_ST_BPM_READY;
			return;
		}
	}
}

// ========= Methods ===================================
MAX326_Handle* MAX326_Create(MAX326_InitStruct init) {

	assert(__init_valid(init));

	MAX326_Handle *handle = malloc(sizeof(MAX326_Handle));
	handle->Init = init;
	handle->CurrState = MAX326_ST_UNINIT;
	handle->CurrPipeline = MAX326_PL_NONE;
	handle->CurrOperation = MAX326_OP_NONE;
	handle->IsOutputReady = false;
	handle->Counter = 0;
	handle->Interrupt = MAX326_IT_NONE;

	if (init.I2CUseDMA) {
		handle->I2CTransmit = __i2c_transmit_dma;
		handle->I2CReceive = __i2c_receive_dma;
	}

	else {
		handle->I2CTransmit = __i2c_transmit_it;
		handle->I2CReceive = __i2c_receive_it;
	}

	return handle;
}

// Family Byte: READ_DEVICE_MODE (0x02) Index Byte: 0x00, Write Byte: 0x00
// The following function initializes the sensor. To place the MAX32664 into
// application mode, the MFIO pin must be pulled HIGH while the board is held
// in reset for 10ms. After 50 addtional ms have elapsed the board should be
// in application mode and will return two bytes, the first 0x00 is a
// successful communcation byte, followed by 0x00 which is the byte indicating
// which mode the IC is in.
void MAX326_Init(MAX326_Handle *handle) {

	// pipeline starts
	if (handle->CurrPipeline == MAX326_PL_NONE) {
		if (handle->CurrState != MAX326_ST_UNINIT)
			return;

		handle->CurrPipeline = MAX326_PL_INIT;

		GPIO_InitTypeDef conf = {0};
		conf.Pin = handle->Init.ResetLine->pin;
		conf.Mode = GPIO_MODE_OUTPUT_PP;
		conf.Pull = GPIO_NOPULL;
		conf.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(handle->Init.ResetLine->port, &conf);

		conf.Pin = handle->Init.MFIOLine->pin;
		conf.Mode = GPIO_MODE_OUTPUT_PP;
		conf.Pull = GPIO_NOPULL;
		conf.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(handle->Init.MFIOLine->port, &conf);

		HAL_GPIO_WriteLine(handle->Init.MFIOLine, GPIO_PIN_SET);
		HAL_GPIO_WriteLine(handle->Init.ResetLine, GPIO_PIN_RESET);
		HAL_Delay(10);
		HAL_GPIO_WriteLine(handle->Init.ResetLine, GPIO_PIN_SET);

		HAL_Delay(1000);

		conf.Pin = handle->Init.MFIOLine->pin;
		conf.Mode = GPIO_MODE_INPUT;
		conf.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(handle->Init.MFIOLine->port, &conf);
		// To be used as an interrupt later

		__read_byte(handle, MAX326_FB_R_DEVMODE, 0x00);
		return;
	}

	// other phases
	handle->CurrPipeline = MAX326_PL_NONE;
	handle->CurrState = MAX326_ST_IDLE;
	handle->Interrupt = MAX326_IT_INIT;
	HAL_NVIC_SetPendingIRQ(handle->Init.IRQLine);
}

void MAX326_ConfigBpm(MAX326_Handle *handle, uint8_t mode) {

	// pipeline starts
	if (handle->CurrPipeline == MAX326_PL_NONE) {
		assert(mode == MAX326_MODE_ONE || mode == MAX326_MODE_TWO);
		handle->CurrState = MAX326_ST_IDLE; // return to "not configured" situation
		handle->UserSelectedMode = mode;
		handle->CurrPipeline = MAX326_PL_CONFBPM;

		MAX326_SetOutputMode(handle, MAX326_WB_ALGDATA); // Just the data
		return;
	}

	// next phases
	assert(handle->Status == MAX326_SB_SUCCESS);

	switch (handle->CurrOperation) {
	case MAX326_OP_SET_OUTPUT_MODE:
		MAX326_SetFifoThreshold(handle, 0x01); // One sample before interrupt is fired.
		break;

	case MAX326_OP_SET_FIFO_THRES:
		MAX326_AgcAlgoControl(handle, ENABLE);
		break;

	case MAX326_OP_AGC_ALGO_CONTROL:
		MAX326_Max30101Control(handle, ENABLE);
		break;

	case MAX326_OP_MAX30101_CONTROL:
		MAX326_MaximFastAlgoControl(handle, handle->UserSelectedMode);
		break;

	case MAX326_OP_MAXFAST_CONTROL:
		MAX326_ReadAlgoSamples(handle);
		break;

	case MAX326_OP_READ_ALGO_SAMPLES:
		handle->SampleRate = handle->I2CRxBuffer[1];
		handle->CurrOperation = MAX326_OP_DELAY;
		handle->Interrupt = MAX326_IT_CONFBPM;
		HAL_NVIC_SetPendingIRQ(handle->Init.IRQLine);
		break;

	default:
		break;
	}
}

void MAX326_ReadBpm(MAX326_Handle *handle) {

	if (handle->UserSelectedMode == MAX326_MODE_ONE) {

		// pipeline starts
		if (handle->CurrPipeline == MAX326_PL_NONE) {
			handle->CurrPipeline = MAX326_PL_MEASURE;

			MAX326_Transaction trans;
			trans = __make_trans(MAX326_TT_READ, MAX326_FB_R_DATA_OUT, MAX326_IB_R_DATA);
			__add_write_byte(&trans, MAX326_MAXFAST_ARRSIZE);
			__add_data_bytes(&trans, MAX326_BF_8, handle->BpmArr, MAX326_MAXFAST_ARRSIZE);
			__trans_exec(handle, trans);
			return;
		}

		// other phases
		// Heart Rate formatting
		handle->Output.HeartRate = (uint16_t) (handle->BpmArr[0]) << 8;
		handle->Output.HeartRate |= (handle->BpmArr[1]);
		handle->Output.HeartRate /= 10;

		// Confidence formatting
		handle->Output.Confidence = handle->BpmArr[2];

		//Blood oxygen level formatting
		handle->Output.Oxygen = (uint16_t) (handle->BpmArr[3]) << 8;
		handle->Output.Oxygen |= handle->BpmArr[4];
		handle->Output.Oxygen /= 10;

		//"Machine State" - has a finger been detected?
		handle->Output.Status = handle->BpmArr[5];

		// "Publish" output
		handle->IsOutputReady = true;
		handle->CurrState = MAX326_ST_BPM_READY;
		handle->CurrPipeline = MAX326_PL_NONE;
		handle->CurrOperation = MAX326_OP_NONE;
		handle->Interrupt = MAX326_IT_OUTPUT;
		HAL_NVIC_SetPendingIRQ(handle->Init.IRQLine);
	}

	else if (handle->UserSelectedMode == MAX326_MODE_TWO) {

		// pipeline starts
		if (handle->CurrPipeline == MAX326_PL_NONE) {
			handle->CurrPipeline = MAX326_PL_MEASURE;

			MAX326_Transaction trans;
			trans = __make_trans(MAX326_TT_READ, MAX326_FB_R_DATA_OUT, MAX326_IB_R_DATA);
			__add_write_byte(&trans, MAX326_MAXFAST_EXT_ARRSIZE);
			__add_data_bytes(&trans, MAX326_BF_8, handle->BpmArr, MAX326_MAXFAST_EXT_ARRSIZE);
			__trans_exec(handle, trans);
			return;
		}

		// other phase
		// Heart Rate formatting
		handle->Output.HeartRate = (uint16_t) (handle->BpmArr[0]) << 8;
		handle->Output.HeartRate |= (handle->BpmArr[1]);
		handle->Output.HeartRate /= 10;

		// Confidence formatting
		handle->Output.Confidence = handle->BpmArr[2];

		//Blood oxygen level formatting
		handle->Output.Oxygen = (uint16_t) (handle->BpmArr[3]) << 8;
		handle->Output.Oxygen |= handle->BpmArr[4];
		handle->Output.Oxygen /= 10.0;

		//"Machine State" - has a finger been detected?
		handle->Output.Status = handle->BpmArr[5];

		//Sp02 r Value formatting
		uint16_t tempVal = (uint16_t) (handle->BpmArr[6]) << 8;
		tempVal |= handle->BpmArr[7];
		handle->Output.RValue = tempVal;
		handle->Output.RValue /= 10.0;

		//Extended Machine State formatting
		handle->Output.ExtStatus = handle->BpmArr[8];

		// Publish output
		handle->IsOutputReady = true;
		handle->CurrState = MAX326_ST_BPM_READY;
		handle->CurrPipeline = MAX326_PL_NONE;
		handle->CurrOperation = MAX326_OP_NONE;
		handle->Interrupt = MAX326_IT_OUTPUT;

		HAL_NVIC_SetPendingIRQ(handle->Init.IRQLine);
	}
}

void MAX326_SetOutputMode(MAX326_Handle *handle, uint8_t outputType) {

	assert(outputType <= MAX326_WB_SENS_ALG_COUNTER); // Bytes between 0x00 and 0x07

	handle->CurrOperation = MAX326_OP_SET_OUTPUT_MODE;

	// Check that communication was successful, not that the IC is outputting
	// correct format.
	__write_byte(handle, MAX326_FB_OUTMODE, SET_FORMAT, outputType);
}

void MAX326_AgcAlgoControl(MAX326_Handle *handle, bool enable) {

	handle->CurrOperation = MAX326_OP_AGC_ALGO_CONTROL;
	__set_enable(handle, MAX326_FB_EN_ALGORITHM, MAX326_IB_EN_AGC_ALGO, enable);
}

void MAX326_Max30101Control(MAX326_Handle *handle, bool senSwitch) {

	handle->CurrOperation = MAX326_OP_MAX30101_CONTROL;
	__set_enable(handle, MAX326_FB_EN_SENSOR, MAX326_IB_R_EN_MAX30101, senSwitch);
}

void MAX326_MaximFastAlgoControl(MAX326_Handle *handle, uint8_t mode) {

	assert(mode == 0 || mode == 1 || mode == 2);

	handle->CurrOperation = MAX326_OP_MAXFAST_CONTROL;
	__set_enable(handle, MAX326_FB_EN_ALGORITHM, MAX326_IB_EN_WHRM_ALGO, mode);
}

void MAX326_ReadAlgoSamples(MAX326_Handle *handle) {

	handle->CurrOperation = MAX326_OP_READ_ALGO_SAMPLES;
	__read_byte_w(handle, MAX326_FB_R_ALGOCONF, MAX326_IB_R_NSAMPLES_SENS, MAX326_WB_R_AGC_NSAMPLES_ID);
}

void MAX326_SetFifoThreshold(MAX326_Handle *handle, uint8_t intThresh) {

	handle->CurrOperation = MAX326_OP_SET_FIFO_THRES;
	__write_byte(handle, MAX326_FB_OUTMODE, WRITE_SET_THRESHOLD, intThresh);
}

void MAX326_ReadSensorHubVersion(MAX326_Handle *handle) {

	handle->CurrOperation = MAX326_OP_READ_SHVERSION;
	const size_t versByteNum = 4;

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_READ, MAX326_FB_IDENTITY, MAX326_IB_R_SENS_HUB_VERS);
	__add_data_bytes(&trans, MAX326_BF_8, NULL, versByteNum);
	__trans_exec(handle, trans);

	handle->Version.Major = handle->I2CRxBuffer[1];
	handle->Version.Minor = handle->I2CRxBuffer[2];
	handle->Version.Revision = handle->I2CRxBuffer[3];
}

// ------ only for version D --------

void MAX326_WriteSystolicVals(MAX326_Handle *handle, uint8_t sysVal1, uint8_t sysVal2, uint8_t sysVal3) {

	const size_t numSysVals = 3;
	uint8_t sysVals[3] = { sysVal1, sysVal2, sysVal3 };

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_WRITE, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_SYSTOLIC_VALUE);
	__add_data_bytes(&trans, MAX326_BF_8, sysVals, numSysVals);
	__trans_exec(handle, trans);
}

void MAX326_ReadSystolicVals(MAX326_Handle *handle) {

	const size_t numSysVals = 3;

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_READ, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_SYSTOLIC_VALUE);
	__add_data_bytes(&trans, MAX326_BF_8, NULL, numSysVals);
	__trans_exec(handle, trans);
}

void MAX326_WriteDiastolicVals(MAX326_Handle *handle, uint8_t diasVal1, uint8_t diasVal2, uint8_t diasVal3) {

	const size_t numSysVals = 3;
	uint8_t sysVals[3] = { diasVal1, diasVal2, diasVal3 };

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_WRITE, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_DIASTOLIC_VALUE);
	__add_data_bytes(&trans, MAX326_BF_8, sysVals, numSysVals);
	__trans_exec(handle, trans);
}

void MAX326_ReadDiastolicVals(MAX326_Handle *handle) {

	const size_t numDiasVals = 3;
	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_READ, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_DIASTOLIC_VALUE);
	__add_data_bytes(&trans, MAX326_BF_8, NULL, numDiasVals);
	__trans_exec(handle, trans);
}

void MAX326_WritePatientBPMedication(MAX326_Handle *handle, MAX326_MedicationFlag med) {

	if (med > MAX326_MF_ISMED) {
		handle->Status = MAX326_SB_INCORR_PARAM;
		return;
	}

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_WRITE, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_BPT_MEDICATION);
	__add_data_bytes(&trans, MAX326_BF_8, &med, 1);
	__trans_exec(handle, trans);
}

void MAX326_ReadPatientBPMedication(MAX326_Handle *handle) {

	__read_byte_w(handle, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF, MAX326_WB_BPT_MEDICATION);
}

void MAX326_WriteBPTAlgoData(MAX326_Handle *handle, uint8_t bptCalibData[]) {

	const size_t numCalibVals = 824;

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_WRITE, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_BPT_CALIB_DATA);
	__add_data_bytes(&trans, MAX326_BF_8, bptCalibData, numCalibVals);
	__trans_exec(handle, trans);
}

void MAX326_ReadBPTAlgoData(MAX326_Handle *handle) {

	const size_t numCalibVals = 824;

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_READ, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_BPT_CALIB_DATA);
	__add_data_bytes(&trans, MAX326_BF_8, NULL, numCalibVals);
	__trans_exec(handle, trans);
}

void MAX326_WritePatientResting(MAX326_Handle *handle, MAX326_RestingFlag resting) {

	if (resting > MAX326_RF_ISREST) {
		handle->Status = MAX326_SB_INCORR_PARAM;
		return;
	}

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_WRITE, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_PATIENT_RESTING);
	__add_data_bytes(&trans, MAX326_BF_8, &resting, 1);
	__trans_exec(handle, trans);
}

void MAX326_ReadPatientResting(MAX326_Handle *handle) {

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_READ, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_PATIENT_RESTING);
	__add_data_bytes(&trans, MAX326_BF_8, NULL, 1);
	__trans_exec(handle, trans);
}

void MAX326_WriteSPO2AlgoCoef(MAX326_Handle *handle, MAX326_SPO2Coef coef) {

	const size_t numCoefVals = 3;
	int32_t coefVals[3] = { coef.A, coef.B, coef.C };

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_WRITE, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_AGC_SPO2_COEFS);
	__add_data_bytes(&trans, MAX326_BF_32, coefVals, numCoefVals);
	__trans_exec(handle, trans);
}

void MAX326_ReadSPO2AlgoCoef(MAX326_Handle *handle) {

	const size_t numCoefVals = 3;

	MAX326_Transaction trans;
	trans = __make_trans(MAX326_TT_READ, MAX326_FB_CHANGE_ALGCONF, MAX326_IB_BPT_CONF);
	__add_write_byte(&trans, MAX326_WB_AGC_SPO2_COEFS);
	__add_data_bytes(&trans, MAX326_BF_32, NULL, numCoefVals);
	__trans_exec(handle, trans);
}
