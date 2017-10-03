#include "cp_core.h"
#include "cp_msg.h"
#include "canpie.h"

#include "stm32f4xx_hal.h"

// Definitions
#define HAL_CAN_TIMEOUT_VALUE  10U
#define MAX_CAN_FILTER_NUMBER	14
#define BUFFER_TX	(0)
#define BUFFER_RX	(1)
#define BUFFER_NOT_USED	(-1)
// external functions
// variables of module
extern CAN_HandleTypeDef hcan1;

static CanRxMsgTypeDef can_rx_msg_fifo0;
//static CanRxMsgTypeDef can_rx_msg_fifo1;
static CanTxMsgTypeDef can_tx_msg;

static int8_t filter_to_cp_buffer[MAX_CAN_FILTER_NUMBER];
static uint8_t tx_mailbox_to_buffer[3];

#if CP_STATISTIC > 0
static uint32_t tx_counter;
static uint32_t rx_counter;
static uint32_t err_counter;
#endif

#define HCAN1 hcan1

struct hal_baudrate
{
	enum CpBitrate_e bitrate;
	uint32_t prescaler;
	uint32_t sjw;
	uint32_t bs1;
	uint32_t bs2;
};

enum DrvInfo_e
{
	eDRV_INFO_OFF = 0, eDRV_INFO_INIT, eDRV_INFO_ACTIVE
};

static const struct hal_baudrate hal_baudrate_timing_36mhz[] =
{
	{ eCP_BITRATE_10K, 225, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 10 KBit/s 87.5%
	{ eCP_BITRATE_20K, 100, CAN_SJW_1TQ, CAN_BS1_14TQ, CAN_BS2_3TQ }, // 20 KBit/s 83.3%
	{ eCP_BITRATE_50K, 45, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 50 KBit/s 87.5%
	{ eCP_BITRATE_100K, 20, CAN_SJW_1TQ, CAN_BS1_14TQ, CAN_BS2_3TQ }, // 100 KBit/s 83.3%
	{ eCP_BITRATE_125K, 18, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 125 KBit/s 87.5%
	{ eCP_BITRATE_250K, 9, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 250 KBit/s 87.5%
	{ eCP_BITRATE_500K, 4, CAN_SJW_1TQ, CAN_BS1_14TQ, CAN_BS2_3TQ }, // 500 KBit/s 83.3%
	{ eCP_BITRATE_800K, 3, CAN_SJW_1TQ, CAN_BS1_12TQ, CAN_BS2_2TQ }, // 800 KBit/s 86.7 %
	{ eCP_BITRATE_1M, 2, CAN_SJW_1TQ, CAN_BS1_14TQ, CAN_BS2_3TQ }  // 1 MBit/s 83.3%
};

#if 0
static const struct hal_baudrate hal_baudrate_timing_42mhz[] =
{
	{ eCP_BITRATE_10K, 280, CAN_SJW_1TQ, CAN_BS1_12TQ, CAN_BS2_2TQ}, // 10 KBit/s 86.7%
	{ eCP_BITRATE_20K, 140, CAN_SJW_1TQ, CAN_BS1_12TQ, CAN_BS2_2TQ}, // 20 KBit/s 86.7%
	{ eCP_BITRATE_50K, 56, CAN_SJW_1TQ, CAN_BS1_12TQ, CAN_BS2_2TQ}, // 50 KBit/s 86.7%
	{ eCP_BITRATE_100K, 28, CAN_SJW_1TQ, CAN_BS1_12TQ, CAN_BS2_2TQ}, // 100 KBit/s 86.7%
	{ eCP_BITRATE_125K, 21, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ}, // 125 KBit/s 87.5%
	{ eCP_BITRATE_250K, 12, CAN_SJW_1TQ, CAN_BS1_11TQ, CAN_BS2_2TQ}, // 250 KBit/s 85.7%
	{ eCP_BITRATE_500K, 6, CAN_SJW_1TQ, CAN_BS1_11TQ, CAN_BS2_2TQ}, // 500 KBit/s 85.7%
	{ eCP_BITRATE_1M, 3, CAN_SJW_1TQ, CAN_BS1_11TQ, CAN_BS2_2TQ}  // 1 MBit/s 85.7%
};
#endif

static const struct hal_baudrate hal_baudrate_timing_42mhz[] =
{
	{ eCP_BITRATE_10K, 200, CAN_SJW_1TQ, CAN_BS1_15TQ, CAN_BS2_5TQ }, // 10 KBit/s 76.19%
	{ eCP_BITRATE_20K, 100, CAN_SJW_1TQ, CAN_BS1_15TQ, CAN_BS2_5TQ }, // 20 KBit/s 76.19%
	{ eCP_BITRATE_50K, 40, CAN_SJW_1TQ, CAN_BS1_15TQ, CAN_BS2_5TQ }, // 50 KBit/s 76.19%
	{ eCP_BITRATE_100K, 20, CAN_SJW_1TQ, CAN_BS1_15TQ, CAN_BS2_5TQ }, // 100 KBit/s 76.19%
	{ eCP_BITRATE_125K, 16, CAN_SJW_1TQ, CAN_BS1_15TQ, CAN_BS2_5TQ }, // 125 KBit/s 76.19%
	{ eCP_BITRATE_250K, 8, CAN_SJW_1TQ, CAN_BS1_15TQ, CAN_BS2_5TQ }, // 250 KBit/s 76.19%
	{ eCP_BITRATE_500K, 4, CAN_SJW_1TQ, CAN_BS1_15TQ, CAN_BS2_5TQ }, // 500 KBit/s 76.19%
	{ eCP_BITRATE_1M, 2, CAN_SJW_1TQ, CAN_BS1_15TQ, CAN_BS2_5TQ }  // 1 MBit/s 76.19%
};

static const struct hal_baudrate hal_baudrate_timing_48mhz[] =
{
	{ eCP_BITRATE_10K, 300, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 10 KBit/s 87.5%
	{ eCP_BITRATE_20K, 150, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 20 KBit/s 87.5%
	{ eCP_BITRATE_50K, 60, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 50 KBit/s 87.5%
	{ eCP_BITRATE_100K, 30, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 100 KBit/s 87.5%
	{ eCP_BITRATE_125K, 24, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 125 KBit/s 87.5%
	{ eCP_BITRATE_250K, 12, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 250 KBit/s 87.5%
	{ eCP_BITRATE_500K, 6, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }, // 500 KBit/s 87.5%
	{ eCP_BITRATE_800K, 4, CAN_SJW_1TQ, CAN_BS1_12TQ, CAN_BS2_2TQ }, // 800 KBit/s 86.7 %
	{ eCP_BITRATE_1M, 3, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ }  // 1 MBit/s 87.5%
};

//-------------------------------------------------------------------
// simulation of CAN message buffer
//
static CpCanMsg_ts atsCanMsgS[CP_BUFFER_MAX];
static CpFifo_ts * aptsFifoS[CP_BUFFER_MAX];
static uint32_t aulAccMaskS[CP_BUFFER_MAX];

static int8_t buffer_direction[CP_BUFFER_MAX]; // 0 tx, 1 rx, -1 not used

//-------------------------------------------------------------------
// these pointers store the callback handlers
//
static CpRcvHandler_Fn /*@null@*/pfnRcvHandlerS = CPP_NULL;
static CpTrmHandler_Fn /*@null@*/pfnTrmHandlerS = CPP_NULL;
static CpErrHandler_Fn /*@null@*/pfnErrHandlerS = CPP_NULL;

static CpStatus_tv get_next_free_filter_number(uint8_t *filter_number);
static HAL_StatusTypeDef can_filter_config(uint32_t ulIdentifierV, uint32_t ulAcceptMaskV, uint8_t ubFormatV, uint8_t filter_number, uint32_t fifo, bool_t activate);
static CpStatus_tv can_filter_init(uint8_t ubBufferIdxV, uint32_t ulIdentifierV, uint32_t ulAcceptMaskV, uint8_t ubFormatV);
static CpStatus_tv can_filter_clear_all(void);
static CpStatus_tv search_for_already_defined_filter(uint8_t ubBufferIdxV, uint8_t *filter_number);

static HAL_StatusTypeDef HAL_CAN_Transmit_IT_MOD(CAN_HandleTypeDef* hcan, uint8_t ubBufferIdxV, uint8_t *used_tx_mailbox);

/*----------------------------------------------------------------------------*\
** Function implementation                                                    **
 **                                                                            **
 \*----------------------------------------------------------------------------*/
static CpStatus_tv CheckParam(const CpPort_ts * ptsPortV, const uint8_t ubBufferIdxV, const uint8_t unReqStateV)
{
	CpStatus_tv tvStatusT = eCP_ERR_CHANNEL;

	//----------------------------------------------------------------
	// test CAN port
	//
	if (ptsPortV != (CpPort_ts *) 0L)
	{
		tvStatusT = eCP_ERR_INIT_MISSING;

		//--------------------------------------------------------
		// check for initialization
		//
		if (ptsPortV->ubDrvInfo >= unReqStateV)
		{
			tvStatusT = eCP_ERR_BUFFER;

			//------------------------------------------------
			// check for valid buffer number
			//
			if (ubBufferIdxV < CP_BUFFER_MAX)
			{
				tvStatusT = eCP_ERR_NONE;
			}
		}
	}

	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreBitrate()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBitrate(CpPort_ts * ptsPortV, int32_t slNomBitRateV, int32_t slDatBitRateV)
{
	uint32_t pclk;
	const struct hal_baudrate *p_hal_baudrate;
	const struct hal_baudrate *p_hal_end;
	CpStatus_tv tvStatusT = eCP_ERR_CHANNEL;

	//----------------------------------------------------------------
	// test CAN port
	//
	if (ptsPortV != (CpPort_ts *) 0L)
	{
		if (ptsPortV->ubDrvInfo > eDRV_INFO_OFF)
		{
			tvStatusT = eCP_ERR_NONE;

			//-----------------------------------------------------
			// test bit-rate
			//
			if ((slNomBitRateV > eCP_BITRATE_1M) || (slNomBitRateV == eCP_BITRATE_NONE))
			{
				tvStatusT = eCP_ERR_BITRATE;
			}
			if ((slDatBitRateV != eCP_BITRATE_NONE) && (slNomBitRateV > slDatBitRateV))
			{
				tvStatusT = eCP_ERR_BITRATE;
			}

			//-----------------------------------------------------
			// configure the btr register here
			//
			if (tvStatusT == eCP_ERR_NONE)
			{
				pclk = HAL_RCC_GetPCLK1Freq();

				switch (pclk)
				{
					case 21000000:
						return (eCP_ERR_NOT_SUPPORTED);
						break;
					case 36000000:
						p_hal_baudrate = hal_baudrate_timing_36mhz;
						p_hal_end = hal_baudrate_timing_36mhz + sizeof(hal_baudrate_timing_36mhz) / sizeof(hal_baudrate_timing_36mhz[0]);
						break;
					case 42000000:
						p_hal_baudrate = hal_baudrate_timing_42mhz;
						p_hal_end = hal_baudrate_timing_42mhz + sizeof(hal_baudrate_timing_42mhz) / sizeof(hal_baudrate_timing_42mhz[0]);
						break;

					case 48000000:
						p_hal_baudrate = hal_baudrate_timing_48mhz;
						p_hal_end = hal_baudrate_timing_48mhz + sizeof(hal_baudrate_timing_48mhz) / sizeof(hal_baudrate_timing_48mhz[0]);
						break;

					default:
						return (eCP_ERR_BITRATE);
						break;
				}

				while (p_hal_baudrate < p_hal_end)
				{
					if (p_hal_baudrate->bitrate == slNomBitRateV)
					{
						HCAN1.Init.Prescaler = p_hal_baudrate->prescaler;
						HCAN1.Init.SJW = p_hal_baudrate->sjw;
						HCAN1.Init.BS1 = p_hal_baudrate->bs1;
						HCAN1.Init.BS2 = p_hal_baudrate->bs2;

						if (HAL_CAN_DeInit(&HCAN1) != HAL_OK)
						{
							return (eCP_ERR_BITRATE);
						}

						if (HAL_CAN_Init(&HCAN1) == HAL_OK)
						{
							// Bitrate change works fine so return;
							return (eCP_ERR_NONE);
						}
						else
						{
							return (eCP_ERR_BITRATE);
						}
					}
					p_hal_baudrate++;
				}
			}
		}
		else
		{
			tvStatusT = eCP_ERR_INIT_MISSING;
		}
	}
	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreBufferConfig()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferConfig(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, uint32_t ulIdentifierV, uint32_t ulAcceptMaskV, uint8_t ubFormatV, uint8_t ubDirectionV)
{
	CpStatus_tv tvStatusT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{
		//--------------------------------------------------------
		// test message format and mask identifier
		//
		switch (ubFormatV & CP_MASK_MSG_FORMAT)
		{
			case CP_MSG_FORMAT_CBFF:
			case CP_MSG_FORMAT_FBFF:
				ulIdentifierV = ulIdentifierV & CP_MASK_STD_FRAME;
				ulAcceptMaskV = ulAcceptMaskV & CP_MASK_STD_FRAME;
				break;

			case CP_MSG_FORMAT_CEFF:
			case CP_MSG_FORMAT_FEFF:
				ulIdentifierV = ulIdentifierV & CP_MASK_EXT_FRAME;
				ulAcceptMaskV = ulAcceptMaskV & CP_MASK_EXT_FRAME;
				break;
		}

		// save identifier
		atsCanMsgS[ubBufferIdxV].ulIdentifier = ulIdentifierV;

		// save format here in message control?
		atsCanMsgS[ubBufferIdxV].ubMsgCtrl = ubFormatV;

		switch (ubDirectionV)
		{
			case eCP_BUFFER_DIR_RCV:
				// save acceptance mask
				aulAccMaskS[ubBufferIdxV] = ulAcceptMaskV;
				buffer_direction[ubBufferIdxV] = BUFFER_RX;
				can_filter_init(ubBufferIdxV, ulIdentifierV, ulAcceptMaskV, ubFormatV);
				break;

			case eCP_BUFFER_DIR_TRM:
				buffer_direction[ubBufferIdxV] = BUFFER_TX;
				break;
		}
	}
	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreBufferGetData()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferGetData(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, uint8_t * pubDestDataV, uint8_t ubStartPosV, uint8_t ubSizeV)
{
	CpStatus_tv tvStatusT;
	uint8_t ubCntT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{
		//--------------------------------------------------------
		// test start position and size
		//
		if ((ubStartPosV + ubSizeV) > CP_DATA_SIZE)
		{
			tvStatusT = eCP_ERR_PARAM;
		}
		else
		{
			//---------------------------------------------------
			// copy data from simulated CAN buffer
			//
			for (ubCntT = ubStartPosV; ubCntT < ubSizeV; ubCntT++)
			{
				*pubDestDataV = CpMsgGetData(&atsCanMsgS[ubBufferIdxV], ubCntT);
				pubDestDataV++;
			}
		}
	}

	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreBufferGetDlc()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferGetDlc(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, uint8_t * pubDlcV)
{
	CpStatus_tv tvStatusT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{
		//----------------------------------------------------------------
		// read DLC from simulated CAN buffer
		//
		*pubDlcV = atsCanMsgS[ubBufferIdxV].ubMsgDLC;
	}
	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreBufferRelease()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferRelease(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV)
{
	CpStatus_tv tvStatusT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{
		buffer_direction[ubBufferIdxV] = BUFFER_NOT_USED;
	}

	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreBufferSend()                                                         //
// send message out of the CAN controller                                     //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferSend(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV)
{
	uint8_t i;
	CpStatus_tv tvStatusT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{
		//-----------------------------------------------------------------
		// setup identifier
		//
		if (CpMsgIsExtended(&atsCanMsgS[ubBufferIdxV]))
		{
			HCAN1.pTxMsg->ExtId = atsCanMsgS[ubBufferIdxV].ulIdentifier;
			HCAN1.pTxMsg->IDE = CAN_ID_EXT;
		}
		else
		{
			HCAN1.pTxMsg->StdId = atsCanMsgS[ubBufferIdxV].ulIdentifier;
			HCAN1.pTxMsg->IDE = CAN_ID_STD;
		}

		//-----------------------------------------------------------------
		// check for RTR bit
		//
		if (CpMsgIsRemote(&atsCanMsgS[ubBufferIdxV]))
		{
			HCAN1.pTxMsg->RTR = CAN_RTR_REMOTE;
		}
		else
		{
			HCAN1.pTxMsg->RTR = CAN_RTR_DATA;
		}

		HCAN1.pTxMsg->DLC = atsCanMsgS[ubBufferIdxV].ubMsgDLC;

		for (i = 0; i < atsCanMsgS[ubBufferIdxV].ubMsgDLC; i++)
		{
			HCAN1.pTxMsg->Data[i] = atsCanMsgS[ubBufferIdxV].aubData[i];
		}

		if (HAL_CAN_Transmit_IT_MOD(&HCAN1, ubBufferIdxV, &tx_mailbox_to_buffer[0]) != HAL_OK)
		{
#ifdef DEBUG
			printf("Transmission error\r\n");
#endif
			return eCP_ERR_TRM_FULL;
		}
	}
	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreBufferSetData()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferSetData(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, uint8_t * pubSrcDataV, uint8_t ubStartPosV, uint8_t ubSizeV)
{
	CpStatus_tv tvStatusT;
	uint8_t ubCntT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{
		//--------------------------------------------------------
		// test start position and size
		//
		if ((ubStartPosV + ubSizeV) > CP_DATA_SIZE)
		{
			tvStatusT = eCP_ERR_PARAM;
		}
		else
		{
			//---------------------------------------------------
			// copy data from simulated CAN buffer
			//
			for (ubCntT = ubStartPosV; ubCntT < ubSizeV; ubCntT++)
			{
				CpMsgSetData(&atsCanMsgS[ubBufferIdxV], ubCntT, *pubSrcDataV);
				pubSrcDataV++;
			}
		}
	}
	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreBufferSetDlc()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferSetDlc(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, uint8_t ubDlcV)
{
	CpStatus_tv tvStatusT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{
		//--------------------------------------------------------
		// write DLC to simulated CAN buffer
		//
		atsCanMsgS[ubBufferIdxV].ubMsgDLC = ubDlcV;
	}

	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreCanMode()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreCanMode(CpPort_ts * ptsPortV, uint8_t ubModeV)
{
	HAL_StatusTypeDef hal_status = HAL_OK;
	CpStatus_tv tvStatusT = eCP_ERR_CHANNEL;

	//----------------------------------------------------------------
	// test CAN port
	//
	if (ptsPortV != (CpPort_ts *) 0L)
	{
		if (ptsPortV->ubDrvInfo > eDRV_INFO_OFF)
		{
			tvStatusT = eCP_ERR_NONE;
//----------------------------------------------------------------
// switch CAN controller into mode "ubModeV"
//
			switch (ubModeV)
			{
				//--------------------------------------------------------
				// Stop the CAN controller (passive on the bus)
				//
				case eCP_MODE_STOP:

					// Transmit mailbox empty Interrupt
					__HAL_CAN_DISABLE_IT(&HCAN1, CAN_IT_TME);

					// FIFO 0 message pending Interrupt
					__HAL_CAN_DISABLE_IT(&HCAN1, CAN_IT_FMP0);

					// FIFO 1 message pending Interrupt
					__HAL_CAN_DISABLE_IT(&HCAN1, CAN_IT_FMP1);

					// Error warning, Error passive, Bus-off, Last error code and Error Interrupts
					__HAL_CAN_DISABLE_IT(&HCAN1, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR);

					// Request initialization
					HAL_CAN_Sleep(&HCAN1);

					break;

					//--------------------------------------------------------
					// Start the CAN controller (active on the bus)
					//
				case eCP_MODE_START:

					// Transmit mailbox empty Interrupt
					__HAL_CAN_ENABLE_IT(&HCAN1, CAN_IT_TME);
#if 1
					(void) HAL_CAN_Receive_IT(&HCAN1, CAN_FIFO0);
					(void) HAL_CAN_Receive_IT(&HCAN1, CAN_FIFO1);
#else
					// FIFO 0 message pending Interrupt
					__HAL_CAN_ENABLE_IT(&HCAN1, CAN_IT_FMP0);

					// FIFO 1 message pending Interrupt
					__HAL_CAN_ENABLE_IT(&HCAN1, CAN_IT_FMP1);
#endif
					// Request initialization
					HAL_CAN_WakeUp(&HCAN1);

					break;

					//--------------------------------------------------------
					// Start the CAN controller (Listen-Only)
					//
				case eCP_MODE_LISTEN_ONLY:
					HCAN1.Init.Mode = CAN_MODE_SILENT;

					// Transmit mailbox empty Interrupt
					__HAL_CAN_ENABLE_IT(&HCAN1, CAN_IT_TME);
#if 1
					(void) HAL_CAN_Receive_IT(&HCAN1, CAN_FIFO0);
					(void) HAL_CAN_Receive_IT(&HCAN1, CAN_FIFO1);
#else
					// FIFO 0 message pending Interrupt
					__HAL_CAN_ENABLE_IT(&HCAN1, CAN_IT_FMP0);

					// FIFO 1 message pending Interrupt
					__HAL_CAN_ENABLE_IT(&HCAN1, CAN_IT_FMP1);
#endif
					HCAN1.Init.Mode = CAN_MODE_SILENT;
					hal_status = HAL_CAN_Init(&HCAN1);

					break;

					//--------------------------------------------------------
					// Other modes are not supported
					//
				default:
					return (eCP_ERR_NOT_SUPPORTED);
					break;
			}
		}
	}

	if (HAL_OK != hal_status)
	{
		return eCP_ERR_INIT_FAIL;
	}
	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreCanState()                                                           //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreCanState(CpPort_ts * ptsPortV, CpState_ts * ptsStateV)
{
	uint32_t hal_error;
	uint32_t hal_esr;

	CpStatus_tv tvStatusT = eCP_ERR_CHANNEL;

	//----------------------------------------------------------------
	// test CAN port
	//
	if (ptsPortV != (CpPort_ts *) 0L)
	{
		if (ptsPortV->ubDrvInfo > eDRV_INFO_OFF)
		{
			tvStatusT = eCP_ERR_NONE;

			// first set the state of the bus
			ptsStateV->ubCanErrState = eCP_STATE_BUS_ACTIVE;

			// so using this function is maybe not the right way because
			// the error is only handled trough the interrupt handler so
			// maybe it is better to read the status register by hand
			hal_error = HAL_CAN_GetError(&HCAN1);

			if (hal_error & HAL_CAN_ERROR_EWG)
			{
				ptsStateV->ubCanErrState = eCP_STATE_BUS_WARN;
			}

			if (hal_error & HAL_CAN_ERROR_EPV)
			{
				ptsStateV->ubCanErrState = eCP_STATE_BUS_PASSIVE;
			}

			if (hal_error & HAL_CAN_ERROR_BOF)
			{
				ptsStateV->ubCanErrState = eCP_STATE_BUS_OFF;
			}

			// now set the errors first start with error none
			ptsStateV->ubCanErrType = eCP_ERR_TYPE_NONE;

			if (hal_error & HAL_CAN_ERROR_STF)
			{
				ptsStateV->ubCanErrType = eCP_ERR_TYPE_STUFF;
			}

			if (hal_error & HAL_CAN_ERROR_FOR)
			{
				ptsStateV->ubCanErrType = eCP_ERR_TYPE_FORM;
			}

			if (hal_error & HAL_CAN_ERROR_ACK)
			{
				ptsStateV->ubCanErrType = eCP_ERR_TYPE_ACK;
			}

			// bit recessive error
			if (hal_error & HAL_CAN_ERROR_BD)
			{
				ptsStateV->ubCanErrType = eCP_ERR_TYPE_BIT0;
			}

			// bit dominant error
			if (hal_error & HAL_CAN_ERROR_BR)
			{
				ptsStateV->ubCanErrType = eCP_ERR_TYPE_BIT1;
			}

			if (hal_error & HAL_CAN_ERROR_CRC)
			{
				ptsStateV->ubCanErrType = eCP_ERR_TYPE_CRC;
			}

			//----------------------------------------------------------------
			// get current error counter
			//
			hal_esr = (HCAN1.Instance->ESR) >> 16;
			ptsStateV->ubCanTrmErrCnt = (uint8_t) (hal_esr & 0x000000FF);
			hal_esr = hal_esr >> 8;
			ptsStateV->ubCanRcvErrCnt = (uint8_t) (hal_esr & 0x000000FF);

		}
	}
	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreDriverInit()                                                         //
// init CAN controller                                                        //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreDriverInit(uint8_t ubPhyIfV, CpPort_ts * ptsPortV, uint8_t ubConfigV)
{
	uint8_t i;
	CpStatus_tv tvStatusT = eCP_ERR_CHANNEL;

	//----------------------------------------------------------------
	// test physical CAN channel
	//
	if (ubPhyIfV == eCP_CHANNEL_1)
	{

#if CP_STATISTIC > 0
		tx_counter = 0;
		rx_counter = 0;
		err_counter = 0;
#endif

		//--------------------------------------------------------
		// test CAN port
		//
		if (ptsPortV != (CpPort_ts *) 0L)
		{
			if (ptsPortV->ubDrvInfo == eDRV_INFO_OFF)
			{
				ptsPortV->ubPhyIf = eCP_CHANNEL_1;
				ptsPortV->ubDrvInfo = eDRV_INFO_INIT;

				//----------------------------------------------
				// hardware initialization
				//

				// release all buffers and
				for (i = eCP_BUFFER_1; i < CP_BUFFER_MAX; i++)
				{
					CpCoreBufferRelease(ptsPortV, i);
				}

				// clear filter to buffer mapping
				for (i = 0; i < MAX_CAN_FILTER_NUMBER; i++)
				{
					filter_to_cp_buffer[i] = BUFFER_NOT_USED;
				}

				HCAN1.Instance = CAN1;
				HCAN1.pTxMsg = &can_tx_msg;
				HCAN1.pRxMsg = &can_rx_msg_fifo0;
				//HCAN1.pRx1Msg = &can_rx_msg_fifo1;
				HCAN1.pRx1Msg = &can_rx_msg_fifo0;

				HCAN1.Init.Mode = CAN_MODE_NORMAL;
				HCAN1.Init.Prescaler = 8;
				HCAN1.Init.SJW = CAN_SJW_1TQ;
				HCAN1.Init.BS1 = CAN_BS1_15TQ;
				HCAN1.Init.BS2 = CAN_BS2_5TQ;

				HCAN1.Init.TTCM = DISABLE;
				HCAN1.Init.ABOM = DISABLE;
				HCAN1.Init.AWUM = DISABLE;
				HCAN1.Init.NART = DISABLE;
				HCAN1.Init.RFLM = DISABLE;
				HCAN1.Init.TXFP = DISABLE;

				if (can_filter_clear_all() != eCP_ERR_NONE)
				{
					return (eCP_ERR_INIT_FAIL);
				}

				if (HAL_CAN_Init(&HCAN1) != HAL_OK)
				{
					return (eCP_ERR_INIT_FAIL);
				}

				tvStatusT = eCP_ERR_NONE;
			}
			else
			{
				//---------------------------------------------
				// already initialized
				//
				tvStatusT = eCP_ERR_INIT_FAIL;
			}
		}
		else
		{
			//-----------------------------------------------------
			// parameter ptsPortV is not correct
			//
			tvStatusT = eCP_ERR_PARAM;
		}
	}
	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreDriverRelease()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreDriverRelease(CpPort_ts * ptsPortV)
{
	CpStatus_tv tvStatusT = eCP_ERR_CHANNEL;
	HAL_StatusTypeDef hal_status;

	//----------------------------------------------------------------
	// test CAN port
	//
	if (ptsPortV != (CpPort_ts *) 0L)
	{
		if (ptsPortV->ubDrvInfo > eDRV_INFO_OFF)
		{
			tvStatusT = CpCoreCanMode(ptsPortV, eCP_MODE_STOP);
			ptsPortV->ubDrvInfo = eDRV_INFO_OFF;

			hal_status = HAL_CAN_DeInit(&HCAN1);

			if ((HAL_OK != hal_status) && (tvStatusT != eCP_ERR_NONE))
			{
				return eCP_ERR_INIT_FAIL;
			}

		}
		else
		{
			tvStatusT = eCP_ERR_INIT_MISSING;
		}
	}
	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreFifoConfig()                                                         //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreFifoConfig(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, CpFifo_ts * ptsFifoV)
{
	CpStatus_tv tvStatusT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{
		if (ptsFifoV != (CpFifo_ts *) 0)
		{
			aptsFifoS[ubBufferIdxV] = ptsFifoV;
		}
		else
		{
			tvStatusT = eCP_ERR_FIFO_PARAM;
		}
	}

	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreFifoRead()                                                           //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreFifoRead(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, CpCanMsg_ts * ptsCanMsgV, uint32_t * pulBufferSizeV)
{
	CpFifo_ts * ptsFifoT;
	CpStatus_tv tvStatusT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{
		if (pulBufferSizeV != (uint32_t *) 0L)
		{
			if (ptsCanMsgV != (CpCanMsg_ts *) 0L)
			{
				ptsFifoT = aptsFifoS[ubBufferIdxV];
				if (CpFifoIsEmpty(ptsFifoT))
				{

				}
			}
		}
	}

	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreFifoRelease()                                                        //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreFifoRelease(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV)
{
	CpStatus_tv tvStatusT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{

	}

	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreFifoWrite()                                                          //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreFifoWrite(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, CpCanMsg_ts * ptsCanMsgV, uint32_t * pulBufferSizeV)
{
	CpFifo_ts * ptsFifoT;
	CpStatus_tv tvStatusT;

	//----------------------------------------------------------------
	// test parameter ptsPortV and ubBufferIdxV
	//
	tvStatusT = CheckParam(ptsPortV, ubBufferIdxV, eDRV_INFO_INIT);
	if (tvStatusT == eCP_ERR_NONE)
	{
		if (pulBufferSizeV != (uint32_t *) 0L)
		{
			if (ptsCanMsgV != (CpCanMsg_ts *) 0L)
			{
				ptsFifoT = aptsFifoS[ubBufferIdxV];
				if (CpFifoIsFull(ptsFifoT))
				{

				}
			}
		}
	}

	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreHDI()                                                                //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreHDI(CpPort_ts * ptsPortV, CpHdi_ts * ptsHdiV)
{
	CpStatus_tv tvStatusT = eCP_ERR_CHANNEL;

	//----------------------------------------------------------------
	// test CAN port
	//
	if (ptsPortV != (CpPort_ts *) 0L)
	{
		if (ptsPortV->ubDrvInfo > eDRV_INFO_OFF)
		{
			tvStatusT = eCP_ERR_NONE;

			if (ptsHdiV != (CpHdi_ts *) 0)
			{
				//--------------------------------------------------
				// get version number
				//
				ptsHdiV->ubVersionMajor = CP_VERSION_MAJOR;
				ptsHdiV->ubVersionMinor = CP_VERSION_MINOR;

				ptsHdiV->ubCanFeatures = 0;
				ptsHdiV->ubDriverFeatures = 0;

				ptsHdiV->ubBufferMax = CP_BUFFER_MAX;

			}
			else
			{
				tvStatusT = eCP_ERR_PARAM;
			}
		}
	}

	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreIntFunctions()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreIntFunctions(CpPort_ts * ptsPortV, CpRcvHandler_Fn pfnRcvHandlerV, CpTrmHandler_Fn pfnTrmHandlerV, CpErrHandler_Fn pfnErrHandlerV)
{
	CpStatus_tv tvStatusT = eCP_ERR_CHANNEL;

	//----------------------------------------------------------------
	// test CAN port
	//
	if (ptsPortV != (CpPort_ts *) 0L)
	{
		if (ptsPortV->ubDrvInfo > eDRV_INFO_OFF)
		{
			tvStatusT = eCP_ERR_NONE;

			//-----------------------------------------------------
			// store the new callback
			//
			pfnRcvHandlerS = pfnRcvHandlerV;
			pfnTrmHandlerS = pfnTrmHandlerV;
			pfnErrHandlerS = pfnErrHandlerV;
		}
	}

	return (tvStatusT);
}

//----------------------------------------------------------------------------//
// CpCoreStatistic()                                                          //
// return statistical information                                             //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreStatistic(CpPort_ts * ptsPortV, CpStatistic_ts * ptsStatsV)
{
	CpStatus_tv tvStatusT = eCP_ERR_CHANNEL;

	//----------------------------------------------------------------
	// test CAN port
	//
	if (ptsPortV != (CpPort_ts *) 0L)
	{
		if (ptsPortV->ubDrvInfo > eDRV_INFO_OFF)
		{
			tvStatusT = eCP_ERR_NONE;

			ptsStatsV->ulErrMsgCount = 0;
			ptsStatsV->ulRcvMsgCount = 0;
			ptsStatsV->ulTrmMsgCount = 0;
		}
	}
	return (tvStatusT);
}

/**
 * @param ulIdentifierV
 * @param ulAcceptMaskV
 * @param ubFormatV
 * @param filter_number
 * @param activate
 * @return
 */
static HAL_StatusTypeDef can_filter_config(uint32_t ulIdentifierV, uint32_t ulAcceptMaskV, uint8_t ubFormatV, uint8_t filter_number, uint32_t fifo, bool_t activate)
{
	CAN_FilterConfTypeDef filter_config;

	// config filter only use 32Bit
	filter_config.FilterNumber = filter_number;
	filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
	filter_config.FilterScale = CAN_FILTERSCALE_32BIT;

	if ((ubFormatV & CP_MASK_MSG_FORMAT) == CP_MSG_FORMAT_CBFF)
	{
		filter_config.FilterIdHigh = ulIdentifierV << 5;
		filter_config.FilterIdLow = 0x0;
		filter_config.FilterMaskIdHigh = ulAcceptMaskV << 5;
		filter_config.FilterMaskIdLow = 0x0; // allows both remote and data frames
	}
	else //  if((ubFormatV & CP_MSG_FORMAT_MASK) == CP_MSG_FORMAT_CEFF)
	{
		filter_config.FilterIdHigh = ulIdentifierV >> 13; // EXTID[28:13]
		filter_config.FilterIdLow = (0x00FF & (ulIdentifierV << 3)) | (1 << 2); // EXTID[12:0]
		filter_config.FilterMaskIdHigh = ulAcceptMaskV >> 13;
		filter_config.FilterMaskIdLow = (0x00FF & (ulAcceptMaskV << 3)) | (1 << 2);
	}

	filter_config.FilterFIFOAssignment = fifo;
	filter_config.FilterActivation = activate;
	filter_config.BankNumber = 14;

	return HAL_CAN_ConfigFilter(&HCAN1, &filter_config);
}

/**
 * @param filter_number
 * @return
 */
static CpStatus_tv get_next_free_filter_number(uint8_t *filter_number)
{
	uint8_t i;

	for (i = 0; i < MAX_CAN_FILTER_NUMBER; ++i)
	{
		if (filter_to_cp_buffer[i] == BUFFER_NOT_USED)
		{
			*filter_number = i;
			return eCP_ERR_NONE;
		}
	}

	return eCP_ERR_INIT_FAIL;
}

/**
 * @param ubBufferIdxV
 * @param filter_number
 * @return
 */
static CpStatus_tv search_for_already_defined_filter(uint8_t ubBufferIdxV, uint8_t *filter_number)
{
	uint8_t i;

	for (i = 0; i < MAX_CAN_FILTER_NUMBER; ++i)
	{
		if (filter_to_cp_buffer[i] == ubBufferIdxV)
		{
			*filter_number = i;
			return eCP_ERR_NONE;
		}
	}
	return eCP_ERR_INIT_FAIL;
}

/**
 * @param ubBufferIdxV
 * @param ulIdentifierV
 * @param ulAcceptMaskV
 * @param ubFormatV
 * @return
 */
static CpStatus_tv can_filter_init(uint8_t ubBufferIdxV, uint32_t ulIdentifierV, uint32_t ulAcceptMaskV, uint8_t ubFormatV)
{
	CpStatus_tv status;
	uint8_t filter_number;
	HAL_StatusTypeDef hal_status;
	static uint32_t fifo_number = CAN_FIFO0;

	// check if the buffer is already defined to a filter
	status = search_for_already_defined_filter(ubBufferIdxV, &filter_number);
	// it is not really nice but I use the status to check if a filter exists or not
	if (eCP_ERR_INIT_FAIL == status)
	{
		// there is no filter assigned to this buffer so get a free one
		status = get_next_free_filter_number(&filter_number);
	}

	if (eCP_ERR_NONE == status)
	{
		// save the buffer index on the filter to buffer table
		filter_to_cp_buffer[filter_number] = ubBufferIdxV;

		// config filter
		hal_status = can_filter_config(ulIdentifierV, ulAcceptMaskV, ubFormatV, filter_number, fifo_number, 1);

#if 0
		// try to use booth FIFO in parallel
		if(fifo_number == CAN_FIFO0)
		{
			fifo_number = CAN_FIFO1;
		}
		else
		{
			fifo_number = CAN_FIFO0;
		}
#endif

		if (HAL_OK == hal_status)
		{
			return eCP_ERR_NONE;
		}
	}

	return eCP_ERR_INIT_FAIL;
}

/**
 *
 * @return
 */
static CpStatus_tv can_filter_clear_all(void)
{
	HAL_StatusTypeDef hal_status;
	uint8_t i;

	for (i = 0; i < MAX_CAN_FILTER_NUMBER; ++i)
	{
		hal_status = can_filter_config(0, 0, 0, i, CAN_FIFO0, 0);

		if (HAL_OK != hal_status)
		{
			return eCP_ERR_INIT_FAIL;
		}
	}

	return eCP_ERR_NONE;
}

#define CAN_TI0R_STID_BIT_POSITION    ((uint32_t)21)  /* Position of LSB bits STID in register CAN_TI0R */
#define CAN_TI0R_EXID_BIT_POSITION    ((uint32_t) 3)  /* Position of LSB bits EXID in register CAN_TI0R */
#define CAN_TDL0R_DATA0_BIT_POSITION  ((uint32_t) 0)  /* Position of LSB bits DATA0 in register CAN_TDL0R */
#define CAN_TDL0R_DATA1_BIT_POSITION  ((uint32_t) 8)  /* Position of LSB bits DATA1 in register CAN_TDL0R */
#define CAN_TDL0R_DATA2_BIT_POSITION  ((uint32_t)16)  /* Position of LSB bits DATA2 in register CAN_TDL0R */
#define CAN_TDL0R_DATA3_BIT_POSITION  ((uint32_t)24)  /* Position of LSB bits DATA3 in register CAN_TDL0R */

/**
 * @brief  Initiates and transmits a CAN frame message.
 * @param  hcan: pointer to a CAN_HandleTypeDef structure that contains
 *         the configuration information for the specified CAN.
 * @param ubBufferIdxV canpie buffer number
 * @param used_tx_mailbox used mailbox for transmit
 * @retval HAL status
 */
static HAL_StatusTypeDef HAL_CAN_Transmit_IT_MOD(CAN_HandleTypeDef* hcan, uint8_t ubBufferIdxV, uint8_t *used_tx_mailbox)
{
	uint32_t transmitmailbox = CAN_TXSTATUS_NOMAILBOX;

	/* Check the parameters */
	assert_param(IS_CAN_IDTYPE(hcan->pTxMsg->IDE));
	assert_param(IS_CAN_RTR(hcan->pTxMsg->RTR));
	assert_param(IS_CAN_DLC(hcan->pTxMsg->DLC));

	if (((hcan->Instance->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) || ((hcan->Instance->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) || ((hcan->Instance->TSR & CAN_TSR_TME2) == CAN_TSR_TME2))
	{
		/* Process Locked */
		__HAL_LOCK(hcan);

		/* Select one empty transmit mailbox */
		if ((hcan->Instance->TSR & CAN_TSR_TME0) == CAN_TSR_TME0)
		{
			transmitmailbox = CAN_TXMAILBOX_0;
		}
		else if ((hcan->Instance->TSR & CAN_TSR_TME1) == CAN_TSR_TME1)
		{
			transmitmailbox = CAN_TXMAILBOX_1;
		}
		else
		{
			transmitmailbox = CAN_TXMAILBOX_2;
		}

		/* Set up the Id */
		hcan->Instance->sTxMailBox[transmitmailbox].TIR &= CAN_TI0R_TXRQ;
		if (hcan->pTxMsg->IDE == CAN_ID_STD)
		{
			assert_param(IS_CAN_STDID(hcan->pTxMsg->StdId));
			hcan->Instance->sTxMailBox[transmitmailbox].TIR |= ((hcan->pTxMsg->StdId << 21U) | hcan->pTxMsg->RTR);
		}
		else
		{
			assert_param(IS_CAN_EXTID(hcan->pTxMsg->ExtId));
			hcan->Instance->sTxMailBox[transmitmailbox].TIR |= ((hcan->pTxMsg->ExtId << 3U) | hcan->pTxMsg->IDE | hcan->pTxMsg->RTR);
		}

		/* Set up the DLC */
		hcan->pTxMsg->DLC &= (uint8_t) 0x0000000F;
		hcan->Instance->sTxMailBox[transmitmailbox].TDTR &= (uint32_t) 0xFFFFFFF0U;
		hcan->Instance->sTxMailBox[transmitmailbox].TDTR |= hcan->pTxMsg->DLC;

		/* Set up the data field */
		hcan->Instance->sTxMailBox[transmitmailbox].TDLR = (((uint32_t) hcan->pTxMsg->Data[3U] << 24U) | ((uint32_t) hcan->pTxMsg->Data[2U] << 16U) | ((uint32_t) hcan->pTxMsg->Data[1U] << 8U)
				| ((uint32_t) hcan->pTxMsg->Data[0U]));
		hcan->Instance->sTxMailBox[transmitmailbox].TDHR = (((uint32_t) hcan->pTxMsg->Data[7U] << 24U) | ((uint32_t) hcan->pTxMsg->Data[6U] << 16U) | ((uint32_t) hcan->pTxMsg->Data[5U] << 8U)
				| ((uint32_t) hcan->pTxMsg->Data[4U]));

		/* Change CAN state */
		switch (hcan->State)
		{
			case (HAL_CAN_STATE_BUSY_RX0):
				hcan->State = HAL_CAN_STATE_BUSY_TX_RX0;
				break;
			case (HAL_CAN_STATE_BUSY_RX1):
				hcan->State = HAL_CAN_STATE_BUSY_TX_RX1;
				break;
			case (HAL_CAN_STATE_BUSY_RX0_RX1):
				hcan->State = HAL_CAN_STATE_BUSY_TX_RX0_RX1;
				break;
			default: /* HAL_CAN_STATE_READY */
				hcan->State = HAL_CAN_STATE_BUSY_TX;
				break;
		}

		/* Set CAN error code to none */
		hcan->ErrorCode = HAL_CAN_ERROR_NONE;

		/* Process Unlocked */
		__HAL_UNLOCK(hcan);

		/* Request transmission */
		hcan->Instance->sTxMailBox[transmitmailbox].TIR |= CAN_TI0R_TXRQ;

		/* save which buffer used on this mailbox */
		used_tx_mailbox[transmitmailbox] = ubBufferIdxV;

		/* Enable Error warning, Error passive, Bus-off,
		 Last error and Error Interrupts */
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR | CAN_IT_TME);
	}
	else
	{
		/* Change CAN state */
		hcan->State = HAL_CAN_STATE_ERROR;

		/* Return function status */
		return HAL_ERROR;
	}

	return HAL_OK;
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan)
{
	uint8_t canpie_buffer_number = 0;
	uint32_t clear_flag = 0;

	if (__HAL_CAN_TRANSMIT_STATUS(hcan, CAN_TXMAILBOX_0))
	{
		canpie_buffer_number = tx_mailbox_to_buffer[CAN_TXMAILBOX_0];
		clear_flag = CAN_FLAG_RQCP0;
	}
	// else
	if (__HAL_CAN_TRANSMIT_STATUS(hcan, CAN_TXMAILBOX_1))
	{
		canpie_buffer_number = tx_mailbox_to_buffer[CAN_TXMAILBOX_1];
		clear_flag = CAN_FLAG_RQCP1;
	}
	// else
	if (__HAL_CAN_TRANSMIT_STATUS(hcan, CAN_TXMAILBOX_2))
	{
		canpie_buffer_number = tx_mailbox_to_buffer[CAN_TXMAILBOX_2];
		clear_flag = CAN_FLAG_RQCP2;
	}
//	else
//	{
//		// what should we do here
//		// return and call error callback or ignore because that could not happen
//	}

	if (CPP_NULL != pfnTrmHandlerS)
	{
		pfnTrmHandlerS(&atsCanMsgS[canpie_buffer_number], canpie_buffer_number);
	}

#if CP_STATISTIC > 0
	tx_counter++;
#endif

	__HAL_CAN_CLEAR_FLAG(hcan, clear_flag);

	// Disable Transmit mailbox empty Interrupt
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_TME);

}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	CpCanMsg_ts *pcan_msg;
	// get filter index
	uint8_t filter_number = hcan->pRxMsg->FMI;
	uint8_t canpie_buffer_number;

	canpie_buffer_number = filter_to_cp_buffer[filter_number];

	pcan_msg = &atsCanMsgS[canpie_buffer_number];

	if (hcan->pRxMsg->IDE == CAN_ID_STD)
	{
		CpMsgSetStdId(pcan_msg, hcan->pRxMsg->StdId);
	}
	else
	{
		CpMsgSetExtId(pcan_msg, hcan->pRxMsg->ExtId);
	}

	if (hcan->pRxMsg->RTR == CAN_RTR_REMOTE)
	{
		CpMsgSetRemote(pcan_msg);
	}
	else
	{
		CpMsgClrRemote(pcan_msg);
	}

	CpMsgSetDlc(pcan_msg, hcan->pRxMsg->DLC);

	pcan_msg->aubData[0] = hcan->pRxMsg->Data[0];
	pcan_msg->aubData[1] = hcan->pRxMsg->Data[1];
	pcan_msg->aubData[2] = hcan->pRxMsg->Data[2];
	pcan_msg->aubData[3] = hcan->pRxMsg->Data[3];
	pcan_msg->aubData[4] = hcan->pRxMsg->Data[4];
	pcan_msg->aubData[5] = hcan->pRxMsg->Data[5];
	pcan_msg->aubData[6] = hcan->pRxMsg->Data[6];
	pcan_msg->aubData[7] = hcan->pRxMsg->Data[7];

	//pcan_msg->ulMsgUser = hcan->pRxMsg->FIFONumber;

	if (CPP_NULL != pfnRcvHandlerS)
	{
		pfnRcvHandlerS(pcan_msg, canpie_buffer_number);
	}

#if CP_STATISTIC > 0
	rx_counter++;
#endif

#if 1
	HAL_CAN_Receive_IT(hcan, hcan->pRxMsg->FIFONumber);
#else
	// FIFO 0 message pending Interrupt
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP0);
	//__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FOV0 | CAN_IT_FMP0);

	// FIFO 1 message pending Interrupt
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_FMP1);
#endif
}

/**
 * @todo enable error callback for error counter
 * @param hcan
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
	static CpState_ts state;
	/**@todo fill state */
#if CP_STATISTIC > 0
	err_counter++;
#endif
	/* Enable Error warning, Error passive, Bus-off, Last error and Error Interrupts and FIFO0, FIFO1 */
	__HAL_CAN_ENABLE_IT(hcan, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR | CAN_IT_TME | CAN_IT_FOV0 | CAN_IT_FMP0 | CAN_IT_FOV1 | CAN_IT_FMP1);

	if (CPP_NULL != pfnErrHandlerS)
	{
		pfnErrHandlerS(&state);
	}
}
