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

static CanRxMsgTypeDef can_rx_msg;
static CanTxMsgTypeDef can_tx_msg;

static uint8_t filter_to_cp_buffer[MAX_CAN_FILTER_NUMBER];
static uint8_t tx_mailbox_to_buffer[3];

struct hal_baudrate
{
	enum CpBitrate_e bitrate;
	uint32_t prescaler;
	uint32_t sjw;
	uint32_t bs1;
	uint32_t bs2;
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
	{	eCP_BITRATE_10K, 280, CAN_SJW_1TQ, CAN_BS1_12TQ, CAN_BS2_2TQ}, // 10 KBit/s 86.7%
	{	eCP_BITRATE_20K, 140, CAN_SJW_1TQ, CAN_BS1_12TQ, CAN_BS2_2TQ}, // 20 KBit/s 86.7%
	{	eCP_BITRATE_50K, 56, CAN_SJW_1TQ, CAN_BS1_12TQ, CAN_BS2_2TQ}, // 50 KBit/s 86.7%
	{	eCP_BITRATE_100K, 28, CAN_SJW_1TQ, CAN_BS1_12TQ, CAN_BS2_2TQ}, // 100 KBit/s 86.7%
	{	eCP_BITRATE_125K, 21, CAN_SJW_1TQ, CAN_BS1_13TQ, CAN_BS2_2TQ}, // 125 KBit/s 87.5%
	{	eCP_BITRATE_250K, 12, CAN_SJW_1TQ, CAN_BS1_11TQ, CAN_BS2_2TQ}, // 250 KBit/s 85.7%
	{	eCP_BITRATE_500K, 6, CAN_SJW_1TQ, CAN_BS1_11TQ, CAN_BS2_2TQ}, // 500 KBit/s 85.7%
	{	eCP_BITRATE_1M, 3, CAN_SJW_1TQ, CAN_BS1_11TQ, CAN_BS2_2TQ}  // 1 MBit/s 85.7%
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
static int8_t buffer_direction[CP_BUFFER_MAX]; // 0 tx, 1 rx, -1 not used

//-------------------------------------------------------------------
// these pointers store the callback handlers
//
static uint8_t (*pfnRcvIntHandler)(CpCanMsg_ts *, uint8_t) = NULL;
static uint8_t (*pfnTrmIntHandler)(CpCanMsg_ts *, uint8_t) = NULL;
static uint8_t (*pfnErrIntHandler)(CpState_ts *) = NULL;

static CpStatus_tv get_next_free_filter_number(uint8_t *filter_number);
static HAL_StatusTypeDef can_filter_config(uint32_t ulIdentifierV, uint32_t ulAcceptMaskV, uint8_t ubFormatV, uint8_t filter_number, bool_t activate);
static CpStatus_tv can_filter_init(uint8_t ubBufferIdxV, uint32_t ulIdentifierV, uint32_t ulAcceptMaskV, uint8_t ubFormatV);
static CpStatus_tv can_filter_clear_all(void);

static HAL_StatusTypeDef HAL_CAN_Transmit_IT_MOD(CAN_HandleTypeDef* hcan, uint8_t ubBufferIdxV, uint8_t *used_tx_mailbox);

/*----------------------------------------------------------------------------*\
** Function implementation                                                    **
 **                                                                            **
 \*----------------------------------------------------------------------------*/

//----------------------------------------------------------------------------//
// CpCoreBitrate()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBitrate(CpPort_ts * ptsPortV, int32_t slNomBitRateV, int32_t slDatBitRateV)
{
	uint32_t pclk;
	const struct hal_baudrate *p_hal_baudrate;
	const struct hal_baudrate *p_hal_end;

//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// test bit-rate
//
	if (slNomBitRateV > eCP_BITRATE_1M)
	{
		return (eCP_ERR_BITRATE);
	}
	if (slDatBitRateV != eCP_BITRATE_NONE)
	{
		return (eCP_ERR_BITRATE);
	}

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
			hcan1.Init.Prescaler = p_hal_baudrate->prescaler;
			hcan1.Init.SJW = p_hal_baudrate->sjw;
			hcan1.Init.BS1 = p_hal_baudrate->bs1;
			hcan1.Init.BS2 = p_hal_baudrate->bs2;

			if (HAL_CAN_DeInit(&hcan1) != HAL_OK)
			{
				return (eCP_ERR_BITRATE);
			}

			if (HAL_CAN_Init(&hcan1) == HAL_OK)
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

	return (eCP_ERR_BITRATE);
}

//----------------------------------------------------------------------------//
// CpCoreBufferConfig()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferConfig(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, uint32_t ulIdentifierV, uint32_t ulAcceptMaskV, uint8_t ubFormatV, uint8_t ubDirectionV)
{
//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// check for valid buffer number
//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

//----------------------------------------------------------------
// test message format and mask identifier
//
	switch (ubFormatV & CP_MSG_FORMAT_MASK)
	{
		case CP_MSG_FORMAT_CBFF:
			ulIdentifierV = ulIdentifierV & CP_MASK_STD_FRAME;
			ulAcceptMaskV = ulAcceptMaskV & CP_MASK_STD_FRAME;
			atsCanMsgS[ubBufferIdxV - 1].ulIdentifier = ulIdentifierV;
			break;

		case CP_MSG_FORMAT_CEFF:
			ulIdentifierV = ulIdentifierV & CP_MASK_EXT_FRAME;
			ulAcceptMaskV = ulAcceptMaskV & CP_MASK_EXT_FRAME;
			atsCanMsgS[ubBufferIdxV - 1].ulIdentifier = ulIdentifierV;
			break;
	}

	// save format here?
	atsCanMsgS[ubBufferIdxV - 1].ubMsgCtrl = ubFormatV;

	switch (ubDirectionV)
	{
		case eCP_BUFFER_DIR_RCV:
			buffer_direction[ubBufferIdxV - 1] = BUFFER_RX;
			can_filter_init(ubBufferIdxV, ulIdentifierV, ulAcceptMaskV, ubFormatV);
			break;

		case eCP_BUFFER_DIR_TRM:
			buffer_direction[ubBufferIdxV - 1] = BUFFER_TX;
			break;
	}

	return (eCP_ERR_NONE);

}

//----------------------------------------------------------------------------//
// CpCoreBufferGetData()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferGetData(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, uint8_t * pubDestDataV, uint8_t ubStartPosV, uint8_t ubSizeV)
{
	uint8_t ubCntT;

//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// check for valid buffer number
//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

//----------------------------------------------------------------
// test start position and size
//
#if CP_CAN_FD > 0
#else
	if (ubStartPosV > CP_DATA_SIZE)
		return (eCP_ERR_PARAM);
	if (ubSizeV > 8)
		return (eCP_ERR_PARAM);
	if ((ubStartPosV + ubSizeV) > 8)
		return (eCP_ERR_PARAM);

#endif

//----------------------------------------------------------------
// copy data from simulated CAN buffer
//
	for (ubCntT = ubStartPosV; ubCntT < ubSizeV; ubCntT++)
	{
		*pubDestDataV = CpMsgGetData(&atsCanMsgS[ubBufferIdxV - 1], ubCntT);
		pubDestDataV++;
	}

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreBufferGetDlc()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferGetDlc(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, uint8_t * pubDlcV)
{
//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// check for valid buffer number
//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

//----------------------------------------------------------------
// read DLC from simulated CAN buffer
//
	*pubDlcV = atsCanMsgS[ubBufferIdxV - 1].ubMsgDLC;

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreBufferRelease()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferRelease(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV)
{
//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// check for valid buffer number
//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreBufferSend()                                                         //
// send message out of the CAN controller                                     //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferSend(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV)
{
	uint8_t i;
//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// check for valid buffer number
//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

	// remove 1 because buffer starts on 1
	ubBufferIdxV = ubBufferIdxV - 1;

	//-----------------------------------------------------------------
	// setup identifier
	//
	if (CpMsgIsExtended(&atsCanMsgS[ubBufferIdxV]))
	{
		hcan1.pTxMsg->ExtId = atsCanMsgS[ubBufferIdxV].ulIdentifier;
		hcan1.pTxMsg->IDE = CAN_ID_EXT;
	}
	else
	{
		hcan1.pTxMsg->StdId = atsCanMsgS[ubBufferIdxV].ulIdentifier;
		hcan1.pTxMsg->IDE = CAN_ID_STD;
	}

	//-----------------------------------------------------------------
	// check for RTR bit
	//
	if (CpMsgIsRemote(&atsCanMsgS[ubBufferIdxV]))
	{
		hcan1.pTxMsg->RTR = CAN_RTR_REMOTE;
	}
	else
	{
		hcan1.pTxMsg->RTR = CAN_RTR_DATA;
	}

	hcan1.pTxMsg->DLC = atsCanMsgS[ubBufferIdxV].ubMsgDLC;

	for (i = 0; i < atsCanMsgS[ubBufferIdxV].ubMsgDLC; i++)
	{
		hcan1.pTxMsg->Data[i] = atsCanMsgS[ubBufferIdxV].aubData[i];
	}

	if (HAL_CAN_Transmit_IT_MOD(&hcan1, ubBufferIdxV + 1, &tx_mailbox_to_buffer[0]) != HAL_OK)
	{
#ifdef DEBUG
		printf("Transmission error\r\n");
#endif

		return eCP_ERR_TRM_FULL;
	}

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreBufferSetData()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferSetData(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, uint8_t * pubSrcDataV, uint8_t ubStartPosV, uint8_t ubSizeV)
{
	uint8_t ubCntT;

//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// check for valid buffer number
//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

//----------------------------------------------------------------
// copy data to simulated CAN buffer
//
	for (ubCntT = ubStartPosV; ubCntT < ubSizeV; ubCntT++)
	{
		CpMsgSetData(&atsCanMsgS[ubBufferIdxV - 1], ubCntT, *pubSrcDataV);
		pubSrcDataV++;
	}

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreBufferSetDlc()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreBufferSetDlc(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, uint8_t ubDlcV)
{
//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// check for valid buffer number
//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

//----------------------------------------------------------------
// write DLC to simulated CAN buffer
//
	atsCanMsgS[ubBufferIdxV - 1].ubMsgDLC = ubDlcV;

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreCanMode()                                                            //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreCanMode(CpPort_ts * ptsPortV, uint8_t ubModeV)
{
	HAL_StatusTypeDef hal_status = HAL_OK;

//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// switch CAN controller into mode "ubModeV"
//
	switch (ubModeV)
	{
		//--------------------------------------------------------
		// Stop the CAN controller (passive on the bus)
		//
		case eCP_MODE_STOP:

			// Disable Transmit mailbox empty Interrupt
			__HAL_CAN_DISABLE_IT(&hcan1, CAN_IT_TME);

			// Disable FIFO 0 message pending Interrupt
			__HAL_CAN_DISABLE_IT(&hcan1, CAN_IT_FMP0);

			// Disable FIFO 1 message pending Interrupt
			__HAL_CAN_DISABLE_IT(&hcan1, CAN_IT_FMP1);

			// Disable Error warning, Error passive, Bus-off, Last error code and Error Interrupts
			__HAL_CAN_DISABLE_IT(&hcan1, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR);

			// Request initialization
			HAL_CAN_Sleep(&hcan1);

			break;

			//--------------------------------------------------------
			// Start the CAN controller (active on the bus)
			//
		case eCP_MODE_START:

			// Disable Transmit mailbox empty Interrupt
			__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_TME);

			// Disable FIFO 0 message pending Interrupt
			__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);

			// Request initialization
			HAL_CAN_WakeUp(&hcan1);

			break;

			//--------------------------------------------------------
			// Start the CAN controller (Listen-Only)
			//
		case eCP_MODE_LISTEN_ONLY:
			hcan1.Init.Mode = CAN_MODE_SILENT;

			// Enable Transmit mailbox empty Interrupt
			__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_TME);

			// Enable FIFO 0 message pending Interrupt
			__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
			hcan1.Init.Mode = CAN_MODE_SILENT;
			hal_status = HAL_CAN_Init(&hcan1);

			break;

			//--------------------------------------------------------
			// Other modes are not supported
			//
		default:
			return (eCP_ERR_NOT_SUPPORTED);
			break;
	}

	if (HAL_OK != hal_status)
	{
		return eCP_ERR_INIT_FAIL;
	}

	return eCP_ERR_NONE;
}

//----------------------------------------------------------------------------//
// CpCoreCanState()                                                           //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreCanState(CpPort_ts * ptsPortV, CpState_ts * ptsStateV)
{
//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// get current error counter
//
	ptsStateV->ubCanTrmErrCnt = 0;
	ptsStateV->ubCanRcvErrCnt = 0;

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreDriverInit()                                                         //
// init CAN controller                                                        //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreDriverInit(uint8_t ubPhyIfV, CpPort_ts * ptsPortV, uint8_t ubConfigV)
{
	uint8_t i;

#if CP_SMALL_CODE == 0
	if (ubPhyIfV != eCP_CHANNEL_1)
	{
		return (eCP_ERR_CHANNEL);
	}

	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

	// release all buffers and
	for (i = eCP_BUFFER_1; i <= CP_BUFFER_MAX; i++)
	{
		CpCoreBufferRelease(ptsPortV, i);
		buffer_direction[i - 1] = BUFFER_NOT_USED;
	}

	// clear filter to buffer mapping
	for (i = 0; i < MAX_CAN_FILTER_NUMBER; i++)
	{
		filter_to_cp_buffer[i] = 0;
	}

	hcan1.Instance = CAN1;
	hcan1.pTxMsg = &can_tx_msg;
	hcan1.pRxMsg = &can_rx_msg;

	hcan1.Init.Prescaler = 8;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_1TQ;
	hcan1.Init.BS1 = CAN_BS1_15TQ;
	hcan1.Init.BS2 = CAN_BS2_5TQ;

	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = DISABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = DISABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;

	if (can_filter_clear_all() != eCP_ERR_NONE)
	{
		return (eCP_ERR_INIT_FAIL);
	}

	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
		return (eCP_ERR_INIT_FAIL);
	}

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreDriverRelease()                                                      //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreDriverRelease(CpPort_ts * ptsPortV)
{
	CpStatus_tv tvStatusT;
	HAL_StatusTypeDef hal_status;

	tvStatusT = CpCoreCanMode(ptsPortV, eCP_MODE_STOP);

	hal_status = HAL_CAN_DeInit(&hcan1);

	if ((HAL_OK != hal_status) && (tvStatusT != eCP_ERR_NONE))
	{
		return eCP_ERR_INIT_FAIL;
	}

	return eCP_ERR_NONE;
}

//----------------------------------------------------------------------------//
// CpCoreFifoConfig()                                                         //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreFifoConfig(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, CpFifo_ts * ptsFifoV)
{
	//----------------------------------------------------------------
	// test CAN port
	//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

	//----------------------------------------------------------------
	// check for valid buffer number
	//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

	if (ptsFifoV != 0)
	{

	}
	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreFifoRead()                                                           //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreFifoRead(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, CpCanMsg_ts * ptsCanMsgV, uint32_t * pulBufferSizeV)
{

	//----------------------------------------------------------------
	// test CAN port
	//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

	//----------------------------------------------------------------
	// check for valid buffer number
	//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

	if (pulBufferSizeV != (uint32_t *) 0L)
	{
		if (ptsCanMsgV != (CpCanMsg_ts *) 0L)
		{

		}
	}

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreFifoRelease()                                                        //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreFifoRelease(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV)
{
	//----------------------------------------------------------------
	// test CAN port
	//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

	//----------------------------------------------------------------
	// check for valid buffer number
	//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreFifoWrite()                                                          //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreFifoWrite(CpPort_ts * ptsPortV, uint8_t ubBufferIdxV, CpCanMsg_ts * ptsCanMsgV, uint32_t * pulBufferSizeV)
{
	//----------------------------------------------------------------
	// test CAN port
	//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

	//----------------------------------------------------------------
	// check for valid buffer number
	//
	if ((ubBufferIdxV < eCP_BUFFER_1) || (ubBufferIdxV > CP_BUFFER_MAX))
	{
		return (eCP_ERR_BUFFER);
	}

	if (pulBufferSizeV != (uint32_t *) 0L)
	{
		if (ptsCanMsgV != (CpCanMsg_ts *) 0L)
		{

		}
	}

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreHDI()                                                                //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreHDI(CpPort_ts * ptsPortV, CpHdi_ts * ptsHdiV)
{
//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// get version number
//
	ptsHdiV->ubVersionMajor = CP_VERSION_MAJOR;
	ptsHdiV->ubVersionMinor = CP_VERSION_MINOR;

	ptsHdiV->ubCanFeatures = 0;
	ptsHdiV->ubDriverFeatures = 0,

	ptsHdiV->ubBufferMax = CP_BUFFER_MAX;
	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreIntFunctions()                                                       //
//                                                                            //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreIntFunctions(CpPort_ts * ptsPortV, uint8_t (*pfnRcvHandler)(CpCanMsg_ts *, uint8_t), uint8_t (*pfnTrmHandler)(CpCanMsg_ts *, uint8_t), uint8_t (*pfnErrHandler)(CpState_ts *))
{
//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

//----------------------------------------------------------------
// store the new callbacks
//
	pfnRcvIntHandler = pfnRcvHandler;
	pfnTrmIntHandler = pfnTrmHandler;
	pfnErrIntHandler = pfnErrHandler;

	return (eCP_ERR_NONE);
}

//----------------------------------------------------------------------------//
// CpCoreStatistic()                                                          //
// return statistical information                                             //
//----------------------------------------------------------------------------//
CpStatus_tv CpCoreStatistic(CpPort_ts * ptsPortV, CpStatistic_ts * ptsStatsV)
{
//----------------------------------------------------------------
// test CAN port
//
#if CP_SMALL_CODE == 0
	if (ptsPortV == (CpPort_ts *) 0L)
	{
		return (eCP_ERR_CHANNEL);
	}
#endif

	ptsStatsV->ulErrMsgCount = 0;
	ptsStatsV->ulRcvMsgCount = 0;
	ptsStatsV->ulTrmMsgCount = 0;

	return (eCP_ERR_NONE);
}

/**
 * @param ulIdentifierV
 * @param ulAcceptMaskV
 * @param ubFormatV
 * @param filter_number
 * @param activate
 * @return
 */
static HAL_StatusTypeDef can_filter_config(uint32_t ulIdentifierV, uint32_t ulAcceptMaskV, uint8_t ubFormatV, uint8_t filter_number, bool_t activate)
{
	CAN_FilterConfTypeDef filter_config;

	// config filter only use 32Bit
	filter_config.FilterNumber = filter_number;
	filter_config.FilterMode = CAN_FILTERMODE_IDMASK;
	filter_config.FilterScale = CAN_FILTERSCALE_32BIT;

	if ((ubFormatV & CP_MSG_FORMAT_MASK) == CP_MSG_FORMAT_CBFF)
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

	filter_config.FilterFIFOAssignment = 0;
	filter_config.FilterActivation = activate;
	filter_config.BankNumber = 14;

	return HAL_CAN_ConfigFilter(&hcan1, &filter_config);
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
		if (filter_to_cp_buffer[i] == 0)
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

	status = get_next_free_filter_number(&filter_number);
	if (eCP_ERR_NONE == status)
	{
		// save buffer idx in filter to buffer table
		filter_to_cp_buffer[filter_number] = ubBufferIdxV;

		// config filter
		hal_status = can_filter_config(ulIdentifierV, ulAcceptMaskV, ubFormatV, filter_number, 1);

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
		hal_status = can_filter_config(0, 0, 0, i, 0);

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
		hcan->pTxMsg->DLC &= (uint8_t) 0x0000000FU;
		hcan->Instance->sTxMailBox[transmitmailbox].TDTR &= (uint32_t) 0xFFFFFFF0U;
		hcan->Instance->sTxMailBox[transmitmailbox].TDTR |= hcan->pTxMsg->DLC;

		/* Set up the data field */
		hcan->Instance->sTxMailBox[transmitmailbox].TDLR = (((uint32_t) hcan->pTxMsg->Data[3U] << 24U) | ((uint32_t) hcan->pTxMsg->Data[2U] << 16U) | ((uint32_t) hcan->pTxMsg->Data[1U] << 8U)
				| ((uint32_t) hcan->pTxMsg->Data[0U]));
		hcan->Instance->sTxMailBox[transmitmailbox].TDHR = (((uint32_t) hcan->pTxMsg->Data[7U] << 24U) | ((uint32_t) hcan->pTxMsg->Data[6U] << 16U) | ((uint32_t) hcan->pTxMsg->Data[5U] << 8U)
				| ((uint32_t) hcan->pTxMsg->Data[4U]));

		if (hcan->State == HAL_CAN_STATE_BUSY_RX)
		{
			/* Change CAN state */
			hcan->State = HAL_CAN_STATE_BUSY_TX_RX;
		}
		else
		{
			/* Change CAN state */
			hcan->State = HAL_CAN_STATE_BUSY_TX;
		}

		/* Set CAN error code to none */
		hcan->ErrorCode = HAL_CAN_ERROR_NONE;

		/* Process Unlocked */
		__HAL_UNLOCK(hcan);

		/* Enable Error warning, Error passive, Bus-off,
		 Last error and Error Interrupts */
		__HAL_CAN_ENABLE_IT(hcan, CAN_IT_EWG | CAN_IT_EPV | CAN_IT_BOF | CAN_IT_LEC | CAN_IT_ERR | CAN_IT_TME);

		// save which buffer used on this mailbox
		used_tx_mailbox[transmitmailbox] = ubBufferIdxV;

		/* Request transmission */
		hcan->Instance->sTxMailBox[transmitmailbox].TIR |= CAN_TI0R_TXRQ;
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

	if (NULL != pfnTrmIntHandler)
	{
		(*pfnTrmIntHandler)(&atsCanMsgS[canpie_buffer_number - 1], canpie_buffer_number);
	}

	__HAL_CAN_CLEAR_FLAG(hcan, clear_flag);

	// Disable Transmit mailbox empty Interrupt
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_TME);

}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
	// get filter index
	uint8_t filter_number = hcan->pRxMsg->FMI;
	uint8_t canpie_buffer_number;

	canpie_buffer_number = filter_to_cp_buffer[filter_number];

	if (NULL != pfnRcvIntHandler)
	{
		(*pfnRcvIntHandler)(&atsCanMsgS[canpie_buffer_number - 1], canpie_buffer_number);
	}

	// Enable FIFO 0 message pending Interrupt
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);

}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{

}
