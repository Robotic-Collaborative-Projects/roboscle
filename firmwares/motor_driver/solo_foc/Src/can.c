/*
 * can.c
 *
 *  Created on: Jun 3, 2023
 *      Author: R.Khorrrambakht
 */

// Function to send FDCAN message
#include "can.h"
#include "stdbool.h"
FDCAN_HandleTypeDef *can_handle;
FDCAN_RxHeaderTypeDef rxHeader;
uint8_t rxData[8];
bool response_flag = false;

void can_init(FDCAN_HandleTypeDef *hfdcan)
{
	can_handle = hfdcan;
	// Enable termination resistor and the CAN transceiver on the board
	HAL_GPIO_WritePin(CAN_TERM_GPIO_Port, CAN_TERM_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CAN_SHTD_GPIO_Port, CAN_SHTD_Pin, GPIO_PIN_RESET);
	// Setup a filter that acts as the driver CAN ID (Only listen to messages that are for us)
	configureCANFilter(0x123);
	// Configure FDCAN reception interrupt
	HAL_FDCAN_ActivateNotification(can_handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
	// Start FDCAN reception
	HAL_FDCAN_Start(can_handle);
}

void sendFDCANRespond(uint32_t identifier, uint8_t *data, uint32_t dataSize)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = identifier;
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	TxHeader.MessageMarker = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(can_handle, &TxHeader, data);
}

// FDCAN reception interrupt callback function
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    // Check if a message is pending in Rx FIFO 0
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0)
    {
        // Retrieve the received message
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData);
        response_flag = true;
    }
}

void can_events()
{
	if(response_flag)
	{
		response_flag = false;
		sendFDCANRespond(rxHeader.Identifier, rxData, sizeof(rxData));
	}
}


void configureCANFilter(uint32_t can_id)
{
    FDCAN_FilterTypeDef sFilterConfig;

    sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = can_id;
	sFilterConfig.FilterID2 = 0x7ff;
	if (HAL_FDCAN_ConfigFilter(can_handle, &sFilterConfig) != HAL_OK)
	{
	Error_Handler();
	}
	/* Configure global filter on the FDCAN instance:
	 Filter all remote frames with STD and EXT ID
	 Reject non matching frames with STD ID and EXT ID */
	if (HAL_FDCAN_ConfigGlobalFilter(can_handle, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK)
	{
	Error_Handler();
	}
}
