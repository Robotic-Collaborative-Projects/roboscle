/*
 * can.h
 *
 *  Created on: Jun 3, 2023
 *      Author: R.Khorrrambakht
 */

#ifndef INC_CAN_H_
#define INC_CAN_H_
#include "main.h"
// Initialize the CAN telemetry subsystem
void can_init(FDCAN_HandleTypeDef *hfdcan);

// Event handler for responding to commands and storing the data. Should be called priodically.
void can_events();

// Function to send FDCAN message
void sendFDCANRespond(uint32_t identifier, uint8_t *data, uint32_t dataSize);
// FDCAN reception interrupt callback function
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
// Configure the CAN filter ad driver ID
void configureCANFilter(uint32_t can_id);

#endif /* INC_CAN_H_ */
