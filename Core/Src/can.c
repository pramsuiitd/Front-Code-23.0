/**
  ******************************************************************************
  * File Name          : CAN.c
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

void CAN_Start(){

	CAN_FilterTypeDef filter1, filter2;

	filter1.FilterBank = 0;

	filter1.FilterActivation = ENABLE;
	filter1.FilterFIFOAssignment = CAN_FILTER_FIFO0;

	filter1.FilterScale = CAN_FILTERSCALE_32BIT;
	filter1.FilterMode = CAN_FILTERMODE_IDMASK;

	filter1.FilterMaskIdHigh = 0x0000;
	filter1.FilterMaskIdLow  = 0x0000;

	filter1.FilterIdLow  = 0x0000;
	filter1.FilterIdHigh = 0x0000;

	filter1.SlaveStartFilterBank = 14;


	filter2.FilterBank = 15;

	filter2.FilterActivation = ENABLE;
	filter2.FilterFIFOAssignment = CAN_FILTER_FIFO1;

	filter2.FilterScale = CAN_FILTERSCALE_32BIT;
	filter2.FilterMode = CAN_FILTERMODE_IDMASK;

	filter2.FilterMaskIdHigh = 0x0000;
	filter2.FilterMaskIdLow  = 0x0000;

	filter2.FilterIdLow  = 0x0000;
	filter2.FilterIdHigh = 0x0000;

	filter2.SlaveStartFilterBank = 14;


	if (HAL_CAN_ConfigFilter(&hcan1, &filter1) != HAL_OK) { // Configuring CAN line according to Filter values
		Error_Handler();
	}
	if (HAL_CAN_ConfigFilter(&hcan2, &filter2)!= HAL_OK){
		Error_Handler();
	}

	if (HAL_CAN_Start(&hcan1) != HAL_OK) { // CAN start
		Error_Handler();
	}

	if(HAL_CAN_Start(&hcan2) != HAL_OK){
		Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_FULL | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK){ // Interrupt activation
		Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING | CAN_IT_RX_FIFO1_FULL | CAN_IT_RX_FIFO1_OVERRUN ) != HAL_OK){ // Interrupt activation
			Error_Handler();
		}
}

void CAN_Tx(CAN_TxHeaderTypeDef TxHeader, CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* data_tx){

		uint32_t Txmailbox = 0x00U;
	    TxHeader.DLC=8;
	    TxHeader.StdId=id; // 11 bits
	    TxHeader.IDE=CAN_ID_STD;
	    TxHeader.RTR=CAN_RTR_DATA;

	    if(HAL_CAN_GetTxMailboxesFreeLevel(hcan)>0) {
	    	if (HAL_CAN_AddTxMessage(hcan, &TxHeader, data_tx, &Txmailbox)!= HAL_OK)
	    		Error_Handler();
	    }
}
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
