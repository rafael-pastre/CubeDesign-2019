/*
*****************************************************************************************************************************
* @project         : USPSat AE1
* @authors(git)    : João Matheus Siqueira Souza (jmssouza)
* @code            : zenith_can_lib.c
* @brief           : Library for CAN Bus control
*****************************************************************************************************************************
*/

#include "main.h"
#include "stm32l4xx_hal.h"
#include "zenith_can_lib.h"

Z_CAN_Package NULL_MSG = {0xffffffff, {0,0,0,0,0,0,0,0}};

void filterConfigCAN(CAN_HandleTypeDef *hcan1)
{

	CAN_FilterTypeDef can1FilterInit;

	can1FilterInit.FilterActivation = ENABLE;
	can1FilterInit.FilterBank = 0;
	can1FilterInit.FilterFIFOAssignment = CAN_RX_FIFO0;
	can1FilterInit.FilterIdHigh = 0x0000;
	can1FilterInit.FilterIdLow = 0x0000;
	can1FilterInit.FilterMaskIdHigh = 0x0000;
	can1FilterInit.FilterMaskIdLow = 0x0000;
	can1FilterInit.FilterMode = CAN_FILTERMODE_IDMASK;
	can1FilterInit.FilterScale = CAN_FILTERSCALE_32BIT;

	HAL_CAN_ConfigFilter(hcan1, &can1FilterInit);
}

uint8_t sendCanMessage(CAN_HandleTypeDef *hcan1, Z_CAN_Package package)
{
	uint32_t pTxMailbox;

	//Mounting the CAN header
	CAN_TxHeaderTypeDef pTxHeader;
	pTxHeader.DLC = 8;
	pTxHeader.IDE = CAN_ID_STD;
	pTxHeader.RTR = CAN_RTR_DATA;
	pTxHeader.StdId = package.identifier;

	if(HAL_CAN_GetTxMailboxesFreeLevel(hcan1) != 0){
		HAL_CAN_AddTxMessage(hcan1, &pTxHeader, package.data, &pTxMailbox);
		return 0;
	}
	else{
		return -1;
	}
}

Z_CAN_Package readCanMessages(CAN_HandleTypeDef *hcan1)
{
	Z_CAN_Package package = NULL_MSG;
	uint32_t RxFifo = CAN_RX_FIFO0;
	CAN_RxHeaderTypeDef pRxHeader;

	if(HAL_CAN_GetRxFifoFillLevel(hcan1, RxFifo) != 0){
		HAL_CAN_GetRxMessage(hcan1, RxFifo, &pRxHeader, package.data);
		package.identifier = pRxHeader.StdId;
	}
	return package;
}

uint8_t isEqual(Z_CAN_Package p1, Z_CAN_Package p2){
	uint8_t i;

	if(p1.identifier != p2.identifier){
		return 0;
	}

	for(i = 0; i < 8; i++){
		if(p1.data[i] != p2.data[i]){
			return 0;
		}
	}

	return 1;
}

