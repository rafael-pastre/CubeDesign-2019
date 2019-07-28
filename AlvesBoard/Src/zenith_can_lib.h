/*
*****************************************************************************************************************************
* @project         : USPSat AE1
* @authors(git)    : João Matheus Siqueira Souza (jmssouza)
* @code            : zenith_can_lib.h
* @brief           : Library for CAN Bus control
*****************************************************************************************************************************
*/

#ifndef ZENITH_CAN_LIB_H_
#define ZENITH_CAN_LIB_H_


typedef struct {
	uint32_t identifier;
	uint8_t data[8];
} Z_CAN_Package;

extern Z_CAN_Package NULL_MSG;// = {-1, {0,0,0,0,0,0,0,0}};

void filterConfigCAN(CAN_HandleTypeDef *hcan1);
uint8_t sendCanMessage(CAN_HandleTypeDef *hcan1, Z_CAN_Package package);
Z_CAN_Package readCanMessages(CAN_HandleTypeDef *hcan1);
uint8_t isEqual(Z_CAN_Package p1, Z_CAN_Package p2);

#endif /* ZENITH_CAN_LIB_H_ */
