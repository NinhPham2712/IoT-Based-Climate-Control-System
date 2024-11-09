#include "main.h"
#include "string.h"
#include "stdio.h"
#include "stdint.h"
#include "DHT22.h"

// Function implementations
void DHT22_Start(void) {
	Set_Pin_Output(GPIOA, GPIO_PIN_5); // set the pin as output
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // pull the pin low
	Delay_Us(1200);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // pull the pin high
	Delay_Us(20);
	Set_Pin_Input(GPIOA, GPIO_PIN_5); // set the pin as input
}

uint8_t DHT22_Check_Response(void) {
	uint8_t resposne = 0;
	Delay_Us(40);
	if (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)) {
		Delay_Us(80); // wait 80 us for response from DHT22
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
			resposne = 1; // successfully receive the response
		else
			resposne = -1; // fail to receive the response
	} else
		return -1;
	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
		; // wait for the pin to go low
	return resposne;
}

uint8_t DHT22_Read_Data(void) {
	uint8_t data;
	for (uint8_t i = 0; i < 8; i++) {
//		Delay_Us(50);
		while (!(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)))
			; // wait for the pin to to go high
		Delay_Us(30);
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5)) // if the pin is high
			data |= 1 << (7 - i); // write 1 bit
		else
			// if the pin is low
			data &= ~(1 << (7 - i)); // write 0 bit
		while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5))
			; // wait for the pin to go low
	}
	return data;
}

void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
	/*Configure GPIO pin */
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
	/*Configure GPIO pin */
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DHT22_Get_Data(DHT22_Data *data) {
	uint8_t status, tempByte1, tempByte2, humidByte1, humidByte2, rcv_checksum, checksum = 0xFF;
	DHT22_Start();
	status = DHT22_Check_Response();
	if (status) {
		humidByte1 = DHT22_Read_Data();
		humidByte2 = DHT22_Read_Data();
		tempByte1 = DHT22_Read_Data();
		tempByte2 = DHT22_Read_Data();
		rcv_checksum = DHT22_Read_Data();
	}
	checksum = humidByte1 + humidByte2 + tempByte1 + tempByte2;

	if (checksum == rcv_checksum) {
		data->temperature = (double) (((tempByte1 << 8) | tempByte2)) / 10.00;
		data->humidity = (double) (((humidByte1 << 8) | humidByte2)) / 10.00;
		return 1;
	}
	else
		return 0;
}
