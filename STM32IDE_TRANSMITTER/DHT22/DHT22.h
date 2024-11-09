#ifndef INC_DHT22_H_
#define INC_DHT22_H_

// Typedef
typedef struct {
	double temperature;
	double humidity;
} DHT22_Data;

// Function prototypes
void DHT22_Start(void);
uint8_t DHT22_Check_Response(void);
uint8_t DHT22_Read_Data(void);
uint8_t DHT22_Get_Data(DHT22_Data *data);
void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

#endif /* INC_DHT22_H_ */
