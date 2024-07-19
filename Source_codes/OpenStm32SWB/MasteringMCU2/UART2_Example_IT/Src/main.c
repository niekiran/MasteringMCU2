/*
 * main.c
 *
 *  Created on: 02-Jun-2018
 *      Author: kiran
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <string.h>
#include "stm32f4xx_hal.h"

/* Private define ------------------------------------------------------------*/
#define TRUE 1
#define FALSE 0

/* Private function prototypes -----------------------------------------------*/
void SystemClockConfig(void);
void UART2_Init(void);
void Error_handler(void);
uint8_t convert_to_capital(uint8_t data);

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
char *user_data = "The application is running\r\n";
uint8_t  data_buffer[100];
uint8_t  recvd_data;
uint32_t count=0;
uint8_t  reception_complete = FALSE;

int main(void)
{
	HAL_Init();
	SystemClockConfig();
	UART2_Init();

	uint16_t len_of_data = strlen(user_data);
	HAL_UART_Transmit(&huart2,(uint8_t*)user_data,len_of_data,HAL_MAX_DELAY);

  while(reception_complete != TRUE)
  {
    HAL_UART_Receive_IT(&huart2,&recvd_data,1);
  }

	while(1);

	return 0;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClockConfig(void)
{


}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
void UART2_Init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if ( HAL_UART_Init(&huart2) != HAL_OK )
	{
		//There is a problem
		Error_handler();
	}
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  huart  Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(recvd_data == '\r')
  {
    reception_complete = TRUE;
    data_buffer[count++]='\r';
    HAL_UART_Transmit(huart,data_buffer,count,HAL_MAX_DELAY);
  }
  else
  {
    data_buffer[count++] = recvd_data;
  }
}

/**
  * @brief  Converts a lowercase ASCII character to its uppercase equivalent.
  * @param  data: The ASCII value of the character to be converted.
  * @retval The ASCII value of the converted uppercase character.
  *         If the input character is not a lowercase letter, it is returned unchanged.
  */
uint8_t convert_to_capital(uint8_t data)
{
	if( data >= 'a' && data <= 'z')
	{
		data = data - ( 'a'- 'A');
	}

	return data;
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_handler(void)
{
	while(1);
}

