
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private define ------------------------------------------------------------*/
enum {
	TRANSFER_WAIT,
	TRANSFER_COMPLETE,
	TRANSFER_ERROR
};
/* Private variables ---------------------------------------------------------*/
unsigned long result1=0, result2=0, result3=0, result4=0, result5=2, result6=2, result7=2, result8=2;
unsigned long initial_value[8] = {0,0,0,0,0,0,0,0};
char ext_result[100];
unsigned long time;
char aRxBuffer[100];
int PACK2=0;
uint16_t TIMERCOUNT = 0;
/* transfer state */
__IO uint32_t wTransferState = TRANSFER_WAIT;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_CAN_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
unsigned long converte(GPIO_TypeDef* SCKPort, uint16_t SCKPin, GPIO_TypeDef* DATAPort, uint16_t DATAPin);

static void CAN_Config(void);
void sendNumber(int n);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
#define EN_SPI()   HAL_GPIO_WritePin(ENABLE_SLAVE_GPIO_Port, ENABLE_SLAVE_Pin, GPIO_PIN_RESET);
#define DIS_SPI()  HAL_GPIO_WritePin(ENABLE_SLAVE_GPIO_Port, ENABLE_SLAVE_Pin, GPIO_PIN_SET);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */

  CAN_Config();

  //reseta o Hx711
  HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_SET);
  HAL_Delay(50);
  HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);
  hspi2.Init.Mode = SPI_MODE_MASTER;
  __HAL_SPI_ENABLE(&hspi2);

  DIS_SPI();

  /* Sinaliza para timer começar a contagem. Ignorar essa parte */
//  HAL_TIM_Base_Start_IT(&htim3);

// int j;

 /* Faz a media dos 10 primeiros valores*/
//for(j = 0;j<10; j++){
//	 result1 = converte(SCK_GPIO_Port, SCK_Pin, DATA1_GPIO_Port, DATA1_Pin)+ result1;
// result2 = converte(SCK_GPIO_Port, SCK_Pin, DATA2_GPIO_Port, DATA2_Pin)+ result2;
////	  HAL_Delay(20);
// result3 = converte(SCK_GPIO_Port, SCK_Pin, DATA3_GPIO_Port, DATA3_Pin)+ result3;
// result4 = converte(SCK_GPIO_Port, SCK_Pin, DATA4_GPIO_Port, DATA4_Pin)+ result4;
////////	 result5 = converte(SCK_GPIO_Port, SCK_Pin, DATA1_GPIO_Port, DATA1_Pin)+ result5;
////////	 result6 = converte(SCK_GPIO_Port, SCK_Pin, DATA2_GPIO_Port, DATA2_Pin)+ result6;
////////	 result7 = converte(SCK_GPIO_Port, SCK_Pin, DATA1_GPIO_Port, DATA1_Pin)+ result7;
////////	 result8 = converte(SCK_GPIO_Port, SCK_Pin, DATA2_GPIO_Port, DATA2_Pin)+ result8;
////
// }
////
// 	initial_value[0] = result1/10;
// 	initial_value[1] = result2/10;
// 	initial_value[2] = result3/10;
// 	initial_value[3] = result4/10;
//// 	initial_value[4] = result5/10;
//// 	initial_value[5] = result6/10;
//// 	initial_value[6] = result7/10;
//// 	initial_value[7] = result8/10;

  //if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
  //	  {
  	  //  Error_Handler();
  //	  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /* Pega valores de 2 HX e subtrai do valor inicial */
	  result1 = converte(SCK_GPIO_Port, SCK_Pin, DATA1_GPIO_Port, DATA1_Pin) - initial_value[0];
	  result2 = converte(SCK_GPIO_Port, SCK_Pin, DATA2_GPIO_Port, DATA2_Pin)- initial_value[1];

	  /* Envia primeiro pacote pela CAN */
	  sendNumber(0);

	  /* Pega valores de 2 HX e subtrai do valor inicial */
	  result3 = converte(SCK_GPIO_Port, SCK_Pin, DATA3_GPIO_Port, DATA3_Pin) - initial_value[2];
	  result4 = converte(SCK_GPIO_Port, SCK_Pin, DATA4_GPIO_Port, DATA4_Pin) - initial_value[3];

	  /* Envia segundo pacote pela CAN */
	  sendNumber(1);

	  result5 = converte(SCK_GPIO_Port, SCK_Pin, DATA5_GPIO_Port, DATA5_Pin) - initial_value[2];
	  result6 = converte(SCK_GPIO_Port, SCK_Pin, DATA6_GPIO_Port, DATA6_Pin) - initial_value[3];

	  sendNumber(2);

	  result7 = converte(SCK_GPIO_Port, SCK_Pin, DATA7_GPIO_Port, DATA7_Pin) - initial_value[2];
	  // result8 = converte(SCK_GPIO_Port, SCK_Pin, DATA8_GPIO_Port, DATA8_Pin) - initial_value[3];

	  sendNumber(3);

	  time = HAL_GetTick(); // tempo em milisegundos
	  // coloca o valor lido e o tempo na string ext_result
	  sprintf(ext_result, "%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t%lu\t%d\t%d\n", result1, result2, result3,result4,result5,result6,result7, time, TIMERCOUNT);
	  // imprime o valor e o tempo no monitor serial
	  //HAL_UART_Transmit(&huart2, (uint8_t*)ext_result, (unsigned)strlen(ext_result), 100);

	  // Habilita o slave (arduino)
	  EN_SPI();

	  /* Inicia a comunicação SPI com o Atmega
	   * Envia dados pelo ext_result e recebe dados pelo aRxBuffer
	   */
	  if(HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t*)ext_result, (uint8_t *)aRxBuffer, 100) != HAL_OK)
	  {
	      // Erro na transmissão
		  Error_Handler();
	  }

	  /* Espera a transmissão acabar */
	  while (wTransferState == TRANSFER_WAIT)
	  {
//		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		  HAL_Delay(500);
	  }

	  switch(wTransferState)
	  {
	    case TRANSFER_COMPLETE :
	      // Transmissão finalizada com sucesso
	    break;
	    default : // TRANSFER_ERROR
	      Error_Handler();
	    break;
	  }

	  DIS_SPI(); // Desabilita o slave

	  HAL_Delay(2); // Sincronização


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 1;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_6TQ;
  hcan.Init.BS2 = CAN_BS2_1TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 99;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SCK_GPIO_Port, SCK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENABLE_SLAVE_GPIO_Port, ENABLE_SLAVE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SCK_Pin */
  GPIO_InitStruct.Pin = SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DATA1_Pin DATA2_Pin DATA3_Pin DATA4_Pin */
  GPIO_InitStruct.Pin = DATA1_Pin|DATA2_Pin|DATA3_Pin|DATA4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DATA5_Pin DATA6_Pin DATA7_Pin DATA8_Pin */
  GPIO_InitStruct.Pin = DATA5_Pin|DATA6_Pin|DATA7_Pin|DATA8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_SLAVE_Pin */
  GPIO_InitStruct.Pin = ENABLE_SLAVE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENABLE_SLAVE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* Função de interrupção da transmissão/recepção */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
//  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  wTransferState = TRANSFER_COMPLETE;
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

/* Função de interrupção de erro */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  wTransferState = TRANSFER_ERROR;
}


static void CAN_Config(void){

  CAN_FilterConfTypeDef  sFilterConfig;
  static CanTxMsgTypeDef        TxMessage;     //--
  static CanRxMsgTypeDef        RxMessage;     //-- codigo original

  /*##-1- Configure the CAN peripheral #######################################*/
   hcan.pTxMsg = &TxMessage;     //-- codigo original
   hcan.pRxMsg = &RxMessage;	  // --



  /*##-2- Configure the CAN Filter ###########################################*/
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = 0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.BankNumber = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  /*##-3- Configure Transmission process #####################################*/
  hcan.pTxMsg->StdId = 0x001;
  hcan.pTxMsg->ExtId = 0x300<<19;
  hcan.pTxMsg->RTR = CAN_RTR_DATA;
  /*configurando como extended id (29 bits)*/
  hcan.pTxMsg->IDE = CAN_ID_EXT;
  hcan.pTxMsg->DLC = 8;
}

/* Monta os pacotes CAN e envia */
void sendNumber(int n){


	if(n == 0){
		hcan.pTxMsg->Data[0] = 0xA;
		hcan.pTxMsg->Data[1] = result1>>16;
		hcan.pTxMsg->Data[2] = result1>>8;
		hcan.pTxMsg->Data[3] = result1;
		hcan.pTxMsg->Data[4] = 0xAA;
		hcan.pTxMsg->Data[5] = result2>>16;
		hcan.pTxMsg->Data[6] = result2>>8;
		hcan.pTxMsg->Data[7] = result2;

}
	else if (n==1){
		hcan.pTxMsg->Data[0] = 0xB;
		hcan.pTxMsg->Data[1] = result3>>16;
		hcan.pTxMsg->Data[2] = result3>>8;
		hcan.pTxMsg->Data[3] = result3;
		hcan.pTxMsg->Data[4] = 0xBB;
		hcan.pTxMsg->Data[5] = result4>>16;
		hcan.pTxMsg->Data[6] = result4>>8;
		hcan.pTxMsg->Data[7] = result4;
	}
	else if (n==2){
			hcan.pTxMsg->Data[0] = 0xC;
			hcan.pTxMsg->Data[1] = result5>>16;
			hcan.pTxMsg->Data[2] = result5>>8;
			hcan.pTxMsg->Data[3] = result5;
			hcan.pTxMsg->Data[4] = 0xCC;
			hcan.pTxMsg->Data[5] = result6>>16;
			hcan.pTxMsg->Data[6] = result6>>8;
			hcan.pTxMsg->Data[7] = result6;
		}
	else if (n==3){
			hcan.pTxMsg->Data[0] = 0xD;
			hcan.pTxMsg->Data[1] = result7>>16;
			hcan.pTxMsg->Data[2] = result7>>8;
			hcan.pTxMsg->Data[3] = result7;
			hcan.pTxMsg->Data[4] = 0xDD;
			hcan.pTxMsg->Data[5] = result8>>16;
			hcan.pTxMsg->Data[6] = result8>>8;
			hcan.pTxMsg->Data[7] = result8;
		}
		 /* Start the Transmission process */

		 if (HAL_CAN_Transmit_IT(&hcan) != HAL_OK)
		 {
		   /* Transmition Error. Se deu erro no envio o LED 13 vai piscar */
			 HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

		 }

}

void HAL_CAN_TxCpltCallback (CAN_HandleTypeDef * hcan){


}

/*void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
	if (((hcan->pRxMsg->ExtId) >> 19) == 0x301){
		TIMERCOUNT = (hcan->pRxMsg->Data[0]<<8) | (hcan->pRxMsg->Data[1]);
	}
	if (HAL_CAN_Receive_IT(hcan, CAN_FIFO0) != HAL_OK)
	  {
	    Error_Handler();
	  }
}*/

//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	cnt++;
//	sprintf(ext_result, "%lu\t%lu\t%lu\t%lu\t%lu\n", result1, result2, result3, result4, cnt);
//	// Habilita o slave (arduino)
//	EN_SPI();
//
//	/* Inicia a comunicação SPI
//	* Envia dados pelo ext_result e recebe dados pelo aRxBuffer
//	*/
//	if(HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t*)ext_result, (uint8_t *)aRxBuffer, 50) != HAL_OK)
//	{
//	  // Erro na transmissão
//	  Error_Handler();
//	}
//
//	/* Espera a transmissão acabar */
//	while (wTransferState == TRANSFER_WAIT)
//	{
//	//		  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//	//		  HAL_Delay(500);
//	}
//
//	switch(wTransferState)
//	{
//	case TRANSFER_COMPLETE :
//	  // Transmissão finalizada com sucesso
//	break;
//	default : // TRANSFER_ERROR
//	  Error_Handler();
//	break;
//	}
//
//	DIS_SPI(); // Desabilita o slave
//
//}

/* Função de captura dos dados do HX711 */
unsigned long converte(GPIO_TypeDef* SCKPort, uint16_t SCKPin, GPIO_TypeDef* DATAPort, uint16_t DATAPin){
	unsigned long captura = 0;
	int i;

	HAL_GPIO_WritePin(SCKPort, SCKPin, GPIO_PIN_RESET);

	while(HAL_GPIO_ReadPin(DATAPort, DATAPin)); // espera DATA ser LOW para iniciar o envio de dados

	for(i = 0; i < 24; i++){ // 24 pulsos
		HAL_GPIO_WritePin(SCKPort, SCKPin, GPIO_PIN_SET);
		captura = captura << 1;
		HAL_GPIO_WritePin(SCKPort, SCKPin, GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(DATAPort, DATAPin)) captura++;
	}

	// ultimo pulso: encerra o envio de dados
	HAL_GPIO_WritePin(SCKPort, SCKPin, GPIO_PIN_SET);
	captura = captura^0x0800000;
	HAL_GPIO_WritePin(SCKPort, SCKPin, GPIO_PIN_RESET);

	return captura;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  //HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
	//  HAL_Delay(200);
	  break;
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
