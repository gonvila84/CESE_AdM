
#include "main.h"
#include "string.h"
#include "asm_func.h"
#include "c_func.h"

/*
 * 	TODOS LOS EJERCICIOS ESTAN AISLADOS MEDIANTE COMPILACION CONDICIONAL, PARA PODER EJECUTARLOS SE DEBE QUITAR LOS COMENTARIOS DE
 *  LAS DEFINICIONES UBICADAS EN LA PARTE SUPERIOR DE ESTE ARCHIVO
 * */


//#define	EXECUTE_EXERCISE1
//#define	EXECUTE_EXERCISE2
//#define	EXECUTE_EXERCISE3
//#define	EXECUTE_EXERCISE4
//#define	EXECUTE_EXERCISE5
//#define	EXECUTE_EXERCISE6
//#define	EXECUTE_EXERCISE7
//#define	EXECUTE_EXERCISE8
//#define	EXECUTE_EXERCISE9


ETH_TxPacketConfig TxConfig;
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);


static void PrivilegiosSVC (void)
{
    uint32_t x = __get_CONTROL ();
    x |= 1;
    __set_CONTROL (x);
    x = __get_CONTROL ();
    x &= ~1u;
    __set_CONTROL (x);
    x = __get_CONTROL ();
    asm_svc ();
    x = __get_CONTROL ();
}

int main(void)
{

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();

  PrivilegiosSVC ();

/*
 * 	TODOS LOS EJERCICIOS ESTAN AISLADOS MEDIANTE COMPILACION CONDICIONAL, PARA PODER EJECUTARLOS SE DEBE QUITAR LOS COMENTARIOS DE
 *  LAS DEFINICIONES UBICADAS EN LA PARTE SUPERIOR DE ESTE ARCHIVO
 * */


  /*
  1) Realizar una función que inicialice un vector con ceros. La función debe tener el siguiente prototipo:

  void zeros (uint32_t * vector, uint32_t longitud);
  */
#ifdef EXECUTE_EXERCISE1
  uint32_t	c_zeros_vin_uint32 [10];
  uint32_t	c_zeros_vlong_uint32 = 10;
  c_zeros	(c_zeros_vin_uint32,c_zeros_vlong_uint32);


  uint32_t	asm_zeros_vin_uint32 [10];
  uint32_t	asm_zeros_vlong_uint32 = 10;
  asm_zeros	(asm_zeros_vin_uint32,asm_zeros_vlong_uint32);
#endif
  /*
  2) Realizar una función que realice el producto de un vector y un escalar (por ejemplo, podría servir para cambiar el nivel de amplitud de una señal).

  void productoEscalar32 (uint32_t * vectorIn, uint32_t * vectorOut, uint32_t longitud, uint32_t escalar);
  */
#ifdef EXECUTE_EXERCISE2
  uint32_t	c_productoEscalar32_vin_uint32 [10] = {1,2,3,4,5,6,7,8,9,10};
  uint32_t	c_productoEscalar32_vout_uint32 [10];
  uint32_t	c_productoEscalar32_vlong_uint32 = 10;
  uint32_t	c_productoEscalar32_escalar_uint32 = 3;
  c_productoEscalar32 (c_productoEscalar32_vin_uint32, c_productoEscalar32_vout_uint32 , c_productoEscalar32_vlong_uint32, c_productoEscalar32_escalar_uint32);

  uint32_t	asm_productoEscalar32_vin_uint32 [10] = {1,2,3,4,5,6,7,8,9,10};
  uint32_t	asm_productoEscalar32_vout_uint32 [10];
  uint32_t	asm_productoEscalar32_vlong_uint32 = 10;
  uint32_t	asm_productoEscalar32_escalar_uint32 = 3;
  asm_productoEscalar32 (asm_productoEscalar32_vin_uint32, asm_productoEscalar32_vout_uint32 , asm_productoEscalar32_vlong_uint32, asm_productoEscalar32_escalar_uint32);
#endif
  /*
  3) Adapte la función del ejercicio 2 para realizar operaciones sobre vectores de 16 bits:

  void productoEscalar16 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
  */
#ifdef EXECUTE_EXERCISE3
  uint16_t	c_productoEscalar16_vin_uint16 [10] = {1,2,3,4,5,6,7,8,9,10};
  uint16_t	c_productoEscalar16_vout_uint16 [10];
  uint32_t	c_productoEscalar16_vlong_uint32 = 10;
  uint16_t	c_productoEscalar16_escalar_uint16 = 3;
  c_productoEscalar16 (c_productoEscalar16_vin_uint16, c_productoEscalar16_vout_uint16, c_productoEscalar16_vlong_uint32, c_productoEscalar16_escalar_uint16);

  uint16_t	asm_productoEscalar16_vin_uint16 [10] = {1,2,3,4,5,6,7,8,9,10};
  uint16_t	asm_productoEscalar16_vout_uint16 [10];
  uint32_t	asm_productoEscalar16_vlong_uint32 = 10;
  uint16_t	asm_productoEscalar16_escalar_uint16 = 3;
  asm_productoEscalar16 (asm_productoEscalar16_vin_uint16, asm_productoEscalar16_vout_uint16, asm_productoEscalar16_vlong_uint32, asm_productoEscalar16_escalar_uint16);
#endif
  /*
  4) Adapte la función del ejercicio 3 para saturar el resultado del producto a 12 bits:

  void productoEscalar12 (uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitud, uint16_t escalar);
  */
#ifdef EXECUTE_EXERCISE4
  uint16_t	c_productoEscalar12_vin_uint16 [10] = {1,2,50000,4,2000,9062,7,8,9,4096};
  uint16_t	c_productoEscalar12_vout_uint16 [10];
  uint32_t	c_productoEscalar12_vlong_uint32 = 10;
  uint16_t	c_productoEscalar12_escalar_uint16 = 3;
  c_productoEscalar12 (c_productoEscalar12_vin_uint16, c_productoEscalar12_vout_uint16, c_productoEscalar12_vlong_uint32, c_productoEscalar12_escalar_uint16);

  uint16_t	asm_productoEscalar12_vin_uint16 [10] = {1,2,50000,4,2000,9062,7,8,9,4096};
  uint16_t	asm_productoEscalar12_vout_uint16 [10];
  uint32_t	asm_productoEscalar12_vlong_uint32 = 10;
  uint16_t	asm_productoEscalar12_escalar_uint16 = 3;
  asm_productoEscalar12 (asm_productoEscalar12_vin_uint16, asm_productoEscalar12_vout_uint16, asm_productoEscalar12_vlong_uint32, asm_productoEscalar12_escalar_uint16);
#endif
  /*
  5) Realice una función que implemente un filtro de ventana móvil de 10 valores sobre un vector de muestras.

  void filtroVentana10(uint16_t * vectorIn, uint16_t * vectorOut, uint32_t longitudVectorIn);
  */
#ifdef EXECUTE_EXERCISE5
  uint16_t	c_filtroVentana10_vin_uint16 [10] = {1,2,6,20,5,35,7,2,9,100};
  uint16_t	c_filtroVentana10_vout_uint16 [10];
  uint32_t	c_filtroVentana10_vlong_uint32 = 10;
  c_filtroVentana10 (c_filtroVentana10_vin_uint16, c_filtroVentana10_vout_uint16, c_filtroVentana10_vlong_uint32);

  uint16_t	asm_filtroVentana10_vin_uint16 [10] = {1,2,6,20,5,35,7,2,9,100};
  uint16_t	asm_filtroVentana10_vout_uint16 [10];
  uint32_t	asm_filtroVentana10_vlong_uint32 = 10;
  asm_filtroVentana10 (asm_filtroVentana10_vin_uint16, asm_filtroVentana10_vout_uint16, asm_filtroVentana10_vlong_uint32);
#endif
  /*
  6) Realizar una función que reciba un vector de números signados de 32 bits y los “empaquete” en otro vector de 16 bits. La función deberá adecuar los valores de entrada a la nueva precisión.

  void pack32to16 (int32_t * vectorIn, int16_t *vectorOut, uint32_t longitud);
  */
#ifdef EXECUTE_EXERCISE6
  int32_t asm_pack32to16_vin_int32 [10] = {1,2,3,4,5,6,7,8,9,10};
  int16_t asm_pack32to16_vout_int16 [10];
  uint32_t asm_pack32to16_vlong_uint32 = 10;
  asm_pack32to16 (asm_pack32to16_vin_int32, asm_pack32to16_vout_int16, asm_pack32to16_vlong_uint32);
#endif
  /*
  7) Realizar una función que reciba un vector de números signados de 32 bits y devuelva la posición del máximo del vector.

  int32_t max (int32_t * vectorIn, uint32_t longitud);
  */
#ifdef EXECUTE_EXERCISE7
  int32_t asm_max_vin_int32 [10] = {-8,2,40,35,-3,1200,7,96,9,108};
  uint32_t asm_max_vlong_int32 = 10;
  int32_t asm_max_returnValue_int32_t = -1;
  asm_max_returnValue_int32_t = asm_max (asm_max_vin_int32, asm_max_vlong_int32);

  int32_t c_max_vin_int32 [10] = {-8,2,40,35,-3,1200,7,96,9,108};
  int32_t c_max_vlong_int32 = 10;
  int32_t c_max_returnValue_int32_t = -1;
  c_max_returnValue_int32_t = c_max (c_max_vin_int32, c_max_vlong_int32);
#endif
  /*
  8) Realizar una función que reciba un vector de muestras signadas de 32 bits y lo decime
  descartando una cada N muestras.

  void downsampleM (int32_t * vectorIn, int32_t * vectorOut, uint32_t longitud, uint32_t N);
  */
#ifdef EXECUTE_EXERCISE8
  int32_t asm_downsampleM_vin_int32 [10] = {-8,2,40,35,-3,1200,7,96,9,108};
  int32_t asm_downsampleM_vout_int32 [10] = {0};
  int32_t asm_downsampleM_longitud_int32 = 10;
  uint32_t asm_downsampleM_N_uint32 = 2;
  asm_downsampleM (asm_downsampleM_vin_int32, asm_downsampleM_vout_int32, asm_downsampleM_longitud_int32, asm_downsampleM_N_uint32);

  int32_t c_downsampleM_vin_int32 [10] = {-8,2,40,35,-3,1200,7,96,9,108};
  int32_t c_downsampleM_vout_int32 [10] = {0};
  int32_t c_downsampleM_longitud_int32 = 10;
  uint32_t c_downsampleM_N_uint32 = 2;
  c_downsampleM (c_downsampleM_vin_int32, c_downsampleM_vout_int32, c_downsampleM_longitud_int32, c_downsampleM_N_uint32);
#endif
  /*
  9) Realizar una función que reciba un vector de muestras no signadas de 16 bits e invierta su orden.

  void invertir (uint16_t * vector, uint32_t longitud);
  */
#ifdef EXECUTE_EXERCISE9
  uint16_t	asm_invertir_vin_uint16 [10] = {1,2,3,4,5,6,7,8,9,10};
  uint32_t	asm_invertir_vlong_uint32 = 10;
  asm_invertir (asm_invertir_vin_uint16, asm_invertir_vlong_uint32);

  uint16_t	c_invertir_vin_uint16 [10] = {1,2,3,4,5,6,7,8,9,10};
  uint32_t	c_invertir_vlong_uint32 = 10;
  c_invertir (c_invertir_vin_uint16, c_invertir_vlong_uint32);
#endif
  while (1)
  {

  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
