/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dfsdm.h"
#include "dma.h"
#include "i2c.h"
#include "quadspi.h"
#include "spi.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include "ring_buffer.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//uint8_t TX_Buffer[BUFFER_SIZE] = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define pc_uart &huart1
#define wifi_uart &huart3
#define wifi_spi &hspi1
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  void SPI_Transmit (uint8_t *data, int size);
  uint8_t data[5] = {0x0a,0x0b,0x0c,0x0d,0x0e};
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_I2C2_Init();
  MX_QUADSPI_Init();
  MX_SPI3_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_SPI2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  Ringbuf_init();
  //ST7789_Init();
  /* USER CODE END 2 */
  //SPI_Transmit(data, 5);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    //ST7789_Test();
      
    //HAL_UART_Receive(pc_uart, TX_Data, sizeof(TX_Data), 5000);
    //HAL_UART_Transmit(pc_uart, TX_Data, sizeof(TX_Data), 5000);
    
    //HAL_SPI_Transmit(&hspi2, RX_Data, sizeof(RX_Data), 5000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void SPI_Transmit (uint8_t *data, int size)
{
  uint8_t *temp = data;
  SPI1->CR1 |= (1<<6);
  int i= size;
  while(i>0U)
  {
  while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	if (i > 1U)
	{
	   //uint8_t view1 = *temp;
     SPI1->DR = *((uint16_t *)temp); // load the data into the Data Register
     temp += sizeof(uint16_t);  
	   i -= 2U;
	}	
	else
      {
        *((__IO uint8_t *)&SPI1->DR) = (*temp);
        temp ++;
        i--;
      }
  }

 //  Clear the Overrun flag by reading DR and SR

	while (!((SPI1->SR)&(1<<1))) {};  // wait for TXE bit to set -> This will indicate that the buffer is empty
	while (((SPI1->SR)&(1<<7))) {};  // wait for BSY bit to Reset -> This will indicate that SPI is not busy in communication	

  while (((SPI1->SR)&(1<<0))) 
  {
    uint8_t temp = SPI1->DR;
  }; 
	  uint8_t	temp1 = SPI1->SR;
	
  SPI1->CR1 &= ~(1<<6);
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/* USER CODE BEGIN 4 */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
 my_Uart_isr (pc_uart);
 my_spi_isr (wifi_spi);
 char newLine = "\r";
 char *pnewLine = &newLine;
 if(IsUartDataAvailable(pc_uart))
  {
    int data = Uart_read(pc_uart);
    spi_write(data, wifi_spi);
    /*
    if((Look_for(pnewLine, _rx_buffer2)) == 1)
    {
      //spi_write(data, wifi_spi);
      Uart_write(data, pc_uart);
    }
    */
    //Uart_write(data, pc_uart);
  }
  /* USER CODE END USART1_IRQn 0 */
  //HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */

  /* USER CODE END USART1_IRQn 1 */
}

/*void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
 {
   //UNUSED(huart);
   //HAL_UART_Transmit_DMA(&huart3, Tx_data, 11);
   HAL_SPI_Receive_IT(wifi_spi, RX_Buffer, BUFFER_SIZE);
   HAL_UART_Transmit_IT(pc_uart, RX_Buffer, BUFFER_SIZE);
 }

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
 {
   UNUSED(huart);
   //HAL_UART_Transmit_DMA(&huart1, Rx_data, 11);
   HAL_UART_Receive_IT(pc_uart, TX_Buffer, BUFFER_SIZE);
   HAL_SPI_Transmit_IT(wifi_spi, TX_Buffer, BUFFER_SIZE);
 }*/
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
