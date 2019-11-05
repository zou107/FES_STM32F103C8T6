/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#define DEBUG_ON
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t aRxBuffer1[1];
uint8_t aRxBuffer2[1];
uint8_t aRxBuffer3[1];


unsigned char imu1_buf[33];
unsigned char imu2_buf[33];
unsigned char has_data_uart2 = 0;
unsigned char has_data_uart3 = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define IMU_DATA_LEN   35

//0-2 acc; 3-5 speed; 6-8 angle;
float imu1_data[9]= {0,0,0,0,0,0,0,0,0};
float imu2_data[9]= {0,0,0,0,0,0,0,0,0};

unsigned char imu1_to_pc_buf[IMU_DATA_LEN];  // send to pc
unsigned char imu2_to_pc_buf[IMU_DATA_LEN];
unsigned char adc1_to_pc_buf[IMU_DATA_LEN];             // head num low high check
unsigned char adc2_to_pc_buf[IMU_DATA_LEN];
unsigned char imu_state = 0;
unsigned int adc_value[4] = {0,0,0,0};

unsigned char my_memcpy(unsigned char *dst, const unsigned char *src, unsigned char count)
{
	if (NULL == dst || NULL == src)
		return 1;
	while(count--)
	{
		*dst++ = *src++;
	}		
	return 0;
}

int check_sum(unsigned char *buf) 
{
	unsigned char sum = 0;
	int i = 0;
	
	for (i = 0; i < 10; i++) {
		sum += buf[i];
	}
	
	if (sum == buf[i]) {
		//printf("check ok.\n");
		return 0;
	}
	else
		return -1;
}

void check_imu(unsigned char *buf, unsigned char id)
{
	unsigned char *temp_buf = 0;
	unsigned char *temp = 0;
	float *imu = 0;
	
	if(1 == id) {
		imu_state = imu_state | 0x01;
		temp = imu1_to_pc_buf;
		imu = imu1_data;
		temp[0] = 0xAA;  // head
		temp[1] = 0xA1;  // imu1
	} else if(2 == id) {
		imu_state = imu_state | 0x02;
		temp = imu2_to_pc_buf;
		imu = imu2_data;
		temp[0] = 0xAA;  // head
		temp[1] = 0xA2;  // imu2
	}
	
	my_memcpy((temp+2), buf, 33);
	temp_buf = temp + 2;
	
	if (check_sum(temp_buf) || check_sum(temp_buf+11) || check_sum(temp_buf+22)) {
		//printf("=====check sum failed. id: %d.\n", id);

		if (1 == id)
			imu_state &= ~0x01;
		else
			imu_state &= ~0x02;
		return;
	}
		
	if(temp_buf[0]==0x55 && temp_buf[1]==0x51 
		&& temp_buf[11]==0x55 && temp_buf[12]==0x52 
	    && temp_buf[22]==0x55 && temp_buf[23]==0x53 )
	{
		imu[0] = ((short)(temp_buf[3]<<8 | temp_buf[2]))/32768.0*16;        // X acc
		imu[1] = ((short)(temp_buf[5]<<8 | temp_buf[4]))/32768.0*16;        // Y
		imu[2] = ((short)(temp_buf[7]<<8 | temp_buf[6]))/32768.0*16;        // Z
		
		imu[3] = ((short)(temp_buf[14]<<8| temp_buf[13]))/32768.0*2000;     // X speed
		imu[4] = ((short)(temp_buf[16]<<8| temp_buf[15]))/32768.0*2000;     // Y
		imu[5] = ((short)(temp_buf[18]<<8| temp_buf[17]))/32768.0*2000;     // Z
		
		imu[6] = ((short)(temp_buf[25]<<8| temp_buf[24]))/32768.0*180;      // X angle
		imu[7] = ((short)(temp_buf[27]<<8| temp_buf[26]))/32768.0*180;      // Y
		imu[8] = ((short)(temp_buf[29]<<8| temp_buf[28]))/32768.0*180;      // Z
		
//		if (2 == id)
//			printf("id: %d. \t angle: %.2f \t %.2f \t %.2f \r\n", id, imu[6], imu[7], imu[8]);
        
	}
}

void pack_adc_buf(unsigned char *buf, unsigned int value, int id)
{
    int i = 0;
    buf[0] = 0xAA;
    buf[1] = 0xB1 + id;
    buf[2] = (value & 0x00ff); // low
    buf[3] = (value >> 8);
    buf[4] = buf[0] + buf[1] + buf[2] + buf[3];
    
    for(i = 5; i< IMU_DATA_LEN; i++){
        buf[i] = 0;
    }
}

void twinking_led(unsigned char num)
{
    while(num--) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // LED on
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // LED off
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
    int i = 0;
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

    HAL_UART_Receive_IT(&huart1, aRxBuffer1, 1);
    HAL_UART_Receive_IT(&huart2, aRxBuffer2, 1);
    HAL_UART_Receive_IT(&huart3, aRxBuffer3, 1);
    HAL_TIM_Base_Start_IT(&htim3);
    
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // LED off
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // LED on
//	HAL_Delay(100);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // LED off
//	HAL_Delay(100);

//    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    HAL_Delay(5);
      
    if (has_data_uart2) {
        has_data_uart2 = 0;

        check_imu(imu1_buf, 1);
    }

    if (has_data_uart3) {
        has_data_uart3 = 0;

        check_imu(imu2_buf, 2);
    }
      
    // check imu state.
    if(imu_state == (0x01 | 0x02)) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // LED on
    } else {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // LED off
    }
    
    // read adc value
    for(i = 0; i < 2; i++) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, 0xffff);
        
        adc_value[i] = HAL_ADC_GetValue(&hadc1);
    }
    HAL_ADC_Stop(&hadc1);
    
    pack_adc_buf(adc1_to_pc_buf, adc_value[0], 0);
    pack_adc_buf(adc2_to_pc_buf, adc_value[1], 1);
    
#ifdef DEBUG_ON
        printf("%d,  %d. \n", adc_value[0], adc_value[1]);
        //printf("PA0 Voltage : %.4f \r\n", adc_value[0]*3.3f/4096);
#endif

    
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      
    //printf("test timer.123\n\n");
    //HAL_UART_Transmit(&huart1, "hello!\r\n", 8, 10);
      
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void initAngle()
{
    unsigned char data[3] = {0xFF, 0xAA, 0x52};
    
    HAL_UART_Transmit(&huart2, data, 3, 0xffff);
    HAL_UART_Transmit(&huart3, data, 3, 0xffff);
}

void calibrationACC()
{
    unsigned char data[3] = {0xFF, 0xAA, 0x67};
    
    HAL_UART_Transmit(&huart2, data, 3, 0xffff);
    HAL_UART_Transmit(&huart3, data, 3, 0xffff);
}

static unsigned char temp_uart1[3];
static unsigned char temp_uart2[11];
static unsigned char temp_uart3[11];

void User_UART1_IRQHandler()
{
    static unsigned char counter = 0;
	
    temp_uart1[counter] = aRxBuffer1[0];
    
    if(counter == 0 && temp_uart1[0] != 0xAA) // find head
        return;   
    counter++; 
    
    if(counter == 3) { // receive 3 byte
        counter = 0;
        
        if(temp_uart1[1] == 0x35 && temp_uart1[2] == 0x35) {
            // init angle(clear z)
            initAngle();
            
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); // LED on
        }else if(temp_uart1[1] == 0xCA && temp_uart1[2] == 0xCA) {
            // calibration ACC
            calibrationACC();
            
            //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); // LED off
        }
    }
}

// imu 1
void User_UART2_IRQHandler()
{
    static unsigned char counter = 0;
	static unsigned char temp_imu_state = 0;

    temp_uart2[counter] = aRxBuffer2[0];  

    if(counter == 0 && temp_uart2[0] != 0x55) // find head
        return;   
    counter++; 
    
    if(counter == 11) { // receive 11 byte
        counter = 0;
        
        if(temp_uart2[1] == 0x51) {
            my_memcpy(imu1_buf,temp_uart2,11);
            temp_imu_state = temp_imu_state | 0x01;
        }else if(temp_uart2[1] == 0x52) {
            my_memcpy((imu1_buf+11),temp_uart2,11);
            temp_imu_state = temp_imu_state | 0x02;
        }else if(temp_uart2[1] == 0x53) {
            my_memcpy((imu1_buf+22),temp_uart2,11);
            temp_imu_state = temp_imu_state | 0x04;
        }
        
        if(temp_imu_state == 0x07) { 
            // have reveive 3 package.
            temp_imu_state = 0;
            has_data_uart2 = 1;
        }
    }
}

// imu 2
void User_UART3_IRQHandler()
{
	static unsigned char counter = 0;
	static unsigned char temp_imu_state = 0;

    temp_uart3[counter] = aRxBuffer3[0];  

    if(counter == 0 && temp_uart3[0] != 0x55) // find head
        return;   
    counter++; 
    
    if(counter == 11) { // receive 11 byte
        counter = 0;
        
        if(temp_uart3[1] == 0x51) {
            my_memcpy(imu2_buf,temp_uart3,11);
            temp_imu_state = temp_imu_state | 0x01;
        }else if(temp_uart3[1] == 0x52) {
            my_memcpy((imu2_buf+11),temp_uart3,11);
            temp_imu_state = temp_imu_state | 0x02;
        }else if(temp_uart3[1] == 0x53) {
            my_memcpy((imu2_buf+22),temp_uart3,11);
            temp_imu_state = temp_imu_state | 0x04;
        }
        
        if(temp_imu_state == 0x07) { 
            // have reveive 3 package.
            temp_imu_state = 0;
            has_data_uart3 = 1;
        }
    }    
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1) {
        User_UART1_IRQHandler();
        
        HAL_UART_Receive_IT(&huart1, aRxBuffer1, 1);
    }
    
    if(huart->Instance == USART2) {
        User_UART2_IRQHandler();

        HAL_UART_Receive_IT(&huart2, aRxBuffer2, 1);
    }
    
    if(huart->Instance == USART3) {
        User_UART3_IRQHandler();
        
        HAL_UART_Receive_IT(&huart3, aRxBuffer3, 1);
    }
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        // send data to pc
		if(imu_state & 0x01) {
			imu_state &= (~0x01);
            
#ifndef DEBUG_ON
            HAL_UART_Transmit(&huart1, imu1_to_pc_buf, 35, 0xffff);
#endif
			//printf("send imu 1.\n");
		}
		
		if (imu_state & 0x02) {
			imu_state &= (~0x02);
            
#ifndef DEBUG_ON
            HAL_UART_Transmit(&huart1, imu2_to_pc_buf, 35, 0xffff);
#endif
			//printf("send imu 2.\n");
		}
        
        // send adc value to pc
#ifndef DEBUG_ON
        HAL_UART_Transmit(&huart1, adc1_to_pc_buf, 35, 0xffff);
        HAL_UART_Transmit(&huart1, adc2_to_pc_buf, 35, 0xffff);
#endif
        
        //printf("\nHAL_TIM_PeriodElapsedCallback timer3...\n\n");
    }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
