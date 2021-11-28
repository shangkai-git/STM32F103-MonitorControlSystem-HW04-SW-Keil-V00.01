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
#include "can.h"
#include "i2c.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "stdint.h"
#include "oled.h"
#include "bmp.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct _InputStatus
{
	uint8_t counter;
	uint8_t level;
	uint8_t old_level;
	uint8_t onToOffEdge;
	uint8_t offToOnEdge;
	uint8_t debouncing;
	uint8_t activeSts;
	/* data */
}InputStatus;

RTC_TimeTypeDef sTimestructure;
RTC_DateTypeDef sdatestructure;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DEB_CYCLES			    5   	// debounce_time = DEB_CYCLES x cycle_time 	
#define DEB_CYCLES_LONG		    30  	// debounce_time = DEB_CYCLES x cycle_time 	
#define A2D_AVG_SIZE		    5   	// size of A2D median buffer (must be odd) 	
#define Breathing1_Cycle 	    1000	// breath cycle: output gradually change cycle 	
#define Breathing2_Cycle 	    800		// breath cycle: output gradually change cycle 	
#define Alarm_Time_default      30000   // default value of Alarm time:30s
#define Alarm_set_time_default  600000 // default value of alarm set time
#define bit0                    0x01    //
#define bit1                    0x02
#define bit2                    0x04
#define bit3                    0x08
#define bit4                    0x10
#define bit5                    0x20
#define bit6                    0x40
#define bit7                    0x80
#define Led_Open                GPIO_PIN_RESET
#define Led_Close               GPIO_PIN_SET
#define Pump_open               4999
#define Pump_cycle              19999


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

InputStatus SW_Up 		= {0, 1, 0, 0, 0, 0, 0};
InputStatus SW_Down 	= {0, 1, 0, 0, 0, 0, 0};
InputStatus SW_Left		= {0, 1, 0, 0, 0, 0, 0};
InputStatus SW_Right 	= {0, 1, 0, 0, 0, 0, 0};
InputStatus SW_Ok       = {0, 1, 0, 0, 0, 0, 0};

InputStatus SW_Digital1 = {0, 1, 0, 0, 0, 0, 0};
InputStatus SW_Digital2 = {0, 1, 0, 0, 0, 0, 0};
InputStatus SW_Digital3	= {0, 1, 0, 0, 0, 0, 0};
InputStatus SW_Digital4	= {0, 1, 0, 0, 0, 0, 0};
InputStatus SW_Digital5	= {0, 1, 0, 0, 0, 0, 0};

static bool lsd1_open_sts = false;
static bool lsd2_open_sts = false;
static bool lsd3_open_sts = false;

/*** Battery voltage status ***/
static bool batt_normal_sts     = true;
//static bool batt_lowvolt_sts    = false;
//static bool batt_highvolt_sts   = false;

static uint16_t led1_num = 0;
static uint16_t led2_num = 0;
static uint16_t led3_num = 0;
static uint16_t led4_num = 0;

static uint16_t relay1_num = 0;
static uint32_t relay2_num = 0;
//static uint16_t relay3_num = 0;
//static uint16_t relay4_num = 0;
//static uint16_t relay5_num = 0;
//static uint16_t relay6_num = 0;
static uint16_t relay7_num = 0;

static bool     relay2_out_flag;
uint16_t        relay2_out_num;
static uint32_t Alarm_set_time  = 600000;
uint16_t        Alarm_Time      = 30000;
uint8_t         Alarm_Sts;
uint16_t        Pump_set_time   = Pump_open;



uint8_t SW_Up_Old    = false;
uint8_t SW_Down_Old  = false;
uint8_t SW_Left_Old  = false;
uint8_t SW_Right_Old = false;







static uint16_t hsd1_num = 0;
static uint16_t hsd2_num = 0;
static uint16_t hsd3_num = 0;
static uint16_t hsd4_num = 0;

static uint16_t lsd1_num = 0;
static uint16_t lsd2_num = 0;
static uint16_t lsd3_num = 0;

static uint16_t batt_low_deb_num   = 0;
static uint16_t batt_high_deb_num  = 0;


// static uint16_t hsd1_diag_value = 0;
// static uint16_t hsd2_diag_value = 0;
// static uint16_t hsd3_diag_value = 0;
// static uint16_t hsd4_diag_value = 0;

// static uint16_t lsd1_diag_value = 0;
// static uint16_t lsd2_diag_value = 0;
// static uint16_t lsd3_diag_value = 0;

// static uint16_t relay3_diag_value = 0;
// static uint16_t relay4_diag_value = 0;

/***	define PWM output channel duty cycle value	***/
uint16_t pwm_ch2_val = 0;
uint16_t pwm_ch3_val = 0;
uint16_t pwm_ch4_val = 0;


/***  define ADC Value  ***/
uint16_t voltage_Value[4];
uint16_t current_Value[4];
uint16_t adc_value[11];
uint16_t HC4851_AN_Value[7];
uint16_t Relay_AN_Value;


uint8_t  Vbat_100mv;
uint16_t adc_cnt;

/***	define debounce variables ***/
uint8_t     deb_cnt;
uint8_t     Task_10ms_cnt;
uint8_t     Task_100ms_cnt;
static bool Task_10ms_flag  = true;
static bool Task_100ms_flag = true;
static bool get_input_sts   = false;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//CAN_HandleTypeDef hcan1;

CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t 			TxData[8]   = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t             RxData[8]   = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint32_t 			TxMailbox;
uint32_t 			std_id      = 0x123;
CAN_FilterTypeDef   sFilterConfig;

/*** SPI transmit array ***/
uint8_t	spi1TxData[16] = {0x31, 0x34, 0x37, 0x77, 0x78, 0x79, 0x7a, 0x39, 0x00, 0x07, 0x77, 0x54, 0xd6, 0xa4, 0x31, 0x02};
uint8_t	spi2TxData[16] = {0x00, 0x07, 0x77, 0x54, 0xd6, 0xa4, 0x31, 0x02, 0x31, 0x34, 0x37, 0x77, 0x78, 0x79, 0x7a, 0x39};

uint8_t t   = ' ';
uint8_t a[] = "Hello world!";
uint8_t b[] = "Init success!";
uint8_t c[] = "this is a test data! what are you want?";
uint8_t d[] = "Program Size: Code=4584 RO-data=6432 RW-data=72 ZI-data=2264  ";
uint8_t e[] = "*** Using Compiler 'V5.06 update 6 (build 750)', folder: 'D:/Keil_v5/ARM/ARMCC/Bin'";
uint8_t f[] = "123456789abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ!@#$%^&*()_+/*-+~_=[]{};:,./12345678910";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void INPUT_Read(void);
void INPUT_SW_deb(void);
void LED_Output(void);
void LSD_Output(void);
void HSD_Output(void);
void RELAY_Output(void);
void Get_ADC_Value(void);
uint16_t ADC_Convert(void);
void CAN_Transmit(void);
void SPI_Transmit(void);
void SPI_Receive(void);
void Task_10ms(void);
void Task_100ms(void);
void Batt_Manage(void);
void CAN_Filter_Config(void);  // CAN Filter Configration 
void Info_Display(void);
void OLED_Parament_Set(void);
void PUMP_Parament_Set(void);

void Alarm_Sts_Relay2(void);
void Input_deb(uint8_t level, InputStatus *const pInputStatus);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  CAN_Filter_Config();
  HAL_CAN_Start(&hcan);
  
  HAL_TIM_Base_Start_IT(&htim1);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim4);
  
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  /* ADC1 Calibration */
  HAL_ADCEx_Calibration_Start(&hadc1);

	/*** OLED Function ***/
  OLED_Init();
  OLED_ColorTurn(0);//0正常显示，1 反色显示
  OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
  OLED_Refresh();
	
//  OLED_DrawSquare(5,5,120,58);
//  OLED_Printf((uint8_t *)"this is a test");
	
//  OLED_Refresh();  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    Task_10ms();
    Task_100ms();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	Info_Display();	

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //Timer1 callback function ,interrupt processing;
{
	
	if(TIM1 == htim->Instance)
	{		
		led1_num++;
		led2_num++;
		led3_num++;
		led4_num++;
		
		relay1_num++;
//		relay2_num++;
//		relay3_num++;
//		relay4_num++;
//		relay5_num++;	
//		relay6_num++;

        if (SW_Ok.activeSts)
		{
		    relay7_num++;
        }
        else
        {
            relay7_num = 0;
        }
        
		
		hsd1_num++;
		hsd2_num++;
		hsd3_num++;
		hsd4_num++;
		adc_cnt++;
		Task_10ms_cnt++;
		Task_100ms_cnt++;
		
		batt_low_deb_num++;
		batt_high_deb_num++;
		
		switch(Task_10ms_cnt)
		{
			case 1:
				HAL_GPIO_WritePin(VCC_SW_GPIO_Port,VCC_SW_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(VBAT_SW_GPIO_Port,VBAT_SW_Pin,GPIO_PIN_SET);	
			    break;
			case 2:
				get_input_sts = true;
			    break;
			case 3:
				Task_10ms_flag = true;
			    break;
			case 10:
				Task_10ms_cnt = 0;
			    break;
			default:
			    break;
		}

		switch(Task_100ms_cnt)
		{
//            case 1:
//				HAL_GPIO_TogglePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin);
//			break;
			case 5:
				Task_100ms_flag = true;
				break;
			case 100:
				Task_100ms_cnt = 0;
			default:
			break;
		}

		if (SW_Digital5.activeSts)
		{
			if (lsd1_num < Breathing1_Cycle)	//if switch open, the lsd1_num will increasing,until the num to be 1000;
			{
				lsd1_num++;
			}			
		}
		else
		{
			if (lsd1_num > 0)	//if switch close,the lsd1_num will decreasing,until the num to be 0;
			{
				lsd1_num--;
			}
		}

		if (SW_Digital4.activeSts)
		{
			if (lsd2_num < Breathing1_Cycle)	//if switch open, the lsd1_num will increasing,until the num to be 1000;
			{
				lsd2_num++;
			}			
		}
		else
		{
			if (lsd2_num > 0)	//if switch close,the lsd1_num will decreasing,until the num to be 0;
			{
				lsd2_num--;
			}
		}
		
		if (SW_Digital3.activeSts)
		{
			if (lsd3_num < Breathing2_Cycle)	//if switch open, the lsd1_num will increasing,until the num to be 800;
			{
				lsd3_num++;
			}			
		}
		else
		{
			if (lsd3_num > 0)	//if switch close,the lsd1_num will decreasing,until the num to be 0;
			{
				lsd3_num--;
			}
		}
		LSD_Output();	
//        Alarm_Sts_Relay2();
	}
	if (TIM4 == htim->Instance)
	{
		HAL_GPIO_TogglePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin);
	}
}


void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  /* Get RX message */
  	if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  	{
    	/* Reception Error */
    	Error_Handler();
  	}
  	if ((RxHeader.StdId == 0x122) && (RxHeader.DLC == 8))
  	{
		/* CAN receive test */
		switch (RxData[0])
		{
			case 1:
				HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Close); // led2 close
				HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,Led_Close); // led3 close
				HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,Led_Close); // led4 close
				HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,Led_Open); // led1 open
			    break;
			case 2:
				HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,Led_Close); // led1 close
				HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,Led_Close); // led3 close
				HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,Led_Close); // led4 close
				HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Open); // led2 open
			    break;
			case 3:
				HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,Led_Close); // led1 close
				HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Close); // led2 close
				HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,Led_Close); // led4 close
				HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,Led_Open); // led3 open
			    break;
			case 4:
				HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,Led_Close); // led1 close
				HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Close); // led2 close
				HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,Led_Close); // led3 close
				HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,Led_Open); // led4 open
			    break;
			default:
				HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,Led_Close); // led1 close
				HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Close); // led2 close
				HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,Led_Close); // led3 close
				HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,Led_Close); // led4 close
			    break;
		}
  	}
	else
	{
		HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,Led_Close); // led1 close
		HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Close); // led2 close
		HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,Led_Close); // led3 close
		HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,Led_Close); // led4 close		
	}
}


/*	read input status */
void INPUT_Read(void)
{
	SW_Ok.level			= HAL_GPIO_ReadPin(SW1_IN_GPIO_Port,SW1_IN_Pin);
	SW_Right.level		= HAL_GPIO_ReadPin(SW2_IN_GPIO_Port,SW2_IN_Pin);
	SW_Left.level		= HAL_GPIO_ReadPin(SW3_IN_GPIO_Port,SW3_IN_Pin);
	SW_Down.level		= HAL_GPIO_ReadPin(SW4_IN_GPIO_Port,SW4_IN_Pin);
	SW_Up.level			= HAL_GPIO_ReadPin(SW5_IN_GPIO_Port,SW5_IN_Pin);
	SW_Digital1.level	= HAL_GPIO_ReadPin(DIGITAL1_IN_GPIO_Port,DIGITAL1_IN_Pin);
	SW_Digital2.level	= HAL_GPIO_ReadPin(DIGITAL2_IN_GPIO_Port,DIGITAL2_IN_Pin);
	SW_Digital3.level	= HAL_GPIO_ReadPin(DIGITAL3_IN_GPIO_Port,DIGITAL3_IN_Pin);
	SW_Digital4.level	= HAL_GPIO_ReadPin(DIGITAL4_IN_GPIO_Port,DIGITAL4_IN_Pin);
	SW_Digital5.level	= HAL_GPIO_ReadPin(DIGITAL5_IN_GPIO_Port,DIGITAL5_IN_Pin);	
}

void INPUT_SW_deb(void) 	//switch input debounce
{
	/*NEW DEBOUNCE FUNCTION*/
	Input_deb(SW_Up.level, &SW_Up);
	Input_deb(SW_Down.level, &SW_Down);
	Input_deb(SW_Left.level, &SW_Left);
	Input_deb(SW_Right.level, &SW_Right);
	Input_deb(SW_Ok.level, &SW_Ok);
	Input_deb(SW_Digital1.level, &SW_Digital1);
	Input_deb(SW_Digital2.level, &SW_Digital2);
	Input_deb(SW_Digital3.level, &SW_Digital3);
	Input_deb(SW_Digital4.level, &SW_Digital4);
	Input_deb(SW_Digital5.level, &SW_Digital5);
}



/**
* @brief  SPI Transmit Function
* @retval None
*/
void CAN_Transmit(void)
{
	if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
	{
		Error_Handler();
	}
}


  /**
  * @brief  SPI transmit function
  * @retval None
  * @Author	sk
  * @Update None
  */
void SPI_Transmit(void)
{
	/****SPI1 Message Transmit ****/
	if (HAL_SPI_Transmit(&hspi1, spi1TxData, 16, 5) != HAL_OK)
	{
		Error_Handler();
	}

//	/****SPI2 Message Transmit ****/
//	if (HAL_SPI_Transmit(&hspi2, spi2TxData, 8, 10) != HAL_OK)
//	{
//		Error_Handler();
//	}
}


/**
* @brief  SPI receive function
* @retval None
* @Author sk
* @Update None
*/
void SPI_Receive(void)
{
//	/****SPI1 Message Transmit ****/
//	if (HAL_SPI_Transmit(&hspi1, spi1TxData, 8, 10) != HAL_OK)
//	{
//		Error_Handler();
//	}

}

void LED_Output(void)
{
	/* led4 control	*/
	if(SW_Up.activeSts)
	{		
		if (led4_num < 499)
		{		
//			HAL_GPIO_TogglePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin);
			HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,Led_Open);
			TxData[0] |= bit3;
//			led1_num = 0;
		}
		else if (led4_num < 999)
		{
			HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,Led_Close);
			TxData[0] &=~ bit3;
		}
		else
		{
			led4_num = 0u;			
		}
	}
	else
	{
//		HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,GPIO_PIN_SET);
		led4_num = 0u;
	}

	if (!batt_normal_sts)
	{
		HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,Led_Open);
	}
	else
	{
		HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,Led_Close);
	}

//	if(SW_Up.activeSts && batt_normal_sts)
//	{
//		HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,GPIO_PIN_RESET);
//	}
//	else
//	{
//		HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,GPIO_PIN_SET);
//	}
//	if(SW_Down.activeSts && batt_normal_sts)
//	{
//		HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,GPIO_PIN_RESET);
//	}
//	else
//	{
//		HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,GPIO_PIN_SET);
//	}
//	if(SW_Left.activeSts && batt_normal_sts)
//	{
//		HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,GPIO_PIN_RESET);
//	}
//	else
//	{
//		HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,GPIO_PIN_SET);
//	}
//	if(SW_Right.activeSts && batt_normal_sts)
//	{
//		HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,GPIO_PIN_RESET);
//	}
//	else
//	{
//		HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,GPIO_PIN_SET);
//	}
	if(SW_Ok.activeSts)
	{
		HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED3_OUT_GPIO_Port,LED3_OUT_Pin,GPIO_PIN_SET);
		HAL_GPIO_WritePin(LED4_OUT_GPIO_Port,LED4_OUT_Pin,GPIO_PIN_SET);
	}
	else
	{
//		HAL_GPIO_WritePin(RELAY5_OUT_GPIO_Port,RELAY5_OUT_Pin,GPIO_PIN_RESET);
	}
}

void RELAY_Output(void)
{
	/* M3 wiper Low Temperature Test Function */
//	if (SW_Up.activeSts)
//	{
//		if (relay1_num < 2999)
//		{
////			HAL_GPIO_WritePin(RELAY1_OUT_GPIO_Port,RELAY1_OUT_Pin,GPIO_PIN_RESET);
////			HAL_GPIO_WritePin(RELAY2_OUT_GPIO_Port,RELAY2_OUT_Pin,GPIO_PIN_RESET);
//		}
//		else if (relay1_num < 6999)
//		{
//			HAL_GPIO_WritePin(RELAY1_OUT_GPIO_Port,RELAY1_OUT_Pin,GPIO_PIN_SET);
//			HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,Led_Open);
//			TxData[0] |= bit0;
//		}
//		else if (relay1_num < 8799)
//		{
//			HAL_GPIO_WritePin(RELAY2_OUT_GPIO_Port,RELAY2_OUT_Pin,GPIO_PIN_SET);
//			HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Open);
//			TxData[0] |= bit1;
//		}
//		else if (relay1_num < 11999)
//		{
//			HAL_GPIO_WritePin(RELAY2_OUT_GPIO_Port,RELAY2_OUT_Pin,GPIO_PIN_RESET);
//			HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Close);
//			TxData[0] &=~ bit1;
//		}
//		else
//		{
//			HAL_GPIO_WritePin(RELAY1_OUT_GPIO_Port,RELAY1_OUT_Pin,GPIO_PIN_RESET);
//
//			HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,Led_Close);
//			relay1_num = 0;
//			TxData[0] &=~ bit0;
//		}
//	}
//	else
//	{
//		HAL_GPIO_WritePin(RELAY1_OUT_GPIO_Port,RELAY1_OUT_Pin,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin,Led_Close);
//		HAL_GPIO_WritePin(RELAY2_OUT_GPIO_Port,RELAY2_OUT_Pin,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Close);
//		relay1_num = 0;
//		TxData[0] &=~ bit0;
//        TxData[0] &=~ bit1;
//	}
//	
//	if (relay2_out_flag && Alarm_Sts == true)
//	{
//        if (relay2_out_num < 499)
//        {
//            HAL_GPIO_WritePin(RELAY2_OUT_GPIO_Port,RELAY2_OUT_Pin,GPIO_PIN_SET);            
//        }
//        else if (relay2_out_num < 999)
//        {
//            HAL_GPIO_WritePin(RELAY2_OUT_GPIO_Port,RELAY2_OUT_Pin,GPIO_PIN_RESET);
//        }
//        else
//        {
//            relay2_out_num = 0;
//        }       
//	}
//	else
//	{
//		HAL_GPIO_WritePin(RELAY2_OUT_GPIO_Port,RELAY2_OUT_Pin,GPIO_PIN_RESET);
//	}
	
	if(SW_Ok.activeSts)
	{
        if (relay7_num < Pump_set_time)
        {
            HAL_GPIO_WritePin(RELAY7_OUT_GPIO_Port,RELAY7_OUT_Pin,GPIO_PIN_SET);
            HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Open);
        }
        else if (relay7_num < Pump_cycle)
        {
            HAL_GPIO_WritePin(RELAY7_OUT_GPIO_Port,RELAY7_OUT_Pin,GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Close);
        }
        else
        {
            relay7_num = 0;
        }
	}
	else
	{
		HAL_GPIO_WritePin(RELAY7_OUT_GPIO_Port,RELAY7_OUT_Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Close);
	}
//	
//	if(SW_Left.activeSts)
//	{
//		HAL_GPIO_WritePin(RELAY3_OUT_GPIO_Port,RELAY3_OUT_Pin,GPIO_PIN_SET);
//	}
//	else
//	{
//		HAL_GPIO_WritePin(RELAY3_OUT_GPIO_Port,RELAY3_OUT_Pin,GPIO_PIN_RESET);
//	}
//	
//	if(SW_Right.activeSts)
//	{
//		HAL_GPIO_WritePin(RELAY4_OUT_GPIO_Port,RELAY4_OUT_Pin,GPIO_PIN_SET);
//	}
//	else
//	{
//		HAL_GPIO_WritePin(RELAY4_OUT_GPIO_Port,RELAY4_OUT_Pin,GPIO_PIN_RESET);
//	}
//	
//	if(SW_Ok.activeSts)
//	{
//		HAL_GPIO_WritePin(RELAY1_OUT_GPIO_Port,RELAY1_OUT_Pin,GPIO_PIN_RESET);
//		HAL_GPIO_WritePin(RELAY2_OUT_GPIO_Port,RELAY2_OUT_Pin,GPIO_PIN_RESET);		
//		HAL_GPIO_WritePin(RELAY3_OUT_GPIO_Port,RELAY3_OUT_Pin,GPIO_PIN_RESET);		
//		HAL_GPIO_WritePin(RELAY4_OUT_GPIO_Port,RELAY4_OUT_Pin,GPIO_PIN_RESET);
//		SW_Up.activeSts 	= 0u;
//		SW_Down.activeSts 	= 0u;
//		SW_Left.activeSts	= 0u;
//		SW_Right.activeSts	= 0u;
//	}
//	else
//	{
//		HAL_GPIO_WritePin(RELAY5_OUT_GPIO_Port,RELAY5_OUT_Pin,GPIO_PIN_RESET);
//	}	
}

void HSD_Output(void)
{	
	 /* hsd1 control	*/
	if(SW_Digital1.activeSts)
	{
		if (hsd1_num < 199)
		{		
			HAL_GPIO_WritePin(HSD1_OUT_GPIO_Port,HSD1_OUT_Pin,GPIO_PIN_SET);
		}
		else if(hsd1_num < 299)
		{
			HAL_GPIO_WritePin(HSD1_OUT_GPIO_Port,HSD1_OUT_Pin,GPIO_PIN_RESET);
		}
		else if(hsd1_num < 449)
		{
			HAL_GPIO_WritePin(HSD1_OUT_GPIO_Port,HSD1_OUT_Pin,GPIO_PIN_SET);
		}
		else if(hsd1_num < 1999)
		{	
			HAL_GPIO_WritePin(HSD1_OUT_GPIO_Port,HSD1_OUT_Pin,GPIO_PIN_RESET);
		}
		else
		{
			hsd1_num = 0;				
		}
	}
	else
	{
		HAL_GPIO_WritePin(HSD1_OUT_GPIO_Port,HSD1_OUT_Pin,GPIO_PIN_RESET);
	}


	if(SW_Digital2.activeSts)
	{
		if (hsd2_num < 499)
		{		
			HAL_GPIO_WritePin(HSD2_OUT_GPIO_Port,HSD2_OUT_Pin,GPIO_PIN_SET);
		}
		else if (hsd2_num <999)
		{
			HAL_GPIO_WritePin(HSD2_OUT_GPIO_Port,HSD2_OUT_Pin,GPIO_PIN_RESET);
		}
		else
		{
			hsd2_num = 0;				
		}
	}
	else
	{
		HAL_GPIO_WritePin(HSD2_OUT_GPIO_Port,HSD2_OUT_Pin,GPIO_PIN_RESET);
	}
}


/* Low Side Driver output control */
void LSD_Output(void)
{
	/* lsd1 control, PWM Output Control */	
	if (SW_Digital5.activeSts)
	{
		/*	In the first 1000ms（Breathing1_Cycle）,duty cycle from 0 to 100	*/
		if (lsd1_num < Breathing1_Cycle || !lsd1_open_sts)	//in the first 1000ms, the duty cycle increase from 0 to 100%;
		{
			pwm_ch2_val = lsd1_num/10;	//every 10ms, the duty cycle increase 1;
			if (pwm_ch2_val < 100)			//"htim4.Init.Period" determines the step size	of duty cycle; 100/htim4.init.period = step,Example:100/1000 =0.1%;
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_ch2_val);
			}
			else
			{
				lsd1_open_sts = true;	//Once the output fully open,set the "lsd1_open_sts" to true;
			}
		}
		else
		{
			pwm_ch2_val = 100;
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_ch2_val);	
		}
	}
	else
	{
		if (lsd1_open_sts || lsd1_num > 0)	//during once open cycle,this function only running once;
		{
			pwm_ch2_val = lsd1_num/10;	//every 10ms, the duty cycle increasing 1;
			if (pwm_ch2_val > 0)			//"htim4.Init.Period" determines the step size	of duty cycle; 100/htim4.init.period = step,Example:100/1000 =0.1%;
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_ch2_val);
			}
			else
			{
				lsd1_open_sts = false;	//Once the output fully open,set the "lsd1_open_sts" to true;
			}
		}
		else
		{
//			lsd1_num = 0;
			pwm_ch2_val = 0;
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_ch2_val);	
		}
//		lsd1_open_sts = false;
//		pwm_ch2_val =0;
//		__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,pwm_ch2_val);
//		lsd1_num = 0;
	}
	
	/* lsd2 control	*/	
	if (SW_Digital4.activeSts)
	{
		if (lsd2_num < Breathing1_Cycle && !lsd2_open_sts)
		{	
			pwm_ch3_val = lsd2_num/10;
			if (pwm_ch3_val < 100)
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_ch3_val);
//				pwm_ch3_val++;	
			}
			else
			{
				lsd2_open_sts = true;
			}
		}
		else
		{
			pwm_ch3_val = 100;
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_ch3_val);
		}
	}
	else
	{
		if (lsd2_open_sts || lsd2_num > 0)
		{
			pwm_ch3_val = lsd2_num/10;
			if (pwm_ch3_val > 0)
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_ch3_val);
			}
			else
			{
				lsd2_open_sts = false;
			}		    
		}
		else
		{
			pwm_ch3_val = 0;
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,pwm_ch3_val);
//			lsd2_num = 0;
		}		
	}	

	/* lsd3 control	*/	
	if (SW_Digital3.activeSts)
	{
		if (lsd3_num < Breathing2_Cycle && !lsd3_open_sts)
		{	
			pwm_ch4_val = lsd3_num/8;
			if (pwm_ch4_val < 100)
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm_ch4_val);
//				pwm_ch4_val++;	
			}
			else
			{
				lsd3_open_sts = true;
			}
		}
		else
		{
			pwm_ch4_val = 100;
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm_ch4_val);
		}
	}
	else
	{
		if (lsd3_open_sts || lsd3_num > 0)
		{
			pwm_ch4_val = lsd3_num/8;
			if (pwm_ch4_val > 0)
			{
				__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm_ch4_val);
			}
			else
			{
				lsd3_open_sts = false;
			}		    
		}
		else
		{
			pwm_ch4_val = 0;
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm_ch4_val);
//			lsd3_num = 0;
		}		
	}
}

/**
  * @brief  This function is executed in case of ADC convert.
  * @retval adc_value
  */
uint16_t ADC_Convert(void)
{
	uint16_t adc_tmp;
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 2);
	if (HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1),HAL_ADC_STATE_REG_EOC))
	{
		adc_tmp = HAL_ADC_GetValue(&hadc1) *3300/4096;
	}
	return adc_tmp;
}

/**
  * @brief  Task scheduling
  * @retval None
  */
void Task_10ms(void)
{
	if (true == get_input_sts)
	{
		INPUT_Read();			// read input status 
		INPUT_SW_deb();			// input status debounce 
		Get_ADC_Value();		// read adc value 
		Batt_Manage();
        
		HAL_GPIO_WritePin(VBAT_SW_GPIO_Port,VBAT_SW_Pin,GPIO_PIN_RESET);	//close the pull up power of input circuit 
		HAL_GPIO_WritePin(VCC_SW_GPIO_Port,VCC_SW_Pin,GPIO_PIN_RESET);    	//there has some problems of VCC_SW circuit,MCU couldn't control the VCC_SW;
		get_input_sts = false;	// disable read input flag;
	}
	
	if (true == Task_10ms_flag)
	{		
        LED_Output();
		RELAY_Output();
		LSD_Output();
		HSD_Output();
		Task_10ms_flag = false;
	}	
}


/**
  * @brief  Task scheduling
  * @retval None
  */
void Task_100ms(void)
{
	if(true == Task_100ms_flag)
	{
		Task_100ms_flag = false;
        OLED_Parament_Set();
        PUMP_Parament_Set();
		CAN_Transmit();
		SPI_Transmit();
        Info_Display(); 
        HAL_RTC_GetTime(&hrtc, &sTimestructure, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sdatestructure, RTC_FORMAT_BIN);
    }
}


/**
  * @brief  read adc value from register
  * @retval None
  */
void Get_ADC_Value(void)
{
	uint8_t tmp;
	for (tmp = 0; tmp < 11; tmp++)
	{
		adc_value[tmp] = ADC_Convert();		//存储AD值
		switch (tmp)
		{
			case 0:
					Vbat_100mv = adc_value[tmp] * 100 / 3053 + 5; 
					break;
			case 1:
					voltage_Value[0] = adc_value[tmp];
					break;
			case 2:
					voltage_Value[1] = adc_value[tmp];
					break;
			case 3:
					voltage_Value[2] = adc_value[tmp];
					break;
			case 4:
					voltage_Value[3] = adc_value[tmp];
					break;
			case 5:
					current_Value[0] = adc_value[tmp];
					break;
			case 6:
					current_Value[1] = adc_value[tmp];
					break;
			case 7:
					current_Value[2] = adc_value[tmp];
					break;
			case 8:
					current_Value[3] = adc_value[tmp];
					break;
			case 9:
					HC4851_AN_Value[0] = adc_value[tmp];
					break;
			case 10:
					Relay_AN_Value = adc_value[tmp];
					break;
			default:
			        break;                
		}      
              
		if (1 < tmp && tmp < 8) 							//TxData[]数组最大只有八个元素;
		{
			TxData[tmp] = adc_value[tmp]>>4;
		}
	}
//	Vbat_100mv = adc_value[0] * 6 / 100 + 5;	// Battery Voltage = adc_value[0]
//	Vbat_100mv = adc_value[0] * 100 / 3053 + 5;
	TxData[1] = Vbat_100mv;
	adc_cnt = 0;
//	HAL_GPIO_TogglePin(LED1_OUT_GPIO_Port,LED1_OUT_Pin);
}

/**
  * @brief  input debounce,judegement the input status;
  *         
  * @param  file: 
  * @param  line: 
  * @retval 
  */
void Input_deb(uint8_t level, InputStatus *const pInputStatus)
{
	pInputStatus->level = level;
	if (pInputStatus->level == pInputStatus->old_level)
	{
		if (pInputStatus->debouncing == true)
		{
			pInputStatus->counter++;
			if (pInputStatus->counter > DEB_CYCLES - 1)
			{
				pInputStatus->debouncing = false;
				if (level == 0)
				{
					pInputStatus->onToOffEdge = true;
					pInputStatus->offToOnEdge = false;
					pInputStatus->activeSts = !pInputStatus->activeSts;
				}
				else
				{
					pInputStatus->offToOnEdge = true;
					pInputStatus->onToOffEdge = false;
				}
			}
		}
		else
		{
			pInputStatus->counter = 0;
		}
	}
	else
	{
		pInputStatus->old_level     = pInputStatus->level;  /* store new status */
		pInputStatus->debouncing    = true;	                /* set deb status */
		pInputStatus->counter       = 0;
	}
}

/**
  * @brief  battery state judgement
  *         
  * @param  file: 100mV battery value
  * @param  line: 
  * @retval Batt_normal_sts
  */
void Batt_Manage(void)
{
	if (batt_normal_sts == true)
	{
		if (Vbat_100mv < 20 && batt_low_deb_num > 1000)
		{
			batt_normal_sts = false;
		}
		else
		{
			batt_low_deb_num = 0;
		}
		if (Vbat_100mv > 55 && batt_high_deb_num > 500)
		{
			batt_normal_sts = false;
		}
		else
		{
			batt_high_deb_num = 0;
		}
	}
	else
	{
		if (Vbat_100mv > 40 && batt_low_deb_num > 1000)
		{
			batt_normal_sts = true;
		}
		else
		{
			batt_low_deb_num = 0;
		}
		if (Vbat_100mv < 110 && batt_high_deb_num > 500)
		{
			batt_normal_sts = true;
		}
		else
		{
			batt_high_deb_num = 0;
		}
	}	
}


/**
  * @brief  CAN Filter Configration
  *         
  * @param  file: Configrate the CAN Filter
  * @param  line: 
  * @retval None
  */
void CAN_Filter_Config(void)
{
	sFilterConfig.FilterBank 			= 0;
	sFilterConfig.FilterMode 			= CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale 			= CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh 			= 0x0000;
	sFilterConfig.FilterIdLow 			= 0x0000;
	sFilterConfig.FilterMaskIdHigh 		= 0x0000;
	sFilterConfig.FilterMaskIdLow 		= 0x0000;
	sFilterConfig.FilterFIFOAssignment 	= CAN_RX_FIFO0;
	sFilterConfig.FilterActivation 		= ENABLE;
	sFilterConfig.SlaveStartFilterBank 	= 14;	

	if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	{
		/* Notification Error */
		Error_Handler();
	}

	TxHeader.RTR   			  	= CAN_RTR_DATA;
	TxHeader.IDE   			  	= CAN_ID_STD;
	TxHeader.StdId 			  	= std_id;
 	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC				= 8;
}


/* 信息显示：通过OLED屏幕显示信息 */
void Info_Display(void)
{
    OLED_ShowString(2, 2, "CYC Time:", 16);
    OLED_ShowNum(75, 2, (relay7_num + 1)/1000, 2, 16);
    OLED_ShowChar(91, 2, '.', 16);
    OLED_ShowNum(96, 2, (relay7_num + 1)/100, 1, 16);
//	OLED_ShowNum(75, 2, relay2_num/1000, 4, 16);
	OLED_ShowChar(114, 2, 'S', 16);
    OLED_ShowString(2, 18, "SET Time:", 16);
    OLED_ShowNum(75, 18, (Pump_set_time + 1)/1000, 2, 16);
    OLED_ShowChar(91, 18, '.', 16);
    OLED_ShowNum(96, 18, (Pump_set_time + 1)/100, 1, 16);
//    OLED_ShowNum(75, 18, Alarm_set_time/1000, 4, 16);
    OLED_ShowChar(114, 18, 'S', 16);
    /* 日期时间显示 */
    OLED_ShowString(2, 34, "20", 12);
    OLED_ShowNum(14, 34, sdatestructure.Year, 2, 12);
    OLED_ShowChar(26, 34, '.', 12);
    OLED_ShowNum(32, 34, sdatestructure.Month, 2, 12);
    OLED_ShowChar(44, 34, '.', 12);
    OLED_ShowNum(50, 34, sdatestructure.Date, 2, 12);
    OLED_ShowChar(62, 34, '/', 12);
    OLED_ShowNum(68, 34, sTimestructure.Hours, 2, 12);
    OLED_ShowChar(80, 34, '.', 12);
    OLED_ShowNum(86, 34, sTimestructure.Minutes, 2, 12);
    OLED_ShowChar(98, 34, '.', 12);
    OLED_ShowNum(104, 34, sTimestructure.Seconds, 2, 12);
    /* 更新显示内容 */
	OLED_Refresh(); 
}

/* 设置OLED显示参数 */
void OLED_Parament_Set(void)
{
    if (SW_Up.activeSts != SW_Up_Old)
    {
        SW_Up_Old = SW_Up.activeSts;
        Alarm_set_time = Alarm_set_time + 1000u;
    }
    if (SW_Down.activeSts != SW_Down_Old)
    {
        SW_Down_Old = SW_Down.activeSts;
        Alarm_set_time = Alarm_set_time - 1000u;
    }
}


/* 设置OLED显示参数 */
void PUMP_Parament_Set(void)
{
    if (SW_Left.activeSts != SW_Left_Old)
    {
        SW_Left_Old = SW_Left.activeSts;
        Pump_set_time = Pump_set_time + 100u;
    }
    if (SW_Right.activeSts != SW_Right_Old)
    {
        SW_Right_Old = SW_Right.activeSts;
        Pump_set_time = Pump_set_time - 100u;
    }
}


/* 报警继电器（Relay2）的警报状态处理 */
void Alarm_Sts_Relay2(void)
{
    if (SW_Ok.activeSts)
    {
        relay2_num++;
        if (relay2_num > Alarm_set_time)
        {
            relay2_out_flag = true;     //置位继电器2输出标志；
            Alarm_Sts = true;           //置位报警状态；
        }
        else 
        {
            relay2_out_flag = false;
        }
    }
    else
    {
        relay2_out_flag = false;
        relay2_num = 0;
    }

    /* 满足报警继电器输出条件，继电器输出计时器开始累加 */
    if (relay2_out_flag)
    {
        relay2_out_num++;
    }
    else
    {
        relay2_out_num = 0;
    }

    /* 报警时间倒计时，计时结束后，清除报警继电器输出标志，报警继电器停止周期输出 */
    if (Alarm_Sts)
    {
        Alarm_Time--;
        if (Alarm_Time == 0u)
        {
            relay2_out_flag = false;
            relay2_out_num = 0;
            relay2_num = 0;
            Alarm_Sts = false;
            Alarm_Time = Alarm_Time_default;
        }
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

    HAL_GPIO_WritePin(LED2_OUT_GPIO_Port,LED2_OUT_Pin,Led_Open);
      
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
