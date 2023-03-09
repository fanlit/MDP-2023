/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdlib.h"
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CNT_ERROR_TOLERANCE 10
#define PWM_MAX 1500
#define SERVO_RIGHT_MAX 230
#define SERVO_LEFT_MAX 100
#define SERVO_STRAIGHT 150
#define INTEGRAL_MAX 1000000
#define GREATER_TURN_PWM 1500
#define LESSER_TURN_PWM 1200

// for ultrasonic sensor
#define TRIG_PIN GPIO_PIN_13
#define TRIG_PORT GPIOD

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart3;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for DCMotor */
osThreadId_t DCMotorHandle;
const osThreadAttr_t DCMotor_attributes = {
  .name = "DCMotor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for RefreshOLED */
osThreadId_t RefreshOLEDHandle;
const osThreadAttr_t RefreshOLED_attributes = {
  .name = "RefreshOLED",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderMotorA */
osThreadId_t EncoderMotorAHandle;
const osThreadAttr_t EncoderMotorA_attributes = {
  .name = "EncoderMotorA",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for EncoderMotorB */
osThreadId_t EncoderMotorBHandle;
const osThreadAttr_t EncoderMotorB_attributes = {
  .name = "EncoderMotorB",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_Command_RX */
osThreadId_t UART_Command_RXHandle;
const osThreadAttr_t UART_Command_RX_attributes = {
  .name = "UART_Command_RX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Gyrohandle */
osThreadId_t GyrohandleHandle;
const osThreadAttr_t Gyrohandle_attributes = {
  .name = "Gyrohandle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for IRsensortask */
osThreadId_t IRsensortaskHandle;
const osThreadAttr_t IRsensortask_attributes = {
  .name = "IRsensortask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM5_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
void StartDefaultTask(void *argument);
void DCMotor_task(void *argument);
void RefreshOLED_task(void *argument);
void EncoderMotorA_task(void *argument);
void EncoderMotorB_task(void *argument);
void UART_Command_RX_task(void *argument);
void Gyro(void *argument);
void IRsensor(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// New commands from Rpi board will be received and stored in this buffer
//ICM20948 imu;
//uint8_t aRxBuffer[1];
uint8_t aRxBuffer[5];
uint8_t oledRow0[20];
uint8_t oledRow1[20];
uint8_t oledRow2[20];
uint8_t oledRow3[20];
uint8_t oledRow4[20];
uint8_t oledRow5[20];

uint8_t buff[20];
double totalAngle = 0;
uint8_t ICMAddr = 0x68;
uint32_t prevTick;
int16_t angularSpeed;
double oldTotalAngle = 0;
double correction;

char cmd = '-';
int received = 0;
int turning = -1;
int dir = -2;
float input = 0.0;
int32_t diffA = 0;
int32_t diffB = 0;
int32_t pwmValB;
int32_t pwmValA;
int32_t millisOldA = 0;
int32_t errorOldA = 0;
int32_t errorAreaA = 0;
int32_t millisOldB = 0;
int32_t errorOldB = 0;
int32_t errorAreaB = 0;
float targetCount = 0.0;
double targetAngle = 0;

//for IR sensor
// TODO only one sensor is in use right now.
uint32_t right_adc;
uint32_t left_adc;
double right_sensor;
double left_sensor;
uint32_t right_sensor_int;
uint32_t left_sensor_int;
uint32_t LPF_SUM_right = 0;
uint32_t LPF_SUM_left = 0;
uint32_t counter = 0;

int8_t start = 0;
int8_t isDone = 0;

void SendFeedBack(int done){
	if(done == 1){
		HAL_UART_Transmit(&huart3, "1\n", 2, 0xFFFF);
	}
	if(done == -1){
		HAL_UART_Transmit(&huart3, "-1\n", 3, 0xFFFF);
	}
}

//void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin ) {
//
//	if ( GPIO_Pin == USER_PB_Pin) {
//		// toggle LED
//		if (start == 0){
//			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
//			start = 1;
//		}
//		else
//			start = 0;
// 	    }
//}

//void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
//{
//	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
//	{
//		if (Is_First_Captured==0) // if the first value is not captured
//		{
//			IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
//			Is_First_Captured = 1;  // set the first captured as true
//			// Now change the polarity to falling edge
//			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
//		}
//
//		else if (Is_First_Captured==1)   // if the first is already captured
//		{
//			IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
//			__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter
//
//			if (IC_Val2 > IC_Val1)
//			{
//				Difference = IC_Val2-IC_Val1;
//			}
//
//			else if (IC_Val1 > IC_Val2)
//			{
//				Difference = (0xffff - IC_Val1) + IC_Val2;
//			}
//
//			Distance = Difference * .034/2;
//			Is_First_Captured = 0; // set it back to false
//
//			// set polarity to rising edge
//			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
//			__HAL_TIM_DISABLE_IT(&htim4, TIM_IT_CC1);
//		}
//	}
//}
//
//void delay (uint16_t time)
//{
//	__HAL_TIM_SET_COUNTER(&htim4, 0);
//	while(__HAL_TIM_GET_COUNTER (&htim4) < time);
//}
//
//void HCSR04_Read (void)
//{
//	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
//	delay(10);  // wait for 10 us
//	HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low
//
//	__HAL_TIM_ENABLE_IT(&htim4, TIM_IT_CC1);
//}

void readByte(uint8_t addr, uint8_t* data){
	buff[0] = addr;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr<<1, buff, 1, 10);
	HAL_I2C_Master_Receive(&hi2c1, ICMAddr<<1, data, 2, 20);
}

void writeByte(uint8_t addr, uint8_t data){
	buff[0] = addr;
	buff[1] = data;
	HAL_I2C_Master_Transmit(&hi2c1, ICMAddr << 1, buff, 2, 20);
}

void gyroStart(){
	writeByte(0x07, 0x07);
	osDelay(10);
	writeByte(0x07, 0x00);
	osDelay(10);
}

void gyroInit(){

	writeByte(0x06, 0x00);
	osDelay(10);
	writeByte(0x03, 0x80);
	osDelay(10);
	writeByte(0x07, 0x07);
	osDelay(10);
	writeByte(0x06, 0x01);
	osDelay(10);
	writeByte(0x7F, 0x20);
	osDelay(10);
	writeByte(0x01, 0x2F);
	osDelay(10);
	writeByte(0x0, 0x00);
	osDelay(10);
	writeByte(0x7F, 0x00);
	osDelay(10);
	writeByte(0x07, 0x00);
	osDelay(10);
}

/* Start PWM for DC and servor motors
	 * htim8 channel 1: PWMA - DC motor A
	 * htim8 channel 2: PWMB - DC motor B
	 * htim1 channel 4: Servo motor*/


void StartPMWVal(void){
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void SetPinForward(void){
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_SET);
}

void SetPinReverse(void){
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_SET);
}

void SetPinStop(void){
	HAL_GPIO_WritePin(GPIOA, AIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, AIN1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, DIN1_Pin, GPIO_PIN_RESET);
}

void TurnFullLeft(void){
	turning = 1;
	htim1.Instance -> CCR4 = SERVO_LEFT_MAX;
}
void TurnFullRight(void){
	turning = 1;
	htim1.Instance -> CCR4 = SERVO_RIGHT_MAX;
}
// assume 90 deg.
void TurnStraighten(void){
	turning = 1;
	htim1.Instance -> CCR4 = SERVO_STRAIGHT;
}

void ResetValues(void)
{
	errorOldA = 0;
	errorOldB = 0;
	errorAreaA = 0;
	errorAreaB = 0;
	pwmValA = 0;
	pwmValB = 0;
	totalAngle = 0;
//	isDone = 0;
	__HAL_TIM_SET_COUNTER(&htim2, 0);
	__HAL_TIM_SET_COUNTER(&htim5, 0);
}

int CalcPIDA_Dist(void){
	int32_t errorA = 0;
	int32_t errorRate = 0;
	float_t kp = 0.40;
	float_t ki = 0.005;
	float_t kd = 200;
	int32_t dt = 0;
	int32_t millisNow;
	int32_t pwmValA_temp;
	float count = abs(__HAL_TIM_GET_COUNTER(&htim2));
	if(targetCount - count > 0)
	{
		millisNow = HAL_GetTick();
		dt = millisNow - millisOldA;
		millisOldA = millisNow;
		errorA = targetCount - count;
		errorRate = (errorA - errorOldA)/dt;
		errorOldA = errorA;
		errorAreaA = errorAreaA + errorA*dt;

		if(errorAreaA > INTEGRAL_MAX)
			errorAreaA = INTEGRAL_MAX;
		else if (errorAreaA < -INTEGRAL_MAX)
			errorAreaA = -INTEGRAL_MAX;

		pwmValA_temp = kp*errorA + kd*errorRate + ki*errorAreaA;

		if(pwmValA_temp > PWM_MAX){
			pwmValA_temp = PWM_MAX;
		}
		else if(pwmValA_temp < -PWM_MAX){
			pwmValA_temp = -PWM_MAX;
		}
		return pwmValA_temp;
	}
	else
	{
//		isDone = 1;
		dir = 0;
	}
	return 0;
}

int CalcPIDB_Dist(void){
	int32_t errorB = 0;
	int32_t errorRate = 0;
	float_t kp = 0.40;
	float_t ki = 0.005;
	float_t kd = 200;
	int32_t dt = 0;
	int32_t millisNow;
	int32_t pwmValB_temp;
	float count = abs(__HAL_TIM_GET_COUNTER(&htim5));
	if(targetCount - count > 0)
	{
		millisNow = HAL_GetTick();
		dt = millisNow - millisOldB;
		millisOldB = millisNow;
		errorB = targetCount - count;
		errorRate = (errorB - errorOldB)/dt;
		errorOldB = errorB;
		errorAreaB = errorAreaB + errorB*dt;
		if(errorAreaB > INTEGRAL_MAX)
			errorAreaB = INTEGRAL_MAX;
		else if (errorAreaB < -INTEGRAL_MAX)
			errorAreaB = -INTEGRAL_MAX;

		pwmValB_temp = kp*errorB + kd*errorRate + ki*errorAreaB;

		if(pwmValB_temp > PWM_MAX){
			pwmValB_temp = PWM_MAX;
		}
		else if(pwmValB_temp < -PWM_MAX){
			pwmValB_temp = -PWM_MAX;
		}
		return pwmValB_temp;
	}
	else
	{
		dir = 0;
	}
	return 0;
}

int ServoCorrection(void){
	int corrMultiplier = 6;

	if(dir == 1)
		correction = (double)(SERVO_STRAIGHT + totalAngle*corrMultiplier);
	else if(dir == -1)
		correction = (double)(SERVO_STRAIGHT - totalAngle*corrMultiplier);

	if(correction > SERVO_RIGHT_MAX){
		correction = SERVO_RIGHT_MAX;
	}
	else if(correction < SERVO_LEFT_MAX){
		correction = SERVO_LEFT_MAX;
	}
	return correction;
}

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
  MX_TIM8_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_TIM5_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 5);
  millisOldA = HAL_GetTick();
  millisOldB = HAL_GetTick();
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim5, 0);
//  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1); // for ultra sonic sensor
  HAL_Delay(1000);
//  IMU_Initialise(&imu, &hi2c1, &huart3);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of DCMotor */
  DCMotorHandle = osThreadNew(DCMotor_task, NULL, &DCMotor_attributes);

  /* creation of RefreshOLED */
//  RefreshOLEDHandle = osThreadNew(RefreshOLED_task, NULL, &RefreshOLED_attributes);

  /* creation of EncoderMotorA */
  EncoderMotorAHandle = osThreadNew(EncoderMotorA_task, NULL, &EncoderMotorA_attributes);

  /* creation of EncoderMotorB */
  EncoderMotorBHandle = osThreadNew(EncoderMotorB_task, NULL, &EncoderMotorB_attributes);

  /* creation of UART_Command_RX */
  UART_Command_RXHandle = osThreadNew(UART_Command_RX_task, NULL, &UART_Command_RX_attributes);

  /* creation of Gyrohandle */
  GyrohandleHandle = osThreadNew(Gyro, NULL, &Gyrohandle_attributes);

  /* creation of IRsensortask */
//  IRsensortaskHandle = osThreadNew(IRsensor, NULL, &IRsensortask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 160;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7199;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|Gyro_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, AIN2_Pin|AIN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIN1_Pin|DIN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : OLED_SCL_Pin OLED_SDA_Pin OLED_RST_Pin OLED_DC_Pin
                           LED3_Pin Gyro_Pin */
  GPIO_InitStruct.Pin = OLED_SCL_Pin|OLED_SDA_Pin|OLED_RST_Pin|OLED_DC_Pin
                          |LED3_Pin|Gyro_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : AIN2_Pin AIN1_Pin */
  GPIO_InitStruct.Pin = AIN2_Pin|AIN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIN1_Pin DIN2_Pin */
  GPIO_InitStruct.Pin = DIN1_Pin|DIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_PB_Pin */
  GPIO_InitStruct.Pin = USER_PB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_PB_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
/* When ever there is a message received, this function interrupt will be called*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart);
	cmd = (char) aRxBuffer[0];
	input = (int)(aRxBuffer[1] - '0')*1000 + (int)((char)aRxBuffer[2] - '0')*100 +
			(int)(aRxBuffer[3] - '0')*10 + (int)((char)aRxBuffer[4] - '0');
	targetCount = 0.0;
	if(cmd == 'a' || cmd == 'b')
	{
		targetCount = ((input - 35)/207.345) * 1560;
	}
	else if(cmd == 'c' || cmd == 'f') //left = +ve deg
	{
		targetAngle = totalAngle + input;
	}
	else if (cmd == 'd' || cmd == 'e') // right = -ve deg
	{
		targetAngle = totalAngle - input;
	}
	received = 1;
	HAL_UART_Receive_IT(&huart3, (uint8_t *)aRxBuffer, 5);
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	  sprintf(oledRow1, "eCnt: %5d", __HAL_TIM_GET_COUNTER(&htim2));
//	  sprintf(oledRow2, "Rd:%3d Ld:%3d", (int)right_sensor_int, (int)left_sensor_int);
	  sprintf(oledRow4, "Tang : %5d", (long)totalAngle);
//	  sprintf(oledRow3, "Rd : %3d", (int)right_sensor_int);
	  sprintf(oledRow3, "recvd: %d", received);
	  sprintf(oledRow2, "cnt: %5d", (long)targetCount);
//	  sprintf(oledRow4, "Sensor: %2.3f", Distance);
//	  sprintf(oledRow5, "cnt: %5d", (long)targetCount);
	  osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_DCMotor_task */
/**
* @brief Function implementing the DCMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DCMotor_task */
void DCMotor_task(void *argument)
{
  /* USER CODE BEGIN DCMotor_task */
	StartPMWVal();
	int angleOffset = 7;
	while(1)
	{
		switch (dir)
		{
			case 1:
//				__HAL_TIM_SET_COUNTER(&htim2, 0);
//				__HAL_TIM_SET_COUNTER(&htim5, 0);
				SetPinForward();
				if(turning == 0)
					htim1.Instance -> CCR4 = ServoCorrection();

				else if(turning == 1 && (cmd == 'c' || cmd == 'e'))// left
				{
					pwmValA = GREATER_TURN_PWM;
					pwmValB = LESSER_TURN_PWM;
				}
				else if(turning == 1 && (cmd == 'd' || cmd == 'f')) // right
				{
					pwmValB = GREATER_TURN_PWM;
					pwmValA = LESSER_TURN_PWM;
				}
				break;

			case 0:
				// send completed cmd back to RPi, ready to execute new command
				SetPinStop();
				osDelay(1000);
				SendFeedBack(1);
				ResetValues();
				dir = -2; // should go to default
				break;

			case -1:
//				__HAL_TIM_SET_COUNTER(&htim2, 0);
//				__HAL_TIM_SET_COUNTER(&htim5, 0);
				SetPinReverse();
				if(turning == 0)
					htim1.Instance -> CCR4 = ServoCorrection();

				else if(turning == 1 && (cmd == 'c' || cmd == 'e'))// left
				{
					pwmValA = GREATER_TURN_PWM;
					pwmValB = LESSER_TURN_PWM;
				}
				else if(turning == 1 && (cmd == 'd' || cmd == 'f'))// right
				{
					pwmValB = GREATER_TURN_PWM;
					pwmValA = LESSER_TURN_PWM;
				}
				break;
			default:
				SetPinStop();
		}
		// Reverse if sensor detects car is <= 100mm (10cm)
//		if(cmd == 'a' && right_sensor_int <= 10){
////			cmd = '-';
////			sprintf(oledRow4, "Obstacle!");
////			ResetValues();
////			TurnStraighten();
////			targetCount = ((190 - 30)/207.345) * 1560;
////			turning = 0;
////			dir = -1;
////			cmd = '-';
////			continue;
//		}
	    if(turning == 1)
	    {
		   if((cmd == 'c' || cmd == 'f') && totalAngle >= targetAngle - angleOffset)
		   {
			   SetPinStop();
			   turning = 0;
			   osDelay(500);
			   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			   ResetValues();
			   osDelay(500);
			   targetCount = ((40 - 35)/207.345) * 1560;
			   dir = -1;
		   }
		   else if((cmd == 'd' || cmd == 'e') && totalAngle <= targetAngle + angleOffset)
		   {
			   SetPinStop();
			   turning = 0;
			   osDelay(500);
			   htim1.Instance -> CCR4 = SERVO_STRAIGHT;
			   ResetValues();
			   osDelay(500);
			   targetCount = ((40 - 35)/207.345) * 1560;
			   dir = -1;
		   }
	    }
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, pwmValB);
		__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, pwmValA);
		osDelay(1);
	}
  /* USER CODE END DCMotor_task */
}

/* USER CODE BEGIN Header_RefreshOLED_task */
/**
* @brief Function implementing the RefreshOLED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RefreshOLED_task */
void RefreshOLED_task(void *argument)
{
  /* USER CODE BEGIN RefreshOLED_task */
  /* Infinite loop */
  for(;;)
  {
//	OLED_ShowString(0,0,oledRow0);
	OLED_ShowString(0,10,oledRow1);
	OLED_ShowString(0,20,oledRow2);
	OLED_ShowString(0,30,oledRow3);
	OLED_ShowString(0,40,oledRow4);
	OLED_ShowString(0,50,oledRow5);
	OLED_Refresh_Gram();
	osDelay(10);
  }
  /* USER CODE END RefreshOLED_task */
}

/* USER CODE BEGIN Header_EncoderMotorA_task */
/**
* @brief Function implementing the EncoderMotorA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderMotorA_task */
void EncoderMotorA_task(void *argument)
{
  /* USER CODE BEGIN EncoderMotorA_task */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	int cnt1, cnt2;
	uint32_t tick;
	cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
	tick = HAL_GetTick();

  /* Infinite loop */
  for(;;)
  {
	  // Get the difference between the cnt1 and cnt2 channels every 1000 ticks
	  if(HAL_GetTick() - tick > 1000L)
	  {
		  cnt2 = __HAL_TIM_GET_COUNTER(&htim2);
		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2))
		  {
			  if(cnt2 < cnt1)
				  diffA = cnt1 - cnt2;
			  else
				  diffA = (65535 - cnt2) + cnt1;
		  }
		  else
		  {
			  if(cnt2 > cnt1)
				  diffA = cnt2 - cnt1;
			  else
				  diffA = (65535 - cnt1) + cnt2;
		  }
		  if(diffA == 65535)
			  diffA = 0;
		  cnt1 = __HAL_TIM_GET_COUNTER(&htim2);
		  tick = HAL_GetTick();
	  }
	  if(turning == 0 && (dir == 1 || dir == -1))
			pwmValA = CalcPIDA_Dist();
  }
  /* USER CODE END EncoderMotorA_task */
}

/* USER CODE BEGIN Header_EncoderMotorB_task */
/**
* @brief Function implementing the EncoderMotorB thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_EncoderMotorB_task */
void EncoderMotorB_task(void *argument)
{
  /* USER CODE BEGIN EncoderMotorB_task */
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
	int cnt1, cnt2;
	uint32_t tick;
	cnt1 = __HAL_TIM_GET_COUNTER(&htim5);
	tick = HAL_GetTick();
  /* Infinite loop */
  for(;;)
  {
	  if(HAL_GetTick() - tick > 1000L)
	  {
		  cnt2 = __HAL_TIM_GET_COUNTER(&htim5);
		  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim5))
		  {
			  if(cnt2 < cnt1)
				  diffB = cnt1 - cnt2;
			  else
				  diffB = (65535 - cnt2) + cnt1;
		  }
		  else
		  {
			  if(cnt2 > cnt1)
				  diffB = cnt2 - cnt1;
			  else
				  diffB = (65535 - cnt1) + cnt2;
		  }
		  if(diffB == 65535)
			  diffB = 0;
		  cnt1 = __HAL_TIM_GET_COUNTER(&htim5);
		  tick = HAL_GetTick();
	  }
	  if(turning == 0 && (dir == 1 || dir == -1))
			pwmValB = CalcPIDB_Dist();
  }
  /* USER CODE END EncoderMotorB_task */
}

/* USER CODE BEGIN Header_UART_Command_RX_task */
/**
* @brief Function implementing the UART_Command_RX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Command_RX_task */
void UART_Command_RX_task(void *argument)
{
  /* USER CODE BEGIN UART_Command_RX_task */
  /* Infinite loop */
  for(;;)
  {
	  /* Do a switch case here to inteprete the different commands
	   * Idea is to associate different alphabets to different sets of instructions
	   * i.e: 'a' = go straight
	   * 	  'b' = reverse,
	   * 	  'c' = turn fully left
	   * 	  etc...
	   * 	  split the operational range of the servo motor into equal intervals
	   * 	  and associate it (and other actions) with a particular alphabet.*/

	  if(received == 1)
	  {
		  received = 0;
		  switch(cmd)
		  {
			  case 'a': // move Forward
				  TurnStraighten();
				  osDelay(100);
				  turning = 0;
				  dir = 1;
				  break;
			  case 'b': // move Backward
				  TurnStraighten();
				  osDelay(100);
				  turning = 0;
				  dir = -1;
				  break;
			  case 'c': // turn Forward Left
				  TurnFullLeft();
				  osDelay(100);
				  turning = 1;
				  dir = 1;
				  break;
			  case 'd': // turn Forward Right
				  TurnFullRight();
				  osDelay(100);
				  turning = 1;
				  dir = 1;
				  break;
			  case 'e': // turn Backward Left
				  TurnFullLeft();
				  osDelay(100);
				  turning = 1;
				  dir = -1;
				  break;
			  case 'f': // turn Backward Right
				  TurnFullRight();
				  osDelay(100);
				  turning = 1;
				  dir = -1;
				  break;
			  case 'g':
				  TurnStraighten();
				  dir = 0;
				  break;
			  default:
				  cmd = '-';
				  osDelay(10);
		  }
	  }
//	  sprintf(oledRow5, "cnt: %5d\0", (long)targetCount);
//	  sprintf(oledRow5, "cmd: %s", cmd);
	  sprintf(oledRow5, "cmd: %s\0", aRxBuffer);
//	  HAL_Delay(500);
  }
  /* USER CODE END UART_Command_RX_task */
}

/* USER CODE BEGIN Header_Gyro */
/**
* @brief Function implementing the Gyrohandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Gyro */
void Gyro(void *argument)
{
  /* USER CODE BEGIN Gyro */
	 uint8_t val[2] = {0,0};
	 uint32_t currTick;
	 int16_t offset = -2;
	 gyroInit();
	 prevTick = HAL_GetTick();
	   while (1)
	   {
		   currTick = HAL_GetTick();
		   if(currTick - prevTick >= 100L)
		   {
			   readByte(0x37, val);
			   angularSpeed = (val[0] << 8) | val[1];

			   if((angularSpeed >= -4 && angularSpeed <= 4) || (dir == 0 || dir == -2))
				   totalAngle += 0;
			   else
				   totalAngle += (double)(angularSpeed + offset)*((currTick - prevTick)/16400.0);

			   if(totalAngle >= 720)
			       totalAngle = 0;
			   else if(totalAngle <= -720)
				   totalAngle = 0;
			   prevTick = HAL_GetTick();
		   }
	   }
  /* USER CODE END Gyro */
}

/* USER CODE BEGIN Header_IRsensor */
/**
* @brief Function implementing the IRsensortask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_IRsensor */
void IRsensor(void *argument)
{
  /* USER CODE BEGIN IRsensor */

//	char buffer[100];
  /* Infinite loop */
  for(;;)
  {
	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_Start(&hadc3);
	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  HAL_ADC_PollForConversion(&hadc3,HAL_MAX_DELAY);
	  left_adc = HAL_ADC_GetValue(&hadc2);
	  right_adc = HAL_ADC_GetValue(&hadc3);

	  LPF_SUM_right = LPF_SUM_right+right_adc;
	  LPF_SUM_left = LPF_SUM_left+left_adc;
	  counter++;
	  if(counter>=100)
	  {
	  	right_sensor = LPF_SUM_right/counter;
	  	left_sensor = LPF_SUM_left/counter;

	  	right_sensor_int = (0.0000074673 *pow(right_sensor,2))-(0.042958* right_sensor)+70.9308;
	  	left_sensor_int = (0.0000074673 *pow(left_sensor,2))-(0.042958* left_sensor)+70.9308;

	  	LPF_SUM_right = 0;
	  	LPF_SUM_left = 0;
	  	counter = 0;
	  }
	  osDelay(1);
  }
  /* USER CODE END IRsensor */
}

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
