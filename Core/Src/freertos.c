/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bmp280.h"                 //Own library
#include "i2c.h"                    //HAL library (not included by CubeMX)
#include "printf.h"                 //include own library dedicated to embedded
#include "usart.h"                  //HAL library (not included by CubeMX)
#include "SSD1306_OLED.h"           //library for OLED
#include "GFX_BW.h"                 //library for OLED
#include "fonts/fonts.h"            //library for OLED
#include "arm_math.h"               //math library
#include "tim.h"                    //HAL library (not included by CubeMX)
#include "adc.h"                    //HAL library (not included by CubeMX)
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FFT_SAMPLES 1024
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
typedef struct
{
	float Pressure;
	float Temperature;
} BmpData_t;

typedef struct
{
	uint8_t OutFreqArray[26];
} FftData_t;

/* USER CODE END Variables */
/* Definitions for HeartbeatTask */
osThreadId_t HeartbeatTaskHandle;
const osThreadAttr_t HeartbeatTask_attributes = {
  .name = "HeartbeatTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for Bmp280Task */
osThreadId_t Bmp280TaskHandle;
const osThreadAttr_t Bmp280Task_attributes = {
  .name = "Bmp280Task",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 256 * 4
};
/* Definitions for OledTask */
osThreadId_t OledTaskHandle;
const osThreadAttr_t OledTask_attributes = {
  .name = "OledTask",
  .priority = (osPriority_t) osPriorityBelowNormal,
  .stack_size = 256 * 4
};
/* Definitions for FFTTask */
osThreadId_t FFTTaskHandle;
const osThreadAttr_t FFTTask_attributes = {
  .name = "FFTTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for QueueBmpData */
osMessageQueueId_t QueueBmpDataHandle;
const osMessageQueueAttr_t QueueBmpData_attributes = {
  .name = "QueueBmpData"
};
/* Definitions for QueueFftData */
osMessageQueueId_t QueueFftDataHandle;
const osMessageQueueAttr_t QueueFftData_attributes = {
  .name = "QueueFftData"
};
/* Definitions for TimerBmpData */
osTimerId_t TimerBmpDataHandle;
const osTimerAttr_t TimerBmpData_attributes = {
  .name = "TimerBmpData"
};
/* Definitions for MutexPrintf */
osMutexId_t MutexPrintfHandle;
const osMutexAttr_t MutexPrintf_attributes = {
  .name = "MutexPrintf"
};
/* Definitions for MutexI2C1 */
osMutexId_t MutexI2C1Handle;
const osMutexAttr_t MutexI2C1_attributes = {
  .name = "MutexI2C1"
};
/* Definitions for MutexBmpData */
osMutexId_t MutexBmpDataHandle;
const osMutexAttr_t MutexBmpData_attributes = {
  .name = "MutexBmpData"
};
/* Definitions for SemaphoreBmpQueue */
osSemaphoreId_t SemaphoreBmpQueueHandle;
const osSemaphoreAttr_t SemaphoreBmpQueue_attributes = {
  .name = "SemaphoreBmpQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void _putchar(char character)
{
  // embedded function for printf() for send char to console etc.
	osMutexAcquire(MutexPrintfHandle, osWaitForever); //encapsulated by a mutex
	HAL_UART_Transmit(&huart2, (uint8_t*)&character, 1, 1000);
	osMutexRelease(MutexPrintfHandle);
}

float complexABS(float real, float compl) {
	return sqrtf(real*real+compl*compl);
}
/* USER CODE END FunctionPrototypes */

void StartHeartbeatTask(void *argument);
void StartBmp280Task(void *argument);
void StartOledTask(void *argument);
void StartFFTTask(void *argument);
void TimerBmpDataCallback(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName);
void vApplicationMallocFailedHook(void);

/* USER CODE BEGIN 4 */
void vApplicationStackOverflowHook(xTaskHandle xTask, signed char *pcTaskName)
{
   /* Run time stack overflow checking is performed if
   configCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2. This hook function is
   called if a stack overflow is detected. */
	while(1){}
}
/* USER CODE END 4 */

/* USER CODE BEGIN 5 */
void vApplicationMallocFailedHook(void)
{
   /* vApplicationMallocFailedHook() will only be called if
   configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h. It is a hook
   function that will get called if a call to pvPortMalloc() fails.
   pvPortMalloc() is called internally by the kernel whenever a task, queue,
   timer or semaphore is created. It is also called by various parts of the
   demo application. If heap_1.c or heap_2.c are used, then the size of the
   heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
   FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
   to query the size of free heap space that remains (although it does not
   provide information on how the remaining heap might be fragmented). */
	while(1){}
}
/* USER CODE END 5 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of MutexPrintf */
  MutexPrintfHandle = osMutexNew(&MutexPrintf_attributes);

  /* creation of MutexI2C1 */
  MutexI2C1Handle = osMutexNew(&MutexI2C1_attributes);

  /* creation of MutexBmpData */
  MutexBmpDataHandle = osMutexNew(&MutexBmpData_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of SemaphoreBmpQueue */
  SemaphoreBmpQueueHandle = osSemaphoreNew(1, 1, &SemaphoreBmpQueue_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* creation of TimerBmpData */
  TimerBmpDataHandle = osTimerNew(TimerBmpDataCallback, osTimerPeriodic, NULL, &TimerBmpData_attributes);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of QueueBmpData */
  QueueBmpDataHandle = osMessageQueueNew (8, sizeof(BmpData_t), &QueueBmpData_attributes);

  /* creation of QueueFftData */
  QueueFftDataHandle = osMessageQueueNew (8, sizeof(FftData_t), &QueueFftData_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of HeartbeatTask */
  HeartbeatTaskHandle = osThreadNew(StartHeartbeatTask, NULL, &HeartbeatTask_attributes);

  /* creation of Bmp280Task */
  Bmp280TaskHandle = osThreadNew(StartBmp280Task, NULL, &Bmp280Task_attributes);

  /* creation of OledTask */
  OledTaskHandle = osThreadNew(StartOledTask, NULL, &OledTask_attributes);

  /* creation of FFTTask */
  FFTTaskHandle = osThreadNew(StartFFTTask, NULL, &FFTTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartHeartbeatTask */
/**
  * @brief  Function implementing the HeartbeatTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartHeartbeatTask */
void StartHeartbeatTask(void *argument)
{
  /* USER CODE BEGIN StartHeartbeatTask */
  /* Infinite loop */
  //--------------GPIO--------------------
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  osDelay(500);
  }
  /* USER CODE END StartHeartbeatTask */
}

/* USER CODE BEGIN Header_StartBmp280Task */
/**
* @brief Function implementing the Bmp280Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBmp280Task */
void StartBmp280Task(void *argument)
{
  /* USER CODE BEGIN StartBmp280Task */
  //----------------I2C----------------
	BMP280_t Bmp280; //Handle for BMP280
	BmpData_t _BmpData; //For local variables for the task
	uint32_t DelayTick = osKernelGetTickCount(); //start softtimer

	osMutexAcquire(MutexI2C1Handle, osWaitForever); //create mutex with infinite timeout for blocking, because I2C is also used by OLED
	BMP280_Init(&Bmp280, &hi2c1, 0x76); //For initialization BMP280 need: handle, which I2C, address
	osMutexRelease(MutexI2C1Handle);

	osTimerStart(TimerBmpDataHandle, 100);
  /* Infinite loop */
  for(;;)
  {
	  osMutexAcquire(MutexI2C1Handle, osWaitForever);
	  BMP280_ReadPressureAndTemperature(&Bmp280, &_BmpData.Pressure, &_BmpData.Temperature); //For local variables for the task
	  osMutexRelease(MutexI2C1Handle);

	  if(osOK == osSemaphoreAcquire(SemaphoreBmpQueueHandle, 0)) //wait for semaphore
	  {
      //send to the queue if the semaphore was successfully occupied
		  osMessageQueuePut(QueueBmpDataHandle, &_BmpData, 0, osWaitForever); //put BMP280 data to a queue
	  }

	  printf("Temperature: %.2f, Pressure: %.2f\n\r", _BmpData.Temperature, _BmpData.Pressure);

	  DelayTick += 10; //read every 10 ms
	  osDelayUntil(DelayTick);
  }
  /* USER CODE END StartBmp280Task */
}

/* USER CODE BEGIN Header_StartOledTask */
/**
* @brief Function implementing the OledTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOledTask */
void StartOledTask(void *argument)
{
  /* USER CODE BEGIN StartOledTask */
	//Variables for sprintf
  char Message[32];

	BmpData_t _BmpData; //local structural variable
	FftData_t FftData;

	osMutexAcquire(MutexI2C1Handle, osWaitForever);
	SSD1306_Init(&hi2c1);
	osMutexRelease(MutexI2C1Handle);

	GFX_SetFont(font_8x5);

	SSD1306_Clear(BLACK); //the buffer goes to the main program stack, not to RTOS memory (mallock)

	SSD1306_Display(); // uses commands and data on I2C

  /* Infinite loop */
  for(;;)
  {
  //OLED operation test
	SSD1306_Clear(BLACK);

	osMessageQueueGet(QueueBmpDataHandle, &_BmpData, NULL, 0);

	osMessageQueueGet(QueueFftDataHandle, &FftData, NULL, 0);

	sprintf(Message, "Press [hPa]: %.2f", _BmpData.Pressure);
	GFX_DrawString(0, 10, Message, WHITE, 0);

	sprintf(Message, "Temp ['C]: %.2f", _BmpData.Temperature);
	GFX_DrawString(0, 20, Message, WHITE, 0);

	//---------------------ADC---------------------
	// FFT

	for(uint8_t i = 0; i < 26; i++) // Each frequency
	{
	  GFX_DrawFillRectangle(10+(i*4), 64-FftData.OutFreqArray[i], 3, FftData.OutFreqArray[i], WHITE);
	}

	SSD1306_Display();
  }
  /* USER CODE END StartOledTask */
}

/* USER CODE BEGIN Header_StartFFTTask */
/**
* @brief Function implementing the FFTTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFFTTask */
void StartFFTTask(void *argument)
{
  /* USER CODE BEGIN StartFFTTask */
	arm_rfft_fast_instance_f32 FFTHandler;
	FftData_t FftData;
	int FreqPoint = 0;
	int Offset = 65; // variable noise floor offset

	// FFT

	uint16_t *AdcMicrophone;
	float *FFTInBuffer;
	float *FFTOutBuffer;
	int *Freqs;

	AdcMicrophone = pvPortMalloc(FFT_SAMPLES * sizeof(uint16_t));
	FFTInBuffer = pvPortMalloc(FFT_SAMPLES * sizeof(float));
	FFTOutBuffer = pvPortMalloc(FFT_SAMPLES * sizeof(float));
	Freqs = pvPortMalloc(FFT_SAMPLES * sizeof(int));

	HAL_TIM_Base_Start(&htim2);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)AdcMicrophone, FFT_SAMPLES);

	arm_rfft_fast_init_f32(&FFTHandler, FFT_SAMPLES);

  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(0x01, osFlagsWaitAll, osWaitForever);

	  for(uint32_t i = 0; i < FFT_SAMPLES; i++)
	  {
		  FFTInBuffer[i] =  (float)AdcMicrophone[i];
	  }

	  arm_rfft_fast_f32(&FFTHandler, FFTInBuffer, FFTOutBuffer, 0);

		FreqPoint = 0;
		// calculate abs values and linear-to-dB
		for (int i = 0; i < FFT_SAMPLES; i = i+2)
		{
			Freqs[FreqPoint] = (int)(20*log10f(complexABS(FFTOutBuffer[i], FFTOutBuffer[i+1]))) - Offset;

			if(Freqs[FreqPoint] < 0)
			{
				Freqs[FreqPoint] = 0;
			}
			FreqPoint++;
		}

		FftData.OutFreqArray[0] = (uint8_t)Freqs[1]; // 22 Hz
		FftData.OutFreqArray[1] = (uint8_t)Freqs[2]; // 63 Hz
		FftData.OutFreqArray[2] = (uint8_t)Freqs[3]; // 125 Hz
		FftData.OutFreqArray[3] = (uint8_t)Freqs[4]; // 185 Hz
		FftData.OutFreqArray[4] = (uint8_t)Freqs[5]; // 225 Hz
		FftData.OutFreqArray[5] = (uint8_t)Freqs[6]; // 270 Hz
		FftData.OutFreqArray[6] = (uint8_t)Freqs[7]; // 315 Hz
		FftData.OutFreqArray[7] = (uint8_t)Freqs[8]; // 360 Hz
		FftData.OutFreqArray[8] = (uint8_t)Freqs[9]; // 400 Hz
		FftData.OutFreqArray[9] = (uint8_t)Freqs[10]; // 450 Hz
		FftData.OutFreqArray[10] = (uint8_t)Freqs[11]; // 485 Hz
		FftData.OutFreqArray[11] = (uint8_t)Freqs[12]; // 530 Hz
		FftData.OutFreqArray[12] = (uint8_t)Freqs[13]; // 575 Hz
		FftData.OutFreqArray[13] = (uint8_t)Freqs[14]; // 625 Hz
		FftData.OutFreqArray[14] = (uint8_t)Freqs[17]; // 750 Hz
		FftData.OutFreqArray[15] = (uint8_t)Freqs[18]; // 750 Hz
		FftData.OutFreqArray[16] = (uint8_t)Freqs[19]; // 840 Hz
		FftData.OutFreqArray[17] = (uint8_t)Freqs[20]; // 885 Hz
		FftData.OutFreqArray[18] = (uint8_t)Freqs[21]; // 930 Hz
		FftData.OutFreqArray[19] = (uint8_t)Freqs[22]; // 975 Hz
		FftData.OutFreqArray[20] = (uint8_t)Freqs[23]; // 1000 Hz
		FftData.OutFreqArray[21] = (uint8_t)Freqs[24]; // 1060 Hz
		FftData.OutFreqArray[22] = (uint8_t)Freqs[51]; // 2200 Hz
		FftData.OutFreqArray[23] = (uint8_t)Freqs[104]; // 4500 Hz
		FftData.OutFreqArray[24] = (uint8_t)Freqs[207]; // 9000 Hz
		FftData.OutFreqArray[25] = (uint8_t)Freqs[344]; // 15000 Hz


		osMessageQueuePut(QueueFftDataHandle, &FftData, 0, osWaitForever);
  }
  /* USER CODE END StartFFTTask */
}
//-----------------CALLBACKS---------------------
/* TimerBmpDataCallback function */
void TimerBmpDataCallback(void *argument)
{
  /* USER CODE BEGIN TimerBmpDataCallback */
	osSemaphoreRelease(SemaphoreBmpQueueHandle);
  /* USER CODE END TimerBmpDataCallback */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	if(hadc->Instance == ADC1)
	{
//		SamplesReady = 1;
		osThreadFlagsSet(FFTTaskHandle, 0x01); //a task notification (faster solution of binary semaphore) for set 1 when end coletion samples for FFT
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
