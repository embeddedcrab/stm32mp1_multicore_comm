/**
  ******************************************************************************
  * @file    FreeRTOS/FreeRTOS_ThreadCreation/Src/main.c
  * @author  MCD Application Team
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
*/


/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>

#include "main.h"
#include "cmsis_os.h"
#include "openamp.h"

#include "comm_types.h"


/* Private includes ----------------------------------------------------------*/


/* Private typedef -----------------------------------------------------------*/


/* Private define ------------------------------------------------------------*/


/* Private macro -------------------------------------------------------------*/
#define MAIN_LED_BLINKING_TIME_MS     500U // time
#define MAIN_LED                      LED7

#define RP_MESSAGE_DELAY              50U  // TODO: Make it 20 ms only after initial testing
#define RP_MESSAGE_INITIAL_DELAY      100U


/* Private variables ---------------------------------------------------------*/
osThreadId LedTask_Handle;

osThreadId openAMP_RxTask_Handle;

osThreadId openAMP_Ch0_Handle;
osThreadId OpenAMP_Ch1_Handle;

osSemaphoreId OpenAMPCh0ReceivedSignal;
osSemaphoreId OpenAMPCh1ReceivedSignal;

// Tx and Rx buffer variables for virtual UART 0 & UART 1
uint8_t VirtUart0ChannelBuffTx[sizeof(tCommShmStrData)];

uint8_t VirtUart1ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart1ChannelRxSize = 0;

// Flag will be set once we receive something on this channel
__IO FlagStatus virtUART0RxMsg = RESET;
__IO FlagStatus VirtUart1RxMsg = RESET;

// Virtual UART 0 using openAMP
VIRT_UART_HandleTypeDef virtUART0;
// Virtual UART 1 using openAMP
VIRT_UART_HandleTypeDef virtUART1;

IPCC_HandleTypeDef hipcc;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_IPCC_Init(void);
int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb);

// Callback handler for Virtual UART 1
void VirtUART0_Callback(VIRT_UART_HandleTypeDef *huart);
void VirtUART1_Callback(VIRT_UART_HandleTypeDef *huart);

// Led Blinking Thread
static void LED_Task(void const * argument);

// Receive data from Main Processor through openAMP Threads
static void OpenAMP_Rx_Task(void *argument);

// Send and Receive data to/from Main Processor respectively through openAMP Thread
static void OpenAMP_Ch0_Process(void *argument);
static void OpenAMP_Ch1_Process(void *argument);


// Update data to be sent to Remote Processor
static void UpdateMessage( tCommShmStrData *pMessage, 
          uint32_t length, uint32_t id, const char *pName );

static void SerializeMessage(
  const tCommShmStrData *pMessage, char * pOutMessage );

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* STM32MP1xx HAL library initialization:
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initialize the Systick. */
  HAL_Init();

  /* Configure the system clock */
  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    HAL_RCC_DeInit();
    SystemClock_Config();
  }

  /* HW semaphore Clock enable */
  __HAL_RCC_HSEM_CLK_ENABLE();

  /* Initialize LED */
  BSP_LED_Init(LED5);
  BSP_LED_Init(LED7);
  BSP_LED_Off(LED5);
  BSP_LED_Off(LED7);

  // Initialize all configured peripherals
  MX_IPCC_Init();

  /* OpenAmp initialisation ---------------------------------*/
  MX_OPENAMP_Init(RPMSG_REMOTE, NULL);

  // virtUART0 and virtUART1 initializations and cb association
  if (VIRT_UART_Init(&virtUART0) != VIRT_UART_OK)
  {
    Error_Handler();
  }
  if(VIRT_UART_Init(&virtUART1) != VIRT_UART_OK)
  {
    Error_Handler();
  }
  // Register Callback for RPMSG1 to receive data or any response
  if(VIRT_UART_RegisterCallback(&virtUART0, VIRT_UART_RXCPLT_CB_ID, VirtUART0_Callback) != VIRT_UART_OK)
  {
    Error_Handler();
  }
  if(VIRT_UART_RegisterCallback(&virtUART1, VIRT_UART_RXCPLT_CB_ID, VirtUART1_Callback) != VIRT_UART_OK)
  {
    Error_Handler();
  }

  /* Initialize semaphore for communication b/w OpenAMP Rx Threads */
  OpenAMPCh0ReceivedSignal = osSemaphoreCreate (NULL, 1);
  if(OpenAMPCh0ReceivedSignal == NULL)
  {
	  Error_Handler();
  }


  OpenAMPCh1ReceivedSignal = osSemaphoreCreate (NULL, 1);
  if(OpenAMPCh1ReceivedSignal == NULL)
  {
	  Error_Handler();
  }
  
  /* Create the thread(s) */
  osThreadDef(LedThread, LED_Task, osPriorityNormal, 0, 128);
  LedTask_Handle = osThreadCreate(osThread(LedThread), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(OpenAMP_Rx, OpenAMP_Rx_Task, osPriorityNormal, 0, 512);
  openAMP_RxTask_Handle = osThreadCreate(osThread(OpenAMP_Rx), NULL);

  osThreadDef(OpenAMP_RxP, OpenAMP_Ch0_Process, osPriorityNormal, 0, 512);
  openAMP_Ch0_Handle = osThreadCreate(osThread(OpenAMP_RxP), NULL);

  osThreadDef(OpenAMP_Tx, OpenAMP_Ch1_Process, osPriorityNormal, 0, 512);
  OpenAMP_Ch1_Handle = osThreadCreate(osThread(OpenAMP_Tx), NULL);

  // Check Thread (s) validity
  if( (NULL == LedTask_Handle) || (NULL == openAMP_RxTask_Handle)
    || (NULL == openAMP_Ch0_Handle) || (NULL == OpenAMP_Ch1_Handle))
  {
    Error_Handler();
  }

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  for( ; ; )
  {
    Error_Handler();
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /**Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_DIG;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;

  /**PLL1 Config
  */
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 81;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLFRACV = 0x800;
  RCC_OscInitStruct.PLL.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL2 Config
    */
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 66;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 0x1400;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL2.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL2.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL3 Config
    */
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSE;
  RCC_OscInitStruct.PLL3.PLLM = 2;
  RCC_OscInitStruct.PLL3.PLLN = 34;
  RCC_OscInitStruct.PLL3.PLLP = 2;
  RCC_OscInitStruct.PLL3.PLLQ = 17;
  RCC_OscInitStruct.PLL3.PLLR = 37;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 0x1A04;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL3.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL3.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

    /**PLL4 Config
    */
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSE;
  RCC_OscInitStruct.PLL4.PLLM = 4;
  RCC_OscInitStruct.PLL4.PLLN = 99;
  RCC_OscInitStruct.PLL4.PLLP = 6;
  RCC_OscInitStruct.PLL4.PLLQ = 8;
  RCC_OscInitStruct.PLL4.PLLR = 8;
  RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_0;
  RCC_OscInitStruct.PLL4.PLLFRACV = 0;
  RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
  RCC_OscInitStruct.PLL4.RPDFN_DIS = RCC_RPDFN_DIS_DISABLED;
  RCC_OscInitStruct.PLL4.TPDFN_DIS = RCC_TPDFN_DIS_DISABLED;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
  Error_Handler();
  }
  /**RCC Clock Config
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                |RCC_CLOCKTYPE_PCLK5|RCC_CLOCKTYPE_MPU;
  RCC_ClkInitStruct.MPUInit.MPU_Clock = RCC_MPUSOURCE_PLL1;
  RCC_ClkInitStruct.MPUInit.MPU_Div = RCC_MPU_DIV2;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
  Error_Handler();
  }

  /**Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(24);
}

/* IPCC init function */
static void MX_IPCC_Init(void)
{

  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  Led binking thread
  * @param  argument: Not Using
  * @retval None
  */
void LED_Task(void const * argument)
{
  (void) argument;
  /* Infinite loop */
  for (;;)
  {
    BSP_LED_Toggle(MAIN_LED);
    osDelay(MAIN_LED_BLINKING_TIME_MS);
  }
}

/**
  * @brief  Open AMP receiver Thread
  * @param  argument: Not Using
  * @retval None
  */
void OpenAMP_Rx_Task(void *argument)
{
  (void) argument;

  while (1)
  {
    // Check for messages on opanAMP
    OPENAMP_check_for_message();

    // Check message received flag set by callback function
    if (VirtUart1RxMsg == SET)
    {
      VirtUart1RxMsg = RESET;
      osSemaphoreRelease(OpenAMPCh1ReceivedSignal);
    }
    // yield to process other tasks
    taskYIELD();
  }
}

/**
 * OpenAMP_Ch0_Process
 * @brief  Transmits data to Remote Processor
 * @param  argument: Not Using
 * @retval None
 * 
 * Typical definition of function to send data, 
 * different data shall be added based on some flags and sent to RP
 */
void OpenAMP_Ch0_Process(void *argument)
{
  VIRT_UART_StatusTypeDef status_ = VIRT_UART_OK;
  // Local Variable to Thread
  const char msgName_[SizeOfName] = {"ADC_Channel"};
  tCommShmStrData message_;

  // Initialize message data
  memset(message_.name, '\0', SizeOfName);
  UpdateMessage( &message_, 0, 0, msgName_ );

  // Wait while dummy message signal does not reach here
  while( RESET == virtUART0RxMsg )
  {
    osDelay(RP_MESSAGE_INITIAL_DELAY);
  }

  // forever thread loop
  while (1)
  {
    // Serialize Message into particular format
    SerializeMessage( &message_, VirtUart0ChannelBuffTx );
    // Send Message to Remote Processor
    status_ = VIRT_UART_Transmit(&virtUART0, VirtUart0ChannelBuffTx, (uint16_t)sizeof(tCommShmStrData));
    if( VIRT_UART_OK == status_ )
    {
      BSP_LED_Toggle(LED5);
    }

    // Update Data
    UpdateMessage( &message_, ++(message_.length), ++(message_.id), NULL );

    // Sleep for next run
    osDelay(RP_MESSAGE_DELAY);
  }
}

/**
 * OpenAMP_Ch1_Process
 * @brief  Open AMP transmission Thread
 * @param  argument: Not Using
 * @retval None
 */
void OpenAMP_Ch1_Process(void *argument)
{
  (void) argument;
  osStatus status;

  // forever loop inside thread
  while (1)
  {
    status = osSemaphoreWait(OpenAMPCh1ReceivedSignal, osWaitForever);
    if (status == osOK)
    {
      // TODO: Do some processing instead of sending back response
      // Send back response to respective channel
      VIRT_UART_Transmit(&virtUART1, VirtUart1ChannelBuffRx, VirtUart1ChannelRxSize);
    }
  }
}


void VirtUART0_Callback(VIRT_UART_HandleTypeDef *huart)
{
  uint16_t length_ = huart->RxXferSize < MAX_BUFFER_SIZE ? huart->RxXferSize : MAX_BUFFER_SIZE - 1;

  // Discard dummy message from another Processor and 
  // set a flag to start another thread processing
  (void) length_;

  virtUART0RxMsg = SET;
}

/*
 * This function is called in the context of IdleTask inside the function
 * OPENAMP_check_for_message.
 * OpenAMP is not thread safe, so we can't release the semaphore here because
 * FreeRTOS is not able to manage context switching in this situation.
 */
void VirtUART1_Callback(VIRT_UART_HandleTypeDef *huart)
{
  // Read data received from Main Processor
  VirtUart1ChannelRxSize = huart->RxXferSize < MAX_BUFFER_SIZE ? huart->RxXferSize : MAX_BUFFER_SIZE - 1;
  memcpy(VirtUart1ChannelBuffRx, huart->pRxBuffPtr, VirtUart1ChannelRxSize);

  VirtUart1RxMsg = SET;
}


/**
 * UpdateMessage
 * @brief  Update Message to be sent to Remote Processor
 * @param  pMessage Pointer to Message type
 * @retval None
 */
void UpdateMessage( tCommShmStrData * pMessage, uint32_t length, uint32_t id, const char *pName )
{
  // Update Message attributes
  pMessage->length = length;
  pMessage->id = id;

  // Check for name, if available then update
  if( NULL == pName ){
    return;
  } else{
    size_t len_ = 0;
    len_ = strlen(pName);
    if( (len_ > 0) && (len_ < SizeOfName) ){
      // Update Message
      strncpy(pMessage->name, pName, len_);
      *(pMessage->name + len_) = '\0';
    }
  }
}


void SerializeMessage( const tCommShmStrData * pMessage, char * pOutMessage )
{
  if( (NULL == pMessage->name) || (NULL == pMessage) || (NULL == pOutMessage) ){
    return;
  }

  // Order of message is: length,id,name
  snprintf(pOutMessage, sizeof(tCommShmStrData), "%u,%u,%s", 
            (unsigned)pMessage->length, (unsigned)pMessage->id, pMessage->name);
}

/* Public user code ---------------------------------------------------------*/

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM2 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2) {
    HAL_IncTick();
  }
}

void CoproSync_ShutdownCb(IPCC_HandleTypeDef * hipcc, uint32_t ChannelIndex, IPCC_CHANNELDirTypeDef ChannelDir)
{
  /* Deinitializes all peripherals */
  HAL_DeInit();

  /* Turn off LED7 */
  BSP_LED_Off(LED7);
  BSP_LED_Off(LED5);

  /* When ready, notify the remote processor that we can be shut down */
  HAL_IPCC_NotifyCPU(hipcc, ChannelIndex, IPCC_CHANNEL_DIR_RX);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  // Switch On the main led and halt because of error
  BSP_LED_On(LED7);
  while(1);
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
