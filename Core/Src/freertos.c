/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ServoModule.h"
#include "ImageModule.h"
#include "RangefinderModule.h"
#include "Focus.h"
#include "tcpecho.h"
#include "cmdPC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t rxSize = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
BaseType_t taskTurnBegin = pdFALSE;
BaseType_t taskCuBegin = pdFALSE;
BaseType_t taskJingBegin = pdFALSE;
BaseType_t taskFocusBegin = pdFALSE;
BaseType_t taskRangeBegin = pdFALSE;

uint8_t InitFlag = 1;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
__attribute__((section("._Text_Area"))) uint8_t rxdata_servo[128];
__attribute__((section("._Text_Area"))) uint8_t rxbuff_servo[128];
__attribute__((section("._Text_Area"))) char msg[256];
__attribute__((section("._Text_Area"))) uint8_t rxdata_cu[128];
__attribute__((section("._Text_Area"))) uint8_t rxbuff_cu[128];
__attribute__((section("._Text_Area"))) uint8_t rxdata_jing[128];
__attribute__((section("._Text_Area"))) uint8_t rxbuff_jing[128];
__attribute__((section("._Text_Area"))) uint8_t rxdata_focus[11];
__attribute__((section("._Text_Area"))) uint8_t rxbuff_focus[11];
__attribute__((section("._Text_Area"))) uint8_t rxdata_rangefinder[128];
__attribute__((section("._Text_Area"))) uint8_t rxbuff_rangefinder[128];

__attribute__((section("._Text_Area"))) uint8_t chcmd[64];
// uint8_t uartbuff[128];

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId turnHandle;
osThreadId imgcuHandle;
osThreadId imgjingHandle;
osThreadId focusHandle;
osThreadId rangefinderHandle;
osSemaphoreId BinaryServoHandle;
osSemaphoreId BinaryCuHandle;
osSemaphoreId BinaryJingHandle;
osSemaphoreId BinaryFocusHandle;
osSemaphoreId BinaryRangefinderHandle;

osThreadId mytaskHandle; // 自己添加的任务句柄

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void TurnTask(void const * argument);
void ImgCuTask(void const * argument);
void ImgJingTask(void const * argument);
void FocusTask(void const * argument);
void RangefinderTask(void const * argument);

void mytask(void const * argument); // 自己添加的任务

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of BinaryServo */
  osSemaphoreDef(BinaryServo);
  BinaryServoHandle = osSemaphoreCreate(osSemaphore(BinaryServo), 1);

  /* definition and creation of BinaryCu */
  osSemaphoreDef(BinaryCu);
  BinaryCuHandle = osSemaphoreCreate(osSemaphore(BinaryCu), 1);

  /* definition and creation of BinaryJing */
  osSemaphoreDef(BinaryJing);
  BinaryJingHandle = osSemaphoreCreate(osSemaphore(BinaryJing), 1);

  /* definition and creation of BinaryFocus */
  osSemaphoreDef(BinaryFocus);
  BinaryFocusHandle = osSemaphoreCreate(osSemaphore(BinaryFocus), 1);

  /* definition and creation of BinaryRangefinder */
  osSemaphoreDef(BinaryRangefinder);
  BinaryRangefinderHandle = osSemaphoreCreate(osSemaphore(BinaryRangefinder), 1);

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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of turn */
  osThreadDef(turn, TurnTask, osPriorityBelowNormal, 0, 256);
  turnHandle = osThreadCreate(osThread(turn), NULL);

  /* definition and creation of imgcu */
  osThreadDef(imgcu, ImgCuTask, osPriorityBelowNormal, 0, 256);
  imgcuHandle = osThreadCreate(osThread(imgcu), NULL);

  /* definition and creation of imgjing */
  osThreadDef(imgjing, ImgJingTask, osPriorityBelowNormal, 0, 256);
  imgjingHandle = osThreadCreate(osThread(imgjing), NULL);

  /* definition and creation of focus */
  osThreadDef(focus, FocusTask, osPriorityBelowNormal, 0, 256);
  focusHandle = osThreadCreate(osThread(focus), NULL);

  /* definition and creation of rangefinder */
  osThreadDef(rangefinder, RangefinderTask, osPriorityBelowNormal, 0, 256);
  rangefinderHandle = osThreadCreate(osThread(rangefinder), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	osThreadDef(mytask, mytask, osPriorityNormal, 0, 256);
	mytaskHandle = osThreadCreate(osThread(mytask), NULL);





  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for LWIP */
  memset(ImgSendTypedef_CU.Data,0xff,64);
  memset(ImgSendTypedef_JING.Data,0xff,64);
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
//  client_init();
  udpecho_init();

	HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rxbuff_servo, 128);
	__HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxbuff_cu, 128);
	__HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rxbuff_jing, 128);
	__HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rxbuff_focus, 11);
	__HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, rxbuff_rangefinder, 128);
	__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)chcmd, 64);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);

	HAL_GPIO_WritePin(RF_PWD_GPIO_Port,RF_PWD_Pin,GPIO_PIN_SET);
	// go2init();
	osDelay(1000);
	RangefinderDataFrameSend.FuncCode = SetMultiFreq;
	cSetMultiFreq = 0x0A;
	Send2RangefinderModule(&RangefinderDataFrameSend);
	osDelay(100);
	RangefinderDataFrameSend.FuncCode = SetTarget;
	cSetTarget = 0x01;
	Send2RangefinderModule(&RangefinderDataFrameSend);
//	osDelay(2000);
	vTaskSuspend(mytaskHandle);

  /* Infinite loop */
//	CommandTypedef.turnYaw.f = 30.f;
//	CommandTypedef.turnPitch.f = 60.f;
//	processControl();

  /* Infinite loop */
  for(;;)
  {
//	   if(InitFlag == 1)
	//   {
	// 	  	  state = STATE_GUIDE;
	// 		CommandTypedef.turnYaw.f = 30.f;
	// 		CommandTypedef.turnPitch.f = 60.f;
	// 		processControl();
	// 		InitFlag = 0;
	//   }

//	 if((taskCuBegin == pdTRUE) && (taskJingBegin == pdTRUE) && (InitFlag == 1))
	//  if((taskJingBegin == pdTRUE) && (InitFlag == 1))
	//  {
	//  	go2init();
	//  	InitFlag = 0;
	//  }else if(InitFlag == 0)
	//  {
		// uart2_printf("state:%d\n",state);
		// state = STATE_GUIDE;
		// HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_SET);
//	  if(InitFlag == 1)
//	  {
//		  go2init();
//		  InitFlag = 0;
//	  }
		processControl();
//	  Send2ImgModule(&ImgSendTypedef_CU,&ImgSendDataTypedef_CU);
	//  }

//	state = STATE_GUIDE;
//	CommandTypedef.turnYaw.f = 30.f;
//	CommandTypedef.turnPitch.f = 60.f;
//	processControl();
    osDelay(20);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_TurnTask */
/**
* @brief Function implementing the turn thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_TurnTask */
void TurnTask(void const * argument)
{
  /* USER CODE BEGIN TurnTask */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(BinaryServoHandle, 10) == osOK && taskTurnBegin == pdTRUE)
	  {
		 pcSend.cuErrorCode[0]             			  = rxdata_servo[3];
		 pcSend.cuErrorCode[1]             			  = rxdata_servo[4];
		 
		 TrackingFaultCodeCu.CommunicationFault       = (rxdata_servo[3] & bit(0)) ? 1 : 0;
		 TrackingFaultCodeCu.EncoderFault             = (rxdata_servo[3] & bit(1)) ? 1 : 0;
		 TrackingFaultCodeCu.DriverFaultA             = (rxdata_servo[3] & bit(2)) ? 1 : 0;
		 TrackingFaultCodeCu.DriverFaultE             = (rxdata_servo[3] & bit(3)) ? 1 : 0;
		 TrackingFaultCodeCu.OverSpeedA               = (rxdata_servo[3] & bit(4)) ? 1 : 0;
		 TrackingFaultCodeCu.OverSpeedE               = (rxdata_servo[3] & bit(5)) ? 1 : 0;
		 TrackingFaultCodeCu.ControlOverLimitA        = (rxdata_servo[3] & bit(6)) ? 1 : 0;
		 TrackingFaultCodeCu.ControlOverLimitE        = (rxdata_servo[3] & bit(7)) ? 1 : 0;
		 TrackingFaultCodeCu.PositionOverLimitE       = (rxdata_servo[4] & bit(0)) ? 1 : 0;
		 TrackingFaultCodeCu.PositionUnderLimitE      = (rxdata_servo[4] & bit(1)) ? 1 : 0;
		 TrackingFaultCodeCu.TrackingFaultCuByte1     = (rxdata_servo[4] & bit(2)) ? 1 : 0;
		 TrackingFaultCodeCu.TrackingFaultCuByte2     = (rxdata_servo[4] & bit(3)) ? 1 : 0;
		 TrackingFaultCodeCu.TrackingFaultCuByte3     = (rxdata_servo[4] & bit(4)) ? 1 : 0;
		 TrackingFaultCodeCu.TrackingFaultCuByte4     = (rxdata_servo[4] & bit(5)) ? 1 : 0;


		 pcSend.jingErrorCode[0]                      = rxdata_servo[5];
		 pcSend.jingErrorCode[1]                      = rxdata_servo[6];
		 TrackingFaultCodeJing.FSMControlOverLimitX   = (rxdata_servo[5] & bit(0)) ? 1 : 0;
		 TrackingFaultCodeJing.FSMControlOverLimitY   = (rxdata_servo[5] & bit(1)) ? 1 : 0;
		 TrackingFaultCodeJing.FSMPositionOverLimitX  = (rxdata_servo[6] & bit(0)) ? 1 : 0;
		 TrackingFaultCodeJing.FSMPositionOverLimitY  = (rxdata_servo[6] & bit(1)) ? 1 : 0;
		 TrackingFaultCodeJing.TrackingFaultJing      = (rxdata_servo[6] & bit(2)) ? 1 : 0;


		 ServoRevTypedef.cCuOffsetState               =  rxdata_servo[17];
		 pcSend.cuOffsetState = ServoRevTypedef.cCuOffsetState;
		 ServoRevTypedef.CuOffset_X.s                 =  rxdata_servo[18] << 8 | rxdata_servo[19];
		 ServoRevTypedef.CuOffset_Y.s                 =  rxdata_servo[20] << 8 | rxdata_servo[21];
		 ServoRevTypedef.cJingOffsetState             =  rxdata_servo[22];
		 pcSend.jingOffsetState = ServoRevTypedef.cJingOffsetState;
		 ServoRevTypedef.cJingISOffsetState           =  rxdata_servo[23];
		 ServoRevTypedef.JingOffset_X.s               =  rxdata_servo[24] << 8 | rxdata_servo[25];
		 ServoRevTypedef.JingOffset_Y.s               =  rxdata_servo[26] << 8 | rxdata_servo[27];
		 ServoRevTypedef.cTurnTableTrackState         =  rxdata_servo[28];
		 ServoRevTypedef.cFSMTrackState               =  rxdata_servo[29];
		 // ServoRevTypedef.cElectPositionState          =  rxdata_servo[30];
		 ServoRevTypedef.cTurnTableMotorPowerState    =  rxdata_servo[31];
		 ServoRevTypedef.cJingTrackControlState       =  rxdata_servo[32];
		 for(int i = 0;i < 4;i++)
		 {
		   #if TOP
		   ServoRevTypedef.FSMXPos.u8t[3-i] = rxdata_servo[33+i];
		   ServoRevTypedef.FSMYPos.u8t[3-i] = rxdata_servo[37+i];
		   ServoRevTypedef.ServoYawPos.u8t[3-i] = rxdata_servo[41+i];
		   ServoRevTypedef.ServoPitchPos.u8t[3-i] = rxdata_servo[45+i];
		   ServoRevTypedef.YawSpeed.u8t[3-i] = rxdata_servo[49+i];
		   ServoRevTypedef.PitchSpeed.u8t[3-i] = rxdata_servo[53+i];
		   ServoRevTypedef.FSMXPosZero.u8t[3-i] = rxdata_servo[57+i];
		   ServoRevTypedef.FSMYPosZero.u8t[3-i] = rxdata_servo[61+i];
		   ServoRevTypedef.TurnYawOutput.u8t[3-i] = rxdata_servo[65+i];
		   ServoRevTypedef.TurnPitchOutput.u8t[3-i] = rxdata_servo[69+i];
		   ServoRevTypedef.FSMXOutput.u8t[3-i] = rxdata_servo[73+i];
		   ServoRevTypedef.FSMYOutput.u8t[3-i] = rxdata_servo[77+i];
		   #else
		   ServoRevTypedef.FSMXPos.u8t[i] = rxdata_servo[33+i];
		   ServoRevTypedef.FSMYPos.u8t[i] = rxdata_servo[37+i];
		   ServoRevTypedef.ServoYawPos.u8t[i] = rxdata_servo[41+i];
		   ServoRevTypedef.ServoPitchPos.u8t[i] = rxdata_servo[45+i];
		   ServoRevTypedef.YawSpeed.u8t[i] = rxdata_servo[49+i];
		   ServoRevTypedef.PitchSpeed.u8t[i] = rxdata_servo[53+i];
		   ServoRevTypedef.FSMXPosZero.u8t[i] = rxdata_servo[57+i];
		   ServoRevTypedef.FSMYPosZero.u8t[i] = rxdata_servo[61+i];
		   ServoRevTypedef.TurnYawOutput.u8t[i] = rxdata_servo[65+i];
		   ServoRevTypedef.TurnPitchOutput.u8t[i] = rxdata_servo[69+i];
		   ServoRevTypedef.FSMXOutput.u8t[i] = rxdata_servo[73+i];
		   ServoRevTypedef.FSMYOutput.u8t[i] = rxdata_servo[77+i];
		   #endif
		 }

		 ServoRevTypedef.ServoCrc                     =  rxdata_servo[81];
		 ServoRevTypedef.ServoCheck                   =  rxdata_servo[82];

		//  uart_printf("FSMXPos:%.2f,FSMYPos:%.2f,ServoYawPos:%.2f,ServoPitchPos:%.2f,YawSpeed:%.2f,PitchSpeed:%.2f,FSMXPosZero:%.2f,FSMYPosZero:%.2f,TurnYawOutput:%.2f,TurnPitchOutput:%.2f,FSMXOutput:%.2f,FSMYOutput:%.2f\r\n",ServoRevTypedef.FSMXPos.f,ServoRevTypedef.FSMYPos.f,ServoRevTypedef.ServoYawPos.f,ServoRevTypedef.ServoPitchPos.f,ServoRevTypedef.YawSpeed.f,ServoRevTypedef.PitchSpeed.f,ServoRevTypedef.FSMXPosZero.f,ServoRevTypedef.FSMYPosZero.f,ServoRevTypedef.TurnYawOutput.f,ServoRevTypedef.TurnPitchOutput.f,ServoRevTypedef.FSMXOutput.f,ServoRevTypedef.FSMYOutput.f);
		        // for(int i = 0;i < rxSize;i++)
		        // {
		        //   uart_printf("%02x ",rxdata_servo[i]);
		        // }
//		         打印数据

		//  sprintf(msg,"41:%02x,42:%02x,43:%02x,44:%02x,ServoPitchPos = %.2f, ServoYawPos= %.2f\r\n",rxdata_servo[41],rxdata_servo[42],rxdata_servo[43],rxdata_servo[44],ServoRevTypedef.ServoPitchPos.f, ServoRevTypedef.ServoYawPos.f);
		// 打印错误代码
		// uart_printf()
		// sprintf(msg,"CommunicationFault:%d,EncoderFault:%d,DriverFaultA:%d,DriverFaultE:%d,OverSpeedA:%d,OverSpeedE:%d,ControlOverLimitA:%d,ControlOverLimitE:%d,PositionOverLimitE:%d,PositionUnderLimitE:%d,TrackingFaultCuByte1:%d,TrackingFaultCuByte2:%d,TrackingFaultCuByte3:%d,TrackingFaultCuByte4:%d,yaw:%.2f,pitch:%.2f\r\n",TrackingFaultCodeCu.CommunicationFault,TrackingFaultCodeCu.EncoderFault,TrackingFaultCodeCu.DriverFaultA,TrackingFaultCodeCu.DriverFaultE,TrackingFaultCodeCu.OverSpeedA,TrackingFaultCodeCu.OverSpeedE,TrackingFaultCodeCu.ControlOverLimitA,TrackingFaultCodeCu.ControlOverLimitE,TrackingFaultCodeCu.PositionOverLimitE,TrackingFaultCodeCu.PositionUnderLimitE,TrackingFaultCodeCu.TrackingFaultCuByte1,TrackingFaultCodeCu.TrackingFaultCuByte2,TrackingFaultCodeCu.TrackingFaultCuByte3,TrackingFaultCodeCu.TrackingFaultCuByte4, ServoRevTypedef.ServoYawPos.f,ServoRevTypedef.ServoPitchPos.f);	
		//  HAL_UART_Transmit_DMA(&huart5, (uint8_t*)msg, strlen(msg));
		// uart_printf("%s",msg);
// 
//		         uint8_t msg1[6] = {0xee,0x16,0x02,0x03,0x01,0x04};
//		         	HAL_UART_Transmit(&huart7, msg1, 6,0xff);
//		         	osDelay(200);
//		         	uint8_t msg3[] = {0xee, 0x16, 0x04, 0x03, 0xa1, 0x0a, 0x00, 0xae};
//		         	HAL_UART_Transmit(&huart7, msg3, 8,0xff);
//		         	osDelay(200);
//		         	uint8_t msg2[6] = {0xee,0x16,0x02,0x03,0x04,0x07};
//		         			         	HAL_UART_Transmit(&huart7, msg2, 6,0xff);
		         //	uint8_t msg[8] = {0xaa, 0x01, 0x08, 0x0f, 0x03, 0x00, 0x64, 0xcb};
//		         uint8_t msg3[7] = {0xaa,0x01,0x07,0x1b,0x03,0x00,0xb4};
//		         HAL_UART_Transmit(&huart6, msg3, 7,0xff);
//		         osDelay(200);
//		         uint8_t msg0[12] = {0xaa, 0x01, 0x0c, 0x01, 0x03, 0x00, 0x44, 0x7a, 0x00, 0x00, 0x00, 0x9b};
//		         		  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//		         		  HAL_UART_Transmit_DMA(&huart6, msg0, 12);
//		         		  osDelay(2000);
//		         	uint8_t msg1[12] = {0xaa, 0x01, 0x0c, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa5};
//		         	HAL_UART_Transmit_DMA(&huart6, msg1, 12);
//		         			  osDelay(2000);
//		         			  uint8_t msg2[12] = {0xaa, 0x01, 0x0c, 0x01, 0x03, 0x00, 0xc4, 0x7a, 0x00, 0x00, 0x00, 0x1b};
//		         			  	HAL_UART_Transmit_DMA(&huart6, msg2, 12);
//		         			  			  osDelay(3000);
//		         uint8_t msg4[8] = {0xaa, 0x01, 0x08, 0x0f, 0x03, 0x00, 0x32, 0x9d};
//		         HAL_UART_Transmit(&huart6, msg4, 8,0xff);

		        memset(rxdata_servo,0,sizeof(rxdata_servo));
//		  HAL_UART_Transmit_DMA(&huart5, rxdata_servo, rxSize);
	  }
    osDelay(20);
  }
  /* USER CODE END TurnTask */
}

/* USER CODE BEGIN Header_ImgCuTask */
/**
* @brief Function implementing the imgcu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImgCuTask */
void ImgCuTask(void const * argument)
{
  /* USER CODE BEGIN ImgCuTask */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(BinaryCuHandle, 10) == osOK && taskCuBegin == pdTRUE)
	  {
		ImgRecvDataTypedef_CU.code1.SelfCheck     = (rxdata_cu[3] & bit(0)) ? 1 : 0;
		ImgRecvDataTypedef_CU.code1.Standby       = (rxdata_cu[3] & bit(1)) ? 1 : 0;
		ImgRecvDataTypedef_CU.code1.AutoTrack     = (rxdata_cu[3] & bit(2)) ? 1 : 0;
		ImgRecvDataTypedef_CU.code1.ManualTrack   = (rxdata_cu[3] & bit(3)) ? 1 : 0;

		// uart2_printf("selfcheck:%d,standby:%d,autotrack:%d,manualtrack:%d\r\n",ImgRecvDataTypedef_CU.code1.SelfCheck,ImgRecvDataTypedef_CU.code1.Standby,ImgRecvDataTypedef_CU.code1.AutoTrack,ImgRecvDataTypedef_CU.code1.ManualTrack);
		ImgRecvDataTypedef_CU.code2.CheckState    = (rxdata_cu[4] & bit(7)) ? 1 : 0;
		ImgRecvDataTypedef_CU.code2.InitState     = (rxdata_cu[4] & bit(6)) ? 1 : 0;
		ImgRecvDataTypedef_CU.code2.NormalTrack   = (rxdata_cu[4] & bit(5)) ? 1 : 0;
		ImgRecvDataTypedef_CU.code2.RememberTrack = (rxdata_cu[4] & bit(4)) ? 1 : 0;
		ImgRecvDataTypedef_CU.code2.TrackLose     = (rxdata_cu[4] & bit(3)) ? 1 : 0;
		pcSend.cuEnable = ImgRecvDataTypedef_CU.code2.NormalTrack || ImgRecvDataTypedef_CU.code2.RememberTrack;
		
		// uart2_printf("checkstate:%d,initstate:%d,normaltrack:%d,remembertrack:%d,tracklose:%d\r\n",ImgRecvDataTypedef_CU.code2.CheckState,ImgRecvDataTypedef_CU.code2.InitState,ImgRecvDataTypedef_CU.code2.NormalTrack,ImgRecvDataTypedef_CU.code2.RememberTrack,ImgRecvDataTypedef_CU.code2.TrackLose);

		ImgRecvDataTypedef_CU.WorkState           = rxdata_cu[5];

		ImgRecvDataTypedef_CU.HardVersion         = (uint8_t)(rxdata_cu[8] & 00000111);
		ImgRecvDataTypedef_CU.SoftVersion         = (uint8_t)(rxdata_cu[8] & 01111000);

		ImgRecvDataTypedef_CU.ImgCount            = rxdata_cu[7] << 8 | rxdata_cu[6];
		ImgRecvDataTypedef_CU.DataCount           = rxdata_cu[10] << 8 | rxdata_cu[9];

		int buff[2] = {0,0};
		buff[0]							          = (rxdata_cu[15] << 28 | rxdata_cu[14] << 21 | rxdata_cu[13] << 14 | rxdata_cu[12] << 7 | rxdata_cu[11]);
		buff[1]            						  = (rxdata_cu[20] << 28 | rxdata_cu[19] << 21 | rxdata_cu[18] << 14 | rxdata_cu[17] << 7 | rxdata_cu[16]);
		ImgRecvDataTypedef_CU.Offset_X            = *((float *)&buff[0]);
		ImgRecvDataTypedef_CU.Offset_Y            = *((float *)&buff[1]);
		// uart_printf("11:%02x,12:%02x,13:%02x,14:%02x,15:%02x\r\n",rxdata_cu[11],rxdata_cu[12],rxdata_cu[13],rxdata_cu[14],rxdata_cu[15]);
		// uart_printf("16:%02x,17:%02x,18:%02x,19:%02x,20:%02x\r\n",rxdata_cu[16],rxdata_cu[17],rxdata_cu[18],rxdata_cu[19],rxdata_cu[20]);
		// uart_printf("x:%.2f,y:%.2f\r\n",ImgRecvDataTypedef_CU.Offset_X,ImgRecvDataTypedef_CU.Offset_Y);

		// uart_printf("")

		        // ImgRecvDataTypedef_CU.Offset_X            = (rxdata_cu[15] << 28 | rxdata_cu[14] << 21 | rxdata_cu[13] << 14 | rxdata_cu[12] << 7 | rxdata_cu[11]) / 1.f;
		        // ImgRecvDataTypedef_CU.Offset_Y            = (rxdata_cu[20] << 28 | rxdata_cu[19] << 21 | rxdata_cu[18] << 14 | rxdata_cu[17] << 7 | rxdata_cu[16]) / 1.f;

//		        char msg[128];
		        // sprintf(msg,"x:%.2f,y:%.2f",ImgRecvDataTypedef_CU.Offset_X,ImgRecvDataTypedef_CU.Offset_Y);
		        // HAL_UART_Transmit_DMA(&huart3, (uint8_t *)msg, strlen(msg));
		  memset(rxdata_cu,0,rxSize);
	  }
    osDelay(10);
  }
  /* USER CODE END ImgCuTask */
}

/* USER CODE BEGIN Header_ImgJingTask */
/**
* @brief Function implementing the imgjing thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ImgJingTask */
void ImgJingTask(void const * argument)
{
  /* USER CODE BEGIN ImgJingTask */
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(BinaryJingHandle, 10) == osOK && taskJingBegin == pdTRUE)
	  {
		  	ImgRecvDataTypedef_JING.code1.SelfCheck     = (rxdata_jing[3] & bit(0)) ? 1 : 0;
	        ImgRecvDataTypedef_JING.code1.Standby       = (rxdata_jing[3] & bit(1)) ? 1 : 0;
	        ImgRecvDataTypedef_JING.code1.AutoTrack     = (rxdata_jing[3] & bit(2)) ? 1 : 0;
	        ImgRecvDataTypedef_JING.code1.ManualTrack   = (rxdata_jing[3] & bit(3)) ? 1 : 0;

	        ImgRecvDataTypedef_JING.code2.CheckState    = (rxdata_jing[4] & bit(7)) ? 1 : 0;
	        ImgRecvDataTypedef_JING.code2.InitState     = (rxdata_jing[4] & bit(6)) ? 1 : 0;
	        ImgRecvDataTypedef_JING.code2.NormalTrack   = (rxdata_jing[4] & bit(5)) ? 1 : 0;
	        ImgRecvDataTypedef_JING.code2.RememberTrack = (rxdata_jing[4] & bit(4)) ? 1 : 0;
	        ImgRecvDataTypedef_JING.code2.TrackLose     = (rxdata_jing[4] & bit(3)) ? 1 : 0;
			pcSend.jingEnable = ImgRecvDataTypedef_JING.code2.NormalTrack || ImgRecvDataTypedef_JING.code2.RememberTrack;

	        ImgRecvDataTypedef_JING.WorkState           = rxdata_jing[5];

	        ImgRecvDataTypedef_JING.HardVersion         = (uint8_t)(rxdata_jing[8] & 00000111);
	        ImgRecvDataTypedef_JING.SoftVersion         = (uint8_t)(rxdata_jing[8] & 01111000);

	        ImgRecvDataTypedef_JING.ImgCount            = rxdata_jing[7] << 8 | rxdata_jing[6];
	        ImgRecvDataTypedef_JING.DataCount           = rxdata_jing[10] << 8 | rxdata_jing[9];
	        
			int buff[2] = {0,0};
			buff[0]							            = (rxdata_jing[15] << 28 | rxdata_jing[14] << 21 | rxdata_jing[13] << 14 | rxdata_jing[12] << 7 | rxdata_jing[11]);
	        buff[1]            							= (rxdata_jing[20] << 28 | rxdata_jing[19] << 21 | rxdata_jing[18] << 14 | rxdata_jing[17] << 7 | rxdata_jing[16]);
			ImgRecvDataTypedef_JING.Offset_X            = *((float *)&buff[0]);
	        ImgRecvDataTypedef_JING.Offset_Y            = *((float *)&buff[1]);

			
			// uart_printf("selfcheck:%d,standby:%d,autotrack:%d,manualtrack:%d\r\n",ImgRecvDataTypedef_JING.code1.SelfCheck,ImgRecvDataTypedef_JING.code1.Standby,ImgRecvDataTypedef_JING.code1.AutoTrack,ImgRecvDataTypedef_JING.code1.ManualTrack);
//			uart_printf("checkstate:%d,initstate:%d,normaltrack:%d,remembertrack:%d,tracklose:%d\r\n",ImgRecvDataTypedef_JING.code2.CheckState,ImgRecvDataTypedef_JING.code2.InitState,ImgRecvDataTypedef_JING.code2.NormalTrack,ImgRecvDataTypedef_JING.code2.RememberTrack,ImgRecvDataTypedef_JING.code2.TrackLose);
			// uart_printf("workstate:%d,hardversion:%d,softversion:%d,imgcount:%d,datacount:%d,offsetx:%.2f,offsety:%.2f\r\n",ImgRecvDataTypedef_JING.WorkState,ImgRecvDataTypedef_JING.HardVersion,ImgRecvDataTypedef_JING.SoftVersion,ImgRecvDataTypedef_JING.ImgCount,ImgRecvDataTypedef_JING.DataCount,ImgRecvDataTypedef_JING.Offset_X,ImgRecvDataTypedef_JING.Offset_Y);

//	        char msg[128];
	        // sprintf(msg,"x:%.2f,y:%.2f",ImgRecvDataTypedef_JING.Offset_X,ImgRecvDataTypedef_JING.Offset_Y);
//	         uart_printf("%s",msg);
	        memset(rxdata_cu,0,rxSize);
	  }
    osDelay(10);
  }
  /* USER CODE END ImgJingTask */
}

/* USER CODE BEGIN Header_FocusTask */
/**
* @brief Function implementing the focus thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FocusTask */
void FocusTask(void const * argument)
{
  /* USER CODE BEGIN FocusTask */

  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(BinaryFocusHandle, 10) == osOK && taskFocusBegin == pdTRUE)
	  {
		  getFocusPos.u8t[3] = rxdata_focus[6];
		  getFocusPos.u8t[2] = rxdata_focus[7];
		  getFocusPos.u8t[1] = rxdata_focus[8];
		  getFocusPos.u8t[0] = rxdata_focus[9];

		        char msg[128];
		        sprintf(msg,"%02x,%02x,%02x,%02x,pos:%f",rxdata_focus[6],rxdata_focus[7],rxdata_focus[8],rxdata_focus[9],getFocusPos.f);
		        // HAL_UART_Transmit(&huart5, (uint8_t *)msg, strlen(msg), 0xff);
				uart_printf("%s",msg);
		        memset(rxdata_focus,0,sizeof(rxdata_focus));
	  }

    osDelay(100);
  }
  /* USER CODE END FocusTask */
}

/* USER CODE BEGIN Header_RangefinderTask */
/**
* @brief Function implementing the rangefinder thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_RangefinderTask */
void RangefinderTask(void const * argument)
{
  /* USER CODE BEGIN RangefinderTask */
	// TODO:掉电�???????要重新设�???????
  /* Infinite loop */
  for(;;)
  {
	  if(osSemaphoreWait(BinaryRangefinderHandle, 10) == osOK && taskRangeBegin == pdTRUE)
	  {
		// 打印数据
		// for(int i = 0;i < rxSize;i++)
	  	// {
		// 	uart_printf("%02x ",rxdata_rangefinder[i]);
		// }

		  switch (rxdata_rangefinder[4])
		        {
		          case DeviceCheck:{
		            RangeFinderRevData.HuiBoValue = rxdata_rangefinder[6];
		            RangeFinderRevData.FPGAState  = rxdata_rangefinder[7] & bit(0) ? 1 : 0;
		            RangeFinderRevData.RayState   = rxdata_rangefinder[7] & bit(1) ? 1 : 0;
		            RangeFinderRevData.ZhuBoState = rxdata_rangefinder[7] & bit(2) ? 1 : 0;
		            RangeFinderRevData.HuiBoState = rxdata_rangefinder[7] & bit(3) ? 1 : 0;
		            RangeFinderRevData.TempState  = rxdata_rangefinder[7] & bit(6) ? 1 : 0;
		            RangeFinderRevData.RayOn      = rxdata_rangefinder[7] & bit(7) ? 1 : 0;
		            RangeFinderRevData.PowerState = rxdata_rangefinder[8] & bit(0) ? 1 : 0;
		          }break;
		          case SingleMeasure:{
		            RangeFinderRevData.Distance   = rxdata_rangefinder[6] * 256 + rxdata_rangefinder[7] + rxdata_rangefinder[8] * 0.1;

		          }break;
		          case MultiMeasure:{
		            RangeFinderRevData.Distance   = rxdata_rangefinder[6] * 256 + rxdata_rangefinder[7] + rxdata_rangefinder[8] * 0.1;
					pcSend.rangefinder.f = RangeFinderRevData.Distance;
				}break;
		          case DeviceError:{
		            RangeFinderRevData.FPGAState  = rxdata_rangefinder[8] & bit(0) ? 1 : 0;
		            RangeFinderRevData.RayState   = rxdata_rangefinder[8] & bit(1) ? 1 : 0;
		            RangeFinderRevData.ZhuBoState = rxdata_rangefinder[8] & bit(2) ? 1 : 0;
		            RangeFinderRevData.HuiBoState = rxdata_rangefinder[8] & bit(3) ? 1 : 0;
		            RangeFinderRevData.TempState  = rxdata_rangefinder[8] & bit(6) ? 1 : 0;
		            RangeFinderRevData.RayOn      = rxdata_rangefinder[8] & bit(7) ? 1 : 0;
		          }break;
		          default:
		            break;
		        }
				// uart_printf("distance:%.2f\r\n",RangeFinderRevData.Distance);
				uart_printf("%x,%x,%x,%.2f\n",rxdata_rangefinder[6],rxdata_rangefinder[7],rxdata_rangefinder[8],RangeFinderRevData.Distance);
		//   char msg[128];
		//         sprintf(msg,"distance = %.2f\r\n",RangeFinderRevData.Distance);
		//         HAL_UART_Transmit_DMA(&huart5, (uint8_t*)msg, strlen(msg));
		        memset(rxdata_rangefinder,0,sizeof(rxdata_rangefinder));
	  }
    osDelay(10);
  }
  /* USER CODE END RangefinderTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  UNUSED(Size);
  rxSize = Size;
//   static float fPm1, fPm2, fPm3;
// 	static int iPm1, iPm2, iPm3;
  if(huart == &huart5)
  {
	  taskTurnBegin = pdTRUE;
	  if(rxbuff_servo[0] == 0xEB && rxbuff_servo[1] == 0x90 && rxbuff_servo[Size - 1] == 0xFE)
	  {
		  if(osSemaphoreRelease(BinaryServoHandle) == osOK)
		  {
			  memcpy(rxdata_servo,rxbuff_servo,Size);
			  memset(rxbuff_servo,0,Size);
			  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  }
	  }
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rxbuff_servo, 128);
	  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);
  }else if(huart == &huart3){
	  taskCuBegin = pdTRUE;
	  if(rxbuff_cu[0] == 0xCC && rxbuff_cu[1] == 0xC0 && rxbuff_cu[Size - 1] == 0xBB)
	  {
		  if(osSemaphoreRelease(BinaryCuHandle) == osOK)
		  {
//			  HAL_UART_Transmit_DMA(&huart3, rxdata_cu, rxSize);
			  memcpy(rxdata_cu,rxbuff_cu,Size);
			  memset(rxbuff_cu,0,Size);
			  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  }
	  }
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxbuff_cu, 128);
	  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  }else if(huart == &huart4){
	  taskJingBegin = pdTRUE;
	  if(rxbuff_jing[0] == 0xCC && rxbuff_jing[1] == 0xC0 && rxbuff_jing[Size - 1] == 0xBB)
	  	  {
	  		  if(osSemaphoreRelease(BinaryJingHandle) == osOK)
	  		  {
	  //			  HAL_UART_Transmit_DMA(&huart3, rxdata_cu, rxSize);
	  			  memcpy(rxdata_jing,rxbuff_jing,Size);
	  			  memset(rxbuff_jing,0,Size);
	  			  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  		  }
	  	  }
	  	  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rxbuff_jing, 128);
	  	  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
  }else if(huart == &huart6){
	  taskFocusBegin = pdTRUE;
	  if(rxbuff_focus[0] == 0xaa && rxbuff_focus[1] == 0x01 && rxbuff_focus[Size - 1] == checkData(rxbuff_focus, Size - 1))
	  {
		uart_printf("enter focus\r\n");
		  if(osSemaphoreRelease(BinaryFocusHandle) == osOK)
		  {
			uart_printf("enter\r\n");
			  memcpy(rxdata_focus,rxbuff_focus,Size);
			  memset(rxbuff_focus,0,Size);
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

		  }
//	  HAL_UART_Transmit_DMA(&huart6,"")
	  }
//		  HAL_UART_Transmit(&huart5, rxbuff_focus, Size, 0xff);
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rxbuff_focus, 11);
	  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);

  }else if(huart == &huart7){
	  taskRangeBegin = pdTRUE;
	  if(rxbuff_rangefinder[0] == 0xee && rxbuff_rangefinder[1] == 0x16 && rxbuff_rangefinder[Size - 1] == CheckSumRangefinder(rxbuff_rangefinder + 3, rxbuff_rangefinder[2]))
	  {
		  if(osSemaphoreRelease(BinaryRangefinderHandle) == osOK)
		  {
			  memcpy(rxdata_rangefinder,rxbuff_rangefinder,Size);
			  memset(rxbuff_rangefinder,0,Size);
			  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		  }
	  }

	  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, rxbuff_rangefinder, 128);
	  __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
  }
  else if(huart == &huart2)
  {
	static float fPm1, fPm2, fPm3;
	static int iPm1, iPm2, iPm3;
	
	switch(chcmd[0])
	{
		case 'M':{
			sscanf(chcmd + 1,"%x", &iPm1);
			ServoDataSendTypedef.TurnMode = iPm1;
			ServoDataSend(&ServoSendTypedef,&ServoDataSendTypedef);
			
		}break;
		case 'P':{
			sscanf(chcmd + 1,"%f,%f", &fPm1, &fPm2, &fPm3);
			CommandTypedef.turnYaw.f = fPm1;
			CommandTypedef.turnPitch.f = fPm2;
		}break;
		case 'S':{
			sscanf(chcmd + 1,"%d", &iPm1);
			switch(iPm1)
			{
				case 0:{
					CommandTypedef.go2Zero = 0x01;
					CommandTypedef.standbyEnable = 0x00;
					CommandTypedef.standbyDisable = 0x00;
					CommandTypedef.guideEnable = 0x00;
					CommandTypedef.scanEnable = 0x00;
				}break;
				case 1:{
					CommandTypedef.go2Zero = 0x00;
					CommandTypedef.standbyEnable = 0x01;
					CommandTypedef.standbyDisable = 0x00;
					CommandTypedef.guideEnable = 0x00;
					CommandTypedef.scanEnable = 0x00;
				}break;
				case 2:{
					CommandTypedef.go2Zero = 0x00;
					CommandTypedef.standbyEnable = 0x00;
					CommandTypedef.standbyDisable = 0x01;
					CommandTypedef.guideEnable = 0x00;
					CommandTypedef.scanEnable = 0x00;
				}break;
				case 3:{
					CommandTypedef.go2Zero = 0x00;
					CommandTypedef.standbyEnable = 0x00;
					CommandTypedef.standbyDisable = 0x00;
					CommandTypedef.guideEnable = 0x01;
					CommandTypedef.scanEnable = 0x00;
				}break;
				case 4:{
					CommandTypedef.go2Zero = 0x00;
					CommandTypedef.standbyEnable = 0x00;
					CommandTypedef.standbyDisable = 0x00;
					CommandTypedef.guideEnable = 0x00;
					CommandTypedef.scanEnable = 0x01;
				}break;
				default:
					break;
			}

		}break;
		case 'V':{
			sscanf(chcmd + 1,"%d,%d", &iPm1, &iPm2);
			CommandTypedef.turnYawSpeed = iPm1 * 100;
			CommandTypedef.turnPitchSpeed = iPm2 * 100;
			// ServoDataSendTypedef.Turntable_Yaw = fPm1;
			// ServoDataSendTypedef.Turntable_Pitch = fPm2;
		}break;
		case 'C':{
			ServoDataSendTypedef.TrackingFaultCleanCu = 0x66;
			ServoDataSendTypedef.TrackingFaultCleanJing = 0x33;
		}break;
		case 'F':{
			sscanf(chcmd + 1,"%f,%f", &fPm1, &fPm2);
			CommandTypedef.FSM_Yaw.f = fPm1;
			CommandTypedef.FSM_Pitch.f = fPm2;
		}break;
		default:
		break;
	}

	HAL_UART_Transmit(&huart2, chcmd, Size, 0xff);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)chcmd, 64);
	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
  }
//  else if(huart == &huart2)
//  {
// 	HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
// 	switch(uartbuff[0])
// 	{
// 		case 'E':{
// 			sscanf(uartbuff + 1,"%ld,%ld,%ld,%ld",&pcRecv.exposurecu,&pcRecv.exposurejing,&pcRecv.focusAdjust_cu, &pcRecv.zoomAdjust_cu);

// 		}break;
// 		case 'M':{
// 			sscanf(uartbuff + 1,"%d",&state);
// 			switch(state)
// 			{
// 				case 1:{
// 					CommandTypedef.go2Zero = 0x01;
// 					CommandTypedef.standbyEnable = 0x00;
// 					CommandTypedef.standbyDisable = 0x00;
// 					CommandTypedef.guideEnable = 0x00;
// 					CommandTypedef.scanEnable = 0x00;
// 				}break;
// 				case 2:{
// 					CommandTypedef.go2Zero = 0x00;
// 					CommandTypedef.standbyEnable = 0x01;
// 					CommandTypedef.standbyDisable = 0x00;
// 					CommandTypedef.guideEnable = 0x00;
// 					CommandTypedef.scanEnable = 0x00;
// 				}break;
// 				case 3:{
// 					CommandTypedef.go2Zero = 0x00;
// 					CommandTypedef.standbyEnable = 0x00;
// 					CommandTypedef.standbyDisable = 0x01;
// 					CommandTypedef.guideEnable = 0x00;
// 					CommandTypedef.scanEnable = 0x00;
// 				}break;
// 				case 4:{
// 					CommandTypedef.go2Zero = 0x00;
// 					CommandTypedef.standbyEnable = 0x00;
// 					CommandTypedef.standbyDisable = 0x00;
// 					CommandTypedef.guideEnable = 0x01;
// 					CommandTypedef.scanEnable = 0x00;
// 				}break;
// 				case 5:{
// 					CommandTypedef.go2Zero = 0x00;
// 					CommandTypedef.standbyEnable = 0x00;
// 					CommandTypedef.standbyDisable = 0x00;
// 					CommandTypedef.guideEnable = 0x00;
// 					CommandTypedef.scanEnable = 0x01;
// 				}break;
// 			}
// 		}break;

// 	}
// 	// uart_printf("%d\r\n",state);
// 	// uart_printf("exposurecu:%ld,exposurejing:%ld,focusAdjust_cu:%ld,zoomAdjust_cu:%ld\r\n",pcRecv.exposurecu,pcRecv.exposurejing,pcRecv.focusAdjust_cu,pcRecv.zoomAdjust_cu);
// 	HAL_UARTEx_ReceiveToIdle_IT(&huart2, uartbuff, 16);
// 	__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
//  }
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UARTEx_RxEventCallback can be implemented in the user file.
   */
}
// uint8_t cmd_i = 0;
// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//   /* Prevent unused argument(s) compilation warning */
//   UNUSED(huart);
// 	if(huart == &huart2)
// 	{


// 	if(chcmd[cmd_i] == '\n')
// 	{
// 		chcmd[cmd_i + 1] = '\0';
// 		switch(chcmd[cmd_start])
// 		{
// 			case 'E':{
// 				// uart_printf("%s",chcmd + cmd_start + 1);
// 				uart2_printf("%s",chcmd + cmd_start + 1);
// 				// HAL_UART_Transmit(&huart2, chcmd + cmd_start + 1, cmd_i - cmd_start, 0xff);
// 				// sscanf(chcmd + cmd_start + 1,"%ld,%ld,%ld,%ld",&pcRecv.exposurecu,&pcRecv.exposurejing,&pcRecv.focusAdjust_cu, &pcRecv.zoomAdjust_cu);
// 			}break;
// 			case 'M':{
// 				sscanf(chcmd + cmd_start + 1,"%d",&state);
// 				switch(state)
// 				{
// 					case 1:{
// 						CommandTypedef.go2Zero = 0x01;
// 						CommandTypedef.standbyEnable = 0x00;
// 						CommandTypedef.standbyDisable = 0x00;
// 						CommandTypedef.guideEnable = 0x00;
// 						CommandTypedef.scanEnable = 0x00;
// 					}break;
// 					case 2:{
// 						CommandTypedef.go2Zero = 0x00;
// 						CommandTypedef.standbyEnable = 0x01;
// 						CommandTypedef.standbyDisable = 0x00;
// 						CommandTypedef.guideEnable = 0x00;
// 						CommandTypedef.scanEnable = 0x00;
// 					}break;
// 					case 3:{
// 						CommandTypedef.go2Zero = 0x00;
// 						CommandTypedef.standbyEnable = 0x00;
// 						CommandTypedef.standbyDisable = 0x01;
// 						CommandTypedef.guideEnable = 0x00;
// 						CommandTypedef.scanEnable = 0x00;
// 					}break;
// 					case 4:{
// 						CommandTypedef.go2Zero = 0x00;
// 						CommandTypedef.standbyEnable = 0x00;
// 						CommandTypedef.standbyDisable = 0x00;
// 						CommandTypedef.guideEnable = 0x01;
// 						CommandTypedef.scanEnable = 0x00;
// 					}break;
// 					case 5:{
// 						CommandTypedef.go2Zero = 0x00;
// 						CommandTypedef.standbyEnable = 0x00;
// 						CommandTypedef.standbyDisable = 0x00;
// 						CommandTypedef.guideEnable = 0x00;
// 						CommandTypedef.scanEnable = 0x01;
// 					}break;
// 				}
// 			}break;
// 			default: 
// 				break;
// 		}
// 		cmd_i = 0;
// 	}
// 	else
// 	{
// 		cmd_i++;
// 	}
// 	// uart_printf("%s",chcmd);
// 	HAL_GPIO_WritePin(LED2_GPIO_Port,LED2_Pin,GPIO_PIN_RESET);
// 	HAL_UART_Receive_IT(&huart2, (uint8_t*)&chcmd[cmd_i], 1);
	
// 	}
//   /* NOTE : This function should not be modified, when the callback is needed,
//             the HAL_UART_RxCpltCallback can be implemented in the user file.
//    */
// }

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);
  if(huart == &huart5)
  {
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart5, rxbuff_servo, 128);
	   __HAL_DMA_DISABLE_IT(&hdma_uart5_rx, DMA_IT_HT);

  }else if(huart == &huart3)
  {
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, rxbuff_cu, 128);
	  __HAL_DMA_DISABLE_IT(&hdma_usart3_rx, DMA_IT_HT);
  }else if(huart == &huart4)
  {
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart4, rxbuff_jing, 128);
	  	  	  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx, DMA_IT_HT);
  }else if(huart == &huart6)
  {

	  HAL_UARTEx_ReceiveToIdle_IT(&huart6, rxbuff_focus, 11);
	  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT);
  }else if(huart == &huart7)
  {
	  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, rxbuff_rangefinder, 128);
	  __HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
  }
	else if(huart == &huart2)
	{
		// HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		// uart_printf("error");
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)chcmd, 16);
		__HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);	
	}
  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_ErrorCallback can be implemented in the user file.
   */
}



void mytask(void const * argument)
{
	static switchFlag = 0;
	osDelay(5000);
	
  /* USER CODE BEGIN mytask */
  /* Infinite loop */
  for(;;)
  {
	// if(switchFlag == 0)
	// {
	// 	CommandTypedef.laserAdjust += 50;
	// 	if(CommandTypedef.laserAdjust == 6500)
	// 	{
	// 		switchFlag = 1;
	// 	}
		
	// }
	// else if(switchFlag == 1)
	// {
	// 	CommandTypedef.laserAdjust -= 50;
	// 	if(CommandTypedef.laserAdjust == 50)
	// 	{
	// 		switchFlag = 0;
	// 	}
	// }
	// setFocusPos.f = (float)CommandTypedef.laserAdjust;
	// SendFocusFrame.func = SetPos;
	// SendFocusData(&SendFocusFrame);
	// uart_printf("laserAdjust:%f\r\n",setFocusPos.f);
	// osDelay(100);

	Send2ImgModule(&ImgSendTypedef_JING,&ImgSendDataTypedef_JING);
	osDelay(500);
  }
  /* USER CODE END mytask */
}


/* USER CODE END Application */
