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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24_ELBC.h"
#include <string.h>
#include <stdio.h>
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
SPI_HandleTypeDef Button_hspi1;
SPI_HandleTypeDef EL_hspi2;
SPI_HandleTypeDef ESP32_hspi3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI3_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void EdgeLED_Color(uint8_t Button_Number, uint8_t Green, uint8_t Red, uint8_t Blue, uint8_t *Packet);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint8_t Config_table[38][4] = {
		//ID,Channel,Pipe,Module
		{ 1,  100, 0, 1 },
		{ 2,  100, 1, 1 },
		{ 3,  100, 2, 1 },
		{ 4,  100, 3, 1 },
		{ 5,  100, 4, 1 },
		{ 6,  100, 5, 1 },
		{ 7,  101, 0, 2 },
		{ 8,  101, 1, 2 },
		{ 9,  101, 2, 2 },
		{ 10, 101, 3, 2 },
		{ 11, 101, 4, 2 },
		{ 12, 101, 5, 2 },
		{ 13, 102, 0, 3 },
		{ 14, 102, 1, 3 },
		{ 15, 102, 2, 3 },
		{ 16, 102, 3, 3 },
		{ 17, 102, 4, 3 },
		{ 18, 102, 5, 3 },
		{ 19, 103, 0, 4 },
		{ 20, 103, 1, 4 },
		{ 21, 103, 2, 4 },
		{ 22, 103, 3, 4 },
		{ 23, 103, 4, 4 },
		{ 24, 103, 5, 4 },
		{ 25, 104, 0, 1 },
		{ 26, 104, 1, 1 },
		{ 27, 104, 2, 1 },
		{ 28, 104, 3, 1 },
		{ 29, 104, 5, 1 },
		{ 30, 104, 0, 1 },
		{ 31, 105, 1, 2 },
		{ 32, 105, 2, 2 },
		{ 33, 105, 3, 2 },
		{ 34, 105, 4, 2 },
		{ 35, 105, 0, 2 },
		{ 36, 105, 1, 2 },
		{ 37, 106, 2, 3 },
		{ 38, 106, 3, 3 }
};


uint8_t Module1_Packet[6][16];
uint8_t Module2_Packet[6][16];
uint8_t Module3_Packet[6][16];
uint8_t Module4_Packet[6][16];
uint8_t Button_Packet_Size = 16;
uint8_t EdgeLED_Packet[20];
uint8_t EdgeLED_Packet_Size = 20;
uint8_t ESP32_Packet[8];
uint8_t ESP32_Packet_Size = 8;

uint8_t TxAddr_B_Module1[5] = {0x00,0x34,0x36,0x52,0x53};
uint8_t RxAddr_B_Module1[5] = {0x00,0x34,0x36,0x52,0x53};

uint8_t TxAddr_B_Module2[5] = {0x00,0x34,0x36,0x52,0x53};
uint8_t RxAddr_B_Module2[5] = {0x00,0x34,0x36,0x52,0x53};

uint8_t TxAddr_B_Module3[5] = {0x00,0x34,0x36,0x52,0x53};
uint8_t RxAddr_B_Module3[5] = {0x00,0x34,0x36,0x52,0x53};

uint8_t TxAddr_B_Module4[5] = {0x00,0x34,0x36,0x52,0x53};
uint8_t RxAddr_B_Module4[5] = {0x00,0x34,0x36,0x52,0x53};

uint8_t TxAddr_EL_Module[5] = {0x00,0x4C,0x45,0x52,0x53};
uint8_t RxAddr_EL_Module[5] = {0x00,0x4C,0x45,0x52,0x53};

uint8_t RF_Channel_Module1 = 0;
uint8_t RF_Channel_Module2 = 0;
uint8_t RF_Channel_Module3 = 0;
uint8_t RF_Channel_Module4 = 0;
uint8_t RF_Channel_EL_Module = 1;
uint8_t Uart_Cmd = 0;
uint8_t Temp10 = 0;
uint8_t Pipe_Num=0;

uint8_t Key_Pressed=0;

uint8_t Data_In_Module1 = 0;
uint8_t Data_In_Module2 = 0;
uint8_t Data_In_Module3 = 0;
uint8_t Data_In_Module4 = 0;

uint8_t Module1_Button_Number=0;
uint8_t Module2_Button_Number=0;
uint8_t Module3_Button_Number=0;
uint8_t Module4_Button_Number=0;

uint8_t Payload_available = 0;
uint8_t Module1_Packet_Count = 0;
uint8_t Module2_Packet_Count = 0;
uint8_t Module3_Packet_Count = 0;
uint8_t Module4_Packet_Count = 0;
uint8_t temp,temp2;
uint8_t Rec_Button_Number = 255;

#ifdef Timing_Test
uint32_t interrupt_time1 = 0;
uint32_t interrupt_time2 = 0;
uint32_t time1 = 0;
uint32_t time2 = 0;
uint32_t time3 = 0;
uint32_t time4 = 0;
uint32_t time_2_1 = 0;
uint32_t time_3_2 = 0;
uint32_t key1_time1 = 0;
uint32_t key1_time2 = 0;
uint32_t key2_time1 = 0;
uint32_t key2_time2 = 0;
uint32_t key3_time1 = 0;
uint32_t key3_time2 = 0;
uint32_t key4_time1 = 0;
uint32_t key4_time2 = 0;
uint32_t key1_time = 0;
uint32_t key2_time = 0;
uint32_t key3_time = 0;
uint32_t key4_time = 0;
uint32_t interrupt_time = 0;
uint32_t time_total = 0;
uint32_t ADC_time1 = 0;
uint32_t ADC_time2 = 0;
uint32_t interrupt_counter = 0;
#endif

/*
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  // This is the interrupt callback function for SPI receive complete event
  // It will be called whenever new data is received

  // Read the received data from the SPI data register
  for (int i = 0; i < 8; i++)
  {
    ESP32_receivedData[i] = HAL_SPI_Receive(&hspi, receivedData, 8, 100);
  }
}
*/

void EdgeLED_Color(uint8_t Button_Number, uint8_t Green, uint8_t Red, uint8_t Blue, uint8_t *Packet)
{
	//Packet[0]=Button_Number;
	Packet[0] = Button_Number;
	Packet[1] = Green;
	Packet[2] = Red;
	Packet[3] = Blue;

	nRF24_Transmit(Sel_EL_Module, Packet, EdgeLED_Packet_Size);
	nRF24_Transmit(Sel_EL_Module, Packet, EdgeLED_Packet_Size);
#ifdef Debug
	nRF24_Transmit_Report(Sel_EL_Module);
#endif
}


//Interrupt handler to check which module has new packet
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	interrupt_time1 = HAL_GetTick();
	if (GPIO_Pin == nRF_B_IRQ1_Pin)
	{

#ifdef Debug
		HAL_UART_Transmit(&huart2, (uint8_t *)"Data received in Button module1\n\n",33, HAL_MAX_DELAY);
#endif
		Data_In_Module1 = 1;
	}
	if (GPIO_Pin == nRF_B_IRQ2_Pin)
	{
#ifdef Debug
		HAL_UART_Transmit(&huart2, (uint8_t *)"Data received in Button module2\n\n",33, HAL_MAX_DELAY);
#endif
		Data_In_Module2 = 1;
	  }
	if (GPIO_Pin == nRF_B_IRQ3_Pin)
	{
#ifdef Debug
	    HAL_UART_Transmit(&huart2, (uint8_t *)"Data received in Button module3\n\n",33, HAL_MAX_DELAY);
#endif
		Data_In_Module3 = 1;
	}
	if (GPIO_Pin == nRF_B_IRQ4_Pin)
	{
#ifdef Debug
		HAL_UART_Transmit(&huart2, (uint8_t *)"Data received in Button module4\n\n",33, HAL_MAX_DELAY);
#endif
		Data_In_Module4 = 1;
	}
	interrupt_time2 = HAL_GetTick();
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
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_SPI3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  //RF Modules address and frequency channel configuration
  TxAddr_B_Module1[0] = Config_table[0][0];
  RxAddr_B_Module1[0] = Config_table[0][0];
  TxAddr_B_Module2[0] = Config_table[6][0];
  RxAddr_B_Module2[0] = Config_table[6][0];
  TxAddr_B_Module3[0] = Config_table[12][0];
  RxAddr_B_Module3[0] = Config_table[12][0];
  TxAddr_B_Module4[0] = Config_table[18][0];
  RxAddr_B_Module4[0] = Config_table[18][0];
  RF_Channel_Module1 = Config_table[0][1];
  RF_Channel_Module2 = Config_table[6][1];
  RF_Channel_Module3 = Config_table[12][1];
  RF_Channel_Module4 = Config_table[18][1];


  nRF24_Reset(Sel_B_Module1,nRF24_REG_STATUS);
  nRF24_Reset(Sel_B_Module2,nRF24_REG_STATUS);
  nRF24_Reset(Sel_B_Module3,nRF24_REG_STATUS);
  nRF24_Reset(Sel_B_Module4,nRF24_REG_STATUS);
  nRF24_Reset(Sel_EL_Module,nRF24_REG_STATUS);

  nRF24_Module_Setup(Sel_B_Module1, Receiver_Mode_noAA_noCRC,
		  RF_Channel_Module1, TxAddr_B_Module1, RxAddr_B_Module1,
		  MBPS1_0dBm, Button_Packet_Size);
  nRF24_Power(Sel_B_Module1, Power_Up);
  nRF24_CE_Enable(Sel_B_Module1);
  nRF24_Module_Setup(Sel_B_Module2, Receiver_Mode_noAA_noCRC,
		  RF_Channel_Module2, TxAddr_B_Module2, RxAddr_B_Module2,
		  MBPS1_0dBm, Button_Packet_Size);
  nRF24_Power(Sel_B_Module2, Power_Up);
  nRF24_CE_Enable(Sel_B_Module2);
  nRF24_Module_Setup(Sel_B_Module3, Receiver_Mode_noAA_noCRC,
		  RF_Channel_Module3, TxAddr_B_Module3, RxAddr_B_Module3,
		  MBPS1_0dBm, Button_Packet_Size);
  nRF24_Power(Sel_B_Module3, Power_Up);
  nRF24_CE_Enable(Sel_B_Module3);
  nRF24_Module_Setup(Sel_B_Module4, Receiver_Mode_noAA_noCRC,
		  RF_Channel_Module4, TxAddr_B_Module4, RxAddr_B_Module4,
		  MBPS1_0dBm, Button_Packet_Size);
  nRF24_Power(Sel_B_Module4, Power_Up);
  nRF24_CE_Enable(Sel_B_Module4);
  nRF24_Module_Setup(Sel_EL_Module, Transmitter_Mode_noAA_noCRC,
		  RF_Channel_EL_Module, TxAddr_EL_Module, RxAddr_EL_Module,
		  MBPS1_0dBm, EdgeLED_Packet_Size);
  nRF24_Power(Sel_EL_Module, Power_Up);
  nRF24_CE_Enable(Sel_EL_Module);

#ifdef Debug
  nRF24_Register_Display(Sel_B_Module1);
  nRF24_Register_Display(Sel_B_Module2);
  nRF24_Register_Display(Sel_B_Module3);
  nRF24_Register_Display(Sel_B_Module4);
  nRF24_Register_Display(Sel_EL_Module);
#endif

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
#ifdef Timing_Test
	  time1 = HAL_GetTick();
#endif
	  if(HAL_UART_Receive(&huart2, &Uart_Cmd, 1, 10) == HAL_OK)
	  {
		  if(Uart_Cmd == 'W')
		  {
			  EdgeLED_Color(1, 255, 255, 255, EdgeLED_Packet);
			  HAL_UART_Transmit(&huart2, (uint8_t*)"\nWHITE\n",7, 10);
		  }
		  if(Uart_Cmd == 'R')
		  {
			  EdgeLED_Color(1, 0, 255, 0, EdgeLED_Packet);
			  HAL_UART_Transmit(&huart2, (uint8_t*)"\nRED\n",5, 10);
		  }
		  if(Uart_Cmd == 'B')
		  {
			  EdgeLED_Color(1, 0, 0, 255, EdgeLED_Packet);
			  HAL_UART_Transmit(&huart2, (uint8_t*)"\nBLUE\n",6, 10);
		  }
		  if(Uart_Cmd == 'G')
		  {
			  EdgeLED_Color(1, 255, 0, 0, EdgeLED_Packet);
			  HAL_UART_Transmit(&huart2, (uint8_t*)"\nGREEN\n",7, 10);
		  }
	  }
#ifdef Timing_Test
	  time2 = HAL_GetTick();
#endif
	  if (__HAL_SPI_GET_FLAG(&ESP32_hspi3, SPI_FLAG_RXNE))
	  {
	      // Read the received data from the SPI data register
		  HAL_SPI_Receive(&ESP32_hspi3, ESP32_Packet, ESP32_Packet_Size, 100);
	  }
#ifdef Timing_Test
	  time3 = HAL_GetTick();
#endif

	  if (Data_In_Module1 == 1)
	  {
		  key1_time1 = HAL_GetTick();
#ifdef Debug
		  nRF24_Receive_Report(Sel_B_Module1);
#endif
		  Module1_Packet_Count = 0;

		  uint8_t Data_In_RX_FIFO = 1;
		  //Reading the received packet in FIFO
		  //Checking the status of first bit of FIFO_STATUS
		  while(Data_In_RX_FIFO == 1)
		  {
			  uint8_t FIFO_Reg = nRF24_ReadReg(Sel_B_Module1, nRF24_REG_FIFO_STATUS);
			  if((FIFO_Reg & 1) == 0)
			  {
				  nRF24_Receive(Sel_B_Module1, Module1_Packet[Module1_Packet_Count], Button_Packet_Size);
				  Module1_Packet_Count++;
			  }
			  else
			  {
				  Data_In_RX_FIFO = 0;
			  }
		  }
		  //Printing the received packets
#ifdef Debug
		  HAL_UART_Transmit(&huart2, (uint8_t*)"**********************\n",23, 100);
		  HAL_UART_Transmit(&huart2, (uint8_t*)"Module1 Data:\n",13, 1000);
		  for (uint8_t i = 0; i < Module1_Packet_Count; i++)
		  {
			  HAL_UART_Transmit(&huart2, (uint8_t*)"*****\n", 6, 1000);
			  // Convert RxData elements to char and print
			  for (uint8_t j = 0; j < Button_Packet_Size; j++)
			  {
			      char buffer[4];  // Buffer to hold the converted string
			      sprintf(buffer, "%d", Module1_Packet[i][j]);  // Convert integer to string

			      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000);
			      HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);
			  }
		  }
		  HAL_UART_Transmit(&huart2, (uint8_t*)"\n**********************\n\n",25, 100);
#endif

		  for (uint8_t i = 0; i < Module1_Packet_Count; i++)
		  {
			  //Checking which Button has sent the data
			  Rec_Button_Number = Module1_Packet[i][0];

			  //Checking which key on the button has been pressed
			  Key_Pressed = Module1_Packet[i][1];

		      //Sending Color to the specified EdgeLED based on the key pressed
			  switch(Key_Pressed)
			  {
				case 1:
					EdgeLED_Color(Rec_Button_Number, 255, 255, 255, EdgeLED_Packet);
					break;
				case 2:
				    EdgeLED_Color(Rec_Button_Number, 255,   0,   0, EdgeLED_Packet);
					break;
				case 3:
					EdgeLED_Color(Rec_Button_Number,   0, 255,   0, EdgeLED_Packet);
					break;
				case 4:
				    EdgeLED_Color(Rec_Button_Number,   0,   0, 255, EdgeLED_Packet);
					break;
				default:
				    break;
			  }

			  Rec_Button_Number = 255;
			  Key_Pressed = 255;
		  }

		  Data_In_Module1 = 0;
#ifdef Timing_Test
		  key1_time2 = HAL_GetTick();

		  time_2_1 = time2 - time1;
		  time_3_2 = time3 - time2;
		  interrupt_time = interrupt_time2 - interrupt_time1;
		  key1_time = key1_time2 - key1_time1;
		  key1_time1 = 0;
		  key1_time2 = 0;

		  char time_buffer[10];  // Buffer to hold the converted string

		  sprintf(time_buffer, "%d", time_2_1);  // Convert integer to string
		  HAL_UART_Transmit(&huart2, (uint8_t*)"time_2_1: ", 10, 1000);
		  HAL_UART_Transmit(&huart2, (uint8_t*)time_buffer, strlen(time_buffer), 1000);
		  HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);

		  sprintf(time_buffer, "%d", time_3_2);  // Convert integer to string
		  HAL_UART_Transmit(&huart2, (uint8_t*)"time_3_2: ", 10, 1000);
		  HAL_UART_Transmit(&huart2, (uint8_t*)time_buffer, strlen(time_buffer), 1000);
		  HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);

		  sprintf(time_buffer, "%d", key1_time);  // Convert integer to string
		  HAL_UART_Transmit(&huart2, (uint8_t*)"key1_time: ", 11, 1000);
		  HAL_UART_Transmit(&huart2, (uint8_t*)time_buffer, strlen(time_buffer), 1000);
		  HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);

		  sprintf(time_buffer, "%d", interrupt_time);  // Convert integer to string
		  HAL_UART_Transmit(&huart2, (uint8_t*)"interrupt_time: ", 16, 1000);
		  HAL_UART_Transmit(&huart2, (uint8_t*)time_buffer, strlen(time_buffer), 1000);
		  HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);
#endif
	  }


	  if (Data_In_Module2 == 1)
	  {
#ifdef Debug
		  nRF24_Receive_Report(Sel_B_Module2);
#endif
		  Module2_Packet_Count = 0;

		  uint8_t Data_In_RX_FIFO = 1;
		  //Reading the received packet in FIFO
		  //Checking the status of first bit of FIFO_STATUS
		  while(Data_In_RX_FIFO == 1)
		  {
			  uint8_t FIFO_Reg = nRF24_ReadReg(Sel_B_Module2, nRF24_REG_FIFO_STATUS);
			  if((FIFO_Reg & 1) == 0)
			  {
				  nRF24_Receive(Sel_B_Module2, Module2_Packet[Module2_Packet_Count], Button_Packet_Size);
				  Module2_Packet_Count++;
			  }
			  else
			  {
				  Data_In_RX_FIFO = 0;
			  }
		  }
		  //Printing the received packets
#ifdef Debug
		  HAL_UART_Transmit(&huart2, (uint8_t*)"**********************\n",23, 100);
		  HAL_UART_Transmit(&huart2, (uint8_t*)"Module2 Data:\n",13, 1000);
		  for (uint8_t i = 0; i < Module2_Packet_Count; i++)
		  {
			  HAL_UART_Transmit(&huart2, (uint8_t*)"*****\n", 6, 1000);
			  // Convert RxData elements to char and print
			  for (uint8_t j = 0; j < Button_Packet_Size; j++)
			  {
			      char buffer[4];  // Buffer to hold the converted string
			      sprintf(buffer, "%d", Module2_Packet[i][j]);  // Convert integer to string

			      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000);
			      HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);
			  }
		  }
		  HAL_UART_Transmit(&huart2, (uint8_t*)"\n**********************\n\n",25, 100);
#endif

		  for (uint8_t i = 0; i < Module2_Packet_Count; i++)
		  {
			  //Checking which Button has sent the data
			  Rec_Button_Number = Module2_Packet[i][0];

			  //Checking which key on the button has been pressed
			  Key_Pressed = Module2_Packet[i][1];

		      //Sending Color to the specified EdgeLED based on the key pressed
			  switch(Key_Pressed)
			  {
				case 1:
					EdgeLED_Color(Rec_Button_Number, 255, 255, 255, EdgeLED_Packet);
					break;
				case 2:
				    EdgeLED_Color(Rec_Button_Number, 255,   0,   0, EdgeLED_Packet);
					break;
				case 3:
					EdgeLED_Color(Rec_Button_Number,   0, 255,   0, EdgeLED_Packet);
					break;
				case 4:
				    EdgeLED_Color(Rec_Button_Number,   0,   0, 255, EdgeLED_Packet);
					break;
				default:
				    break;
			  }

			  Rec_Button_Number = 255;
			  Key_Pressed = 255;
		  }

		  Data_In_Module2 = 0;

	  }
	  if (Data_In_Module3 == 1)
	  {
#ifdef Debug
		  nRF24_Receive_Report(Sel_B_Module3);
#endif
		  Module3_Packet_Count = 0;

		  uint8_t Data_In_RX_FIFO = 1;
		  //Reading the received packet in FIFO
		  //Checking the status of first bit of FIFO_STATUS
		  while(Data_In_RX_FIFO == 1)
		  {
			  uint8_t FIFO_Reg = nRF24_ReadReg(Sel_B_Module3, nRF24_REG_FIFO_STATUS);
			  if((FIFO_Reg & 1) == 0)
			  {
				  nRF24_Receive(Sel_B_Module3, Module3_Packet[Module3_Packet_Count], Button_Packet_Size);
				  Module3_Packet_Count++;
			  }
			  else
			  {
				  Data_In_RX_FIFO = 0;
			  }
		  }
		  //Printing the received packets
#ifdef Debug
		  HAL_UART_Transmit(&huart2, (uint8_t*)"**********************\n",23, 100);
		  HAL_UART_Transmit(&huart2, (uint8_t*)"Module3 Data:\n",13, 1000);
		  for (uint8_t i = 0; i < Module3_Packet_Count; i++)
		  {
			  HAL_UART_Transmit(&huart2, (uint8_t*)"*****\n", 6, 1000);
			  // Convert RxData elements to char and print
			  for (uint8_t j = 0; j < Button_Packet_Size; j++)
			  {
			      char buffer[4];  // Buffer to hold the converted string
			      sprintf(buffer, "%d", Module3_Packet[i][j]);  // Convert integer to string

			      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000);
			      HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);
			  }
		  }
		  HAL_UART_Transmit(&huart2, (uint8_t*)"\n**********************\n\n",25, 100);
#endif

		  for (uint8_t i = 0; i < Module3_Packet_Count; i++)
		  {
			  //Checking which Button has sent the data
			  Rec_Button_Number = Module3_Packet[i][0];

			  //Checking which key on the button has been pressed
			  Key_Pressed = Module3_Packet[i][1];

		      //Sending Color to the specified EdgeLED based on the key pressed
			  switch(Key_Pressed)
			  {
				case 1:
					EdgeLED_Color(Rec_Button_Number, 255, 255, 255, EdgeLED_Packet);
					break;
				case 2:
				    EdgeLED_Color(Rec_Button_Number, 255,   0,   0, EdgeLED_Packet);
					break;
				case 3:
					EdgeLED_Color(Rec_Button_Number,   0, 255,   0, EdgeLED_Packet);
					break;
				case 4:
				    EdgeLED_Color(Rec_Button_Number,   0,   0, 255, EdgeLED_Packet);
					break;
				default:
				    break;
			  }

			  Rec_Button_Number = 255;
			  Key_Pressed = 255;
		  }

		  Data_In_Module3 = 0;

	  }
	  if (Data_In_Module4 == 1)
	  {
#ifdef Debug
		  nRF24_Receive_Report(Sel_B_Module4);
#endif
		  Module4_Packet_Count = 0;

		  uint8_t Data_In_RX_FIFO = 1;
		  //Reading the received packet in FIFO
		  //Checking the status of first bit of FIFO_STATUS
		  while(Data_In_RX_FIFO == 1)
		  {
			  uint8_t FIFO_Reg = nRF24_ReadReg(Sel_B_Module4, nRF24_REG_FIFO_STATUS);
			  if((FIFO_Reg & 1) == 0)
			  {
				  nRF24_Receive(Sel_B_Module4, Module4_Packet[Module4_Packet_Count], Button_Packet_Size);
				  Module4_Packet_Count++;
			  }
			  else
			  {
				  Data_In_RX_FIFO = 0;
			  }
		  }
		  //Printing the received packets
#ifdef Debug
		  HAL_UART_Transmit(&huart2, (uint8_t*)"**********************\n",23, 100);
		  HAL_UART_Transmit(&huart2, (uint8_t*)"Module4 Data:\n",13, 1000);
		  for (uint8_t i = 0; i < Module4_Packet_Count; i++)
		  {
			  HAL_UART_Transmit(&huart2, (uint8_t*)"*****\n", 6, 1000);
			  // Convert RxData elements to char and print
			  for (uint8_t j = 0; j < Button_Packet_Size; j++)
			  {
			      char buffer[4];  // Buffer to hold the converted string
			      sprintf(buffer, "%d", Module4_Packet[i][j]);  // Convert integer to string

			      HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 1000);
			      HAL_UART_Transmit(&huart2, (uint8_t*)"\n", 1, 1000);
			  }
		  }
		  HAL_UART_Transmit(&huart2, (uint8_t*)"\n**********************\n\n",25, 100);
#endif

		  for (uint8_t i = 0; i < Module4_Packet_Count; i++)
		  {
			  //Checking which Button has sent the data
			  Rec_Button_Number = Module4_Packet[i][0];

			  //Checking which key on the button has been pressed
			  Key_Pressed = Module4_Packet[i][1];

		      //Sending Color to the specified EdgeLED based on the key pressed
			  switch(Key_Pressed)
			  {
				case 1:
					EdgeLED_Color(Rec_Button_Number, 255, 255, 255, EdgeLED_Packet);
					break;
				case 2:
				    EdgeLED_Color(Rec_Button_Number, 255,   0,   0, EdgeLED_Packet);
					break;
				case 3:
					EdgeLED_Color(Rec_Button_Number,   0, 255,   0, EdgeLED_Packet);
					break;
				case 4:
				    EdgeLED_Color(Rec_Button_Number,   0,   0, 255, EdgeLED_Packet);
					break;
				default:
				    break;
			  }

			  Rec_Button_Number = 255;
			  Key_Pressed = 255;
		  }

		  Data_In_Module4 = 0;

	  }

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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  Button_hspi1.Instance = SPI1;
  Button_hspi1.Init.Mode = SPI_MODE_MASTER;
  Button_hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  Button_hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  Button_hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  Button_hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  Button_hspi1.Init.NSS = SPI_NSS_SOFT;
  Button_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  Button_hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  Button_hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  Button_hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  Button_hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&Button_hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  EL_hspi2.Instance = SPI2;
  EL_hspi2.Init.Mode = SPI_MODE_MASTER;
  EL_hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  EL_hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  EL_hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  EL_hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  EL_hspi2.Init.NSS = SPI_NSS_SOFT;
  EL_hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  EL_hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  EL_hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  EL_hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  EL_hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&EL_hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  ESP32_hspi3.Instance = SPI3;
  ESP32_hspi3.Init.Mode = SPI_MODE_MASTER;
  ESP32_hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  ESP32_hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  ESP32_hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  ESP32_hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  ESP32_hspi3.Init.NSS = SPI_NSS_SOFT;
  ESP32_hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  ESP32_hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  ESP32_hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  ESP32_hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  ESP32_hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&ESP32_hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, nRF_EL_CSN_Pin|nRF_B_CSN1_Pin|nRF_B_CSN4_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, nRF_EL_CE_Pin|T_LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, nRF_B_CE1_Pin|nRF_B_CE2_Pin|nRF_B_CE3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, nRF_B_CSN2_Pin|nRF_B_CSN3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, nRF_B_CE4_Pin|T_LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ESP32_CS_GPIO_Port, ESP32_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : nRF_EL_CSN_Pin nRF_B_CSN1_Pin nRF_B_CE4_Pin nRF_B_CSN4_Pin */
  GPIO_InitStruct.Pin = nRF_EL_CSN_Pin|nRF_B_CSN1_Pin|nRF_B_CE4_Pin|nRF_B_CSN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : nRF_EL_IRQ_Pin nRF_B_IRQ1_Pin nRF_B_IRQ4_Pin */
  GPIO_InitStruct.Pin = nRF_EL_IRQ_Pin|nRF_B_IRQ1_Pin|nRF_B_IRQ4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : nRF_EL_CE_Pin */
  GPIO_InitStruct.Pin = nRF_EL_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(nRF_EL_CE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : nRF_B_CE1_Pin nRF_B_CSN2_Pin nRF_B_CE2_Pin nRF_B_CSN3_Pin
                           nRF_B_CE3_Pin */
  GPIO_InitStruct.Pin = nRF_B_CE1_Pin|nRF_B_CSN2_Pin|nRF_B_CE2_Pin|nRF_B_CSN3_Pin
                          |nRF_B_CE3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : nRF_B_IRQ2_Pin nRF_B_IRQ3_Pin */
  GPIO_InitStruct.Pin = nRF_B_IRQ2_Pin|nRF_B_IRQ3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : T_LED2_Pin */
  GPIO_InitStruct.Pin = T_LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(T_LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : T_Key2_Pin T_Key1_Pin */
  GPIO_InitStruct.Pin = T_Key2_Pin|T_Key1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : T_LED1_Pin */
  GPIO_InitStruct.Pin = T_LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(T_LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ESP32_CS_Pin */
  GPIO_InitStruct.Pin = ESP32_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ESP32_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
