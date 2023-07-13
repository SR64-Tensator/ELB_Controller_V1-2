// Functions to manage the nRF24L01+ transceiver

#include <nrf24_ELBC.h>
#include "main.h"

#define ELBC

void nRF24_CSN_Select(uint16_t Sel_CSN_Pin)
{
	switch (Sel_CSN_Pin)
	{
		case Sel_EL_Module:
			HAL_GPIO_WritePin(GPIOC, nRF_EL_CSN_Pin, GPIO_PIN_RESET);
			break;
		case Sel_B_Module1:
			HAL_GPIO_WritePin(GPIOC, nRF_B_CSN1_Pin, GPIO_PIN_RESET);
			break;
		case Sel_B_Module2:
			HAL_GPIO_WritePin(GPIOB, nRF_B_CSN2_Pin, GPIO_PIN_RESET);
			break;
		case Sel_B_Module3:
			HAL_GPIO_WritePin(GPIOB, nRF_B_CSN3_Pin, GPIO_PIN_RESET);
			break;
		case Sel_B_Module4:
			HAL_GPIO_WritePin(GPIOC, nRF_B_CSN4_Pin, GPIO_PIN_RESET);
			break;
	}
}

void nRF24_CSN_UnSelect(uint16_t Sel_CSN_Pin)
{
	switch (Sel_CSN_Pin)
	{
		case Sel_EL_Module:
			HAL_GPIO_WritePin(GPIOC, nRF_EL_CSN_Pin, GPIO_PIN_SET);
			break;
		case Sel_B_Module1:
			HAL_GPIO_WritePin(GPIOC, nRF_B_CSN1_Pin, GPIO_PIN_SET);
			break;
		case Sel_B_Module2:
			HAL_GPIO_WritePin(GPIOB, nRF_B_CSN2_Pin, GPIO_PIN_SET);
			break;
		case Sel_B_Module3:
			HAL_GPIO_WritePin(GPIOB, nRF_B_CSN3_Pin, GPIO_PIN_SET);
			break;
		case Sel_B_Module4:
			HAL_GPIO_WritePin(GPIOC, nRF_B_CSN4_Pin, GPIO_PIN_SET);
			break;
	}
}

void nRF24_CE_Enable(uint16_t Sel_CE_Pin)
{
	switch (Sel_CE_Pin)
	{
		case Sel_EL_Module:
			HAL_GPIO_WritePin(GPIOA, nRF_EL_CE_Pin, GPIO_PIN_SET);
			break;
		case Sel_B_Module1:
			HAL_GPIO_WritePin(GPIOB, nRF_B_CE1_Pin, GPIO_PIN_SET);
			break;
		case Sel_B_Module2:
			HAL_GPIO_WritePin(GPIOB, nRF_B_CE2_Pin, GPIO_PIN_SET);
			break;
		case Sel_B_Module3:
			HAL_GPIO_WritePin(GPIOB, nRF_B_CE3_Pin, GPIO_PIN_SET);
			break;
		case Sel_B_Module4:
			HAL_GPIO_WritePin(GPIOC, nRF_B_CE4_Pin, GPIO_PIN_SET);
			break;
	}
}

void nRF24_CE_Disable(uint16_t Sel_CE_Pin)
{
	switch (Sel_CE_Pin)
	{
	case Sel_EL_Module:
		HAL_GPIO_WritePin(GPIOA, nRF_EL_CE_Pin, GPIO_PIN_RESET);
		break;
	case Sel_B_Module1:
		HAL_GPIO_WritePin(GPIOB, nRF_B_CE1_Pin, GPIO_PIN_RESET);
		break;
	case Sel_B_Module2:
		HAL_GPIO_WritePin(GPIOB, nRF_B_CE2_Pin, GPIO_PIN_RESET);
		break;
	case Sel_B_Module3:
		HAL_GPIO_WritePin(GPIOB, nRF_B_CE3_Pin, GPIO_PIN_RESET);
		break;
	case Sel_B_Module4:
		HAL_GPIO_WritePin(GPIOC, nRF_B_CE4_Pin, GPIO_PIN_RESET);
		break;
	}
}


// Writes a single byte to the particular register
// input:
//   Sel_CSN_Pin - Select the RF Module
//   reg - number of register to write
//   data - value to write
void nRF24_WriteReg(uint16_t Sel_CSN_Pin, uint8_t reg, uint8_t data)
{
	uint8_t buf[2];
	buf[0] = reg | 1<<5;                                  //Set the 6th bit of the reg to make it a write command
	buf[1] = data;

	nRF24_CSN_Select(Sel_CSN_Pin);                  //Pull CSN pin low to select the nRF24 on SPI BUS
	if(Sel_CSN_Pin != Sel_EL_Module)
	{
		HAL_SPI_Transmit(&Button_hspi1, buf, 2, 1000);    //Write command and register address (001AAAAA) and then the data to be written
		nRF24_CSN_UnSelect(Sel_B_Module1);                //Pull CSN pin high to unselect the nRF24 on SPI BUS
	}
	else
	{
		HAL_SPI_Transmit(&EL_hspi2, buf, 2, 1000);        //Write command and register address (001AAAAA) and then the data to be written
		nRF24_CSN_UnSelect(Sel_EL_Module);                //Pull CSN pin high to unselect the nRF24 on SPI BUS
	}
	nRF24_CSN_UnSelect(Sel_CSN_Pin);                  //Pull CSN pin low to select the nRF24 on SPI BUS
}

// Writes multiple bytes starting from a particular register
// input:
//   Sel_CSN_Pin - Select the RF Module
//   reg - number of register to write
//   data - pointer to the buffer with data to write
//   size - number of bytes to write
void nRF24_WriteMBReg(uint16_t Sel_CSN_Pin, uint8_t reg, uint8_t *data, uint8_t size)
{
	uint8_t buf[2];
	buf[0] = reg | 1<<5;                                      //Set the 6th bit of the reg to make it a write command
//	buf[1] = data;

	nRF24_CSN_Select(Sel_CSN_Pin);                      //Pull CSN pin low to select the nRF24 on SPI BUS
	if(Sel_CSN_Pin != Sel_EL_Module)
	{
		HAL_SPI_Transmit(&Button_hspi1, buf, 1, 1000);        //Write command and register address (001AAAAA)
		HAL_SPI_Transmit(&Button_hspi1, data, size, 1000);    //Send the data to be written in that address
	}
	else
	{
		HAL_SPI_Transmit(&EL_hspi2, buf, 1, 1000);            //Write command and register address (001AAAAA)
		HAL_SPI_Transmit(&EL_hspi2, data, size, 1000);        //Send the data to be written in that address
	}
	nRF24_CSN_UnSelect(Sel_CSN_Pin);                    //Pull CSN pin high to unselect the nRF24 on SPI BUS
}

// Reads a value of register
// input:
//   Sel_CSN_Pin - Select the RF Module
//   reg - number of register to read
// return: value of register
uint8_t nRF24_ReadReg(uint16_t Sel_CSN_Pin, uint8_t reg) {

	uint8_t data = 0;

	nRF24_CSN_Select(Sel_CSN_Pin);                            //Pull CSN pin low to select the nRF24 on SPI BUS
	if(Sel_CSN_Pin != Sel_EL_Module)
	{
		HAL_SPI_Transmit(&Button_hspi1, &reg, 1, 1000);       //Sending the read command and register address (000AAAAA)
		HAL_SPI_Receive(&Button_hspi1, &data, 1, 1000);       //Reading the one byte received
	}
	else
	{
		HAL_SPI_Transmit(&EL_hspi2, &reg, 1, 1000);           //Sending the read command and register address (000AAAAA)
		HAL_SPI_Receive(&EL_hspi2, &data, 1, 1000);           //Reading the one byte received
	}
	nRF24_CSN_UnSelect(Sel_CSN_Pin);                          //Pull CSN pin high to unselect the nRF24 on SPI BUS

	return data;
}

// Reads multiple bytes starting from a particular register
// input:
//   reg - number of register to write
//   data - pointer to the buffer with data to write
//   size - number of bytes to write
void nRF24_ReadMBReg(uint16_t Sel_CSN_Pin, uint8_t reg, uint8_t *data, uint8_t size)
{

	nRF24_CSN_Select(Sel_CSN_Pin);                            //Pull CSN pin low to select the nRF24 on SPI BUS
	if(Sel_CSN_Pin != Sel_EL_Module)
	{
		HAL_SPI_Transmit(&Button_hspi1, &reg, 1, 1000);       //Sending the read command and register address (000AAAAA)
		HAL_SPI_Receive(&Button_hspi1, data, size, 1000);     //Reading the received bytes

	}
	else
	{
		HAL_SPI_Transmit(&EL_hspi2, &reg, 1, 1000);           //Sending the read command and register address (000AAAAA)
		HAL_SPI_Receive(&EL_hspi2, data, size, 1000);         //Reading the received bytes
	}
	nRF24_CSN_UnSelect(Sel_CSN_Pin);		                  //Pull CSN pin low to select the nRF24 on SPI BUS

}

// Send the command to the nRF24
// input:
//   Sel_CSN_Pin - Select the RF Module
//   cmd - Command based on the nRF24l01 Command register
void nRF24_SendCMD(uint16_t Sel_CSN_Pin, uint8_t cmd)
{

	nRF24_CSN_Select(Sel_CSN_Pin);                      //Pull CSN pin low to select the nRF24 on SPI BUS
	if(Sel_CSN_Pin != Sel_EL_Module)
	{
		HAL_SPI_Transmit(&Button_hspi1, &cmd, 1, 1000);       //Transmit the command
	}
	else
	{
		HAL_SPI_Transmit(&EL_hspi2, &cmd, 1, 1000);           //Transmit the command
	}
	nRF24_CSN_UnSelect(Sel_CSN_Pin);		              //Pull CSN pin low to select the nRF24 on SPI BUS
}

// Reset all the nRF24's registers
// input:
//   Sel_CSN_Pin - Select the RF Module
//   reg - number of register to write
void nRF24_Reset(uint16_t Sel_CSN_Pin, uint8_t Reg)
{

		if(Reg == nRF24_REG_STATUS)
		{
			nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_STATUS, 0x70);
		}
		else if(Reg == nRF24_REG_FIFO_STATUS)
		{
			nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_FIFO_STATUS, 0x11);
		}
}


void nRF24_Module_Setup(uint16_t Sel_CSN_Pin, uint8_t mode, uint8_t RF_Ch, uint8_t *Tx_Addr, uint8_t *Rx_Addr, uint8_t RF_Power_Setup, uint8_t Payloadsize)
{
	/*Pipe0_Addr[4] = Cat_Mod_Pipe[0][0][0];
	Pipe1_Addr[4] = Cat_Mod_Pipe[0][0][1];
	Pipe2_Addr = Cat_Mod_Pipe[0][0][2];
	Pipe3_Addr = Cat_Mod_Pipe[0][0][3];
	Pipe4_Addr = Cat_Mod_Pipe[0][0][4];
	Pipe5_Addr = Cat_Mod_Pipe[0][0][5];*/

	uint8_t rx_addr_p0[5],rx_addr_p1[5],rx_addr_p2,rx_addr_p3,rx_addr_p4,rx_addr_p5;
	rx_addr_p0[0] = *Rx_Addr;
	rx_addr_p0[1] = *(Rx_Addr+1);
	rx_addr_p0[2] = *(Rx_Addr+2);
	rx_addr_p0[3] = *(Rx_Addr+3);
	rx_addr_p0[4] = *(Rx_Addr+4);
	rx_addr_p1[0] = (*Rx_Addr)+1;
	rx_addr_p1[1] = *(Rx_Addr+1);
	rx_addr_p1[2] = *(Rx_Addr+2);
	rx_addr_p1[3] = *(Rx_Addr+3);
	rx_addr_p1[4] = *(Rx_Addr+4);
	rx_addr_p2 = rx_addr_p1[0]+1;
	rx_addr_p3 = rx_addr_p2+1;
	rx_addr_p4 = rx_addr_p3+1;
	rx_addr_p5 = rx_addr_p4+1;

	switch(mode)
	{
    case Receiver_Mode_AA:
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_CONFIG, 0x09);                      //PRIM_RX=1, PWR_UP=0, EN_CRC=1
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_AA, 0x3F);                       //Enable AA for all pipes
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_RXADDR, 0x3F);                   //Enable RX addresses for all pipes
		break;
    case Receiver_Mode_noAA_CRC:
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_CONFIG, 0x09);                      //PRIM_RX=1, PWR_UP=0, EN_CRC=1
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_AA, 0x00);                       //Disable AA for all pipes
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_RXADDR, 0x3F);                   //Enable RX addresses for all pipes
		break;
    case Receiver_Mode_noAA_noCRC:
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_CONFIG, 0x01);                      //PRIM_RX=1, PWR_UP=0, EN_CRC=0
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_AA, 0x00);                       //Disable AA for all pipes
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_RXADDR, 0x3F);                   //Enable RX addresses for all pipes
		break;
    case Transmitter_Mode_AA:
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_CONFIG, 0x08);                      //PRIM_RX=0, PWR_UP=0, EN_CRC=1
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_AA, 0x3F);                       //Enable AA for all pipes
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_RXADDR, 0x01);                   //Enable RX addresses for pipe0
		break;
    case Transmitter_Mode_noAA_CRC:
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_CONFIG, 0x08);                      //PRIM_RX=0, PWR_UP=0, EN_CRC=1
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_AA, 0x00);                       //Disable AA for all pipes
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_RXADDR, 0x01);                   //Enable RX addresses for pipe0
		break;
    case Transmitter_Mode_noAA_noCRC:
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_CONFIG, 0x00);                      //PRIM_RX=0, PWR_UP=0, EN_CRC=0
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_AA, 0x00);                       //Disable AA for all pipes
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_EN_RXADDR, 0x01);                   //Enable RX addresses for pipe0
		break;
	}

	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_SETUP_AW, 0x03);                        //5 Bytes Length Address
	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_SETUP_RETR, 0x8A);                      //ARD=2000us, ARC=10
	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RF_CH, RF_Ch);                          //Set the Frequency Channel to 2

	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RF_SETUP, RF_Power_Setup);              //Power in dBm, Speed in Mbps, LNA_HCURR=1

	if((mode==Transmitter_Mode_AA)||(mode==Transmitter_Mode_noAA_CRC)||(mode==Transmitter_Mode_noAA_noCRC))
	{
		nRF24_WriteMBReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P0, Tx_Addr, 5);          //Set the Pipe0 address to {0x53, 0x52, 0x36, 0x34, 0x00}
		nRF24_WriteMBReg(Sel_CSN_Pin, nRF24_REG_TX_ADDR, Tx_Addr, 5);             //Set the TX address to {0x53, 0x52, 0x36, 0x34, 0x00}
	}
	else if((mode==Receiver_Mode_AA)||(mode==Receiver_Mode_noAA_CRC)||(mode==Receiver_Mode_noAA_noCRC))
	{
		nRF24_WriteMBReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P0, rx_addr_p0, 5);       //Set the Pipe0 address
		nRF24_WriteMBReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P1, rx_addr_p1, 5);       //Set the Pipe1 address
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P2, rx_addr_p2);            //Set the Pipe2 address
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P3, rx_addr_p3);            //Set the Pipe3 address
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P4, rx_addr_p4);            //Set the Pipe4 address
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P5, rx_addr_p5);            //Set the Pipe5 address
		nRF24_WriteMBReg(Sel_CSN_Pin, nRF24_REG_TX_ADDR, Tx_Addr, 5);             //Set the TX address
	}

	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RX_PW_P0, Payloadsize);                 //Set the Pipe0 Payload size
	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RX_PW_P1, Payloadsize);                 //Set the Pipe1 Payload size
	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RX_PW_P2, Payloadsize);                 //Set the Pipe2 Payload size
	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RX_PW_P3, Payloadsize);                 //Set the Pipe3 Payload size
	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RX_PW_P4, Payloadsize);                 //Set the Pipe4 Payload size
	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_RX_PW_P5, Payloadsize);                 //Set the Pipe5 Payload size
}


uint8_t nRF24_Transmit(uint16_t Sel_CSN_Pin, uint8_t *data, uint8_t Payload_Size)
{
	uint8_t cmdtosend = 0;
	uint8_t Temp_Register;
    uint8_t Tr_Success = 0;

	nRF24_CSN_Select(Sel_CSN_Pin);                                                      //Pull CSN pin low to select the module1 on SPI BUS


	cmdtosend = nRF24_CMD_W_TX_PAYLOAD;                                                 //Payload transmit command
	if(Sel_CSN_Pin != Sel_EL_Module)
	{
		HAL_SPI_Transmit(&Button_hspi1, &cmdtosend, 1, 100);
		HAL_SPI_Transmit(&Button_hspi1, data, Payload_Size, 100);
	}
	else
	{
		HAL_SPI_Transmit(&EL_hspi2, &cmdtosend, 1, 100);
		HAL_SPI_Transmit(&EL_hspi2, data, Payload_Size, 100);
	}
	nRF24_CE_Enable(Sel_CSN_Pin);                                                       //Enable the module1
	HAL_Delay(1);
	nRF24_CE_Disable(Sel_CSN_Pin);
	Temp_Register = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_STATUS);                       //Reading the STATUS register
	Temp_Register = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_STATUS);                       //Reading the STATUS register
	if((Temp_Register & (1<<5)) != 0)                                                   //check TX_EMPTY flag to be 1 along with Reserved pin to be 0
	{
#ifdef Debug
		HAL_UART_Transmit(&huart2, (uint8_t *)"\nData Transmitted Successfully\n",31,100);
#endif
		//Remove the interrupt pin
		Temp_Register = Temp_Register | (1<<5);                                         //Write 1 to TX_DS to reset it
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_STATUS, Temp_Register);
		Tr_Success = 1;
	}
	if((Temp_Register & (1<<4)) != 0)                                                   //check TX_EMPTY flag to be 1 along with Reserved pin to be 0
	{
#ifdef Debug
		HAL_UART_Transmit(&huart2, (uint8_t *)"Maximum Retry Reached\n",22,100);
#endif
		//Remove the interrupt pin
		Temp_Register = Temp_Register | (1<<4);                                         //Write 1 to MAX_RT to reset it
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_STATUS, Temp_Register);
	}

	//nRF24_SendCMD(Sel_CSN_Pin, nRF24_CMD_FLUSH_TX);

	nRF24_CSN_UnSelect(Sel_CSN_Pin);
	return Tr_Success;
}


uint8_t isDataAvailable(uint16_t Sel_CSN_Pin)
{
	uint8_t STATUS_Register = 0;
	uint8_t Receiving_Pipe = 255;

	nRF24_CSN_Select(Sel_CSN_Pin);                                       //Pull CSN pin low to select the module1 on SPI BUS

	STATUS_Register = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_STATUS);               //Reading the status register
	Receiving_Pipe = STATUS_Register & 0x0E;                                //Reading the bits 1,2 and 3 of the status for Pipe number
	Receiving_Pipe = Receiving_Pipe >> 1;                                  //Shift the bits to right to derive the pipe number
    if((STATUS_Register & (1<<6)) != 0)
	{
       	STATUS_Register = STATUS_Register | (1<<6);
       	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_STATUS, STATUS_Register);           //Writing 1 to RX_DR to clear bit
		nRF24_CSN_UnSelect(Sel_CSN_Pin);                                 //Release the CSN pin to unselect the module1 on SPI BUS
	}

    nRF24_CSN_UnSelect(Sel_CSN_Pin);                                     //Release the CSN pin to unselect the module1 on SPI BUS
    return Receiving_Pipe;
}

uint8_t isDataAvailable_it(uint16_t Sel_CSN_Pin)
{
	uint8_t STATUS_Register = 0;
	uint8_t Receiving_Pipe = 255;

	nRF24_CSN_Select(Sel_CSN_Pin);                                       //Pull CSN pin low to select the module1 on SPI BUS

	STATUS_Register = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_STATUS);               //Reading the status register
	Receiving_Pipe = STATUS_Register & 0x0E;                                //Reading the bits 1,2 and 3 of the status for Pipe number
	Receiving_Pipe = Receiving_Pipe >> 1;                                  //Shift the bits to right to derive the pipe number
    if((STATUS_Register & (1<<6)) != 0)
	{
       	STATUS_Register = STATUS_Register | (1<<6);
       	nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_STATUS, STATUS_Register);           //Writing 1 to RX_DR to clear bit
		nRF24_CSN_UnSelect(Sel_CSN_Pin);                                 //Release the CSN pin to unselect the module1 on SPI BUS
	}

    nRF24_CSN_UnSelect(Sel_CSN_Pin);                                     //Release the CSN pin to unselect the module1 on SPI BUS
    return Receiving_Pipe;
}


void nRF24_Receive(uint16_t Sel_CSN_Pin, uint8_t *data, uint8_t Payload_Size)
{
	uint8_t cmdtosend = 0;
	uint8_t Temp_Register = 0;

	if(Sel_CSN_Pin != Sel_EL_Module)
	{
		nRF24_CSN_Select(Sel_CSN_Pin);                                                    //Pull CSN pin low to select the module1 on SPI BUS
		cmdtosend = nRF24_CMD_R_RX_PAYLOAD;                                                 //Payload transmit command
		HAL_SPI_Transmit(&Button_hspi1, &cmdtosend, 1, 100);
		HAL_SPI_Receive(&Button_hspi1, data, Payload_Size, 100);                            //Receive the payload
		HAL_Delay(1);
		Temp_Register = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_STATUS);
		Temp_Register = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_STATUS);
		if((Temp_Register & (1<<6)) != 0)                                                   //check TX_DR flag to reset the interrupt pin
		{
#ifdef Debug
			HAL_UART_Transmit(&huart2, (uint8_t *)"\nData Received in the RX FIFO\n",31,100);
#endif
			//Remove the interrupt pin
			Temp_Register = Temp_Register | (1<<6);                                         //Write 1 to TX_DR to reset interrupt pin
			nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_STATUS, Temp_Register);
		}
#ifndef ELBC
		HAL_Delay(1);
		cmdtosend = nRF24_CMD_FLUSH_RX;                                                     //Flush the RX Buffer
		nRF24_SendCMD(Sel_CSN_Pin, cmdtosend);
#endif
		nRF24_CSN_UnSelect(Sel_CSN_Pin);
	}
	else
	{
		nRF24_CSN_Select(Sel_CSN_Pin);                                       //Pull CSN pin low to select the EdgeLED Module on SPI BUS
		cmdtosend = nRF24_CMD_R_RX_PAYLOAD;                                    //Payload transmit command
		HAL_SPI_Transmit(&EL_hspi2, &cmdtosend, 1, 100);
		HAL_SPI_Receive(&EL_hspi2, data, Payload_Size, 100);                             //Receive the payload
		Temp_Register = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_STATUS);
		if((Temp_Register & (1<<6)) != 0)                                                   //check TX_DR flag to reset the interrupt pin
		{
#ifdef Debug
			HAL_UART_Transmit(&huart2, (uint8_t *)"\nData Received in the RX FIFO\n",31,100);
#endif
			//Remove the interrupt pin
			Temp_Register = Temp_Register | (1<<6);                                         //Write 1 to TX_DR to reset interrupt pin
			nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_STATUS, Temp_Register);
		}
#ifndef ELBC
		HAL_Delay(1);
	    cmdtosend = nRF24_CMD_FLUSH_RX;                                        //Flush the RX Buffer
		nRF24_SendCMD(Sel_EL_Module, cmdtosend);
#endif
		nRF24_CSN_UnSelect(Sel_CSN_Pin);                                     //Release the CSN pin to unselect the EdgeLED Module on SPI BUS
		//Release the CSN pin to unselect the module1 on SPI BUS
	}
}


// Read all the Register data
void nRF24_ReadAllRegs (uint16_t Sel_CSN_Pin, uint8_t *data)
{
	for (int i=0; i<10; i++)
	{
		data[i] = nRF24_ReadReg(Sel_CSN_Pin, i);
	}

	nRF24_ReadMBReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P0, &data[10], 5);
	nRF24_ReadMBReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P1, &data[15], 5);
	data[20] = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P2);
	data[21] = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P3);
	data[22] = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P4);
	data[23] = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_RX_ADDR_P5);
	nRF24_ReadMBReg(Sel_CSN_Pin, nRF24_REG_TX_ADDR, &data[24], 5);

	for (int i=29; i<38; i++)
	{
		data[i] = nRF24_ReadReg(Sel_CSN_Pin, (i-12));
	}
}


void nRF24_Power(uint16_t Sel_CSN_Pin, uint8_t On_Off_Cmd)
{
	uint8_t Temp_Reg;

	Temp_Reg = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_CONFIG);

	if(On_Off_Cmd == Power_Up)
	{
		Temp_Reg = Temp_Reg | (1<<1);
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_CONFIG, Temp_Reg);
	}
	else if(On_Off_Cmd == Power_Down)
	{
		Temp_Reg = Temp_Reg & 0x7D;
		nRF24_WriteReg(Sel_CSN_Pin, nRF24_REG_CONFIG, Temp_Reg);
	}
}


void nRF24_Transmit_Report(uint16_t Sel_CSN_Pin)
{
	uint8_t Temp_reg,Temp_reg2;

	Temp_reg = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_FIFO_STATUS);
	HAL_UART_Transmit(&huart2, (uint8_t*)"Transmit Report\n",16, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"**********************\n",23, 100);
	HAL_UART_Transmit(&huart2, (uint8_t*)"FIFO STATUS Register: ",22, 100);
	nRF24_sendRegisterValue(Temp_reg);

	if((Temp_reg & (1<<4)) != 0)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"TX FIFO empty\n",14, 100);
	} else
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"Data in TX FIFO\n",16, 100);
	}
	Temp_reg = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_OBSERVE_TX);
	HAL_UART_Transmit(&huart2, (uint8_t*)"OBSERVE TX Register: ",21, 100);
	nRF24_sendRegisterValue(Temp_reg);
	Temp_reg2 = Temp_reg>>4;
	HAL_UART_Transmit(&huart2, (uint8_t*)"PLOS Count: ",12, 100);
	nRF24_sendRegisterValue(Temp_reg2);
	Temp_reg = Temp_reg & 0x0F;
	HAL_UART_Transmit(&huart2, (uint8_t*)"ARC Count: ",11, 100);
	nRF24_sendRegisterValue(Temp_reg);
	Temp_reg = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_STATUS);
	HAL_UART_Transmit(&huart2, (uint8_t*)"STATUS Register: ",17, 100);
	nRF24_sendRegisterValue(Temp_reg);
	if((Temp_reg & (1<<0)) != 0)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"TX FIFO Full\n",13, 100);
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)"**********************\n",23, 100);

}

void nRF24_Receive_Report(uint16_t Sel_CSN_Pin)
{
	uint8_t FIFO_reg,STATUS_reg;

	HAL_UART_Transmit(&huart2, (uint8_t*)"**********************\n",23, 100);

	switch(Sel_CSN_Pin)
	{
	case Sel_B_Module1:
		HAL_UART_Transmit(&huart2, (uint8_t*)"Module1 receive report\n",23, 100);
		break;
	case Sel_B_Module2:
		HAL_UART_Transmit(&huart2, (uint8_t*)"Module2 receive report\n",23, 100);
		break;
	case Sel_B_Module3:
		HAL_UART_Transmit(&huart2, (uint8_t*)"Module3 receive report\n",23, 100);
		break;
	case Sel_B_Module4:
		HAL_UART_Transmit(&huart2, (uint8_t*)"Module4 receive report\n",23, 100);
		break;
	case Sel_EL_Module:
		HAL_UART_Transmit(&huart2, (uint8_t*)"EdgeLED Module receive report\n",30, 100);
		break;
	default:
		break;
	}

	FIFO_reg = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_FIFO_STATUS);
	STATUS_reg = nRF24_ReadReg(Sel_CSN_Pin, nRF24_REG_STATUS);

	HAL_UART_Transmit(&huart2, (uint8_t*)"FIFO STATUS Register: ",22, 100);
	nRF24_sendRegisterValue(FIFO_reg);
	if((FIFO_reg & (1<<0)) != 0)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"RX FIFO empty\n",14, 100);
	} else
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"Data in RX FIFO\n",16, 100);
	}
	if((FIFO_reg & (1<<1)) != 0)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"RX FIFO Full\n",13, 100);
	} else
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"Available locations in RX FIFO\n",31, 100);
	}


	HAL_UART_Transmit(&huart2, (uint8_t*)"STATUS Register: ",17, 100);
	nRF24_sendRegisterValue(STATUS_reg);
	if((STATUS_reg & (1<<6)) != 0)
	{
		HAL_UART_Transmit(&huart2, (uint8_t*)"Data Ready in RX FIFO\n\n",14, 100);
	}
	STATUS_reg = (STATUS_reg>>1) & 0x07;
	HAL_UART_Transmit(&huart2, (uint8_t*)"Data in Pipe Number: ",21, 100);
	nRF24_sendRegisterValue(STATUS_reg);

}


void nRF24_Register_Display(uint16_t Sel_CSN_Pin)
{
	uint8_t Reg_Data[40];

	nRF24_ReadAllRegs(Sel_CSN_Pin, Reg_Data);
	switch(Sel_CSN_Pin)
	{
	case Sel_B_Module1:
		HAL_UART_Transmit(&huart2, (uint8_t*)"Module1 Registers\n",18, HAL_MAX_DELAY);
		break;
	case Sel_B_Module2:
		HAL_UART_Transmit(&huart2, (uint8_t*)"Module2 Registers\n",18, HAL_MAX_DELAY);
		break;
	case Sel_B_Module3:
		HAL_UART_Transmit(&huart2, (uint8_t*)"Module3 Registers\n",18, HAL_MAX_DELAY);
		break;
	case Sel_B_Module4:
		HAL_UART_Transmit(&huart2, (uint8_t*)"Module4 Registers\n",18, HAL_MAX_DELAY);
		break;
	case Sel_EL_Module:
		HAL_UART_Transmit(&huart2, (uint8_t*)"EdgeLED Module Registers\n",25, HAL_MAX_DELAY);
		break;
	}
	HAL_UART_Transmit(&huart2, (uint8_t*)"******************\n",19, HAL_MAX_DELAY);

	HAL_UART_Transmit(&huart2, (uint8_t*)"CONFIG:",7, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[0]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"EN_AA_reg:",10, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[1]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"EN_RXADR_reg:",13, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[2]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"SETUP_AW_reg:",13, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[3]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"SETUP_RETR_reg:",15, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[4]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RF_CH_reg:",10, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[5]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RF_SETUP_reg:",13, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[6]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RF_STATUS_reg:",14, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[7]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"OBSERVE_TX_reg:",15, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[8]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_ADR_P0_reg:\n",15, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[10]);
	nRF24_sendRegisterValue(Reg_Data[11]);
	nRF24_sendRegisterValue(Reg_Data[12]);
	nRF24_sendRegisterValue(Reg_Data[13]);
	nRF24_sendRegisterValue(Reg_Data[14]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_ADR_P1_reg:\n",15, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[15]);
	nRF24_sendRegisterValue(Reg_Data[16]);
	nRF24_sendRegisterValue(Reg_Data[17]);
	nRF24_sendRegisterValue(Reg_Data[18]);
	nRF24_sendRegisterValue(Reg_Data[19]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_ADR_P2_reg:",15, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[20]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_ADR_P3_reg:",15, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[21]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_ADR_P4_reg:",15, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[22]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_ADR_P5_reg:",15, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[23]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"TX_ADR_reg:\n",12, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[24]);
	nRF24_sendRegisterValue(Reg_Data[25]);
	nRF24_sendRegisterValue(Reg_Data[26]);
	nRF24_sendRegisterValue(Reg_Data[27]);
	nRF24_sendRegisterValue(Reg_Data[28]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_PW_P0_reg:",13, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[29]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_PW_P1_reg:",13, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[30]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_PW_P2_reg:",13, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[31]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_PW_P3_reg:",13, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[32]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_PW_P4_reg:",13, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[33]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"RX_PW_P5_reg:",13, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[34]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"FIFO_STATUS_reg:",16, HAL_MAX_DELAY);
	nRF24_sendRegisterValue(Reg_Data[35]);
	HAL_UART_Transmit(&huart2, (uint8_t*)"******************\n\n",20, HAL_MAX_DELAY);
}

void nRF24_sendRegisterValue(uint8_t value)
{
  char buffer[10];
  sprintf(buffer, "0x%02X\r\n", value); // Convert value to hexadecimal string
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
}


