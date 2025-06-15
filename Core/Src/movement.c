/*
 * Movement.c
 *
 *  Created on: May 16, 2025
 *      Author: garre
 */

#include <movement.h>
#include <memory.h>
#include "support.h"
#include "nrf24.h"
#include "usb_device.h"

uint8_t val_X;
uint8_t val_Y;
uint8_t val_S;
uint8_t dir_R;
uint32_t wd = 0;
uint32_t blink = 0;

#define HEX_CHARS      "0123456789ABCDEF"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim11;

extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);



uint8_t nRF24_payload[32];

// Pipe number
nRF24_RXResult pipe;

uint32_t i, j, k;

void Toggle_LED()
{
    HAL_GPIO_TogglePin(LD2_GPIO_Port,LD2_Pin);
}

// Length of received pay_load
uint8_t payload_length;

// Helpers for transmit mode demo

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT         (uint32_t)0x000FFFFF

// Result of packet transmission
typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

nRF24_TXResult tx_res;

// Function to transmit data packet
// input:
//   pBuf - pointer to the buffer with data to transmit
//   length - length of the data buffer in bytes
// return: one of nRF24_TX_xx values
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length)
{
	volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
	uint8_t status;

	// Deassert the CE pin (in case if it still high)
	nRF24_CE_L();

	// Transfer a data from the specified buffer to the TX FIFO
	nRF24_WritePayload(pBuf, length);

	// Start a transmission by asserting CE pin (must be held at least 10us)
	nRF24_CE_H();

	// Poll the transceiver status register until one of the following flags will be set:
	//   TX_DS  - means the packet has been transmitted
	//   MAX_RT - means the maximum number of TX retransmits happened
	// note: this solution is far from perfect, better to use IRQ instead of polling the status
	do {
		status = nRF24_GetStatus();
		if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT)) {
			break;
		}
	} while (wait--);

	// Deassert the CE pin (Standby-II --> Standby-I)
	nRF24_CE_L();

	if (!wait) {
		// Timeout
		return nRF24_TX_TIMEOUT;
	}


	// Clear pending IRQ flags
    nRF24_ClearIRQFlags();

	if (status & nRF24_FLAG_MAX_RT) {
		// Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
		return nRF24_TX_MAXRT;
	}

	if (status & nRF24_FLAG_TX_DS) {
		// Successful transmission
		return nRF24_TX_SUCCESS;
	}

	// Some banana happens, a payload remains in the TX FIFO, flush it
	nRF24_FlushTX();

	return nRF24_TX_ERROR;
}

void send_payload(uint8_t* payload, uint8_t length)
{

    // Set operational mode (PTX == transmitter)
    nRF24_SetOperationalMode(nRF24_MODE_TX);

    // Clear any pending IRQ flags
    nRF24_ClearIRQFlags();


	// Transmit a packet
	tx_res = nRF24_TransmitPacket(payload, length);
	switch (tx_res) {
		case nRF24_TX_SUCCESS:

			break;
		case nRF24_TX_TIMEOUT:

			break;
		case nRF24_TX_MAXRT:

			break;
		default:

			break;
	}

	HAL_Delay(5);
    // Set operational mode (PRX == receiver)
    nRF24_SetOperationalMode(nRF24_MODE_RX);


    // Put the transceiver to the RX mode
    nRF24_CE_H();
}

// this fonction check the dipswitch to select a channel when the robot start.
uint8_t channel(void)
{
	uint8_t channel = 115;
	uint8_t set_channel = 0;
	if(HAL_GPIO_ReadPin (DSW_0_GPIO_Port, DSW_0_Pin)) set_channel += 1;
	if(HAL_GPIO_ReadPin (DSW_1_GPIO_Port, DSW_1_Pin)) set_channel += 2;
	if(HAL_GPIO_ReadPin (DSW_2_GPIO_Port, DSW_2_Pin)) set_channel += 4;
	if(HAL_GPIO_ReadPin (DSW_3_GPIO_Port, DSW_3_Pin)) set_channel += 8;

	switch(set_channel)
	{
	case 0 :
		channel = 115;
		break;
	case 1 :
		channel = 10;
		break;
	case 2 :
		channel = 20;
		break;
	case 3 :
		channel = 30;
		break;
	case 4 :
		channel = 40;
		break;
	case 5 :
		channel = 50;
		break;
	case 6 :
		channel = 60;
		break;
	case 7 :
		channel = 70;
		break;
	case 8 :
		channel = 80;
		break;
	case 9 :
		channel = 90;
		break;
	case 10 :
		channel = 100;
		break;
	case 11 :
		channel = 110;
		break;
	case 12 :
		channel = 120;
		break;
	case 13 :
		channel = 125;
		break;
	case 14 :
		channel = 127;
		break;
	case 15 :
		channel = 115;
		break;
	}

	return channel;
}

void movement(void)
{
	HAL_TIM_PWM_Start( &htim1,TIM_CHANNEL_1 );
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,1500);

	HAL_TIM_PWM_Start( &htim2,TIM_CHANNEL_1 );
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,1500);

	HAL_TIM_PWM_Start( &htim11,TIM_CHANNEL_1 );
	__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1,1500);

	HAL_Delay(500);
	// RX/TX disabled
	nRF24_CE_L();

	// Configure the nRF24L01+

	HAL_Delay(500);
	if (!nRF24_Check())
	{

		while (1)
		{
			Toggle_LED();
			HAL_Delay(100);
			if (nRF24_Check()) break;
		}
	}




	// Initialize the nRF24L01 to its default state
	nRF24_Init();




	// This is simple receiver/transmitter :
	//   - pipe#1 address	: '0xE7 0x1C 0xE4'
	//   - TX address		: '0xE7 0x1C 0xE3'
	//   - payload			: 5 bytes
	//   - RF channel		: 115 (2515MHz)
	//   - data rate		: 250kbps (minimum possible, to increase reception reliability)
	//   - CRC scheme		: 2 byte

	// The transmitter sends a 5-byte packets to the address '0xE7 0x1C 0xE3' without Auto-ACK (ShockBurst disabled)

	// Disable ShockBurst for all RX pipes
	nRF24_DisableAA(0xFF);

	// Set RF channel
	nRF24_SetRFChannel(channel());

	// Set data rate
	nRF24_SetDataRate(nRF24_DR_250kbps);

	// Set CRC scheme
	nRF24_SetCRCScheme(nRF24_CRC_2byte);

	// Set address width, its common for all pipes (RX and TX)
	nRF24_SetAddrWidth(3);

	// Configure RX PIPE#1
	static const uint8_t nRF24_ADDR_Rx[] = { 0xE7, 0x1C, 0xE3 };
	nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR_Rx); // program address for RX pipe #1
	nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_OFF, 5); // Auto-ACK: disabled, payload length: 5 bytes

	// Configure TX PIPE
	static const uint8_t nRF24_ADDR_Tx[] = { 0xE7, 0x1C, 0xE4 };
	nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR_Tx); // program TX address

	// Set TX power (maximum)
	nRF24_SetTXPower(nRF24_TXPWR_0dBm);

	// Set operational mode (PRX == receiver)
	nRF24_SetOperationalMode(nRF24_MODE_RX);

	// Wake the transceiver
	nRF24_SetPowerMode(nRF24_PWR_UP);

	// Put the transceiver to the RX mode
	nRF24_CE_H();

	// The main loop
	while (1)
	{
		HAL_Delay(2);
		// watch dog to make hall motor break if no comunication for 100 cycles
		if (++wd > 100)
		{
			// to make all motor break
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,1500);
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,1500);
			__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1,1500);

			HAL_Delay(100);

			// to stop all PWM
			HAL_TIM_PWM_Stop( &htim1,TIM_CHANNEL_1 );
			HAL_TIM_PWM_Stop( &htim2,TIM_CHANNEL_1 );
			HAL_TIM_PWM_Stop( &htim11,TIM_CHANNEL_1 );


			while (1)
			{
				Toggle_LED();
				HAL_Delay(100);
			}

		}

		//
		// Constantly poll the status of the RX FIFO and get a payload if FIFO is not empty
		//
		// This is far from best solution, but it's ok for testing purposes
		// More smart way is to use the IRQ pin :)
		//
		if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY)
		{
			wd = 0;

			if (++blink > 200)
			{
				Toggle_LED();
				blink = 0;
			}
			// Get a payload from the transceiver
			pipe = nRF24_ReadPayload(nRF24_payload, &payload_length);

			// Clear all pending IRQ flags
			nRF24_ClearIRQFlags();

			// Print a payload contents to UART


			HAL_Delay(2);
			uint8_t message[32] = {0xaa,0x44,0x11,0x22,0x55};
			send_payload(message, 5);
			//send_payload(nRF24_payload, payload_length);

			dir_R = nRF24_payload[0];
			val_S = nRF24_payload[1];
			val_X = nRF24_payload[2];
			val_Y = nRF24_payload[3];
		}

		if (dir_R == 0xaa)
		{
			// speed of the drum
			uint32_t speed_D = 1500 + val_S*500/256;
			if (speed_D > 2000) speed_D = 2000;

			//speed of the left wheel
			uint32_t speed_L = 1000 + val_Y*1000/256 - (128 - val_X)*500/128;
			if (speed_L < 1000) speed_L = 1000;
			else if (speed_L > 2000) speed_L = 2000;

			//speed of the right wheel
			uint32_t speed_R = 1000 + val_Y*1000/256 + (128 - val_X)*500/128;
			if (speed_R < 1000) speed_R = 1000;
			else if (speed_R > 2000) speed_R =2000;

			__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1,speed_D);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,speed_L);
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,speed_R);
		}
		else if (dir_R == 0xbb)
		{
			// speed of the drum
			uint32_t speed_D = 1500 - val_S*500/256;
			if (speed_D < 1000) speed_D = 1000;

			//speed of the left wheel
			uint32_t speed_L = 2000 - val_Y*1000/256 + (128 - val_X)*500/128;
			if (speed_L < 1000) speed_L = 1000;
			else if (speed_L > 2000) speed_L = 2000;

			//speed of the right wheel
			uint32_t speed_R = 2000 - val_Y*1000/256 - (128 - val_X)*500/128;
			if (speed_R < 1000) speed_R = 1000;
			else if (speed_R > 2000) speed_R =2000;

			__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1,speed_D);
			__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,speed_L);
			__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,speed_R);

		}
		else if (dir_R == 0xcc)
		{
			uint32_t wait = 4500;
			while(wait--)
			{

				__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,1500);
				__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1,1500);
				__HAL_TIM_SetCompare(&htim11, TIM_CHANNEL_1,1500);
				HAL_Delay(200);
				Toggle_LED();
			}
		}


	}

}
