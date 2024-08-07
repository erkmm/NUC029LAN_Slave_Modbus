

//void UART0_IRQHandler(void) {
//	if(!UART_GET_INT_FLAG(UART0, UART_ISR_TOUT_INT_Msk)){//time out oldu mu?
//	//if(UART0->ISR & UART_ISR_RDA_INT_Msk)
//	if (UART_GET_INT_FLAG(UART0, UART_ISR_RDA_INT_Msk)) {//threshold geldi
//		UART_Read(UART0, RxData,8);
////		while (!UART_GET_RX_EMPTY(UART0)) {//rx buf data var mi?
////			for(rxIndex = 0; rxIndex < 8; rxIndex++){
////			RxData[rxIndex] = UART0->RBR;}
//		}}}//}



#include <stdint.h>
#include "modbusSlave.h"
#include "NUC029xAN.h"
#define BUFSIZE 256
#include "modbus_crc.h"

extern uint8_t RxData[BUFSIZE];
extern uint8_t TxData[BUFSIZE];

void transmit_data(uint8_t *data, uint8_t size){
	// we will calculate the CRC in this function itself
	uint16_t crc = crc16(data, size);
	data[size] = crc&0xFF;   // CRC LOW
	data[size+1] = (crc>>8)&0xFF;  // CRC HIGH

	UART_Write(UART0, data, size+2);
	//HAL_UART_Transmit(&huart2, data, size+2, 1000);
}

void modbusException (uint8_t exceptioncode)
{
	//| SLAVE_ID | FUNCTION_CODE | Exception code | CRC     |
	//| 1 BYTE   |  1 BYTE       |    1 BYTE      | 2 BYTES |

	TxData[0] = RxData[0];       // slave ID
	TxData[1] = RxData[1]|0x80;  // adding 1 to the MSB of the function code
	TxData[2] = exceptioncode;   // Load the Exception code
	transmit_data(TxData, 3);    // send Data... CRC will be calculated in the function
}


uint8_t readHoldingRegs (void)
{
	uint16_t startAddr = ((RxData[2]<<8)|RxData[3]);  // start Register Address

	uint16_t numRegs = ((RxData[4]<<8)|RxData[5]);   // number to registers master has requested
	if ((numRegs<1)||(numRegs>125))  // maximum no. of Registers as per the PDF
	{
		modbusException (ILLEGAL_DATA_VALUE);  // send an exception
		return 0;
	}

	uint16_t endAddr = startAddr+numRegs-1;  // end Register
	if (endAddr>49)  // end Register can not be more than 49 as we only have record of 50 Registers in total
	{
		modbusException(ILLEGAL_DATA_ADDRESS);   // send an exception
		return 0;
	}
	// Prepare TxData buffer

	//| SLAVE_ID | FUNCTION_CODE | BYTE COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE    | N*2 BYTES | 2 BYTES |

	TxData[0] = SLAVE_ID;  // slave ID
	TxData[1] = RxData[1];  // function code
	TxData[2] = numRegs*2;  // Byte count
	uint8_t indx = 3;  // we need to keep track of how many bytes has been stored in TxData Buffer

	for (uint8_t i=0; i<numRegs; i++)   // Load the actual data into TxData buffer
	{
		TxData[indx++] = (Holding_Registers_Database[startAddr]>>8)&0xFF;  // extract the higher byte
		TxData[indx++] = (Holding_Registers_Database[startAddr])&0xFF;   // extract the lower byte
		startAddr++;  // increment the register address
	}

	transmit_data(TxData, indx);  // send data... CRC will be calculated in the function itself
	return 1;   // success
}
