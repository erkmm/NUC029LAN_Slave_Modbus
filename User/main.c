/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 14/10/01 10:36a $
 * @brief    Template project for NUC029 series MCU
 *
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NUC029xAN.h"
#include "modbus_crc.h"
#include "modbusSlave.h"

#define BUFSIZE 256

unsigned char RxData[BUFSIZE]= {0};
unsigned char TxData[BUFSIZE];
//unsigned char Txli[8]={0x01, 0x03, 0x02, 0x12, 0x34, 0x33, 0xB5};
volatile uint32_t g_u32comRbytes = 0;
volatile uint32_t g_u32comRhead  = 0;
volatile uint32_t g_u32comRtail  = 0;


void UART0_Init() {
	/*---------------------------------------------------------------------------------------------------------*/
	/* Init UART                                                                                               */
	/*---------------------------------------------------------------------------------------------------------*/
	SYS->IPRSTC2 |= SYS_IPRSTC2_UART0_RST_Msk;
	SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

	/* Configure UART0 and set UART0 Baudrate */
    UART0->FUN_SEL = UART_FUNC_SEL_UART;
	UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
	UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
	UART0->FCR = UART_FCR_RFITL_8BYTES | UART_FCR_RTS_TRI_LEV_8BYTES;
//	UART0->FCR = UART0->FCR = (UART0->FCR & (~UART_FCR_RFITL_Msk)) | UART_FCR_RFITL_8BYTES;
	UART_SetTimeoutCnt(UART0,60);
	UART0->IER = UART_IER_RDA_IEN_Msk|UART_IER_THRE_IEN_Msk|UART_IER_RTO_IEN_Msk|UART_IER_TIME_OUT_EN_Msk;
	NVIC_EnableIRQ(UART0_IRQn);
}

void Timer_Init(void){
	TIMER0->TCSR = 0
					| TIMER_TCSR_IE_Msk
					| TIMER_PERIODIC_MODE
					| TIMER_TCSR_CEN_Msk
					| (25 << TIMER_TCSR_PRESCALE_Pos);
	TIMER0->TCMPR = 20*86400; // 10ms period
	NVIC_EnableIRQ(TMR0_IRQn);
}

void TMR0_IRQHandler(void) {
	if ((TIMER0->TISR & (1 << 0) ? 1 : 0) == 1) {
		/* Clear Timer0 time-out interrupt flag */
		TIMER0->TISR |= (1 << 0);
		//readHoldingRegs();

		UART_Write(UART0, RxData, 8);
		}
	}

void UART0_IRQHandler(void) {
	uint8_t u8InChar = 0xFF;

	if (UART0->ISR & UART_ISR_RDA_INT_Msk) {

		while (UART_IS_RX_READY(UART0)) {

			// while(UART_GET_RX_EMPTY(UART0) == 0) {
			/* Get the character from UART Buffer */
			u8InChar = UART0->RBR; /* Get Data from UART RX  */
			printf("%X",u8InChar);

			/* Check if buffer full */
			if (g_u32comRbytes < BUFSIZE) {
				/* Enqueue the character */
				RxData[g_u32comRtail] = u8InChar;
				g_u32comRtail =(g_u32comRtail == (BUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
				g_u32comRbytes++;
			}
		}
	} //}

	if (UART0->ISR & UART_ISR_TOUT_INT_Msk) {

		while (UART_GET_RX_EMPTY(UART0) == 0) {

			u8InChar = UART0->RBR; /* Get Data from UART RX  */
			printf("%X",u8InChar);

			/* Check if buffer full */
			if (g_u32comRbytes < BUFSIZE) {
				/* Enqueue the character */
				RxData[g_u32comRtail] = u8InChar;
				g_u32comRtail =(g_u32comRtail == (BUFSIZE - 1)) ? 0 : (g_u32comRtail + 1);
				g_u32comRbytes++;
		}
	}
 }

    if(UART0->ISR & UART_ISR_THRE_INT_Msk) {
        uint16_t tmp;
        tmp = g_u32comRtail;
        if(g_u32comRhead != tmp) {
            u8InChar = RxData[g_u32comRhead];
            UART_WRITE(UART0, u8InChar);
            g_u32comRhead = (g_u32comRhead == (BUFSIZE - 1)) ? 0 : (g_u32comRhead + 1);
            g_u32comRbytes--;
        }
    }

}


void SYS_Init(void) {
	/* Unlock protected registers */
	SYS_UnlockReg();

	/* Enable IP clock */
	CLK->APBCLK = 0
	        |CLK_APBCLK_TMR0_EN_Msk
			|CLK_APBCLK_UART0_EN_Msk;

	/* Update System Core Clock */
	/* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and cyclesPerUs automatically. */
	SystemCoreClockUpdate();

	/* Set P3 multi-function pins for UART0 RXD and TXD  */
	SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
	SYS->P3_MFP |= (SYS_MFP_P30_RXD0 | SYS_MFP_P31_TXD0);

	/* Lock protected registers */
	SYS_LockReg();
}

int main() {
	SYS_Init();

	/* Init UART0 to 115200-8n1 for print message */
	UART0_Init();
	Timer_Init();

	while (1){
//		UART_Write(UART0,RxData + g_u32comRbytes,8);
//		for (volatile int time = 0; time < 0x10000; time++){}
		//	if (RxData[0] == SLAVE_ID) {
		//		switch (RxData[1]) {
		//		case 0x03: {
		//			readHoldingRegs();
		//		}
		//			break;
		//		default:
		//			modbusException(ILLEGAL_FUNCTION);
		//		}
		//	} else {
		//		modbusException(ILLEGAL_FUNCTION);
		//	}
		 //UART_Write(UART0,RxData,8);



		//for (volatile int time = 0; time < 0x10000; time++){}
	}

}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
