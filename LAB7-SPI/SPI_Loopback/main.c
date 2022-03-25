/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/04/20 2:56p $
 * @brief
 *           Implement SPI Master loop back transfer.
 *           This sample code needs to connect SPI0_MISO0 pin and SPI0_MOSI0 pin together.
 *           It will compare the received data with transmitted data.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"

#define PLL_CLOCK           50000000

#define DATAX0      0x32
#define DATAX1		  0x33
#define DATAY0		  0x34
#define DATAY1      0x35
#define DATAZ0      0x36
#define DATAZ1      0x37

void SYS_Init(void)
{
    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Enable external 12 MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Select HXT as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Select HCLK as the clock source of SPI2 */
    CLK_SetModuleClock(SPI2_MODULE, CLK_CLKSEL1_SPI2_S_HCLK, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable SPI2 peripheral clock */
    CLK_EnableModuleClock(SPI2_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Setup SPI2 multi-function pins */
    SYS->GPD_MFP = SYS_GPD_MFP_PD0_SPI2_SS0 | SYS_GPD_MFP_PD1_SPI2_CLK | SYS_GPD_MFP_PD2_SPI2_MISO0 | SYS_GPD_MFP_PD3_SPI2_MOSI0;
    SYS->ALT_MFP = SYS_ALT_MFP_PD0_SPI2_SS0 | SYS_ALT_MFP_PD1_SPI2_CLK | SYS_ALT_MFP_PD2_SPI2_MISO0 | SYS_ALT_MFP_PD3_SPI2_MOSI0;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a master, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Set IP clock divider. SPI clock rate = 2 MHz */
    SPI_Open(SPI2, SPI_MASTER, SPI_MODE_3, 8, 1000000);

		SPI2->DIVIDER = (SPI2->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | 24;
    
		/* set SS at low level trigger */
    SPI2->SSR &= ~SPI_SSR_SS_LVL_Msk;

		/* disable the auto SS */
    SPI2->SSR &= ~SPI_SSR_AUTOSS_Msk;
		
		/* set SPI at master mode */
    SPI2->CNTRL |= SPI_MASTER;
		/* set clock idle state at high */
		SPI2->CNTRL |= SPI_CNTRL_CLKP_Msk;
		/* set transmitt data at falling edge, recieve data at rising edge */
    SPI2->CNTRL |= SPI_CNTRL_TX_NEG_Msk;
    SPI2->CNTRL &= ~SPI_CNTRL_RX_NEG_Msk;
		/* set bit length of word transfer */
		SPI2->CNTRL |= ((SPI2->CNTRL & ~SPI_CNTRL_TX_BIT_LEN_Msk) | (0x08 << SPI_CNTRL_TX_BIT_LEN_Pos));  
}

void SPI_Write(uint8_t reg, uint8_t data)
{
    SPI2->TX[0] = 0x00 | reg;
    
    SPI2->SSR |= SPI_SSR_SSR_Msk;                       //ss start
    
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;               //wait until transfer done
    while((SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) == 1);
    
    SPI2->TX[0] = data;                                 //send data
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;
    while((SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) == 1);

    SPI2->SSR &= ~SPI_SSR_SSR_Msk;                      //set ss high
}

uint8_t SPI_Read(uint8_t reg)
{
    uint8_t temp;
    
    SPI2->TX[0] = 0x80 | reg;
    
    SPI2->SSR |= SPI_SSR_SSR_Msk;                       //ss start
    
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;               //wait until transfer done
    while((SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) == 1);
    
    SPI2->CNTRL |= SPI_CNTRL_GO_BUSY_Msk;               //receive data
    while((SPI2->CNTRL & SPI_CNTRL_GO_BUSY_Msk) == 1);
    
    temp = SPI2->RX[0];
    
    SPI2->SSR &= ~SPI_SSR_SSR_Msk;                      //set ss high
    
    return temp;
}

void ADXL_Init()
{
    SPI_Write(0x2D,0x08);
	  SPI_Write(0x31,0x0B);
		SPI_Write(0x38,0x80);
}


/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
		uint8_t TEMP[6];
		int16_t accX, accY, accZ;
		float accXX, accYY, accZZ;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init system, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();
	
		/* Init ADXL */
		ADXL_Init();

    printf("\n");
    
		while(1){
			
			TEMP[0]=SPI_Read(DATAX0);
			TEMP[1]=SPI_Read(DATAX1);
			TEMP[2]=SPI_Read(DATAY0);
			TEMP[3]=SPI_Read(DATAY1);
			TEMP[4]=SPI_Read(DATAZ0);
			TEMP[5]=SPI_Read(DATAZ1);

			accX=(TEMP[1]<<8) | TEMP[0];
			accY=(TEMP[3]<<8) | TEMP[2];
			accZ=(TEMP[5]<<8) | TEMP[4];
		
			accXX=(float)accX/256;
			accYY=(float)accY/256;
			accZZ=(float)accZ/256;
			
			printf("x:%.2f   ", accXX - 0.02); 
			printf("y:%.2f   ", accYY + 0.03); 
			printf("z:%.2f \n" ,accZZ + 0.93); 
			CLK_SysTickDelay(300000);
		}
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

