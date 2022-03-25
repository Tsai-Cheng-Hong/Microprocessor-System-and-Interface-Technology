/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/04/13 10:17a $
 * @brief
 *           Show how to generate time-out reset system event while WDT time-out reset delay period expired.
 *
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"
#include "Scankey.h"

#define PLL_CLOCK           50000000

int f=0;
volatile uint8_t g_u8IsWDTTimeoutINT = 0;

void WDT_IRQHandler(void)
{
	WDT_CLEAR_TIMEOUT_INT_FLAG();
	printf("WDT time-out interrupt occurred.\n");
	
	if(f==0)
		printf("No Problem~~~\n");  
	else
	  printf("Alarm!!!~~~reset!!!\n");
	
	g_u8IsWDTTimeoutINT = 1;
}

void GPAB_IRQHandler(void)
{
	PA->ISRC = BIT2;
	printf("Change!!!\n");
	PB11=0;
	f=~f;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock and internal 10 kHz */
    CLK_EnableXtalRC((CLK_PWRCON_XTL12M_EN_Msk | CLK_PWRCON_OSC10K_EN_Msk));

    /* Waiting for clock ready */
    CLK_WaitClockReady((CLK_CLKSTATUS_XTL12M_STB_Msk | CLK_CLKSTATUS_OSC10K_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);
    
    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);    
    
    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));
    
    /* Enable WDT module clock */
    CLK_EnableModuleClock(WDT_MODULE);    
        
    /* Select WDT module clock source */
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, NULL);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PB multi-function pins for UART0 RXD, TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
	uint8_t number;
	//u8Option = getchar();
	
	/* Unlock protected registers */
	SYS_UnlockReg();

	/* Init System, peripheral clock and multi-function I/O */
  SYS_Init();

  /* Lock protected registers */
  SYS_LockReg();

  /* Init UART0 for printf */
  UART0_Init();
	
	GPIO_SetMode(PA, BIT2, GPIO_PMD_QUASI);
	PA3=0;

  PA->PMD = (PA->PMD & (~GPIO_PMD_PMD2_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD2_Pos);
  PA->IMD |= (GPIO_IMD_EDGE << 2);
  PA->IEN |= (BIT2 << GPIO_IEN_IR_EN_Pos);
		
  NVIC_EnableIRQ(GPAB_IRQn);
	GPIO->DBNCECON = (GPIO_DBNCECON_ICLK_ON_Msk | GPIO_DBCLKSRC_LIRC | GPIO_DBCLKSEL_1024);
	PA->DBEN |= (BIT2);
    
  GPIO_ENABLE_DEBOUNCE(PA,BIT2);
		
	printf("Start Lab5\n");
		
	SYS_UnlockReg();

	g_u8IsWDTTimeoutINT = 0;
		
  if(f==0){
		WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_1026CLK, TRUE, FALSE);
		WDT_EnableInt();
		NVIC_EnableIRQ(WDT_IRQn);
	}

	number=ScanKey();
	if(number==1) f=1;

	while(1){
		if(f==0)
			printf("safe!\n");
		else
		  printf("Alarm!!!\n");

		WDT_Open(WDT_TIMEOUT_2POW14, WDT_RESET_DELAY_1026CLK, TRUE, FALSE);
		WDT_EnableInt();
	  NVIC_EnableIRQ(WDT_IRQn);
		CLK_SysTickDelay(8000000);
	}
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
