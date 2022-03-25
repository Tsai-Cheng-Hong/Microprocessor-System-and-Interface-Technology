/****************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 1 $
 * $Date: 14/12/08 11:50a $
 * @brief    Transmit and receive data from PC terminal through RS232 interface.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"
#include "GPIO.h"
#include "SYS.h"
#include "UART.h"
#include <string.h>


#define PLL_CLOCK 50000000
#define RXBUFSIZE 1024
#define result 
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8RecData[RXBUFSIZE]  = {0};
volatile uint32_t g_u32comRbytes = 0;
volatile int32_t g_bWait         = TRUE;
/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void UART_TEST_HANDLE(void);
void UART_FunctionTest(void);

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

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP &= ~(SYS_GPB_MFP_PB0_Msk | SYS_GPB_MFP_PB1_Msk);
    SYS->GPB_MFP |= SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

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

/*---------------------------------------------------------------------------------------------------------*/
/* UART Test Sample                                                                                        */
/* Test Item                                                                                               */
/* It sends the received data to HyperTerminal.                                                            */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf and testing */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("\nUART Demo Program\n");

    /* UART sample function */
    UART_FunctionTest();

    while(1);
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 0 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART02_IRQHandler(void)
{
    UART_TEST_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART_TEST_HANDLE()
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32IntSts = UART0->ISR;

    if(u32IntSts & UART_ISR_RDA_INT_Msk)
    {
        /* Get all the input characters */
        while(UART_IS_RX_READY(UART0))
        {
            /* Get the character from UART Buffer */
            u8InChar = UART_READ(UART0);
            if(u8InChar == 0x0D)
            {
							printf("%s\n", g_u8RecData);
							if(g_u8RecData[0]==0x72 && g_u8RecData[1]==0x65 && g_u8RecData[2]==0x64 && g_u8RecData[3]==0x20 && g_u8RecData[4]==0x6F && g_u8RecData[5]==0x6E){
									PA14=0;
							}
							if(g_u8RecData[0]==0x72 && g_u8RecData[1]==0x65 && g_u8RecData[2]==0x64 && g_u8RecData[3]==0x20 && g_u8RecData[4]==0x6F && g_u8RecData[5]==0x66 && g_u8RecData[6]==0x66){
									PA14=1;
							}
							if(g_u8RecData[0]==0x67 && g_u8RecData[1]==0x72 && g_u8RecData[2]==0x65 && g_u8RecData[3]==0x65 && g_u8RecData[4]==0x6E && g_u8RecData[5]==0x20 && g_u8RecData[6]==0x6F && g_u8RecData[7]==0x6E){
									PA13=0;
							}
							if(g_u8RecData[0]==0x67 && g_u8RecData[1]==0x72 && g_u8RecData[2]==0x65 && g_u8RecData[3]==0x65 && g_u8RecData[4]==0x6E && g_u8RecData[5]==0x20 && g_u8RecData[6]==0x6F && g_u8RecData[7]==0x66 && g_u8RecData[8]==0x66){
									PA13=1;
							}
							if(g_u8RecData[0]==0x62 && g_u8RecData[1]==0x6C && g_u8RecData[2]==0x75 && g_u8RecData[3]==0x65 && g_u8RecData[4]==0x20 && g_u8RecData[5]==0x6F && g_u8RecData[6]==0x6E){
									PA12=0;
							}
							if(g_u8RecData[0]==0x62 && g_u8RecData[1]==0x6C && g_u8RecData[2]==0x75 && g_u8RecData[3]==0x65 && g_u8RecData[4]==0x20 && g_u8RecData[5]==0x6F && g_u8RecData[6]==0x66 && g_u8RecData[6]==0x66){
									PA12=1;
							}	
							memset(g_u8RecData,1,sizeof(g_u8RecData));
							g_u32comRbytes=0;
            }else{
							g_u8RecData[g_u32comRbytes] = u8InChar;
							g_u32comRbytes++;
						}
						if(u8InChar == '0'){
							g_bWait = FALSE;
            }
        }
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest()
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Function Test                                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will print input char on terminal      |\n");
    printf("|    Please enter any to start     (Press '0' to exit)      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        Using a RS232 cable to connect UART0 and PC.
        UART0 is set to debug port. UART0 is enable RDA and RLS interrupt.
        When inputting char to terminal screen, RDA interrupt will happen and
        UART0 will print the received char on screen.
    */

    /* Enable Interrupt and install the call back function */
		printf("\n");
    UART_EnableInt(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_TOUT_IEN_Msk));
    while(g_bWait);

    /* Disable Interrupt */
    UART_DisableInt(UART0, (UART_IER_RDA_IEN_Msk | UART_IER_THRE_IEN_Msk | UART_IER_TOUT_IEN_Msk));
    g_bWait = TRUE;
    printf("\nUART Demo End.\n");

}

