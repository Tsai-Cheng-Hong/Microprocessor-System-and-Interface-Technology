/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 2 $
 * $Date: 14/12/29 3:22p $
 * @brief
 *           Show a I2C Master how to access Slave.
 *           This sample code needs to work with I2C_Slave.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NUC100Series.h"
#include "GPIO.h"
#include "SYS.h"
#include "i2c.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000
#define ADXL_W_ADR	0xA6
#define ADXL_R_ADR	0xA7
#define DATAX0      0x32
#define DATAX1		  0x33
#define DATAY0		  0x34
#define DATAY1      0x35
#define DATAZ0      0x36
#define DATAZ1      0x37

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset IP */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
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

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable I2C0 module clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFP = SYS_GPB_MFP_PB0_UART0_RXD | SYS_GPB_MFP_PB1_UART0_TXD;

    /* Set GPA multi-function pins for I2C0 SDA and SCL */
    SYS->GPA_MFP = SYS_GPA_MFP_PA8_I2C0_SDA | SYS_GPA_MFP_PA9_I2C0_SCL;
}

void I2C_Write(uint8_t INDEX,uint8_t DATA)
{
	I2C_Trigger(I2C0,1,0,1,0);
  I2C_WAIT_READY(I2C0);
	
	I2C0->I2CDAT = ADXL_W_ADR;
  I2C_Trigger(I2C0, 0, 0, 1, 0);   //clr si flag
    I2C_WAIT_READY(I2C0);	//poll si flag
	
	//send write address
	I2C0->I2CDAT = INDEX;	
	I2C_Trigger(I2C0,0, 0, 1, 0);   //clr si 	
	  I2C_WAIT_READY(I2C0);//poll si flag
	//send write data
	I2C0->I2CDAT = DATA;	
	I2C_Trigger(I2C0,0, 0, 1, 0);   //clr si 	
  I2C_WAIT_READY(I2C0);	//poll si flag
}

uint8_t I2C_Read(uint8_t INDEX)
{	
  uint8_t TEMP;
  //send i2c start
  I2C_Trigger(I2C0,1, 0, 1, 0);	//set start
  I2C_WAIT_READY(I2C0);	//poll si flag
	
  //send to Write port
  I2C0->I2CDAT = ADXL_W_ADR;
  I2C_Trigger(I2C0,0, 0, 1, 0);	//clr si
  I2C_WAIT_READY(I2C0); //poll si flag
	
  //send INDEX
  I2C0->I2CDAT = INDEX;
  I2C_Trigger(I2C0,0, 0, 1, 0);	//clr si
  I2C_WAIT_READY(I2C0);	//poll si flag
	
  //send i2c start
  I2C_Trigger(I2C0,1, 0, 1, 0);	//set start
  I2C_WAIT_READY(I2C0);	//poll si flag
  
	//send to Read port
  I2C0->I2CDAT = (ADXL_R_ADR);
  I2C_Trigger(I2C0,0, 0, 1, 0);	//clr si
  I2C_WAIT_READY(I2C0);	//poll si flag		
  
	//receive data
  I2C0->I2CDAT = 0xFF;
  I2C_Trigger(I2C0,0, 0, 1, 0);   //clr si	
  I2C_WAIT_READY(I2C0);	//poll si flag
  TEMP = I2C0->I2CDAT;
  I2C_Trigger(I2C0,0, 1, 1, 0);   //clr si and set stop

  //while(I2C_I2CON_STO);

  return TEMP;
}

void ADXL_Init()
{
	I2C_Write(0x2D,0x08);
	I2C_Write(0x31,0x0B);
	I2C_Write(0x38,0x80);
}

uint8_t ADXL_Read_DataX()
{
	int8_t x0,x1,xaxisData;
  x0=I2C_Read(0x32);
	x1=I2C_Read(0x33);
	xaxisData=((x1 << 8) | x0);
  return xaxisData;
}
uint8_t ADXL_Read_DataY()
{
	int8_t y0,y1;
	int16_t yaxisData;
  y0=I2C_Read(0x34);
	y1=I2C_Read(0x35);
	yaxisData=((y1 << 8) | y0);
  return yaxisData;
}
uint8_t ADXL_Read_DataZ()
{
	int8_t z0,z1,zaxisData;
  z0=I2C_Read(0x36);
	z1=I2C_Read(0x37);
	zaxisData=((z1 << 8) | z0);
  return zaxisData;
}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
	uint8_t TEMP[6];
	int16_t accX, accY, accZ;
	float accXX, accYY, accZZ;
	SYS_UnlockReg();
	SYS_Init();
	SYS_LockReg();
	I2C0_Init();
	UART0_Init();
	ADXL_Init();	
		
	while(1){
	  TEMP[0]=I2C_Read(DATAX0);
    TEMP[1]=I2C_Read(DATAX1);
    TEMP[2]=I2C_Read(DATAY0);
    TEMP[3]=I2C_Read(DATAY1);
    TEMP[4]=I2C_Read(DATAZ0);
    TEMP[5]=I2C_Read(DATAZ1);

		accX=(TEMP[1]<<8) | TEMP[0];
		accY=(TEMP[3]<<8) | TEMP[2];
		accZ=(TEMP[5]<<8) | TEMP[4];
		
		accXX=(float)accX/256;
		accYY=(float)accY/256;
		accZZ=(float)accZ/256;
		
		printf("x:%.2f   ",accXX-0.11);		
		printf("y:%.2f   ",accYY+0.16);	
		printf("z:%.2f\n",accZZ);
		CLK_SysTickDelay(500000);
	}
}



