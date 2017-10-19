/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 4 $
 * $Date: 15/03/19 1:12p $
 * @brief    Change duty cycle and period of output waveform by EPWM Double Buffer function.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK           72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       EPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle EPWM0 interrupt event
 */
void EPWM0_IRQHandler(void)
{
    static int toggle = 0;

    EPWM_SET_DIVIDER(EPWM0, EPWM_CH0, EPWM_CLK_DIV_2);
    // Update EPWM0 channel 0 period and duty
    if(toggle == 0)
    {
        EPWM_SET_CNR(EPWM0, EPWM_CH0, 99);
        EPWM_SET_CMR(EPWM0, EPWM_CH0, 39);
    }
    else
    {
        EPWM_SET_CNR(EPWM0, EPWM_CH0, 199);
        EPWM_SET_CMR(EPWM0, EPWM_CH0, 99);
    }
    /* Reload PWM period and duty values */
    EPWM0->PWMCON |= EPWM_PWMCON_LOAD_Msk;
    toggle ^= 1;
    // Clear channel 0 period interrupt flag
    EPWM_ClearPeriodIntFlag(EPWM0, 0);
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

    /* Switch HCLK clock source to external XTAL 12MHz and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Enable EPWM0 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Set P5 multi-function pins for CLKO */
    SYS->P5_MFP = SYS->P5_MFP & (~SYS_MFP_P55_Msk) | SYS_MFP_P55_CLKO;

    /* Set P0 multi-function pins for EPWM Channel 0(PWM0_CH0) ~ channel 5(PWM0_CH5) */
    SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P01_Msk | SYS_MFP_P02_Msk | SYS_MFP_P03_Msk | SYS_MFP_P04_Msk | SYS_MFP_P05_Msk);
    SYS->P0_MFP |= (SYS_MFP_P00_EPWM0_CH0 | SYS_MFP_P01_EPWM0_CH1 | SYS_MFP_P02_EPWM0_CH2 | SYS_MFP_P03_EPWM0_CH3 | SYS_MFP_P04_EPWM0_CH4 | SYS_MFP_P05_EPWM0_CH5);
}

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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          EPWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use EPWM0 channel 0 to output waveform\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: EPWM0_CH0(P0.0)\n");
    printf("\nUse double buffer feature.\n");

    /*
        EPWM0 channel 0 waveform of this sample shown below:

        |<-        CNR + 1  clk     ->|  CNR + 1 = 199 + 1 CLKs
                       |<-CMR+1 clk ->|  CMR + 1 = 99 + 1 CLKs
                                      |<-   CNR + 1  ->|  CNR + 1 = 99 + 1 CLKs
                                               |<CMR+1>|  CMR + 1 = 39 + 1 CLKs

         ______________                _______          _
      __|      100     |_____100______|   40  |___60___|      PWM waveform
    */


    /*
      Configure EPWM0 channel 0 init period and duty.
      Period is __HXT / (prescaler * clock divider * (PWMP + 1))
      Duty ratio = (PWM_Duty + 1) / (PWMP + 1)
      Period = 12 MHz / (2 * 1 * (199 + 1)) =  30000 Hz
      Duty ratio = (99 + 1) / (199 + 1) = 50%
    */
    /* set EPWM0 channel 0 output configuration */
    EPWM_ConfigOutputChannel(EPWM0, EPWM_CH0, 30000, 50);

    /* Enable EPWM0 channel 0 period interrupt */
    EPWM_EnablePeriodInt(EPWM0, NULL, EPWM_PERIOD_INT_UNDERFLOW);
    NVIC_EnableIRQ(EPWM0_IRQn);

    /* enable all EPWM channels output */
    EPWM_EnableOutput(EPWM0, 0x3);

    /* Start */
    EPWM_Start(EPWM0, NULL);

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
