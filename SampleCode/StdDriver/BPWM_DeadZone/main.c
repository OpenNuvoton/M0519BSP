/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 5 $
 * $Date: 15/03/24 10:20a $
 * @brief    Demonstrate how to use BPWM Dead Zone function.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"


#define PLL_CLOCK       72000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       BPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM0 interrupt event
 */
void BPWM0_IRQHandler(void)
{
    static uint32_t cnt;
    static uint32_t out;

    // Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times.
    if(++cnt == 100)
    {
        if(out)
            BPWM_EnableOutput(BPWM0, 0x3);
        else
            BPWM_DisableOutput(BPWM0, 0x3);
        out ^= 1;
        cnt = 0;
    }
    // Clear channel 0 period interrupt flag
    BPWM_ClearPeriodIntFlag(BPWM0, 0);
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

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Enable BPWM0 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Set P5 multi-function pins for CLKO */
    SYS->P5_MFP = SYS->P5_MFP & (~SYS_MFP_P55_Msk) | SYS_MFP_P55_CLKO;

    /* Set P5 multi-function pins for BPWM0 Channel 0 */
    SYS->P5_MFP = SYS->P5_MFP & (~SYS_MFP_P56_Msk) | SYS_MFP_P56_BPWM0_CH0; /* Enable P5.6 PWM function pin(BPWM0_CH0) */
    SYS->P5_MFP = SYS->P5_MFP & (~SYS_MFP_P57_Msk) | SYS_MFP_P57_BPWM0_CH1; /* Enable P5.7 PWM function pin(BPWM0_CH1) */
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

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output all BPWM0 channels with different\n");
    printf("  frequency and duty, enable dead zone function of all BPWM0 pairs.\n");
    printf("  And also enable/disable BPWM0 output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0_CH0(P5.6), BPWM0_CH1(P5.7)\n");

    /* BPWM0 channel 0 frequency is 100Hz, duty 30%, */
    BPWM_ConfigOutputChannel(BPWM0, BPWM_CH0, 100, 30);
    BPWM_EnableDeadZone(BPWM0, BPWM_CH0, 400);

    /* Enable output of all BPWM0 channels */
    BPWM_EnableOutput(BPWM0, 0x3);

    /* Enable BPWM0 channel 0 period interrupt, use channel 0 to measure time. */
    BPWM_EnablePeriodInt(BPWM0, BPWM_CH0, 0);
    NVIC_EnableIRQ(BPWM0_IRQn);

    /* Start */
    BPWM_Start(BPWM0, 0x3);

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
