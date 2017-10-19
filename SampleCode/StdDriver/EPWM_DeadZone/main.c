/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 4 $
 * $Date: 15/03/19 1:12p $
 * @brief    Demonstrate how to use EPWM Dead Zone function.
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
    static uint32_t cnt;
    static uint32_t out;

    // Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times.
    if(++cnt == 100)
    {
        if(out)
        {
            EPWM_MASK_OUTPUT(EPWM0, 0, 0);
        }
        else
        {
            EPWM_MASK_OUTPUT(EPWM0, 0x3F, 0);
        }
        out ^= 1;
        cnt = 0;
    }
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

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

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
    printf("|                          EPWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output all EPWM0 channels with different\n");
    printf("  frequency and duty, enable dead zone function of all EPWM0 pairs.\n");
    printf("  And also enable/disable EPWM0 output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: EPWM0_CH0(P0.0), EPWM0_CH1(P0.1), EPWM0_CH2(P0.2),\n");
    printf("                         EPWM0_CH3(P0.3), EPWM0_CH4(P0.4), EPWM0_CH5(P0.5)\n");

    /*Set Pwm mode as complementary mode*/
    EPWM_ENABLE_COMPLEMENTARY_MODE(EPWM0);

    /* EPWM0 pair 0 frequency is 100Hz, duty 30%, */
    EPWM_ConfigOutputChannel(EPWM0, EPWM_CH0, 100, 30);

    /* EPWM0 pair 1 frequency is 100Hz, duty 50%, */
    EPWM_ConfigOutputChannel(EPWM0, EPWM_CH2, 100, 50);

    /* EPWM0 pair 2 frequency is 100Hz, duty 70%, */
    EPWM_ConfigOutputChannel(EPWM0, EPWM_CH4, 100, 70);

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Note that all channels share one dead zone configuration */
    EPWM_EnableDeadZone(EPWM0, EPWM_CH0, 40);
    EPWM_EnableDeadZone(EPWM0, EPWM_CH2, 40);
    EPWM_EnableDeadZone(EPWM0, EPWM_CH4, 40);
    /* Lock protected registers */
    SYS_LockReg();

    /* Enable EPWM0 channel 0 period interrupt, use channel 0 to measure time. */
    EPWM_EnablePeriodInt(EPWM0, NULL, EPWM_PERIOD_INT_UNDERFLOW);
    NVIC_EnableIRQ(EPWM0_IRQn);

    /* enable all EPWM channels output */
    EPWM_EnableOutput(EPWM0, 0x3);

    /* Start */
    EPWM_Start(EPWM0, NULL);

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
