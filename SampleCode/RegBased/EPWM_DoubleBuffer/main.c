/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 5 $
 * $Date: 15/03/19 1:12p $
 * @brief    Change duty cycle and period of output waveform by EPWM Double Buffer function.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"

#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT
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
    static int toggle = 0;

    /* Update EPWM0 channel 0 period and duty */
    if(toggle == 0)
    {
        EPWM0->PWMP = 99;
        EPWM0->PWM0 = 39;
    }
    else
    {
        EPWM0->PWMP = 199;
        EPWM0->PWM0 = 99;
    }
    /* Reload PWM period and duty values */
    EPWM0->PWMCON |= EPWM_PWMCON_LOAD_Msk;
    toggle ^= 1;
    /* Clear channel 0 period interrupt flag */
    EPWM0->PWMSTS = EPWM_PWMSTS_PWMF_Msk;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(1);

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HXT;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* Enable EPWM0 module clock */
    CLK->APBCLK |= CLK_APBCLK_EPWM0_EN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Set P0 multi-function pins for EPWM Channel 0(PWM0_CH0) ~ channel 5(PWM0_CH5) */
    SYS->P0_MFP &= ~(SYS_MFP_P00_Msk | SYS_MFP_P01_Msk | SYS_MFP_P02_Msk | SYS_MFP_P03_Msk | SYS_MFP_P04_Msk | SYS_MFP_P05_Msk);
    SYS->P0_MFP |= (SYS_MFP_P00_EPWM0_CH0 | SYS_MFP_P01_EPWM0_CH1 | SYS_MFP_P02_EPWM0_CH2 | SYS_MFP_P03_EPWM0_CH3 | SYS_MFP_P04_EPWM0_CH4 | SYS_MFP_P05_EPWM0_CH5);
}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
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
    printf("|                          EPWM Driver Sample Code                       |\n");
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


    /* Set channel 0 clock divider to 2 */
    EPWM0->PWMCON = (EPWM0->PWMCON & ~(EPWM_PWMCON_PWMDIV_Msk)) | (EPWM_CLK_DIV_2);

    /*
      Configure EPWM0 channel 0 init period and duty.
      Period is __HXT / (clock divider * (PWMP + 1))
      Duty ratio = (PWM_Duty + 1) / (PWMP + 1)
      Period = 12 MHz / (2 * (199 + 1)) =  30000 Hz
      Duty ratio = (99 + 1) / (199 + 1) = 50%
    */
    EPWM0->PWM0 = 99;
    EPWM0->PWMP = 199;

    /* Enable EPWM channel 0 period interrupt */
    EPWM0->PWMCON |= EPWM_PWMCON_PWMI_EN_Msk;
    NVIC_EnableIRQ(EPWM0_IRQn);

    /* Enable all EPWM0 output channels */
    GPIO->PWMPOEN &= ~(GPIO_PWMPOEN_HZ_Odd0_Msk | GPIO_PWMPOEN_HZ_Even0_Msk);

    /* Start */
    EPWM0->PWMCON |= EPWM_PWMCON_PWMRUN_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
