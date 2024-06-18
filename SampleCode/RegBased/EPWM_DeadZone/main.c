/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 4 $
 * $Date: 15/03/18 11:53a $
 * @brief    Demonstrate how to use EPWM Dead Zone function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
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
    static uint32_t cnt;
    static uint32_t out;

    /* Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times. */
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
    /* Clear EPWM0 period interrupt flag */
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
    printf("  This sample code will output all EPWM0 channels with different\n");
    printf("  frequency and duty, enable dead zone function of all EPWM0 pairs.\n");
    printf("  And also enable/disable EPWM0 output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: EPWM0_CH0(P0.0), EPWM0_CH1(P0.1), EPWM0_CH2(P0.2),\n");
    printf("                         EPWM0_CH3(P0.3), EPWM0_CH4(P0.4), EPWM0_CH5(P0.5)\n");

    /* Note that all EPWM0 channels share one period */
    /* EPWM0 pair 0 frequency is 100Hz, duty 30%, */
    /* Assume EPWM0 output frequency is 100Hz and duty ratio is 30%, user can calculate EPWM0 settings by follows.
       duty ratio = (PWM_Duty0+1)/(PWMP+1)
       cycle time = PWMP+1
       High level = PWM_Duty0+1
       EPWM0 clock source frequency = __HXT = 12000000
       (CNR+1) = EPWM0 clock source frequency/clock source divider/EPWM0 output frequency
               = 12000000/2/100 = 60000
       (Note: PWMP is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       PWMP = 59999
       duty ratio = 30% ==> (PWM_Duty0+1)/(PWMP+1) = 30% ==> PWM_Duty0 = (PWMP+1)*0.3-1 = 60000*30/100-1
       PWM_Duty0 = 17999
       Clock divider is EPWM_CLK_DIV_2 : clock divider = 2
    */

    /*Set Pwm mode as complementary mode*/
    EPWM_ENABLE_COMPLEMENTARY_MODE(EPWM0);

    /* Set EPWM0 Timer clock divider select */
    EPWM0->PWMCON = (EPWM0->PWMCON & ~(EPWM_PWMCON_PWMDIV_Msk)) | (EPWM_CLK_DIV_2);

    /* Set EPWM0 pair 0 duty */
    EPWM0->PWM0 = 17999;

    /* Set EPWM0 Timer period */
    EPWM0->PWMP = 59999;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* enable and configure dead zone */
    /* Note that all channels share one dead zone configuration */
    EPWM0->PDTC = (EPWM0->PDTC & ~EPWM_PDTC_DTCNT_Msk) | 40;
    EPWM0->PDTC |= EPWM_PDTC_DTEN0_Msk;

    /* EPWM0 pair 1 frequency is 100Hz, duty 50%,
       PWMP = 59999
       duty ratio = 50% ==> (PWM_Duty0+1)/(PWMP+1) = 50% ==> PWM_Duty0 = (PWMP+1)*0.5-1 = 60000*50/100-1
       PWM_Duty0 = 29999
    */
    /* Set EPWM0 pair 1 duty */
    EPWM0->PWM2 = 29999;

    /* enable dead zone */
    EPWM0->PDTC |= EPWM_PDTC_DTEN2_Msk;

    /* EPWM0 pair 2 frequency is 100Hz, duty 70%,
       PWMP = 59999
       duty ratio = 70% ==> (PWM_Duty0+1)/(PWMP+1) = 70% ==> PWM_Duty0 = (PWMP+1)*0.7-1 = 60000*70/100-1
       PWM_Duty0 = 41999
    */
    /* Set EPWM0 channel 2 duty */
    EPWM0->PWM4 = 41999;

    /* Enable dead zone */
    EPWM0->PDTC |= EPWM_PDTC_DTEN4_Msk;
    /* Lock protected registers */
    SYS_LockReg();

    /* Enable EPWM0 period interrupt, use channel 0 to measure time. */
    EPWM0->PWMCON |= EPWM_PWMCON_PWMI_EN_Msk;
    NVIC_EnableIRQ(EPWM0_IRQn);

    /* Enable all EPWM0 output channels */
    GPIO->PWMPOEN &= ~(GPIO_PWMPOEN_HZ_Odd0_Msk | GPIO_PWMPOEN_HZ_Even0_Msk);

    /* Start */
    EPWM0->PWMCON |= EPWM_PWMCON_PWMRUN_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
