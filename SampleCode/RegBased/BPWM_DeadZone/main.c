/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 4 $
 * $Date: 15/03/24 10:20a $
 * @brief    Demonstrate how to use BPWM Dead Zone function.
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

    /* Channel 0 frequency is 100Hz, every 1 second enter this IRQ handler 100 times. */
    if(++cnt == 100)
    {
        if(out)
            BPWM0->POE |= BPWM_POE_POE0_Msk | BPWM_POE_POE1_Msk;
        else
            BPWM0->POE &= ~(BPWM_POE_POE0_Msk | BPWM_POE_POE1_Msk);
        out ^= 1;
        cnt = 0;
    }
    /* Clear channel 0 period interrupt flag */
    BPWM0->PIIR = BPWM_PIIR_PWMIF0_Msk;
}

void SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(1);

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    u32TimeOutCnt = __HIRC;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk))
		if(--u32TimeOutCnt == 0) break;
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

    /* Enable BPWM0 module clock */
    CLK->APBCLK |= CLK_APBCLK_BPWM0_EN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Set P5 multi-function pins for BPWM0 Channel 0 and channel 1 */
    SYS->P5_MFP = (SYS->P5_MFP & ~SYS_MFP_P56_Msk) | SYS_MFP_P56_BPWM0_CH0; /* Enable P5.6 PWM function pin(BPWM0_CH0) */
    SYS->P5_MFP = (SYS->P5_MFP & ~SYS_MFP_P57_Msk) | SYS_MFP_P57_BPWM0_CH1; /* Enable P5.7 PWM function pin(BPWM0_CH1) */
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
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output all BPWM0 channels with different\n");
    printf("  frequency and duty, enable dead zone function of all BPWM0 pairs.\n");
    printf("  And also enable/disable BPWM0 output every 1 second.\n");
    printf("  I/O configuration:\n");
    printf("    waveform output pin: BPWM0_CH0(P5.6), BPWM0_CH1(P5.7)\n");

    /* BPWM0 channel 0 frequency is 100Hz, duty 30%, */
    /* Assume BPWM0 output frequency is 100Hz and duty ratio is 30%, user can calculate BPWM0 settings by follows.
       duty ratio = (CMR+1)/(CNR+1)
       cycle time = CNR+1
       High level = CMR+1
       BPWM clock source frequency = __HXT = 12000000
       (CNR+1) = BPWM clock source frequency/prescaler/clock source divider/BPWM output frequency
               = 12000000/2/1/100 = 60000
       (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
       CNR = 59999
       duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30% ==> CMR = (CNR+1)*0.3-1 = 60000*30/100-1
       CMR = 17999
       Prescale value is 1 : prescaler= 2
       Clock divider is BPWM_CSR_DIV1 : clock divider =1
    */

    /* Set Pwm mode */
    BPWM0->PCR |= BPWM_PCR_CH0MOD_Msk;

    /* Set BPWM0 Timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, BPWM_CH0, 1); /* Divided by 2 */

    /* Set BPWM0 Timer clock divider select */
    BPWM_SET_DIVIDER(BPWM0, BPWM_CH0, BPWM_CLK_DIV_1);

    /* Set BPWM0 Timer duty */
    BPWM0->CMR0 = 17999;

    /* Set BPWM0 Timer period */
    BPWM0->CNR0 = 59999;

    /* enable and configure dead zone */
    BPWM0->PPR = (BPWM0->PPR & ~BPWM_PPR_DZI01_Msk) | (40 << BPWM_PPR_DZI01_Pos);
    BPWM0->PCR |= BPWM_PCR_DZEN01_Msk;

    /* Enable output of all BPWM0 channels */
    BPWM0->POE |= BPWM_POE_POE0_Msk | BPWM_POE_POE1_Msk;
    GPIO->PWMPOEN &= ~GPIO_PWMPOEN_HZ_BPWM_Msk;

    /* Enable BPWM0 channel 0 period interrupt, use channel 0 to measure time. */
    BPWM0->PIER = BPWM_PIER_PWMPIE0_Msk;
    NVIC_EnableIRQ(BPWM0_IRQn);

    /* Start */
    BPWM0->PCR |= BPWM_PCR_CH0EN_Msk | BPWM_PCR_CH1EN_Msk;

    while(1);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
