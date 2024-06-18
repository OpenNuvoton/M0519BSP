/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 7 $
 * $Date: 15/07/09 11:56a $
 * @brief    Show the usage of GPIO interrupt function.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"


#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK       72000000


/**
 * @brief       GPIO group 0 (P0/P1/P2/P3/P4) IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The GPIO group 0 (P0/P1/P2/P3/P4) default IRQ, declared in startup_M0519.s.
 */
void GPG0_IRQHandler(void)
{
    /* To check if P1.3 interrupt occurred */
    if(P1->ISF & BIT3)
    {
        P1->ISF = BIT3;
        printf("P1.3 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all P0, P1, P2, P3 and P4 interrupts */
        P0->ISF = P0->ISF;
        P1->ISF = P1->ISF;
        P2->ISF = P2->ISF;
        P3->ISF = P3->ISF;
        P4->ISF = P4->ISF;
        printf("Un-expected interrupts.\n");
    }
}

/**
 * @brief       GPIO group 1 (P5/P6/P7/P8/P9/PA) IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The GPIO group 1 (P5/P6/P7/P8/P9/PA) default IRQ, declared in startup_M0519.s.
 */
void GPG1_IRQHandler(void)
{
    /* To check if P6.5 interrupt occurred */
    if(P6->ISF & BIT5)
    {
        P6->ISF = BIT5;
        printf("P6.5 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all P5, P6, P7, P8 and P9, PA interrupts */
        P5->ISF = P5->ISF;
        P6->ISF = P6->ISF;
        P7->ISF = P7->ISF;
        P8->ISF = P8->ISF;
        P9->ISF = P9->ISF;
        PA->ISF = PA->ISF;
        printf("Un-expected interrupts.\n");
    }
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
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV = (CLK->CLKDIV & (~CLK_CLKDIV_HCLK_N_Msk)) | CLK_CLKDIV_HCLK(1);

    /* Set PLL to Power-down mode */
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART_S_Msk)) | CLK_CLKSEL1_UART_S_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

}

void UART0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
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

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------------------------+\n");
    printf("|    GPIO P1.3 and P6.5 Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO Interrupt Function Test                                                                        */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("P1.3 and P6.5 are used to test interrupt ......\n");

    /* Set the driving type of PWM output ports which are controlled by GPIO mode register */
    GPIO->PWMPOEN = 0;

    /* Configure P1.3 as Input mode and enable interrupt by rising edge trigger */
    P1->PMD = (P1->PMD & (~GPIO_PMD_PMD3_Msk)) | (GPIO_PMD_INPUT << GPIO_PMD_PMD3_Pos);
    P1->IMD |= (GPIO_IMD_EDGE << 3);
    P1->IEN |= (BIT3 << GPIO_IEN_IR_EN_Pos);
    NVIC_EnableIRQ(GPG0_IRQn);

    /* Configure P6.5 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    P6->PMD = (P6->PMD & (~GPIO_PMD_PMD5_Msk)) | (GPIO_PMD_QUASI << GPIO_PMD_PMD5_Pos);
    P6->IMD |= (GPIO_IMD_EDGE << 5);
    P6->IEN |= (BIT5 << GPIO_IEN_IF_EN_Pos);
    NVIC_EnableIRQ(GPG1_IRQn);

    /* Waiting for interrupts */
    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
