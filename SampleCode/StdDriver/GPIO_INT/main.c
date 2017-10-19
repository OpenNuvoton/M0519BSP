/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * $Revision: 4 $
 * $Date: 15/03/19 1:12p $
 * @brief    Show the usage of GPIO interrupt function.
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"


#define PLL_CLOCK   72000000


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
    if(GPIO_GET_INT_FLAG(P1, BIT3))
    {
        GPIO_CLR_INT_FLAG(P1, BIT3);
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
    if(GPIO_GET_INT_FLAG(P6, BIT5))
    {
        GPIO_CLR_INT_FLAG(P6, BIT5);
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

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

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
    GPIO_SetMode(P1, BIT3, GPIO_PMD_INPUT);
    GPIO_EnableInt(P1, 3, GPIO_INT_RISING);
    NVIC_EnableIRQ(GPG0_IRQn);

    /* Configure P6.5 as Quasi-bidirection mode and enable interrupt by falling edge trigger */
    GPIO_SetMode(P6, BIT5, GPIO_PMD_QUASI);
    GPIO_EnableInt(P6, 5, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPG1_IRQn);

    /* Waiting for interrupts */
    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
