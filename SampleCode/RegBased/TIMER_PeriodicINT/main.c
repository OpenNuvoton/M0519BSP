/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/03/19 1:12p $
 * @brief    Implement timer counting in periodic mode.
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_au32TMRINTCount[4] = {0};


/**
 * @brief       Timer0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer0 default IRQ, declared in startup_M0519.s.
 */
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        g_au32TMRINTCount[0]++;
    }
}

/**
 * @brief       Timer1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer1 default IRQ, declared in startup_M0519.s.
 */
void TMR1_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);

        g_au32TMRINTCount[1]++;
    }
}

/**
 * @brief       Timer2 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer2 default IRQ, declared in startup_M0519.s.
 */
void TMR2_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER2);

        g_au32TMRINTCount[2]++;
    }
}

/**
 * @brief       Timer3 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The Timer3 default IRQ, declared in startup_M0519.s.
 */
void TMR3_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER3) == 1)
    {
        /* Clear Timer3 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER3);

        g_au32TMRINTCount[3]++;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable IRC22M clock */
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for IRC22M clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_IRC22M_STB_Msk));

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_HIRC;

    /* Set PLL to Power-down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;

    /* Enable external 12 MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = CLK_PLLCON_72MHz_HXT;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Enable peripheral clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk |
                  CLK_APBCLK_TMR0_EN_Msk | CLK_APBCLK_TMR1_EN_Msk | CLK_APBCLK_TMR2_EN_Msk | CLK_APBCLK_TMR3_EN_Msk;

    /* Peripheral clock source */
    CLK->CLKSEL1 = CLK_CLKSEL1_UART_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD, TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(PllClock, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32InitCount;

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
    printf("|    Timer Periodic and Interrupt Sample Code    |\n");
    printf("+------------------------------------------------+\n\n");

    printf("# Timer Settings:\n");
    printf("  Timer0: Clock source HCLK; Periodic mode; Enable interrupt; Presacle 10; 1 interrupt tick/sec.\n");
    printf("  Timer1: Clock source HCLK; Periodic mode; Enable interrupt; Presacle 4;  2 interrupt ticks/sec.\n");
    printf("  Timer2: Clock source HCLK; Periodic mode; Enable interrupt; Presacle 8;  4 interrupt ticks/sec.\n");
    printf("  Timer3: Clock source HCLK; Periodic mode; Enable interrupt; Presacle 2;  8 interrupt ticks/sec.\n");
    printf("# Check Timer0 ~ Timer3 interrupt counts are reasonable or not.\n\n");

    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER0->TCMPR = (SystemCoreClock / 10);
    TIMER0->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER0, (10 - 1));

    /* Open Timer1 in periodic mode, enable interrupt and 2 interrupt ticks per second */
    TIMER1->TCMPR = ((SystemCoreClock / 4) / 2);
    TIMER1->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER1, (4 - 1));

    /* Open Timer2 in periodic mode, enable interrupt and 4 interrupt ticks per second */
    TIMER2->TCMPR = ((SystemCoreClock / 8) / 4);
    TIMER2->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER2, (8 - 1));

    /* Open Timer3 in periodic mode, enable interrupt and 8 interrupt ticks per second */
    TIMER3->TCMPR = ((SystemCoreClock / 2) / 8);
    TIMER3->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER3, (2 - 1));

    /* Enable Timer0 ~ Timer3 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);
    NVIC_EnableIRQ(TMR1_IRQn);
    NVIC_EnableIRQ(TMR2_IRQn);
    NVIC_EnableIRQ(TMR3_IRQn);

    /* Clear Timer0 ~ Timer3 interrupt counts to 0 */
    g_au32TMRINTCount[0] = g_au32TMRINTCount[1] = g_au32TMRINTCount[2] = g_au32TMRINTCount[3] = 0;
    u32InitCount = g_au32TMRINTCount[0];

    /* Start Timer0 ~ Timer3 counting */
    TIMER_Start(TIMER0);
    TIMER_Start(TIMER1);
    TIMER_Start(TIMER2);
    TIMER_Start(TIMER3);

    /* Check Timer0 ~ Timer3 interrupt counts */
    printf("# Timer interrupt counts :\n");
    while(u32InitCount < 20)
    {
        if(g_au32TMRINTCount[0] != u32InitCount)
        {
            printf("TMR0:%3d    TMR1:%3d    TMR2:%3d    TMR3:%3d\n",
                   g_au32TMRINTCount[0], g_au32TMRINTCount[1], g_au32TMRINTCount[2], g_au32TMRINTCount[3]);
            u32InitCount = g_au32TMRINTCount[0];

            if((g_au32TMRINTCount[1] > (g_au32TMRINTCount[0] * 2 + 1)) || (g_au32TMRINTCount[1] < (g_au32TMRINTCount[0] * 2 - 1)) ||
                    (g_au32TMRINTCount[2] > (g_au32TMRINTCount[0] * 4 + 1)) || (g_au32TMRINTCount[2] < (g_au32TMRINTCount[0] * 4 - 1)) ||
                    (g_au32TMRINTCount[3] > (g_au32TMRINTCount[0] * 8 + 1)) || (g_au32TMRINTCount[3] < (g_au32TMRINTCount[0] * 8 - 1)))
            {
                printf("*** FAIL ***\n");
                goto lexit;
            }
        }
    }

    printf("*** PASS ***\n");

lexit:

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
