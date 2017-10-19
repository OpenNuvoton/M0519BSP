/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/03/19 1:13p $
 * @brief    Show how to use the timer2 capture function to capture timer2 counter value.
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"

#define PLL_CLOCK           72000000


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
    /* Clear Timer0 time-out interrupt flag */
    TIMER_ClearIntFlag(TIMER0);

    g_au32TMRINTCount[0]++;
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
    /* Clear Timer1 time-out interrupt flag */
    TIMER_GetIntFlag(TIMER1);

    g_au32TMRINTCount[1]++;
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
    if(TIMER_GetCaptureIntFlag(TIMER2) == 1)
    {
        /* Clear Timer2 capture interrupt flag */
        TIMER_ClearCaptureIntFlag(TIMER2);

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
    /* Clear Timer3 time-out interrupt flag */
    TIMER_GetIntFlag(TIMER3);

    g_au32TMRINTCount[3]++;
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCON_IRC22M_EN_Msk);

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable HXT */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL and SysTick source to HCLK/2*/
    CLK_SetCoreClock(PLL_CLOCK);
    CLK_SetSysTickClockSrc(CLK_CLKSEL0_STCLK_S_HCLK_DIV2);

    /* Enable peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(TMR2_MODULE);
    CLK_EnableModuleClock(TMR3_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P3 multi-function pins for UART0 RXD, TXD */
    SYS->P3_MFP = SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD;
    /* Set P4 multi-function pins for TM2 and TM3 */
    SYS->P4_MFP = SYS_MFP_P46_TM2 | SYS_MFP_P47_TM3;
}

void UART0_Init(void)
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
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    volatile uint32_t u32InitCount;
    uint32_t au32CAPValus[10];

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+---------------------------------------------------+\n");
    printf("|    Timer External Capture Function Sample Code    |\n");
    printf("+---------------------------------------------------+\n\n");

    printf("# Timer Settings:\n");
    printf("  Timer3: Clock source is HCLK; Toggle-output mode and frequency is 1 Hz.\n");
    printf("  Timer2: Clock source is HCLK; Clock prescale is 72; Continuous counting mode;\n");
    printf("           TCMP is 0xFFFFFF; Capture pin and capture interrupt enable;\n");
    printf("# Generate 1 Hz frequency from TM3 and connect TM3 pin to TM2 capture pin.\n");
    printf("# Get 1000000 counts when each TM2 capture interrupt occurred.\n\n");

    /* Set TM3 toggle-output frequency to 1 Hz */
    TIMER_Open(TIMER3, TIMER_TOGGLE_MODE, 2);

    /* Set Timer2 default setting */
    TIMER_Open(TIMER2, TIMER_CONTINUOUS_MODE, 1);

    /* Enable TM2 capture function */
    TIMER_SET_PRESCALE_VALUE(TIMER2, (72 - 1));
    TIMER_SET_CMP_VALUE(TIMER2, 0xFFFFFF);
    TIMER_EnableCapture(TIMER2, TIMER_CAPTURE_FREE_COUNTING_MODE, TIMER_CAPTURE_FALLING_EDGE);
    TIMER_EnableCaptureInt(TIMER2);

    /* Enable Timer2 NVIC */
    NVIC_EnableIRQ(TMR2_IRQn);

    /* Clear Timer2 interrupt counts to 0 */
    u32InitCount = g_au32TMRINTCount[2] = 0;

    /* Start Timer2 and Timer3 counting */
    TIMER_Start(TIMER2);
    TIMER_Start(TIMER3);

    /* Check TM2 capture interrupt counts */
    while(g_au32TMRINTCount[2] < 10)
    {
        if(g_au32TMRINTCount[2] != u32InitCount)
        {
            au32CAPValus[u32InitCount] = TIMER_GetCaptureData(TIMER2);
            printf("[%2d] - %4d\n", g_au32TMRINTCount[2], au32CAPValus[u32InitCount]);
            if(u32InitCount > 1)
            {
                if((au32CAPValus[u32InitCount] - au32CAPValus[u32InitCount - 1]) != 1000000)
                {
                    printf("*** FAIL ***\n");
                    while(1);
                }
            }
            u32InitCount = g_au32TMRINTCount[2];
        }
    }

    /* Stop Timer2 and Timer3 counting */
    TIMER_Close(TIMER2);
    TIMER_Close(TIMER3);

    if(g_au32TMRINTCount[2] == 10)
    {
        printf("*** PASS ***\n");
    }
    else
    {
        printf("*** FAIL *** (%d)\n", g_au32TMRINTCount[2]);
    }

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
