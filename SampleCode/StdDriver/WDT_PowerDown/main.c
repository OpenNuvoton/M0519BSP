/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/03/19 1:13p $
 * @brief    Use WDT time-out interrupt event to wake-up system.
 * @note
 * Copyright (C) 2013 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"

#define PLL_CLOCK           72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8IsWDTWakeupINT = 0;


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power-down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    printf("System enter to power-down mode.\n\n");

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(UART0)
        if(--u32TimeOutCnt == 0) break;

    CLK->PWRCON |= CLK_PWRCON_PD_WU_INT_EN_Msk;

    SCB->SCR = 4;

    __WFI();
}

/**
 * @brief       IRQ Handler for WDT and WWDT Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The WDT_IRQHandler is default IRQ of WDT and WWDT, declared in startup_M0519.s.
 */
void WDT_IRQHandler(void)
{
    if((WDT_GET_TIMEOUT_INT_FLAG() == 1) && (WDT_GET_TIMEOUT_WAKEUP_FLAG() == 1))
    {
        /* Clear WDT time-out interrupt and wake-up flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
        WDT_CLEAR_TIMEOUT_WAKEUP_FLAG();

        g_u8IsWDTWakeupINT = 1;

        printf("WDT time-out wake-up interrupt occurred.\n");
    }
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
    CLK_EnableModuleClock(WDT_MODULE);

    /* Peripheral clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_PLL, CLK_CLKDIV_UART(1));
    CLK_SetModuleClock(WDT_MODULE, CLK_CLKSEL1_WDT_S_LIRC, 0);

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
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Use P0.0 to check time-out period time */
    GPIO->PWMPOEN = 0;
    GPIO_SetMode(P0, BIT0, GPIO_PMD_OUTPUT);
    P00 = 0;

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+----------------------------------------------+\n");
    printf("|    WDT Power-down and Wake-up Sample Code    |\n");
    printf("+----------------------------------------------+\n\n");

    printf("# WDT Settings:\n");
    printf("  Clock source is 10 kHz; Enable interrupt; Enable Wake-up; Time-out interval is 2^16 * WDT clock.\n");
    printf("# When WDT start counting, system will generate a WDT time-out interrupt after 6.5536 ~ 6.656 s.\n");
    printf("  Measure P0.0 high period to check time-out interval and system can be waken-up by WDT time-out event.\n\n");

    P00 = 1;

    /* Because of all bits can be written in WDT Control Register are write-protected;
       To program it needs to disable register protection first. */
    SYS_UnlockReg();

    /* Enable WDT wake-up function and select time-out interval to 2^16 * WDT clock then start WDT counting */
    g_u8IsWDTWakeupINT = 0;
    WDT_Open(WDT_TIMEOUT_2POW16, WDT_RESET_DELAY_1026CLK, FALSE, TRUE);

    /* Enable WDT interrupt function */
    WDT_EnableInt();

    /* Enable WDT NVIC */
    NVIC_EnableIRQ(WDT_IRQn);

    /* System entry into Power-down Mode */
    PowerDownFunction();

    /* Check if WDT time-out interrupt and wake-up interrupt flag occurred */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(1)
    {
        if(((CLK->PWRCON & CLK_PWRCON_PD_WU_STS_Msk) == CLK_PWRCON_PD_WU_STS_Msk) && (g_u8IsWDTWakeupINT == 1))
            break;

        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for System or WDT interrupt time-out!\n");
            break;
        }
    }

    P00 = 0;

    printf("Wake-up test PASS.\n");

    /* Clear Power-down wake-up interrupt flag */
    CLK->PWRCON |= CLK_PWRCON_PD_WU_STS_Msk;

    /* Disable WDT counting */
    WDT_Close();

    while(1);
}

/*** (C) COPYRIGHT 2013 Nuvoton Technology Corp. ***/
