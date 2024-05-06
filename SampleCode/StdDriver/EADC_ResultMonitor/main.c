/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 5 $
 * $Date: 15/03/19 1:12p $
 * @brief    Monitor the conversion result of channel 2 by the digital compare function.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"


#define PLL_CLOCK           72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcCmp0IntFlag;
volatile uint32_t g_u32AdcCmp1IntFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12 MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Set core clock as PLL_CLOCK from PLL */
    CLK_SetCoreClock(PLL_CLOCK);

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Enable EADC module clock */
    CLK_EnableModuleClock(EADC_MODULE);

    /* ADC clock source is 72 MHz, set divider to 8, EADC clock is 72/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV_EADC(8));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Set P5 multi-function pins for CLKO */
    SYS->P5_MFP = (SYS->P5_MFP & (~SYS_MFP_P55_Msk)) | SYS_MFP_P55_CLKO;

    /* Configure the P60 - P63 ADC analog input pins.  */
    SYS->P6_MFP &= ~(SYS_MFP_P60_Msk | SYS_MFP_P61_Msk |
                     SYS_MFP_P62_Msk | SYS_MFP_P63_Msk);
    SYS->P6_MFP |= (SYS_MFP_P60_EADC0_CH0 | SYS_MFP_P61_EADC0_CH1 |
                    SYS_MFP_P62_EADC0_CH2 | SYS_MFP_P63_EADC0_CH3);

    /* Disable the P60 - P63 digital input path to avoid the leakage current. */
    GPIO_DISABLE_DIGITAL_PATH(P6, 0xF);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Init UART                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
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
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest()
{
    uint32_t u32TimeOutCnt;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|          EADC compare function (result monitor) sample code          |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\nIn this test, software will compare the conversion result of channel 2.\n");

    /* Enable the A/D converter */
    EADC_Open(EADC, NULL);

    /* Configure the sample module A0 for analog input channel 2 and ADINT0 trigger source */
    EADC_ConfigSampleModule(EADC, EADC_SMPA0, EADC_ADINT0_TRIGGER, 2);

    /* Enable EADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 0: channel 2 is less than 0x800; match count is 5.\n");
    EADC_ENABLE_CMP0(EADC, EADC_CMPSMPL_A0, EADC_ADCMPR_CMPCOND_LESS_THAN, 0x800, 0x5);

    /* Enable EADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 1 : channel 2 is greater than or equal to 0x800; match count is 5.\n");
    EADC_ENABLE_CMP1(EADC, EADC_CMPSMPL_A0, EADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL, 0x800, 0x5);

    /* Enable sample module A0 interrupt source */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1);

    /* Clear the A/D ADINT3 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, (0x1 << 3));
    /* Enable ADINT3 interrupt */
    EADC_ENABLE_INT(EADC, (0x1 << 3));
    NVIC_EnableIRQ(EADC3_IRQn);

    /* Clear the EADC comparator 0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, (0x1 << 4));
    /* Enable EADC comparator 0 interrupt */
    EADC_ENABLE_CMP_INT(EADC, 0);

    /* Clear the EADC comparator 1 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, (0x1 << 5));
    /* Enable EADC comparator 1 interrupt */
    EADC_ENABLE_CMP_INT(EADC, 1);

    /* Reset the EADC interrupt indicator and trigger sample module 0 to start A/D conversion */
    g_u32AdcCmp0IntFlag = 0;
    g_u32AdcCmp1IntFlag = 0;
    EADC_START_CONV(EADC, 0x1);

    /* Wait EADC compare interrupt */
    u32TimeOutCnt = EADC_TIMEOUT;
    while((g_u32AdcCmp0IntFlag == 0) && (g_u32AdcCmp1IntFlag == 0))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for EADC compare interrupt time-out!\n");
            return;
        }
    }

    /* Disable the sample module A0 interrupt source */
    EADC_DISABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1);

    /* Disable EADC comparator interrupt */
    EADC_DISABLE_CMP_INT(EADC, 0);
    EADC_DISABLE_CMP_INT(EADC, 1);
    /* Disable compare function */
    EADC_DISABLE_CMP0(EADC);
    EADC_DISABLE_CMP1(EADC);

    if(g_u32AdcCmp0IntFlag == 1)
    {
        printf("Comparator 0 interrupt occurs.\nThe conversion result of channel 2 is less than 0x800\n");
    }
    else
    {
        printf("Comparator 1 interrupt occurs.\nThe conversion result of channel 2 is greater than or equal to 0x800\n");
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC3_IRQHandler(void)
{
    if(EADC_GET_INT_FLAG(EADC, (0x1 << 6)))
    {
        g_u32AdcCmp0IntFlag = 1;
        EADC_CLR_INT_FLAG(EADC, (0x1 << 6));/* Clear the A/D compare flag 0 */
    }

    if(EADC_GET_INT_FLAG(EADC, (0x1 << 7)))
    {
        g_u32AdcCmp1IntFlag = 1;
        EADC_CLR_INT_FLAG(EADC, (0x1 << 7));/* Clear the A/D compare flag 1 */
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_IRQn);

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    printf("Exit EADC sample code\n");

    while(1);

}

