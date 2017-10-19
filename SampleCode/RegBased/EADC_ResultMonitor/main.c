/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 4 $
 * $Date: 15/03/19 1:12p $
 * @brief    Monitor the conversion result of channel 2 by the digital compare function.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"

#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK           72000000

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcCmp0IntFlag;
volatile uint32_t g_u32AdcCmp1IntFlag;

/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184 MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(1);

    /* Enable external XTAL 12 MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For SYS_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* Enable EADC module clock */
    CLK->APBCLK |= CLK_APBCLK_EADC_EN_Msk;

    /* EADC clock source is 72 MHz, set divider to 8, EADC clock is 72/8 MHz */
    CLK->CLKDIV  = (CLK->CLKDIV & ~CLK_CLKDIV_EADC_N_Msk) | (((8) - 1) << CLK_CLKDIV_EADC_N_Pos);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Configure the P60 - P63 EADC analog input pins.  */
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
    /* Reset UART IP */
    SYS->IPRSTC2 |=  SYS_IPRSTC2_UART0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_UART0_RST_Msk;

    /* Configure UART0 and set UART0 Baudrate */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
    UART0->LCR = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest()
{
    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|          EADC compare function (result monitor) sample code          |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\nIn this test, software will compare the conversion result of channel 2.\n");

    /* Set the EADC internal sampling time, input mode as single-end and enable the A/D converter */
    EADC->ADCR = EADC_ADCR_AD_EN_Msk;
    /* Configure the sample module A0 for analog input channel 2 and software trigger source */
    EADC->ADSPCRA[0] = EADC_ADINT0_TRIGGER | EADC_ADSPCR_CHSEL(2);

    /* Enable EADC comparator 0. Compare condition: conversion result < 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 0: channel 2 is less than 0x800; match count is 5.\n");
    EADC->ADCMPR[0] = 0x800 << EADC_ADCMPR_CMPD_Pos |
                      0x5 << EADC_ADCMPR_CMPMATCNT_Pos |
                      0x0 << EADC_ADCMPR_CMPSMPL_Pos |
                      EADC_ADCMPR_CMPCOND_LESS_THAN |
                      EADC_ADCMPR_ADCMP_EN_Msk;

    /* Enable EADC comparator 1. Compare condition: conversion result >= 0x800; match Count=5 */
    printf("   Set the compare condition of comparator 1 : channel 2 is greater than or equal to 0x800; match count is 5.\n");
    EADC->ADCMPR[1] = 0x800 << EADC_ADCMPR_CMPD_Pos |
                      0x5 << EADC_ADCMPR_CMPMATCNT_Pos |
                      0x0 << EADC_ADCMPR_CMPSMPL_Pos |
                      EADC_ADCMPR_CMPCOND_GREATER_OR_EQUAL |
                      EADC_ADCMPR_ADCMP_EN_Msk;

    /* Enable sample module A/D ADINT0 interrupt */
    EADC->ADINTSRCTL[0] = 0x1;

    /* Clear the A/D ADINT3 interrupt flag for safe */
    EADC->ADSR1 = EADC_ADSR1_ADF3_Msk;
    /* Enable ADINT3 interrupt */
    EADC->ADCR |= EADC_ADCR_ADIE3_Msk;
    NVIC_EnableIRQ(EADC3_IRQn);

    /* Clear the EADC comparator 0 interrupt flag for safe */
    EADC->ADSR1 = EADC_ADSR1_ADCMPF0_Msk;
    /* Enable EADC comparator 0 interrupt */
    EADC->ADCMPR[0] |= EADC_ADCMPR_ADCMPIE_Msk;

    /* Clear the EADC comparator 1 interrupt flag for safe */
    EADC->ADSR1 = EADC_ADSR1_ADCMPF1_Msk;
    /* Enable EADC comparator 1 interrupt */
    EADC->ADCMPR[1] |= EADC_ADCMPR_ADCMPIE_Msk;

    /* Reset the EADC interrupt indicator and trigger sample module A0 to start A/D conversion */
    g_u32AdcCmp0IntFlag = 0;
    g_u32AdcCmp1IntFlag = 0;
    EADC->ADSSTR |= (0x1 << 0);

    /* Wait EADC compare interrupt */
    while((g_u32AdcCmp0IntFlag == 0) && (g_u32AdcCmp1IntFlag == 0));

    /* Disable the sample module A0 interrupt source */
    EADC->ADINTSRCTL[0] &= ~0x1;

    /* Disable EADC comparator interrupt */
    EADC->ADCMPR[0] &= ~EADC_ADCMPR_ADCMPIE_Msk;
    EADC->ADCMPR[1] &= ~EADC_ADCMPR_ADCMPIE_Msk;
    /* Disable compare function */
    EADC->ADCMPR[0] &= ~EADC_ADCMPR_ADCMP_EN_Msk;
    EADC->ADCMPR[1] &= ~EADC_ADCMPR_ADCMP_EN_Msk;

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
    if(EADC->ADSR1 & EADC_ADSR1_ADCMPF0_Msk)
    {
        g_u32AdcCmp0IntFlag = 1;
        EADC->ADSR1 |= EADC_ADSR1_ADCMPF0_Msk;  /* Clear the A/D compare flag 0 */
    }

    if(EADC->ADSR1 & EADC_ADSR1_ADCMPF1_Msk)
    {
        g_u32AdcCmp1IntFlag = 1;
        EADC->ADSR1 |= EADC_ADSR1_ADCMPF1_Msk;  /* Clear the A/D compare flag 1 */
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
    NVIC_DisableIRQ(EADC3_IRQn);

    /* Reset EADC module */
    SYS->IPRSTC2 |= SYS_IPRSTC2_EADC_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_EADC_RST_Msk;

    /* Disable EADC IP clock */
    CLK->APBCLK &= ~CLK_APBCLK_EADC_EN_Msk;

    printf("Exit EADC sample code\n");

    while(1);

}
