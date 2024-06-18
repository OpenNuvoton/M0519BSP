/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 15/03/12 3:18p $
 * @brief    Use ADINT interrupt to do the EADC continuous scan conversion.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 *
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "M0519.h"

#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK           72000000

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;

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
    CyclesPerUs     = SystemCoreClock / 1000000;  // For CLK_SysTickDelay()

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
    uint8_t  u32SAMPLECount = 0;
    int32_t  i32ConversionData[8] = {0};
    uint32_t u32TimeOutCnt = 0;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      ADINT trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 2 cycles of conversion result from the specified channels.\n");

    printf("Single end input (sample module A channel 0, 1, 2 and 3)\n");
    /* Enable the A/D converter */
    EADC->ADCR = EADC_ADCR_AD_EN_Msk;

    /* Configure the sample module A4 for analog input channel 0 and enable ADINT0 trigger source */
    EADC->ADSPCRA[4] = EADC_ADINT0_TRIGGER | EADC_ADSPCR_CHSEL(0);
    /* Configure the sample module A5 for analog input channel 1 and enable ADINT0 trigger source */
    EADC->ADSPCRA[5] = EADC_ADINT0_TRIGGER | EADC_ADSPCR_CHSEL(1);
    /* Configure the sample module A6 for analog input channel 2 and enable ADINT0 trigger source */
    EADC->ADSPCRA[6] = EADC_ADINT0_TRIGGER | EADC_ADSPCR_CHSEL(2);
    /* Configure the sample module A7 for analog input channel 3 and enable ADINT0 trigger source */
    EADC->ADSPCRA[7] = EADC_ADINT0_TRIGGER | EADC_ADSPCR_CHSEL(3);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC->ADSR1 = EADC_ADSR1_ADF0_Msk;

    /* Enable the sample module A7 interrupt */
    EADC->ADCR |= EADC_ADCR_ADIE0_Msk;  /* Enable sample module A/D ADINT0 interrupt */
    EADC->ADINTSRCTL[0] = 0x1ul << 7;  /* Enable sample module A7 interrupt source */
    NVIC_EnableIRQ(EADC0_IRQn);

    /* Reset the EADC indicator and trigger sample module A7 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    g_u32COVNUMFlag = 0;
    EADC->ADSSTR |= (0x1 << 7);

    /* Wait EADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
    u32TimeOutCnt = EADC_TIMEOUT;
    while(g_u32AdcIntFlag == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for EADC interrupt time-out!\n");
            return;
        }
    }

    /* Reset the EADC interrupt indicator */
    g_u32AdcIntFlag = 0;

    /* Wait EADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
    u32TimeOutCnt = EADC_TIMEOUT;
    while(g_u32AdcIntFlag == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for EADC interrupt time-out!\n");
            return;
        }
    }

    /* Reset the EADC interrupt indicator */
    g_u32AdcIntFlag = 0;

    /* Disable the sample module A7 interrupt */
    EADC->ADINTSRCTL[0] &= ~(0x1 << 7); /* Disable sample module A7 interrupt source */

    /* Get the conversion result of the sample module */
    for(u32SAMPLECount = 0; u32SAMPLECount < 4; u32SAMPLECount++)
        i32ConversionData[u32SAMPLECount] = (EADC->ADDRA[u32SAMPLECount + 4] & EADC_ADDR_RSLT_Msk);

    /* Wait conversion done */
    u32TimeOutCnt = EADC_TIMEOUT;
    while(EADC->ADSR0 != 0xF0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for EADC conversion done time-out!\n");
            return;
        }
    }

    /* Get the conversion result of the sample module */
    for(u32SAMPLECount = 4; u32SAMPLECount < 8; u32SAMPLECount++)
        i32ConversionData[u32SAMPLECount] = (EADC->ADDRA[u32SAMPLECount] & EADC_ADDR_RSLT_Msk);

    for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 8; g_u32COVNUMFlag++)
        printf("Conversion result of channel %d: 0x%X (%d)\n", (g_u32COVNUMFlag % 4), i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);

}



/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC->ADSR1 = EADC_ADSR1_ADF0_Msk;      /* Clear the A/D ADINT0 interrupt flag */
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
    SYS->IPRSTC2 |= SYS_IPRSTC2_EADC_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_EADC_RST_Msk;

    /* Disable EADC IP clock */
    CLK->APBCLK &= ~CLK_APBCLK_EADC_EN_Msk;

    printf("Exit EADC sample code\n");

    while(1);

}

