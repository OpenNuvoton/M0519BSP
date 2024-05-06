/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 5 $
 * $Date: 15/03/19 1:12p $
 * @brief    Trigger EADC by writing ADSSTR register.
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
volatile uint32_t g_u32AdcIntFlag;

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

    /* EADC clock source is 72 MHz, set divider to 8, EADC clock is 72/8 MHz */
    CLK_SetModuleClock(EADC_MODULE, 0, CLK_CLKDIV_EADC(8));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Set P5 multi-function pins for CLKO */
    SYS->P5_MFP = (SYS->P5_MFP & (~SYS_MFP_P55_Msk)) | SYS_MFP_P55_CLKO;

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
    int32_t  i32ConversionData;
    uint32_t u32TimeOutCnt;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      SWTRG trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("Single end input (channel 2 only)\n");
    /* Enable the A/D converter */
    EADC_Open(EADC, NULL);

    /* Configure the sample module A0 for analog input channel 2 and software trigger source.*/
    EADC_ConfigSampleModule(EADC, EADC_SMPA0, EADC_SOFTWARE_TRIGGER, 2);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, 0x1);

    /* Enable the sample module 0 interrupt.  */
    EADC_ENABLE_INT(EADC, 0x1); /* Enable sample module A/D ADINT0 interrupt */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1); /* Enable sample module 0 interrupt */
    NVIC_EnableIRQ(EADC0_IRQn);

    /* Reset the EADC interrupt indicator and trigger sample module A0 to start A/D conversion */
    g_u32AdcIntFlag = 0;
    EADC_START_CONV(EADC, 0x1);

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

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC, 0x1);

    /* Get the conversion result of the sample module 0 */
    i32ConversionData = EADC_GET_CONV_DATA(EADC, EADC_SMPA0);
    printf("Conversion result of channel 2: 0x%X (%d)\n\n", i32ConversionData, i32ConversionData);
}



/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC, 0x1);      /* Clear the A/D ADINT0 interrupt flag */
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
