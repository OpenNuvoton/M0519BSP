/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 5 $
 * $Date: 15/03/19 1:12p $
 * @brief    Show how to trigger EADC by timer.
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

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

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

void TIMER0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init TIMER0                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set timer0 periodic time-out period is 3us if timer clock is 72 MHz */
    TIMER_SET_CMP_VALUE(TIMER0, 216);

    /* Start timer counter in periodic mode and enable timer interrupt trigger EADC */
    TIMER0->TCSR = TIMER_PERIODIC_MODE;
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest()
{
    int32_t  i32ConversionData[6] = {0};
    uint32_t u32TimeOutCnt;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      Timer trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");
    printf("Single end input (channel 2 only)\n");
    /* Enable the A/D converter */
    EADC_Open(EADC, NULL);

    /* Configure the sample module A0 for analog input channel 2 and enable Timer0 trigger source */
    EADC_ConfigSampleModule(EADC, EADC_SMPA0, EADC_TIMER0_TRIGGER, 2);

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC_CLR_INT_FLAG(EADC, 0x1);

    /* Enable the sample module 0 interrupt.  */
    EADC_ENABLE_INT(EADC, 0x1); /* Enable sample module A/D ADINT0 interrupt */
    EADC_ENABLE_SAMPLE_MODULE_INT(EADC, 0, 0x1); /* Enable sample module 0 interrupt */
    NVIC_EnableIRQ(EADC0_IRQn);

    printf("Conversion result of channel 2:\n");

    /* Reset the EADC indicator and enable Timer0 counter */
    g_u32AdcIntFlag = 0;
    g_u32COVNUMFlag = 0;
    TIMER_Start(TIMER0);

    while(1)
    {
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

        /* Get the conversion result of the sample module 0 */
        i32ConversionData[g_u32COVNUMFlag - 1] = EADC_GET_CONV_DATA(EADC, EADC_SMPA0);

        if(g_u32COVNUMFlag > 6)
            break;
    }

    /* Disable Timer0 counter */
    TIMER_Stop(TIMER0);

    /* Disable the ADINT0 interrupt */
    EADC_DISABLE_INT(EADC, 0x1);

    for(g_u32COVNUMFlag = 0; (g_u32COVNUMFlag) < 6; g_u32COVNUMFlag++)
        printf("                                0x%X (%d)\n", i32ConversionData[g_u32COVNUMFlag], i32ConversionData[g_u32COVNUMFlag]);
}



/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC0_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    g_u32COVNUMFlag++;
    EADC_CLR_INT_FLAG(EADC, 0x1);/* Clear the A/D ADINT0 interrupt flag */
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

    /* Init TIMER0 for EADC */
    TIMER0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC0_IRQn);

    /* Reset Timer0 module */
    SYS_ResetModule(TMR0_RST);

    /* Reset EADC module */
    SYS_ResetModule(EADC_RST);

    /* Disable Timer0 IP clock */
    CLK_DisableModuleClock(TMR0_MODULE);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC_MODULE);

    printf("Exit EADC sample code\n");

    while(1);

}
