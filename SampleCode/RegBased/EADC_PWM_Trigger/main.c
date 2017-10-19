/****************************************************************************
 * @file     main.c
 * @version  V2.0
 * $Revision: 6 $
 * $Date: 15/03/24 10:24a $
 * @brief    Demonstrate how to trigger EADC by BPWM.
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
volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define Function Prototypes                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
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

    /* Enable BPWM0 module clock */
    CLK->APBCLK |= CLK_APBCLK_BPWM0_EN_Msk;

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

void BPWM0_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init BPWM0                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set BPWM0 timer clock prescaler */
    BPWM_SET_PRESCALER(BPWM0, 0, 1);

    /* Set edge-aligned type */
    BPWM0->PCR &= ~BPWM_PCR_PWM01TYPE_Msk;

    /* Set Pwm mode as Auto-reload mode */
    BPWM0->PCR |= BPWM_PCR_CH0MOD_Msk;

    /* Set BPWM0 timer duty */
    BPWM_SET_CMR(BPWM0, 0, 108);

    /* Set BPWM0 timer period */
    BPWM_SET_CNR(BPWM0, 0, 216);

    /* Enable output of all BPWM0 channels */
    BPWM0->POE |= BPWM_POE_POE1_Msk | BPWM_POE_POE0_Msk;
    GPIO->PWMPOEN &= ~GPIO_PWMPOEN_HZ_BPWM_Msk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest()
{
    int32_t  i32ConversionData[6] = {0};

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                       BPWM trigger mode test                          |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    printf("Single end input (channel 2 only)\n");
    /* Enable the A/D converter */
    EADC->ADCR = EADC_ADCR_AD_EN_Msk;
    /* Configure the sample module A0 for analog input channel 2 and enable BPWM0 trigger source */
    EADC->ADSPCRA[0] = EADC_PWM20_TRIGGER | EADC_ADSPCR_CHSEL(2);

    /* Enable PWM2_CH0 period point trigger EADC */
    EADC->SMPTRGA[0] |= EADC_SMPTRGA_PWM20PEN_Msk;

    /* Clear the A/D ADINT0 interrupt flag for safe */
    EADC->ADSR1 = EADC_ADSR1_ADF0_Msk;

    /* Enable the sample module A0 interrupt */
    EADC->ADCR |= EADC_ADCR_ADIE0_Msk;  /* Enable sample module A/D ADINT0 interrupt */
    EADC->ADINTSRCTL[0] = 0x1;         /* Enable sample module A0 interrupt source */
    NVIC_EnableIRQ(EADC0_IRQn);

    printf("Conversion result of channel 2:\n");

    /* Reset the EADC indicator and enable BPWM0 channel 0 timer */
    g_u32AdcIntFlag = 0;
    g_u32COVNUMFlag = 0;
    BPWM0->PCR |= BPWM_PCR_CH0EN_Msk;

    while(1)
    {
        /* Wait EADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
        while(g_u32AdcIntFlag == 0);

        /* Reset the EADC interrupt indicator */
        g_u32AdcIntFlag = 0;

        /* Get the conversion result of the sample module A0 */
        i32ConversionData[g_u32COVNUMFlag - 1] = (EADC->ADDRA[0] & EADC_ADDR_RSLT_Msk);

        if(g_u32COVNUMFlag > 6)
            break;
    }

    /* Disable BPWM0 channel 0 timer */
    BPWM0->PCR &= ~BPWM_PCR_CH0EN_Msk;

    /* Disable the ADINT0 interrupt */
    EADC->ADCR &= ~EADC_ADCR_ADIE0_Msk;

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

    /* Init BPWM0 for EADC */
    BPWM0_Init();

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

    /* Reset BPWM0 module */
    SYS->IPRSTC2 |= SYS_IPRSTC2_BPWM0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_BPWM0_RST_Msk;

    /* Disable EADC IP clock */
    CLK->APBCLK &= ~CLK_APBCLK_EADC_EN_Msk;

    /* Disable BPWM0 IP clock */
    CLK->APBCLK &= ~CLK_APBCLK_BPWM0_EN_Msk;

    printf("Exit EADC sample code\n");

    while(1);

}
