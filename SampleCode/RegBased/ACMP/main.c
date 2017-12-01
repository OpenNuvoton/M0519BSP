
/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 4 $
 * $Date: 15/03/19 1:12p $
 * @brief    Demonstrate how ACMP works with internal band-gap voltage.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M0519.h"


/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for print message */
    UART_Init();

    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|        M0519 ACMP Sample Code         |\n");
    printf("+---------------------------------------+\n");

    printf("\nThis sample code demonstrates ACMP0 function. Using ACMP0_P (P8.4) as ACMP0\n");
    printf("positive input and using internal band-gap voltage as the negative input\n");
    printf("The compare result reflects on ACMP0_O (P8.7)\n");

    /* Configure ACMP0. Enable ACMP0, enable interrupt and select internal reference voltage as negative input. */
    ACMP->CR[0] = ACMP_CR_ACMP_EN_Msk | ACMP_CR_ACMPIE_Msk | ACMP_CR_CN_Msk;

    /* Enable ACMP interrupt */
    NVIC_EnableIRQ(ACMP_IRQn);

    while(1);

}

void ACMP_IRQHandler(void)
{
    static uint32_t u32Cnt = 0;

    /* Clear ACMP0 interrupt flag */
    ACMP->SR = ACMP_SR_ACMPF0_Msk;
    /* Check Comparator 0 Output Status */
    if(ACMP->SR & ACMP_SR_CO0_Msk)
        printf("ACMP0_P voltage > Band-gap voltage (%d)\n", u32Cnt);
    else
        printf("ACMP0_P voltage <= Band-gap voltage (%d)\n", u32Cnt);

    u32Cnt++;
}


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable external 12 MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Select HXT as the clock source of UART */
    CLK->CLKSEL1 &= (~CLK_CLKSEL1_UART_S_Msk);

    /* Enable UART and ACMP clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_ACMP_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
		/* Set P8.4 and P8.7 multi-function configuration as ACMP0 positive input pin (ACMP0_P) and ACMP0 output pin (ACMP0_O). */
		SYS->P8_MFP &= ~(SYS_MFP_P87_Msk | SYS_MFP_P84_Msk);
		SYS->P8_MFP |= (SYS_MFP_P87_ACMP0_O | SYS_MFP_P84_ACMP0_P);


    /* Disable digital input path of analog pin ACMP0_P (P8.4) to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(P8, BIT4);

		/* Set P3 multi-function pins for UART0 RXD and TXD */
		SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
		SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);
}

void UART_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LCR = UART_LCR_WLS_Msk;
    /* Using mode 2 calculation: UART bit rate = UART peripheral clock rate / (BRD setting + 2) */
    /* UART peripheral clock rate 12MHz; UART bit rate 115200 bps. */
    /* 12000000 / 115200 bps ~= 104 */
    /* 104 - 2 = 0x66. */
    UART0->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (0x66);
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/


