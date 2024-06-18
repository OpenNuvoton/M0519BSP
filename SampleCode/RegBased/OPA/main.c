
/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/03/19 1:12p $
 * @brief    Demonstrate how OPA works with schmitt trigger buffer.
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M0519.h"


/* Function prototype declaration */
void SYS_Init(void);
void UART_Init(void);

int32_t main(void)
{
    uint32_t u32DelayCnt;

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
    printf("|         M0519 OPA Sample Code         |\n");
    printf("+---------------------------------------+\n");

    printf("\nThis sample code demonstrates OPA digital output function. OP0_P (P8.0) is the OPA0\n");
    printf("positive input pin, OP0_N (P8.1) is the negative input and OP0_O (P8.2) is the OPA output pin.\n");

    printf("When the voltage of the positive input is greater than the voltage of the negative input,\n");
    printf("the digital output state, OPDO0 (OPASR[0]), will be set to 1; otherwise, it will be cleared to 0.\n");
    printf("This sample code will show the relation between the OPA's inputs and show a sequence ");
    printf("number when detecting a transition of OPA's digital output.\n");
    printf("Press any key to start ...");
    getchar();
    printf("\n");

    /* Enable OPA0 schmitt trigger buffer */
    OPA_ENABLE_SCH_TRIGGER(OPA, 0);

    /* Power on the OPA0 circuit */
    OPA_POWER_ON(OPA, 0);

    /* Delay for OPA stable time */
    for(u32DelayCnt = 0; u32DelayCnt < 1500; u32DelayCnt++) __NOP();

    /* Clear OPA0 interrupt flag */
    OPA_CLR_INT_FLAG(OPA, 0);

    /* Enable OPA0 interrupt function */
    OPA_ENABLE_INT(OPA, 0);

    /* Enable ACMP interrupt */
    NVIC_EnableIRQ(ACMP_IRQn);

    while(1);

}

void ACMP_IRQHandler(void)
{
    static uint32_t u32Cnt = 0;

    /* Clear OPA0 interrupt flag */
    OPA_CLR_INT_FLAG(OPA, 0);
    /* Check OPA0 digital output state */
    if(OPA_GET_DIGITAL_OUTPUT(OPA, 0))
        printf("OP0_P voltage > OP0_N voltage (%d)\n", u32Cnt);
    else
        printf("OP0_P voltage <= OP0_N voltage (%d)\n", u32Cnt);

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

    /* Enable UART, ACMP and OPA peripheral clocks */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_ACMP_EN_Msk | CLK_APBCLK_OPA_EN_Msk;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set P8.0, P8.1 and P8.2 as OPA0 positive input pin (OP0_P), OPA0 negative input pin (OP0_N) and OPA0 output pin (OP0_O). */
    SYS->P8_MFP = SYS_MFP_P80_OP0_P | SYS_MFP_P81_OP0_N | SYS_MFP_P82_OP0_O;

    /* Disable digital input path of analog pins, OP0_P (P8.0) and OP0_N (P8.1), to prevent leakage. */
    GPIO_DISABLE_DIGITAL_PATH(P8, BIT0);
    GPIO_DISABLE_DIGITAL_PATH(P8, BIT1);

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


