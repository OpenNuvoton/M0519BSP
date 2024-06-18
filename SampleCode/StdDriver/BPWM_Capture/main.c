/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 5 $
 * $Date: 15/03/24 10:20a $
 * @brief    Capture the BPWM0 Channel 0 waveform by BPWM0 Channel 1.
 * @note
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
#define PLL_CLOCK           72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/**
 * @brief       BPWM0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle BPWM0 interrupt event
 */
void BPWM0_IRQHandler(void)
{
    uint32_t u32CapIntFlag1;

    /* Handle BPWM0 Capture function */
    u32CapIntFlag1 = BPWM0->CCR;

    /* BPWM0 channel 1 Capture interrupt */
    if(u32CapIntFlag1 & BPWM_CCR_CAPIF1_Msk)
    {
        BPWM0->CCR &= (BPWM_CCR_MASK | BPWM_CCR_CAPIF1_Msk);
    }
}

/*--------------------------------------------------------------------------------------*/
/* Capture function to calculate the input waveform information                         */
/* u32Count[4] : Keep the internal counter value when input signal rising / falling     */
/*               happens                                                                */
/*                                                                                      */
/* time    A    B     C     D                                                           */
/*           ___   ___   ___   ___   ___   ___   ___   ___                              */
/*      ____|   |_|   |_|   |_|   |_|   |_|   |_|   |_|   |_____                        */
/* index              0 1   2 3                                                         */
/*                                                                                      */
/* The capture internal counter down count from 0x10000, and reload to 0x10000 after    */
/* input signal falling happens (Time B/C/D)                                            */
/*--------------------------------------------------------------------------------------*/
int32_t CalPeriodTime(BPWM_T *pwm, uint32_t u32Ch)
{
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod;
    uint32_t u32TimeOutCnt;

    /* Clear Capture Falling Indicator (Time A) */
    BPWM_ClearCaptureIntFlag(pwm, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH);

    /* Wait for Capture Falling Indicator */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(BPWM_GetCaptureIntFlag(pwm, u32Ch) < 2)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for BPWM Capture Falling Indicator time-out!\n");
            return -1;
        }
    }

    /* Clear Capture Falling Indicator (Time B) */
    BPWM_ClearCaptureIntFlag(pwm, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH);

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(BPWM_GetCaptureIntFlag(pwm, u32Ch) < 2)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM Capture Falling Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Falling and Rising Indicator */
        BPWM_ClearCaptureIntFlag(pwm, u32Ch, BPWM_CAPTURE_INT_FALLING_LATCH | BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = BPWM_GET_CAPTURE_FALLING_DATA(pwm, u32Ch);

        /* Wait for Capture Rising Indicator */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(BPWM_GetCaptureIntFlag(pwm, u32Ch) < 2)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM Capture Rising Indicator time-out!\n");
                return -1;
            }
        }

        /* Clear Capture Rising Indicator */
        BPWM_ClearCaptureIntFlag(pwm, u32Ch, BPWM_CAPTURE_INT_RISING_LATCH);

        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = BPWM_GET_CAPTURE_RISING_DATA(pwm, u32Ch);
    }

    u16RisingTime = u32Count[1];

    u16FallingTime = u32Count[0];

    u16HighPeriod = u32Count[1] - u32Count[2];

    u16LowPeriod = 0x10000 - u32Count[1];

    u16TotalPeriod = 0x10000 - u32Count[2];

    printf("PWM generate: \nHigh Period=7199 ~ 7201, Low Period=16799 ~ 16801, Total Period=23999 ~ 24001\n");
    printf("Capture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeriod, u16LowPeriod, u16TotalPeriod);
    if((u16HighPeriod < 7199) || (u16HighPeriod > 7201) || (u16LowPeriod < 16799) || (u16LowPeriod > 16801) || (u16TotalPeriod < 23999) || (u16TotalPeriod > 24001))
    {
        printf("Capture Test Fail!!\n");
        return -1;
    }
    else
    {
        printf("Capture Test Pass!!\n");
        return 0;
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_OSC22M_EN_Msk);

    /* Waiting for Internal RC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_OSC22M_STB_Msk);

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HIRC, CLK_CLKDIV_HCLK(1));

    /* Enable external XTAL 12MHz clock */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for external XTAL clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Switch HCLK clock source to external XTAL 12MHz and HCLK source divide 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Enable BPWM0 module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Set P5 multi-function pins for CLKO */
    SYS->P5_MFP = (SYS->P5_MFP & ~SYS_MFP_P55_Msk) | SYS_MFP_P55_CLKO;

    /* Set P5 multi-function pins for BPWM0 Channel 0 */
    SYS->P5_MFP = (SYS->P5_MFP & ~SYS_MFP_P56_Msk) | SYS_MFP_P56_BPWM0_CH0; /* Enable P5.6 PWM function pin(BPWM0_CH0) */
    SYS->P5_MFP = (SYS->P5_MFP & ~SYS_MFP_P57_Msk) | SYS_MFP_P57_BPWM0_CH1; /* Enable P5.7 PWM function pin(BPWM0_CH1) */
}

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
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32TimeOutCnt;

    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          PWM Driver Sample Code                        |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 1 to capture\n  the signal from BPWM0 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    BPWM0_CH0(P5.6 BPWM0 channel 0) <--> BPWM0_CH1(P5.7 BPWM0 channel 1)\n\n");
    printf("Use BPWM0 Channel 1(P5.7) to capture the BPWM0 Channel 0(P5.6) Waveform\n");

    while(1)
    {
        printf("\nPress any key to start PWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 Channel 0 as PWM output function.                                      */
        /*--------------------------------------------------------------------------------------*/

        /* Assume PWM output frequency is 250Hz and duty ratio is 30%, user can calculate PWM settings by follows.
           duty ratio = (CMR+1)/(CNR+1)
           cycle time = CNR+1
           High level = CMR+1
           PWM clock source frequency = __HXT = 12000000
           (CNR+1) = PWM clock source frequency/prescaler/clock source divider/PWM output frequency
                   = 12000000/2/1/250 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 23999
           duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30%
           CMR = 7199
           Prescale value is 1 : prescaler= 2
           Clock divider is PWM_CSR_DIV1 : clock divider =1
        */

        /* set BPWM0 channel 0 output configuration */
        BPWM_ConfigOutputChannel(BPWM0, BPWM_CH0, 250, 30);

        /* Enable BPWM0 Output path for BPWM0 channel 0 */
        BPWM_EnableOutput(BPWM0, 0x1);

        /* Enable Timer for BPWM0 channel 0 */
        BPWM_Start(BPWM0, 0x1);

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 channel 1  for capture function                                        */
        /*--------------------------------------------------------------------------------------*/

        /* If input minimum frequency is 250Hz, user can calculate capture settings by follows.
           Capture clock source frequency = __HXT = 12000000 in the sample code.
           (CNR+1) = Capture clock source frequency/prescaler/clock source divider/minimum input frequency
                   = 12000000/2/1/250 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 0xFFFF
           (Note: In capture mode, user should set CNR to 0xFFFF to increase capture frequency range.)
        */

        /* set BPWM0 channel 1 capture configuration */
        BPWM_ConfigCaptureChannel(BPWM0, BPWM_CH1, 166, 0);

        /* Enable capture falling edge interrupt for BPWM0 channel 1 */
        BPWM_EnableCaptureInt(BPWM0, BPWM_CH1, BPWM_CAPTURE_INT_FALLING_LATCH);

        /* Enable PWMB NVIC interrupt */
        NVIC_EnableIRQ((IRQn_Type)(BPWM0_IRQn));

        /* Enable Timer for BPWM0 channel 1 */
        BPWM_Start(BPWM0, 0x2);

        /* Enable Capture Function for BPWM0 channel 1 */
        BPWM_EnableCapture(BPWM0, 0x2);

        /* Wait until BPWM0 channel 1 Timer start to count */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(BPWM0->PDR1 == 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM0 channel 1 Timer start to count time-out!\n");
                goto lexit;
            }
        }

        /* Capture the Input Waveform Data */
        if( CalPeriodTime(BPWM0, BPWM_CH1) < 0 ) goto lexit;

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                      */
        /* Set BPWM0 Timer loaded value(CNR) as 0. When BPWM0 internal counter(PDR) reaches to 0, disable BPWM0 Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set BPWM0 channel 0 loaded value as 0 */
        BPWM_Stop(BPWM0, 0x1);

        /* Wait until BPWM0 channel 0 Timer Stop */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(BPWM0->PDR0 != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM0 channel 0 Timer Stop time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for BPWM0 channel 0 */
        BPWM_ForceStop(BPWM0, 0x1);

        /* Disable BPWM0 Output path for BPWM0 channel 0 */
        BPWM_DisableOutput(BPWM0, 0x1);

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 1 (Recommended procedure method 1)                                                      */
        /* Set BPWM0 Timer loaded value(CNR) as 0. When BPWM0 internal counter(PDR) reaches to 0, disable BPWM0 Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Disable BPWM0 NVIC */
        NVIC_DisableIRQ((IRQn_Type)(BPWM0_IRQn));

        /* Set loaded value as 0 for BPWM0 channel 1 */
        BPWM_Stop(BPWM0, 0x2);

        /* Wait until BPWM0 channel 1 current counter reach to 0 */
        u32TimeOutCnt = SystemCoreClock;  /* 1 second time-out */
        while(BPWM0->PDR1 != 0)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for BPWM0 channel 1 current counter reach to 0 time-out!\n");
                goto lexit;
            }
        }

        /* Disable Timer for BPWM0 channel 1 */
        BPWM_ForceStop(BPWM0, 0x2);

        /* Disable Capture Function and Capture Input path for BPWM0 channel 1 */
        BPWM_DisableCapture(BPWM0, 0x2);

        /* Clear Capture Interrupt flag for BPWM0 channel 1 */
        BPWM_ClearCaptureIntFlag(BPWM0, BPWM_CH1, BPWM_CAPTURE_INT_FALLING_LATCH);
    }

lexit:

    while(1);
}




