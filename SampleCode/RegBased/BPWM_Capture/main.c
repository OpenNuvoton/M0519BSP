/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 4 $
 * $Date: 15/03/24 10:19a $
 * @brief    Capture the BPWM0 Channel 0 waveform by BPWM0 Channel 1.
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"


#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK       72000000

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
void CalPeriodTime()
{
    uint16_t u32Count[4];
    uint32_t u32i;
    uint16_t u16RisingTime, u16FallingTime, u16HighPeroid, u16LowPeroid, u16TotalPeroid;

    /* Clear Capture Falling Indicator (Time A) */
    BPWM0->CCR = (BPWM0->CCR & BPWM_CCR_MASK) | BPWM_CCR_CFLRI1_Msk;

    /* Wait for Capture Falling Indicator  */
    while((BPWM0->CCR >> BPWM_CCR_CFLRI1_Pos & 1) == 0);
    /* Clear Capture Falling Indicator (Time B)*/
    BPWM0->CCR = (BPWM0->CCR & BPWM_CCR_MASK) | BPWM_CCR_CFLRI1_Msk;

    u32i = 0;

    while(u32i < 4)
    {
        /* Wait for Capture Falling Indicator */
        while((BPWM0->CCR >> BPWM_CCR_CFLRI1_Pos & 1) == 0);

        /* Clear Capture Falling Indicator */
        BPWM0->CCR = (BPWM0->CCR & BPWM_CCR_MASK) | BPWM_CCR_CFLRI1_Msk;

        /* Clear Capture Rising Indicator */
        BPWM0->CCR = (BPWM0->CCR & BPWM_CCR_MASK) | BPWM_CCR_CRLRI1_Msk;

        /* Get Capture Falling Latch Counter Data */
        u32Count[u32i++] = BPWM0->CFLR1;

        /* Wait for Capture Rising Indicator */
        while((BPWM0->CCR >> BPWM_CCR_CRLRI1_Pos & 1) == 0);

        /* Clear Capture Rising Indicator */
        BPWM0->CCR = (BPWM0->CCR & BPWM_CCR_MASK) | BPWM_CCR_CRLRI1_Msk;

        /* Get Capture Rising Latch Counter Data */
        u32Count[u32i++] = BPWM0->CRLR1;
    }

    u16RisingTime = u32Count[1];

    u16FallingTime = u32Count[0];

    u16HighPeroid = u32Count[1] - u32Count[2];

    u16LowPeroid = 0x10000 - u32Count[1];

    u16TotalPeroid = 0x10000 - u32Count[2];

    printf("PWM generate: \nHigh Period=7199 ~ 7201, Low Period=16799 ~ 16801, Total Period=23999 ~ 24001\n");
    printf("capture Result: Rising Time = %d, Falling Time = %d \nHigh Period = %d, Low Period = %d, Total Period = %d.\n\n",
           u16RisingTime, u16FallingTime, u16HighPeroid, u16LowPeroid, u16TotalPeroid);
    if((u16HighPeroid < 7199) || (u16HighPeroid > 7201) || (u16LowPeroid < 16799) || (u16LowPeroid > 16801) || (u16TotalPeroid < 23999) || (u16TotalPeroid > 24001))
        printf("Capture Test Fail!!\n");
    else
        printf("Capture Test Pass!!\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable Internal RC 22.1184MHz clock */
    CLK->PWRCON |= CLK_PWRCON_OSC22M_EN_Msk;

    /* Waiting for Internal RC clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_OSC22M_STB_Msk));

    /* Switch HCLK clock source to Internal RC and HCLK source divide 1 */
    CLK->CLKSEL0 &= ~CLK_CLKSEL0_HCLK_S_Msk;
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_HIRC;
    CLK->CLKDIV &= ~CLK_CLKDIV_HCLK_N_Msk;
    CLK->CLKDIV |= CLK_CLKDIV_HCLK(1);

    /* Enable external XTAL 12MHz clock */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for external XTAL clock ready */
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk));

    /* Set core clock as PLL_CLOCK from PLL */
    CLK->PLLCON = PLLCON_SETTING;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_PLL_STB_Msk));
    CLK->CLKSEL0 &= (~CLK_CLKSEL0_HCLK_S_Msk);
    CLK->CLKSEL0 |= CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CycylesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = PLL_CLOCK / 1000000;  // For SYS_SysTickDelay()

    /* Enable UART module clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk;

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /* Enable BPWM0 module clock */
    CLK->APBCLK |= CLK_APBCLK_BPWM0_EN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Set P5 multi-function pins for BPWM0 Channel 0 and channel 1 */
    SYS->P5_MFP = SYS->P5_MFP & (~SYS_MFP_P56_Msk) | SYS_MFP_P56_BPWM0_CH0; /* Enable P5.6 PWM function pin(BPWM0_CH0) */
    SYS->P5_MFP = SYS->P5_MFP & (~SYS_MFP_P57_Msk) | SYS_MFP_P57_BPWM0_CH1; /* Enable P5.7 PWM function pin(BPWM0_CH1) */
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
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will use BPWM0 channel 1 to capture\n  the signal from BPWM0 channel 0.\n");
    printf("  I/O configuration:\n");
    printf("    BPWM0_CH0(P5.6 BPWM0 channel 0) <--> BPWM0_CH1(P5.7 BPWM0 channel 1)\n\n");
    printf("Use BPWM0 Channel 1(P5.7) to capture the BPWM0 Channel 0(P5.6) Waveform\n");

    while(1)
    {
        printf("\nPress any key to start BPWM Capture Test\n");
        getchar();

        /*--------------------------------------------------------------------------------------*/
        /* Set the BPWM0 Channel 0 as BPWM output function.                                     */
        /*--------------------------------------------------------------------------------------*/

        /* Assume BPWM output frequency is 250Hz and duty ratio is 30%, user can calculate BPWM settings by follows.
           duty ratio = (CMR+1)/(CNR+1)
           cycle time = CNR+1
           High level = CMR+1
           BPWM clock source frequency = __HXT = 12000000
           (CNR+1) = BPWM clock source frequency/prescaler/clock source divider/BPWM output frequency
                   = 12000000/2/1/250 = 24000
           (Note: CNR is 16 bits, so if calculated value is larger than 65536, user should increase prescale value.)
           CNR = 23999
           duty ratio = 30% ==> (CMR+1)/(CNR+1) = 30%
           CMR = 7199
           Prescale value is 1 : prescaler= 2
           Clock divider is PWM_CSR_DIV1 : clock divider =1
        */
        /* Set Pwm mode */
        BPWM0->PCR |= BPWM_PCR_CH0MOD_Msk;

        /* Set BPWM0 Timer clock prescaler */
        BPWM0->PPR = (BPWM0->PPR & ~(BPWM_PPR_CP01_Msk)) | (1 << BPWM_PPR_CP01_Pos);

        /* Set BPWM0 Timer clock divider select */
        BPWM0->CSR = (BPWM0->CSR & ~(BPWM_CSR_CSR0_Msk)) | (BPWM_CLK_DIV_1 << BPWM_CSR_CSR0_Pos);

        /* Set BPWM0 Timer duty */
        BPWM0->CMR0 = 7199;

        /* Set BPWM0 Timer period */
        BPWM0->CNR0 = 23999;

        /* Enable BPWM0 Output path for BPWM0 channel 0 */
        BPWM0->POE |= BPWM_POE_POE0_Msk;
        GPIO->PWMPOEN &= ~GPIO_PWMPOEN_HZ_BPWM_Msk;

        /* Enable Timer for BPWM0 channel 0 */
        BPWM0->PCR |= BPWM_PCR_CH0EN_Msk;

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
        /* Set Pwm mode */
        BPWM0->PCR |= BPWM_PCR_CH1MOD_Msk;

        /* Set BPWM0 Timer clock prescaler */
        BPWM0->PPR = (BPWM0->PPR & ~(BPWM_PPR_CP01_Msk)) | (1 << BPWM_PPR_CP01_Pos);

        /* Set BPWM0 Timer clock divider select */
        BPWM0->CSR = (BPWM0->CSR & ~(BPWM_CSR_CSR1_Msk)) | (BPWM_CLK_DIV_1 << BPWM_CSR_CSR1_Pos);

        /* Set BPWM0 Timer loaded value */
        BPWM0->CNR1 = 0xFFFF;

        /* Enable capture falling edge interrupt for BPWM0 channel 1 */
        BPWM0->CCR = (BPWM0->CCR & BPWM_CCR_MASK) | BPWM_CCR_CFL_IE1_Msk;

        /* Enable BPWM0 NVIC interrupt */
        NVIC_EnableIRQ((IRQn_Type)(BPWM0_IRQn));

        /* Enable Capture Function for BPWM0 channel 1 */
        BPWM0->CCR = (BPWM0->CCR & BPWM_CCR_MASK) | BPWM_CCR_CAPCH1EN_Msk;

        /* Enable Timer for BPWM0 channel 1 */
        BPWM0->PCR |= BPWM_PCR_CH1EN_Msk;

        /* Wait until BPWM0 channel 1 Timer start to count */
        while(BPWM0->PDR1 == 0);

        /* Enable capture input path for BPWM0 channel 1 */
        BPWM0->CAPENR |= BPWM_CAPENR_CINEN1_Msk;

        /* Capture the Input Waveform Data */
        CalPeriodTime();
        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 0 (Recommended procedure method 1)                                                      */
        /* Set BPWM0 Timer loaded value(CNR) as 0. When BPWM0 internal counter(PDR) reaches to 0, disable BPWM0 Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Set BPWM0 channel 0 loaded value as 0 */
        BPWM0->CNR0 = 0;

        /* Wait until BPWM0 channel 0 Timer Stop */
        while(BPWM0->PDR0 != 0);

        /* Disable Timer for BPWM0 channel 0 */
        BPWM0->PCR &= ~BPWM_PCR_CH0EN_Msk;

        /* Disable BPWM0 Output path for BPWM0 channel 0 */
        BPWM0->POE &= ~BPWM_POE_POE0_Msk;

        /*------------------------------------------------------------------------------------------------------------*/
        /* Stop BPWM0 channel 1 (Recommended procedure method 1)                                                      */
        /* Set BPWM0 Timer loaded value(CNR) as 0. When BPWM0 internal counter(PDR) reaches to 0, disable BPWM0 Timer */
        /*------------------------------------------------------------------------------------------------------------*/

        /* Disable BPWM0 NVIC */
        NVIC_DisableIRQ((IRQn_Type)(BPWM0_IRQn));

        /* Set loaded value as 0 for BPWM0 channel 1 */
        BPWM0->CNR1 = 0;

        /* Wait until BPWM0 channel 1 current counter reach to 0 */
        while(BPWM0->PDR1 != 0);

        /* Disable Timer for BPWM0 channel 1 */
        BPWM0->PCR &= ~BPWM_PCR_CH1EN_Msk;

        /* Disable Capture Function for BPWM0 channel 1 */
        BPWM0->CCR = (BPWM0->CCR & BPWM_CCR_MASK) & ~BPWM_CCR_CAPCH1EN_Msk;

        /* Clear Capture Interrupt flag for BPWM0 channel 1 */
        BPWM0->CCR = (BPWM0->CCR & BPWM_CCR_MASK) | (BPWM_CCR_CAPIF1_Msk);

        /* Disable Capture Input path for BPWM0 channel 1 */
        BPWM0->CAPENR &= ~BPWM_CAPENR_CINEN1_Msk;
    }
}

/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
