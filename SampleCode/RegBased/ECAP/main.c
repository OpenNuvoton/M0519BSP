/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 15/03/24 6:08p $
 * @brief    Show how to use ECAP interface to get input frequency
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "M0519.h"

#define PLLCON_SETTING  CLK_PLLCON_72MHz_HXT
#define PLL_CLOCK       72000000


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t u32Status;
uint32_t u32IC0Hold;
/*---------------------------------------------------------------------------------------------------------*/
/*  Timer0 IRQ Handler                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void TMR0_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER0) == 1)
    {
        /* Clear Timer0 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER0);

        /*P8.0 gpio toggle */
        P80 ^= 1;
    }
}
/*---------------------------------------------------------------------------------------------------------*/
/*  ECAP0 IRQ Handler                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CAP0_IRQHandler(void)
{
    /* Get input Capture status */
    u32Status = ECAP_GET_INT_STATUS(ECAP0);

    /* Check input capture channel 0 flag */
    if((u32Status & ECAP_STATUS_CAPF0_Msk) == ECAP_STATUS_CAPF0_Msk)
    {
        /* Clear input capture channel 0 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPF0_Msk);

        /* Get input capture counter hold value */
        u32IC0Hold = ECAP0->HOLD0;
    }

    /* Check input capture channel 1 flag */
    if((u32Status & ECAP_STATUS_CAPF1_Msk) == ECAP_STATUS_CAPF1_Msk)
    {
        /* Clear input capture channel 1 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPF1_Msk);
    }

    /* Check input capture channel 2 flag */
    if((u32Status & ECAP_STATUS_CAPF2_Msk) == ECAP_STATUS_CAPF2_Msk)
    {
        /* Clear input capture channel 2 flag */
        ECAP_CLR_CAPTURE_FLAG(ECAP0, ECAP_STATUS_CAPF2_Msk);
    }

    /* Check input capture compare-match flag */
    if((u32Status & ECAP_STATUS_CMPF_Msk) == ECAP_STATUS_CMPF_Msk)
    {
        /* Clear input capture compare-match flag */
        ECAP_CLR_CMP_MATCH_FLAG(ECAP0);
    }

    /* Check input capture overflow flag */
    if((u32Status & ECAP_STATUS_CAPF2_Msk) == ECAP_STATUS_CAPF2_Msk)
    {
        /* Clear input capture overflow flag */
        ECAP_CLR_OVF_FLAG(ECAP0);
    }
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

    /* Enable UART & ECAP0 & TMR0 module clock */
    CLK->APBCLK |= (CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_ECAP0_EN_Msk | CLK_APBCLK_TMR0_EN_Msk);

    /* Select UART module clock source */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GP3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP = SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD;

    /* Set P2.3 for ECAP0_IC0*/
    SYS->P2_MFP = SYS_MFP_P23_ECAP0_IC0;

}

void UART0_Init(void)
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

void ECAP0_Init(void)
{
    /* Enable ECAP0*/
    ECAP_ENABLE_CNT(ECAP0);

    /* Select Reload function */
    ECAP_SET_CNT_CLEAR_EVENT(ECAP0, ECAP_CNT_CLR_BY_CAPTURE);

    /* Enable ECAP0 Input Channel 0*/
    ECAP_ENABLE_INPUT_CHANNEL(ECAP0, ECAP_CTL0_CAPEN0_Msk);

    /* Enalbe ECAP0 source from IC0 */
    ECAP_SEL_INPUT_SRC(ECAP0, ECAP_IC0, ECAP_CAP_INPUT_SRC_FROM_IC);

    /* Select IC0 detect risign edge */
    ECAP_SEL_CAPTURE_EDGE(ECAP0, ECAP_IC0, ECAP_RISING_EDGE);

    /* Input Channel 0 interrupt enabled */
    ECAP_ENABLE_INT(ECAP0, ECAP_CTL0_CAPIEN0_Msk);

    NVIC_EnableIRQ(ECAP0_IRQn);
}

void Timer0_Init(void)
{
    /* Open Timer0 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER0->TCMPR = (SystemCoreClock / 200);
    TIMER0->TCSR = TIMER_TCSR_IE_Msk | TIMER_PERIODIC_MODE;
    TIMER_SET_PRESCALE_VALUE(TIMER0, (10 - 1));

    /* Enable Timer0 NVIC */
    NVIC_EnableIRQ(TMR0_IRQn);

}
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint32_t u32Hz, u32Hz_DET;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n");
    printf("+----------------------------------------------------------+\n");
    printf("|   M0519 Enhanced Input Capture Timer Driver Sample Code  |\n");
    printf("+----------------------------------------------------------+\n");
    printf("\n");
    printf("  !! GPIO P8.0 toggle periodically    !!\n");
    printf("  !! Connect P8.0 --> P2.3(ECAP0_IC0) !!\n\n");
    printf("     Press any key to start test\n\n");
    getchar();

    /* Initial ECAP0 function */
    ECAP0_Init();

    /* Initial Timer0 function */
    Timer0_Init();

    /* Configure P8.0 as output mode */
    P8->PMD = (P8->PMD & (~GPIO_PMD_PMD0_Msk)) | (GPIO_PMD_OUTPUT << GPIO_PMD_PMD0_Pos);

    u32Hz_DET = 0;

    /* Start Timer0 counting */
    TIMER_Start(TIMER0);

    /* Delay 200ms */
    CLK_SysTickDelay(200000);

    /* Init & clear ECAP inuterrupt status flags */
    u32Status = ECAP_GET_INT_STATUS(ECAP0);
    ECAP0->STATUS = u32Status;

    /* ECAP_CNT starts up-counting */
    ECAP_CNT_START(ECAP0);

    while(1)
    {
        if(u32Status != 0)
        {
            /* Input Capture status is changed, and get a new hold value of input capture counter */
            u32Status = 0;

            /* Caculate the IC0 input frequency */
            u32Hz_DET = SystemCoreClock / (u32IC0Hold + 1);

            /* If IC0 input frequency is changed, print new frquency */
            if(u32Hz != u32Hz_DET)
            {
                u32Hz = u32Hz_DET;
                printf("\nECAP0_IC0 input frequency is %d (Hz)\n", u32Hz);
                printf("\nPlease press any to exit ECAP sample code\n");
            }
        }

        if((DEBUG_PORT->FSR & UART_FSR_RX_EMPTY_Msk) != 0)
            continue;
        else
        {
            TIMER_Stop(TIMER0); //Disable timer Counting.
            break;
        }
    }
    /* Disable External Interrupt */
    NVIC_DisableIRQ(ECAP0_IRQn);

    /* Reset ECAP module */
    SYS->IPRSTC2 |= SYS_IPRSTC2_ECAP0_RST_Msk ;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_ECAP0_RST_Msk ;

    /* Reset Timer0 module */
    SYS->IPRSTC2 |= SYS_IPRSTC2_TMR0_RST_Msk ;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_TMR0_RST_Msk ;

    /* Disable ECAP0*/
    ECAP_DISABLE_CNT(ECAP0);

    /* Disable timer0 IP clock */
    CLK->APBCLK &= ~CLK_APBCLK_TMR0_EN_Msk;

    /* Disable ECAP IP clock */
    CLK->APBCLK &= ~CLK_APBCLK_ECAP0_EN_Msk;

    printf("\nExit ECAP sample code\n");

    while(1);
}



