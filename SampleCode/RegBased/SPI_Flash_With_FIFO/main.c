/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 2 $
 * $Date: 15/03/19 1:12p $
 * @brief
 *           SPI Flash Driver Sample Code
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "M0519.h"
#include "spi_flash.h"

#define TEST_PAGE_NUM 10  /* page numbers */
#define BYTE_PER_PAGE 256 /* byte per page */

uint8_t SrcArray[BYTE_PER_PAGE];
uint8_t DestArray[BYTE_PER_PAGE];
uint32_t g_u32SystickCount;

/* Function prototype declaration */
void SYS_Init(void);
void UART0_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t u32Error = 0;
    uint32_t u32ID;
    uint32_t u32Duration_W, u32Duration_R;

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART0_Init();

    /* Configure SPI0 as a master, clock idle low, 8-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    SPI0->CNTRL = SPI_MASTER | (8 << SPI_CNTRL_TX_BIT_LEN_Pos) | SPI_CNTRL_TX_NEG_Msk;
    /* Disable the automatic slave selection function. Configure slave selection signal as low-active. */
    SPI0->SSR = 0;
    /* Set IP clock divider. SPI clock rate = HCLK / ((0+1)*2) = 6 MHz */
    SPI0->DIVIDER = (SPI0->DIVIDER & (~SPI_DIVIDER_DIVIDER_Msk)) | 0;

    /* Enable FIFO mode. */
    SPI0->CNTRL |= SPI_CNTRL_FIFO_Msk;

    printf("\n\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|            M0519 SPI Flash with FIFO Mode Sample Code             |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates how to access a Winbond 25Q16 SPI flash with FIFO buffers.\n");
    printf("I/O connection:\n");
    printf("   SPI0_SS   (P2.6) -- W25Q16 /CS\n   SPI0_CLK  (P2.7) -- W25Q16 CLK\n");
    printf("   SPI0_MISO (P5.1) -- W25Q16 DO \n   SPI0_MOSI (P5.0) -- W25Q16 DI\n");
    printf("The whole SPI flash will be erased. Afterward this sample code will write, read and compare %d-page data.\n", TEST_PAGE_NUM);

    /* Wait ready */
    SpiFlash_WaitReady(SPI0);

    if((u32ID = SpiFlash_ReadMidDid(SPI0)) != 0xEF14)
    {
        printf("Wrong ID, 0x%x\n", u32ID);
        goto lexit;
    }
    else
    {
        printf("Flash found: W25Q16\n");
    }

    printf("Erase chip ...");

    /* Erase SPI flash */
    SpiFlash_ChipErase(SPI0);

    /* Wait ready */
    SpiFlash_WaitReady(SPI0);

    printf("[OK]\n");

    /* Initiate source data buffer */
    for(u32ByteCount = 0; u32ByteCount < BYTE_PER_PAGE; u32ByteCount++)
    {
        SrcArray[u32ByteCount] = u32ByteCount;
    }

    printf("Start to write data to Flash ...");

    g_u32SystickCount = 0;

    /* Configure SysTick */
    /* 1 milli-second per tick */
    SysTick->LOAD = 1000 * CyclesPerUs; /* 1000 us */
    /* Clear SysTick current value register and system tick counter flag */
    SysTick->VAL  = (0x00);
    /* Core clock used for SysTick timer; enable system tick interrupt; start counting. */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Program SPI flash */
    u32FlashAddress = 0;
    for(u32PageNumber = 0; u32PageNumber < TEST_PAGE_NUM; u32PageNumber++)
    {
        /* page program */
        SpiFlash_PageProgram(SPI0, u32FlashAddress, SrcArray, BYTE_PER_PAGE);
        SpiFlash_WaitReady(SPI0);
        u32FlashAddress += 0x100;
    }

    /* Log the duration of write operation */
    u32Duration_W = g_u32SystickCount;
    /* Stop SysTick timer */
    SysTick->CTRL = 0;

    printf("[OK] (%d ms)\n", u32Duration_W);

    /* clear destination data buffer */
    for(u32ByteCount = 0; u32ByteCount < BYTE_PER_PAGE; u32ByteCount++)
    {
        DestArray[u32ByteCount] = 0;
    }

    printf("Read & Compare ...");

    g_u32SystickCount = 0;
    /* Core clock used for SysTick timer; enable system tick interrupt; start counting. */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;

    /* Read SPI flash */
    u32FlashAddress = 0;
    for(u32PageNumber = 0; u32PageNumber < TEST_PAGE_NUM; u32PageNumber++)
    {
        /* page read */
        SpiFlash_ReadData(SPI0, u32FlashAddress, DestArray, BYTE_PER_PAGE);
        u32FlashAddress += 0x100;

        for(u32ByteCount = 0; u32ByteCount < BYTE_PER_PAGE; u32ByteCount++)
        {
            if(DestArray[u32ByteCount] != SrcArray[u32ByteCount])
                u32Error ++;
        }
    }

    /* Log the duration of read operation */
    u32Duration_R = g_u32SystickCount;
    /* Stop SysTick timer */
    SysTick->CTRL = 0;

    if(u32Error == 0)
    {
        printf("[OK] (%d ms)\n", u32Duration_R);
    }
    else
    {
        printf("[FAIL]\n");
    }
   
lexit:

    /* Reset SPI0 */
    SYS->IPRSTC2 |= SYS_IPRSTC2_SPI0_RST_Msk;
    SYS->IPRSTC2 &= ~SYS_IPRSTC2_SPI0_RST_Msk;

    printf("\nExit SPI driver sample code.\n");

    while(1);
}

void SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable external 12 MHz XTAL */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Waiting for clock ready */
    u32TimeOutCnt = __HIRC;
    while(!(CLK->CLKSTATUS & CLK_CLKSTATUS_XTL12M_STB_Msk))
		if(--u32TimeOutCnt == 0) break;

    /* Select HXT as the clock source of HCLK */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLK_S_Msk)) | CLK_CLKSEL0_HCLK_S_HXT;

    /* Select HXT as the clock source of UART; select HCLK as the clock source of SPI0. */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~(CLK_CLKSEL1_UART_S_Msk | CLK_CLKSEL1_SPI0_S_Msk))) | (CLK_CLKSEL1_UART_S_HXT | CLK_CLKSEL1_SPI0_S_HCLK);

    /* Enable UART and SPI0 clock */
    CLK->APBCLK = CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_SPI0_EN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Setup SPI0 multi-function pins */
    SYS->P2_MFP &= ~(SYS_MFP_P26_Msk | SYS_MFP_P27_Msk);
    SYS->P2_MFP |= (SYS_MFP_P26_SPI0_SS | SYS_MFP_P27_SPI0_CLK);
    SYS->P5_MFP &= ~(SYS_MFP_P51_Msk | SYS_MFP_P50_Msk);
    SYS->P5_MFP |= (SYS_MFP_P51_SPI0_MISO | SYS_MFP_P50_SPI0_MOSI);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();
}

void UART0_Init(void)
{
    /* Word length is 8 bits; 1 stop bit; no parity bit. */
    UART0->LCR = UART_LCR_WLS_Msk;
    /* Using mode 2 calculation: UART bit rate = UART peripheral clock rate / (BRD setting + 2) */
    /* UART peripheral clock rate 12 MHz; UART bit rate 115200 bps. */
    /* 12000000 / 115200 bps ~= 104 */
    /* 104 - 2 = 0x66. */
    UART0->BAUD = UART_BAUD_DIV_X_EN_Msk | UART_BAUD_DIV_X_ONE_Msk | (0x66);
}

void SysTick_Handler(void)
{
    g_u32SystickCount++;
    /* Clear SysTick current value register and system tick counter flag */
    SysTick->VAL  = (0x00);
}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/
