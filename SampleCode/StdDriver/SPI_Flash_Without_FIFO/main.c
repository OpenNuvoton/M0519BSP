/******************************************************************************
 * @file     main.c
 * @version  V2.00
 * $Revision: 3 $
 * $Date: 15/03/19 1:13p $
 * @brief
 *           SPI Flash Driver Sample Code
 * @note
 * Copyright (C) 2014 Nuvoton Technology Corp. All rights reserved.
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
    UART_Open(UART0, 115200);

    /* Configure SPI0 as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 6 MHz */
    SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, 6000000);

    /* Disable the automatic slave selection function. */
    SPI_DisableAutoSS(SPI0);

    printf("\n\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("|          M0519 SPI Flash without FIFO Mode Sample Code            |\n");
    printf("+-------------------------------------------------------------------+\n");
    printf("\n");
    printf("\nThis sample code demonstrates how to access a Winbond 25Q16 SPI flash without FIFO buffers.\n");
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
    SPI_Close(SPI0);

    printf("\nExit SPI driver sample code.\n");

    while(1);
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable external 12 MHz XTAL */
    CLK_EnableXtalRC(CLK_PWRCON_XTL12M_EN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Switch HCLK clock source to HXT and HCLK source divider is 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLK_S_HXT, CLK_CLKDIV_HCLK(1));

    /* Select HXT as the clock source of UART0 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART_S_HXT, CLK_CLKDIV_UART(1));

    /* Select HCLK as the clock source of SPI0 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL1_SPI0_S_HCLK, MODULE_NoMsk);

    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);
    /* Enable SPI0 peripheral clock */
    CLK_EnableModuleClock(SPI0_MODULE);

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

void SysTick_Handler(void)
{
    g_u32SystickCount++;
    /* Clear SysTick current value register and system tick counter flag */
    SysTick->VAL  = (0x00);
}


/*** (C) COPYRIGHT 2014 Nuvoton Technology Corp. ***/

