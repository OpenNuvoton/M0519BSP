/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * $Revision: 1 $
 * $Date: 15/03/19 6:54p $
 * @brief    M0519 SD Card Sample Code
 *
 * @note
 * Copyright (C) 2011 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "M0519.h"
#include "diskio.h"
#include "ff.h"

#define PLLCON_SETTING      CLK_PLLCON_50MHz_HXT
#define PLL_CLOCK           50000000

FATFS FatFs[1];               /* File system object for logical drive */
uint8_t g_au8FsBuf[1024];


FRESULT scan_files(char* path /* Start node to be scanned (also used as work area) */);
void UART_Init(void);
void SYS_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    DRESULT dres;
    FRESULT fres;
    char StringBuf[256] = { 0 };
    FIL file1;
    UINT RetCnt;

    SYS_UnlockReg();
    
    SYS_Init();
    
    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);
    
    printf("\n\n");
    printf("+-------------------------------------------------+\n");
    printf("|            M0519 SD Card Sample Code            |\n");
    printf("+-------------------------------------------------+\n");
    printf("This sample code will list the files on root of SD card and create a \"SDTest.txt\" file.\n");
    printf("I/O connection:\n");
    printf("   P0.0: SD card detection pin\n");
    printf("   P0.1: SD card power control pin\n");
    printf("   P9.7 (SPI1_SS) connects with SD card DAT3 pin\n");
    printf("   P9.6 (SPI1_MOSI) connects with SD card CMD pin\n");
    printf("   P9.5 (SPI1_MISO) connects with SD card DAT0 pin\n");
    printf("   P9.4 (SPI1_CLK) connects with SD card CLK pin\n");
    printf("\n");
    
    
    dres = disk_read(0, g_au8FsBuf, 2, 1);
    if(dres != RES_OK)
    {
        printf("Disk read fail! (%d)\n", dres);
        while(1);
    }
    fres = f_mount(0, &FatFs[0]);
    if(fres != FR_OK)
    {
        printf("Disk read fail! (%d)\n", fres);
        while(1);
    }

    /* Show all dir/files on root */
    printf("Scan files:\n");
    fres = scan_files(StringBuf);
    if(fres != FR_OK)
    {
        goto lexit;
    }
    printf("\n");
    
    printf("Create file \"SDTest.txt\"\n");
    fres = f_open(&file1, "SDTest.txt", FA_WRITE);
    if(fres == FR_NO_FILE)
    {
        fres = f_open(&file1, "SDTest.txt", FA_CREATE_NEW | FA_WRITE);
    }
    if(fres != FR_OK)
    {
        printf("Open file error! (%d)\n", fres);
        goto lexit;
    }
    
    fres = f_write(&file1, "SD card test", 12, &RetCnt);
    if (fres != FR_OK)
    {
        printf("Write file error! (%d)\n", fres);
        f_close(&file1);
        goto lexit;
    }
    
    f_close(&file1);
    
lexit:    
        
    printf("Exit!\n");
    while(1) __NOP();
}



FRESULT scan_files(char* path /* Start node to be scanned (also used as work area) */)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;
    char *fn;   /* This function is assuming non-Unicode cfg. */
    char buf[256];

#if _USE_LFN
    static char lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
    fno.lfname = lfn;
    fno.lfsize = sizeof lfn;
#endif

    res = f_opendir(&dir, path);                       /* Open the directory */
    if(res == FR_OK)
    {
        printf("File list:\n");
        i = strlen(path);
        for(;;)
        {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if(res != FR_OK || fno.fname[0] == 0) break;   /* Break on error or end of dir */
            if(fno.fname[0] == '.') continue;              /* Ignore dot entry */
#if _USE_LFN
            fn = *fno.lfname ? fno.lfname : fno.fname;
#else
            fn = fno.fname;
#endif
            
            if(fno.fattrib & AM_HID)
                continue;
            
            if(fno.fattrib & AM_DIR)                       /* It is a directory */
            {
                sprintf(&path[i], "%s\\", fn);
                printf("\t%s\n", path);
            }
            else                                           /* It is a file. */
            {
                sprintf(buf, "%s", fn);
                printf("\t%s\n", buf);

            }
        }
//        f_closedir(&dir);
    }
    else
        printf("Open dir fail. (%d)\n", res);

    return res;
}


/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime(void)
{
    unsigned long tmr;

    tmr = 0x00000;

    return tmr;
}




void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock (Internal RC 22.1184 MHz) */
    CLK->PWRCON |= CLK_PWRCON_IRC22M_EN_Msk;

    /* Waiting for HIRC clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_IRC22M_STB_Msk);

    /* Switch HCLK clock source to HIRC */
    CLK->CLKSEL0 = CLK_CLKSEL0_HCLK_S_HIRC;

    /* Set PLL to Power-down mode and PLL_STB bit in CLKSTATUS register will be cleared by hardware.*/
    CLK->PLLCON |= CLK_PLLCON_PD_Msk;
    
    /* Enable HXT clock (external XTAL 12MHz) */
    CLK->PWRCON |= CLK_PWRCON_XTL12M_EN_Msk;

    /* Enable PLL and Set PLL frequency */
    CLK->PLLCON = PLLCON_SETTING;
    
    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_CLKSTATUS_PLL_STB_Msk | CLK_CLKSTATUS_XTL12M_STB_Msk);

    /* Switch HCLK clock source to PLL, STCLK to HCLK/2 */
    CLK->CLKSEL0 = CLK_CLKSEL0_STCLK_S_HCLK_DIV2 | CLK_CLKSEL0_HCLK_S_PLL;

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate PllClock, SystemCoreClock and CyclesPerUs automatically. */
    //SystemCoreClockUpdate();
    PllClock        = PLL_CLOCK;            // PLL
    SystemCoreClock = PLL_CLOCK / 1;        // HCLK
    CyclesPerUs     = SystemCoreClock / 1000000;  // For CLK_SysTickDelay()

    /* Enable UART and SPI1 peripheral clock */
    CLK->APBCLK |= CLK_APBCLK_UART0_EN_Msk | CLK_APBCLK_SPI1_EN_Msk;

    /* Select UART module clock source as HXT and UART module clock divider as 1 */
    CLK->CLKSEL1 &= ~CLK_CLKSEL1_UART_S_Msk;
    CLK->CLKSEL1 |= CLK_CLKSEL1_UART_S_HXT;
    CLK->CLKDIV &= ~(CLK_CLKDIV_UART_N_Msk);
    

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set P3 multi-function pins for UART0 RXD and TXD */
    SYS->P3_MFP &= ~(SYS_MFP_P30_Msk | SYS_MFP_P31_Msk);
    SYS->P3_MFP |= (SYS_MFP_P30_UART0_RXD | SYS_MFP_P31_UART0_TXD);

    /* Set SPI1 multi-function pins for SD card interface */
    SYS->P9_MFP &= ~(SYS_MFP_P97_Msk | SYS_MFP_P96_Msk | SYS_MFP_P95_Msk | SYS_MFP_P94_Msk);
    SYS->P9_MFP |= (SYS_MFP_P97_SPI1_SS | SYS_MFP_P96_SPI1_MOSI | SYS_MFP_P95_SPI1_MISO | SYS_MFP_P94_SPI1_CLK);

}


