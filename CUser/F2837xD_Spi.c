//###########################################################################
//
// FILE:   F2837xD_Spi.c
//
// TITLE:  F2837xD SPI Initialization & Support Functions.
//
//###########################################################################
// $TI Release: F2837xD Support Library v210 $
// $Release Date: Tue Nov  1 14:46:15 CDT 2016 $
// $Copyright: Copyright (C) 2013-2016 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

//
// Included Files
//
#include "F2837xD_device.h"
#include "F2837xD_Examples.h"

//
// Calculate BRR: 7-bit baud rate register value
// SPI CLK freq = 500 kHz
// LSPCLK freq  = CPU freq / 4  (by default)
// BRR          = (LSPCLK freq / SPI CLK freq) - 1
//
#if CPU_FRQ_200MHZ
#define SPI_BRR        ((200E6 / 4) / 5E6) - 1 // 12.5 MHz
#endif

#if CPU_FRQ_150MHZ
#define SPI_BRR        ((150E6 / 4) / 500E3) - 1
#endif

#if CPU_FRQ_120MHZ
#define SPI_BRR        ((120E6 / 4) / 500E3) - 1
#endif


//
// InitSPI - This function initializes the SPI to a known state
//
void InitSpi(void)
{
    // GPIO配置
    //InitSpiaGpio();

    // 上电复位后，SPI工作在标准模式下，禁止SPI FIFO功能
    // Initialize SPI FIFO registers
    //SpiaRegs.SPIFFTX.all=0xE040;
    //SpiaRegs.SPIFFRX.all=0x204f;
    //SpiaRegs.SPIFFTX.all=0xe021;      // Enable FIFO's, set TX FIFO level to 1
    //SpiaRegs.SPIFFRX.all=0x0;//0x0021;      // Set RX FIFO level to 8
    //SpiaRegs.SPIFFCT.all=0x0;

    // SPI 寄存器配置

    //SpiaRegs.SPICCR.all =0x000F;    // Reset on, output at rising edge, 16-bit char bits
    // Set reset low before configuration changes
    // Clock polarity (0 == rising, 1 == falling)
    // 16-bit character
    // Disable loop-back
    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpiaRegs.SPICCR.bit.SPICHAR = (16-1);
    SpiaRegs.SPICCR.bit.SPILBK = 0;

    SpicRegs.SPICCR.bit.SPISWRESET = 0;
    SpicRegs.SPICCR.bit.CLKPOLARITY = 0;
    SpicRegs.SPICCR.bit.SPICHAR = (16-1);
    SpicRegs.SPICCR.bit.SPILBK = 0;

    //SpiaRegs.SPICTL.all =0x0006;    // CLOCK PHASE=0, Master Mode, enable talk, and SPI int disabled.
    // Enable master (0 == slave, 1 == master)
    // Enable transmission (Talk)
    // Clock phase (0 == normal, 1 == delayed)
    // SPI interrupts are disabled
    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpiaRegs.SPICTL.bit.TALK = 1;
    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
    SpiaRegs.SPICTL.bit.SPIINTENA = 0;

    SpicRegs.SPICTL.bit.MASTER_SLAVE = 1;
    SpicRegs.SPICTL.bit.TALK = 1;
    SpicRegs.SPICTL.bit.CLK_PHASE = 0;
    SpicRegs.SPICTL.bit.SPIINTENA = 0;

    // Set the baud rate
    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;
    SpicRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR; //SpiaRegs.SPIBRR = 0x1;           // SPI Baud Rate = LSPCLK/(SPIBRR+1), 根据书上公式，LSPCLK=37.5MHz so=37.5/4=9.375MHz

    SpiaRegs.SPICCR.all = 0x008F;    // 在改变设置前将RESET清零，并在设置结束后将其置位
    SpicRegs.SPICCR.all = 0x008F;    // 在改变设置前将RESET清零，并在设置结束后将其置位

    // Set FREE bit
    // Halting on a breakpoint will not halt the SPI
    SpiaRegs.SPIPRI.bit.FREE = 1;   // breakpoints don't disturb xmission
    SpicRegs.SPIPRI.bit.FREE = 1;   // breakpoints don't disturb xmission


    // 唤醒MAX5307
    GpioDataRegs.GPDSET.bit.GPIO103 = 1;             //cs=1
    NOP;
    NOP;
    GpioDataRegs.GPDCLEAR.bit.GPIO103 = 1;           //cs=0

    SpicRegs.SPITXBUF=0xfffc;                       //MAX5307唤醒字符
    while(SpicRegs.SPISTS.bit.INT_FLAG!=1){NOP;}    // 数据传完后INT_FLAG会置位

    GpioDataRegs.GPDSET.bit.GPIO103 = 1;             //cs=1为下一次做准备

    SpicRegs.SPICCR.bit.SPISWRESET=0;               //通过reset 清楚SPI中断标志INT_FLAG
    NOP;
    NOP;
    // Release the SPI from reset
    SpicRegs.SPICCR.bit.SPISWRESET=1;               // Relinquish SPI from Reset

}

//
////
//// InitSPI - This function initializes the SPI to a known state
////
//void InitSpi_A(void)
//{
//    // GPIO配置
//    //InitSpiaGpio();
//
//    // 上电复位后，SPI工作在标准模式下，禁止SPI FIFO功能
//    // Initialize SPI FIFO registers
//    //SpiaRegs.SPIFFTX.all=0xE040;
//    //SpiaRegs.SPIFFRX.all=0x204f;
//    //SpiaRegs.SPIFFTX.all=0xe021;      // Enable FIFO's, set TX FIFO level to 1
//    //SpiaRegs.SPIFFRX.all=0x0;//0x0021;      // Set RX FIFO level to 8
//    //SpiaRegs.SPIFFCT.all=0x0;
//
//    // SPI 寄存器配置
//
//    //SpiaRegs.SPICCR.all =0x000F;    // Reset on, output at rising edge, 16-bit char bits
//    // Set reset low before configuration changes
//    // Clock polarity (0 == rising, 1 == falling)
//    // 16-bit character
//    // Disnable loop-back
//    SpiaRegs.SPICCR.bit.SPISWRESET = 0;
//    SpiaRegs.SPICCR.bit.CLKPOLARITY = 0;
//    SpiaRegs.SPICCR.bit.SPICHAR = (16-1);
//    SpiaRegs.SPICCR.bit.SPILBK = 0;
//
//    //SpiaRegs.SPICTL.all =0x0006;    // CLOCK PHASE=0, Master Mode, enable talk, and SPI int disabled.
//    // Enable master (0 == slave, 1 == master)
//    // Enable transmission (Talk)
//    // Clock phase (0 == normal, 1 == delayed)
//    // SPI interrupts are disabled
//    SpiaRegs.SPICTL.bit.MASTER_SLAVE = 1;
//    SpiaRegs.SPICTL.bit.TALK = 1;
//    SpiaRegs.SPICTL.bit.CLK_PHASE = 0;
//    SpiaRegs.SPICTL.bit.SPIINTENA = 0;
//
//    // Set the baud rate
//    SpiaRegs.SPIBRR.bit.SPI_BIT_RATE = SPI_BRR;
//    //SpiaRegs.SPIBRR = 0x1;           // SPI Baud Rate = LSPCLK/(SPIBRR+1), 根据书上公式，LSPCLK=37.5MHz so=37.5/4=9.375MHz
//
//    SpiaRegs.SPICCR.all = 0x008F;    // 在改变设置前将RESET清零，并在设置结束后将其置位
//
//    // Set FREE bit
//    // Halting on a breakpoint will not halt the SPI
//    SpiaRegs.SPIPRI.bit.FREE = 1;   // breakpoints don't disturb xmission
//
//
//    // 唤醒MAX5307
//    GpioDataRegs.GPBSET.bit.GPIO57 = 1;             //cs=1
//    NOP;
//    NOP;
//    GpioDataRegs.GPBCLEAR.bit.GPIO57 = 1;           //cs=0
//
//    SpiaRegs.SPITXBUF=0xfffc;                       //MAX5307唤醒字符
//    while(SpiaRegs.SPISTS.bit.INT_FLAG!=1){NOP;}    // 数据传完后INT_FLAG会置位
//
//    GpioDataRegs.GPBSET.bit.GPIO57 = 1;             //cs=1为下一次做准备
//
//    SpiaRegs.SPICCR.bit.SPISWRESET=0;               //通过reset 清楚SPI中断标志INT_FLAG
//    NOP;
//    NOP;
//    // Release the SPI from reset
//    SpiaRegs.SPICCR.bit.SPISWRESET=1;               // Relinquish SPI from Reset
//
//}

//
// End of file
//
