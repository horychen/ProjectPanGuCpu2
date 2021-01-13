#include "ACMExpr.h"

//
// Globals
//
struct IPC_MEMORY_WRITE Write;
struct IPC_MEMORY_READ Read;
#pragma DATA_SECTION( Read, "SHARERAMGS1");
#pragma DATA_SECTION(Write, "SHARERAMGS0"); // GS0 is write


void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr);
void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }
    return;
}
#define STOP_LED1  GpioDataRegs.GPDCLEAR.bit.GPIO124=1;
#define STOP_LED2  GpioDataRegs.GPBCLEAR.bit.GPIO33=1;
#define START_LED1 GpioDataRegs.GPDSET.bit.GPIO124=1;
#define START_LED2 GpioDataRegs.GPBSET.bit.GPIO33=1;


//
// Main
//
void main(void)
{


    //
    // Wait until Shared RAM is available.
    //
    while(!( MemCfgRegs.GSxMSEL.bit.MSEL_GS0))
    {
    }
    START_LED1

    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    //
    InitSysCtrl();


    //
    // Step 2. Initialize GPIO:
    // This example function is found in the F2837xD_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example

    // 初始化SPI，用于与DAC芯片MAX5307通讯。
    //GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 0; // Configure GPIO57 as C\S\ signal for MAX5307
    InitSpi();

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    //
    InitPieCtrl();


//    #ifdef _FLASH
//        // Call Flash Initialization to setup flash waitstates
//        // This function must reside in RAM
//        InitFlash();
//        // Gain pump semaphore
//        //SeizeFlashPump();
//    #endif

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    //
    InitPieVectTable();


    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;  // This is needed to write to EALLOW protected registers
    //PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize the Device Peripheral. This function can be
    //         found in F2837xD_CpuTimers.c
    //
    //InitCpuTimers();   // For this example, only initialize the Cpu Timers

    //
    // Configure CPU-Timer0 to interrupt every second:
    // c2_FREQ in MHz, 1 second Period (in uSeconds)
    //
    //ConfigCpuTimer(&CpuTimer0, 200, 1000000);

    //
    // To ensure precise timing, use write-only instructions to write to the
    // entire register.
    //
    //CpuTimer0Regs.TCR.all = 0x4000;

    //
    // Step 5. User specific code, enable interrupts:
    //

    //
    // Enable CPU int1 which is connected to CPU-Timer 0
    //
    //IER |= M_INT1;

    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    Write.position_cmd_elec = 100;
    Write.speed_cmd_elec = 600;
    IPCLtoRFlagSet(IPC_FLAG10);

    START_LED2

    while(1)
    {
        if(IPCRtoLFlagBusy(IPC_FLAG7) == 1){
            DAC_MAX5307(1, Read.dac_buffer[0] ); //71us 10khz
            DAC_MAX5307(2, Read.dac_buffer[1] ); //71us 10khz
            DAC_MAX5307(3, Read.dac_buffer[2] ); //71us 10khz
            DAC_MAX5307(4, Read.dac_buffer[3] ); //71us 10khz
            DAC_MAX5307(5, Read.dac_buffer[4] ); //71us 10khz
            DAC_MAX5307(6, Read.dac_buffer[5] ); //71us 10khz
            DAC_MAX5307(7, Read.dac_buffer[6] ); //71us 10khz
            DAC_MAX5307(8, Read.dac_buffer[7] ); //71us 10khz
            IPCRtoLFlagAcknowledge (IPC_FLAG7);
        }

        if(IPCLtoRFlagBusy(IPC_FLAG10) == 0) // if not busy
        {
            Write.position_cmd_elec += 0.01;
            Write.speed_cmd_elec -= 0.01;

            // Set a flag to notify CPU02 that data is available
            IPCLtoRFlagSet(IPC_FLAG10);
        }
    }
}

//
// cpu_timer0_isr - CPU Timer0 ISR
//
//__interrupt void cpu_timer0_isr(void)
//{
//   EALLOW;
//   CpuTimer0.InterruptCount++;
//   GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;
//   EDIS;
//
//   PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
//}

//
// End of file
//
