#include "ACMExpr.h"
#include "F2837xD_Cla_typedefs.h"  // F2837xD CLA Type definitions
#include "F2837xD_device.h"        // F2837xD Headerfile Include File
#include "F2837xD_Examples.h"      // F2837xD Examples Include File
#include "hw_can.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_types.h"
#include "can.h"


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



//*****************************************************************************
// A counter that keeps track of the number of times the transmit was
// successful.
//*****************************************************************************
volatile unsigned long g_ulMsgCount = 0;

//*****************************************************************************
// A flag to indicate that some transmission error occurred.
//*****************************************************************************
volatile unsigned long g_bErrFlag = 0;

tCANMsgObject sTXCANMessage_ID0x01;
tCANMsgObject sRXCANMessage_ID0x01;
unsigned char ucTXMsgData_ID0x01[8], ucRXMsgData_ID0x01[8];
tCANMsgObject sTXCANMessage_ID0x03;
tCANMsgObject sRXCANMessage_ID0x03;
unsigned char ucTXMsgData_ID0x03[8], ucRXMsgData_ID0x03[8];

#define TX_ID0x01_OBJID 1
#define RX_ID0x01_OBJID 2
#define TX_ID0x03_OBJID 3
#define RX_ID0x03_OBJID 4

// Prototype statements for functions found within this file.
interrupt void sciaRxFifoIsr(void);
void scia_echoback_init(void);
void scia_fifo_init(void);
void scia_xmit(int a);
unsigned char SciReceivedChar[6];
Uint32 sci_pos;
Uint32 can_pos_ID0x01;
Uint32 can_pos_ID0x03; // knee joint encoder
Uint32 can_pos_prev;

REAL dataStoreCan[2000];
REAL dataStoreSci[2000];

int32 deltaPos;
int16 dataWidth = 10;
Uint16 startRecode = 0;
int16 dataIndex = 0;
Uint16 TestCount=0;

// 注意，Eureka扩展板和测试板使用的WE信号管脚不同
#define EUREKA_BOARD

#ifdef EUREKA_BOARD
#define ENCODER485_WRITE_ENABLE  GpioDataRegs.GPESET.bit.GPIO137 = 1;
#define ENCODER485_WRITE_DISABLE  GpioDataRegs.GPECLEAR.bit.GPIO137 = 1;
#else
#define ENCODER485_WRITE_ENABLE  GpioDataRegs.GPASET.bit.GPIO8 = 1;
#define ENCODER485_WRITE_DISABLE  GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;
#endif

void get_sci_angle(){

    SciaRegs.SCIFFRX.bit.RXFIFORESET = 0;
    DELAY_US(10);
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;

    // 半双工模式
    ENCODER485_WRITE_ENABLE
    scia_xmit(2);
    TestCount++;
    DELAY_US(5);
    ENCODER485_WRITE_DISABLE

//    int retryTime;
//    for(retryTime=0; retryTime<10;retryTime++)
//    {
//        DELAY_US(400);
//        if(SciaRegs.SCIFFRX.bit.RXFFST >= 6)
//        {
//            int i;
//            for(i=0;i<6;i++)
//            {
//                SciReceivedChar[i]=SciaRegs.SCIRXBUF.all & 0x00FF;  // Read data
//            }
//            sci_pos = (Uint32)(SciReceivedChar[4] *65536) + (Uint32)(SciReceivedChar[3] *256) + (Uint32)(SciReceivedChar[2]);
//            //angle_sci = pos * 360 *(double)7.6293945e-6 /64;
//            TestCount2++;
//            break;
//        }
//    }
}

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



    // Initialize the CAN controller
    CANInit(CANA_BASE);
    CANInit(CANB_BASE);

    // Setup CAN to be clocked
    CANClkSourceSelect(CANA_BASE, 0);
    CANClkSourceSelect(CANB_BASE, 0);

    // Set up the bit rate for the CAN bus.  This function sets up the CAN
    // bus timing for a nominal configuration.  You can achieve more control
    // over the CAN bus timing by using the function CANBitTimingSet() instead
    // of this one, if needed.
    // In this example, the CAN bus is set to 500 kHz.  In the function below,
    // the call to SysCtlClockGet() is used to determine the clock rate that
    // is used for clocking the CAN peripheral.  This can be replaced with a
    // fixed value if you know the value of the system clock, saving the extra
    // function call.  For some parts, the CAN peripheral is clocked by a fixed
    // 8 MHz regardless of the system clock in which case the call to
    // SysCtlClockGet() should be replaced with 8000000.  Consult the data
    // sheet for more information about CAN peripheral clocking.
    CANBitRateSet(CANA_BASE, 200000000, 500000);
    CANBitRateSet(CANB_BASE, 200000000, 500000);


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
    PieVectTable.SCIA_RX_INT = &sciaRxFifoIsr;
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

    PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
    PieCtrlRegs.PIEIER9.bit.INTx1 = 1;   // PIE Group 9, INT1 SCIA receive
    IER = 0x100;                         // Enable CPU INT
    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    Write.position_cmd_elec = 100;
    Write.speed_cmd_elec = 600;
    IPCLtoRFlagSet(IPC_FLAG10);

    START_LED2



    scia_fifo_init();      // Initialize the SCI FIFO
    scia_echoback_init();  // Initialize SCI for echoback
    // Enable test mode and select external loopback
//    HWREG(CANA_BASE + CAN_O_CTL) |= CAN_CTL_TEST;
//    HWREG(CANA_BASE + CAN_O_TEST) = CAN_TEST_EXL;

    // Enable the CAN for operation.
    CANEnable(CANA_BASE);
    CANEnable(CANB_BASE);

    // Initialize the message object that will be used for sending CAN
    // messages.
    sTXCANMessage_ID0x01.ui32MsgID = 1;                        // CAN message ID - use 1
    sTXCANMessage_ID0x01.ui32MsgIDMask = 0;                    // no mask needed for TX
    sTXCANMessage_ID0x01.ui32Flags = MSG_OBJ_TX_INT_ENABLE;    // enable interrupt on TX
    sTXCANMessage_ID0x01.ui32MsgLen = 3;     // size of message is
    ucTXMsgData_ID0x01[0] = sTXCANMessage_ID0x01.ui32MsgLen;
    ucTXMsgData_ID0x01[1] = sTXCANMessage_ID0x01.ui32MsgID;
    ucTXMsgData_ID0x01[2] = 1;
    sTXCANMessage_ID0x01.pucMsgData = ucTXMsgData_ID0x01;           // ptr to message content

    // Initialize the message object that will be used for recieving CAN
    // messages.
    *(unsigned long *)ucRXMsgData_ID0x01 = 0;
    sRXCANMessage_ID0x01.ui32MsgID = 1;                        // CAN message ID - use 1
    sRXCANMessage_ID0x01.ui32MsgIDMask = 0;                    // no mask needed for TX
    sRXCANMessage_ID0x01.ui32Flags = MSG_OBJ_NO_FLAGS;         //
    sRXCANMessage_ID0x01.ui32MsgLen = sizeof(ucRXMsgData_ID0x01);     // size of message is 4
    sRXCANMessage_ID0x01.pucMsgData = ucRXMsgData_ID0x01;           // ptr to message content

    // Initialize the message object that will be used for sending CAN
    // messages.
    sTXCANMessage_ID0x03.ui32MsgID = 3;                        // CAN message ID - use 3
    sTXCANMessage_ID0x03.ui32MsgIDMask = 0;                    // no mask needed for TX
    sTXCANMessage_ID0x03.ui32Flags = MSG_OBJ_TX_INT_ENABLE;    // enable interrupt on TX
    sTXCANMessage_ID0x03.ui32MsgLen = 0x03;     // size of message is
    ucTXMsgData_ID0x03[0] = sTXCANMessage_ID0x03.ui32MsgLen;
    ucTXMsgData_ID0x03[1] = sTXCANMessage_ID0x03.ui32MsgID;
    ucTXMsgData_ID0x03[2] = 0x01;
    sTXCANMessage_ID0x03.pucMsgData = ucTXMsgData_ID0x03;           // ptr to message content

    // Initialize the message object that will be used for recieving CAN
    // messages.
    *(unsigned long *)ucRXMsgData_ID0x03 = 0;
    sRXCANMessage_ID0x03.ui32MsgID = 3;                        // CAN message ID - use 3
    sRXCANMessage_ID0x03.ui32MsgIDMask = 0;                    // no mask needed for TX
    sRXCANMessage_ID0x03.ui32Flags = MSG_OBJ_NO_FLAGS;         //
    sRXCANMessage_ID0x03.ui32MsgLen = sizeof(ucRXMsgData_ID0x03);     // size of message is 4
    sRXCANMessage_ID0x03.pucMsgData = ucRXMsgData_ID0x03;           // ptr to message content

    // Enter loop to send messages.  A new message will be sent once per
    // second.  The 4 bytes of message content will be treated as an unsigned
    // long and incremented by one each time.

    // Setup the message object being used to receive messages
    CANMessageSet(CANA_BASE, RX_ID0x01_OBJID, &sRXCANMessage_ID0x01, MSG_OBJ_TYPE_RX);
    CANMessageSet(CANA_BASE, RX_ID0x03_OBJID, &sRXCANMessage_ID0x03, MSG_OBJ_TYPE_RX);

    int i;
    for(i=0;i<2000;i++)
    {
        dataStoreCan[i] = 0;
        dataStoreSci[i] = 0;
    }


    while(1)
    {
        if(IPCRtoLFlagBusy(IPC_FLAG7) == 1){

            DAC_MAX5307(1, Read.dac_buffer[0] ); //71us 10khz
            DAC_MAX5307(2, Read.dac_buffer[1] ); //71us 10khz
            DAC_MAX5307(3, Read.dac_buffer[2] ); //71us 10khz
            DAC_MAX5307(4, Read.dac_buffer[3] ); //71us 10khz
            //            DAC_MAX5307(5, Read.dac_buffer[4] ); //71us 10khz
            //            DAC_MAX5307(6, Read.dac_buffer[5] ); //71us 10khz
            //            DAC_MAX5307(7, Read.dac_buffer[6] ); //71us 10khz
            //            DAC_MAX5307(8, Read.dac_buffer[7] ); //71us 10khz

            IPCRtoLFlagAcknowledge (IPC_FLAG7);
        }

        if(IPCLtoRFlagBusy(IPC_FLAG10) == 0) // if not busy
        {
            Write.position_cmd_elec += 0.01;
            Write.speed_cmd_elec -= 0.01;

            Write.SCI_position_elec = sci_pos;
            Write.CAN_position_elec_ID0x01 = can_pos_ID0x01;
            Write.CAN_position_elec_ID0x03 = can_pos_ID0x03;
            // Set a flag to notify CPU02 that data is available
            IPCLtoRFlagSet(IPC_FLAG10);
        }

        get_sci_angle();
        CANMessageSet(CANA_BASE, TX_ID0x01_OBJID, &sTXCANMessage_ID0x01, MSG_OBJ_TYPE_TX);
        DELAY_US(5);
        CANMessageGet(CANA_BASE, RX_ID0x01_OBJID, &sRXCANMessage_ID0x01, true);
        can_pos_ID0x01 = (Uint32)(ucRXMsgData_ID0x01[5]*65536)+ (Uint32)(ucRXMsgData_ID0x01[4] * 256) + (Uint32)(ucRXMsgData_ID0x01[3]);

        CANMessageSet(CANA_BASE, TX_ID0x03_OBJID, &sTXCANMessage_ID0x03, MSG_OBJ_TYPE_TX);
        DELAY_US(5);
        CANMessageGet(CANA_BASE, RX_ID0x03_OBJID, &sRXCANMessage_ID0x03, true);
        can_pos_ID0x03 = (Uint32)(ucRXMsgData_ID0x03[5]*65536)+ (Uint32)(ucRXMsgData_ID0x03[4] * 256) + (Uint32)(ucRXMsgData_ID0x03[3]);

        DELAY_US(900);


        //        deltaPos = (int32)(can_pos_ID0x03 - can_pos_prev);
        //        if(deltaPos < -65536)
        //        {
        //            deltaPos += 131072;
        //        }
        //        if(deltaPos > 65536)
        //        {
        //            deltaPos -= 131072;
        //        }
        //        if( deltaPos < (-1)*dataWidth){
        //            dataIndex++;
        //            startRecode = 1;
        //        }else if( deltaPos > dataWidth){
        //            dataIndex--;
        //            startRecode = 1;
        //        }
        //
        //        if(dataIndex>=2000){
        //            dataIndex = 1999;
        //        }else if (dataIndex<0){
        //            dataIndex = 0;
        //        }
        //
        //        if(startRecode == 1)
        //        {
        //            can_pos_prev = can_pos_ID0x03;
        //            dataStoreCan[dataIndex] = (REAL)(can_pos_ID0x03/131072.0*360.0);
        //            dataStoreSci[dataIndex] = (REAL)(sci_pos/8388608.0*360.0);
        //            startRecode = 0;
        //        }

    }
}


void scia_echoback_init()
{
    // Note: Clocks were turned on to the SCIA peripheral
    // in the InitSysCtrl() function

    SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                   // No parity,8 char bits,
                                   // async mode, idle-line protocol

//    SciaRegs.SCICCR.bit.PARITYENA =1;
    SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                   // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all =0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA =1;
    SciaRegs.SCICTL2.bit.RXBKINTENA =1;

//    SciaRegs.SCIHBAUD.all    =0x0002;  // 9600 baud @LSPCLK = 50MHz (200 MHz SYSCLK).
//    SciaRegs.SCILBAUD.all    =0x008B;
//    SciaRegs.SCIHBAUD.all    =0x0001;  // 14400 baud @LSPCLK = 50MHz (200 MHz SYSCLK).
//    SciaRegs.SCILBAUD.all    =0x00B1;
//    SciaRegs.SCIHBAUD.all    =0x0003;  // 14400 baud @LSPCLK = 100MHz (200 MHz SYSCLK).
//    SciaRegs.SCILBAUD.all    =0x0063;
//    SciaRegs.SCIHBAUD.all =0x0000; //LSPCLK=100M
//    SciaRegs.SCILBAUD.all =0x0018; //CLK=0.5M

    SciaRegs.SCIHBAUD.all =0x0000; //LSPCLK=100M
    SciaRegs.SCILBAUD.all =0x0004; //CLK=2.5M

    SciaRegs.SCICTL1.all =0x0023;  // Relinquish SCI from Reset
}

// Transmit a character from the SCI
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF.all =a;

}

void scia_msg(char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

// Initialize the SCI FIFO
void scia_fifo_init()
{
    SciaRegs.SCIFFTX.all=0xC040;
    SciaRegs.SCIFFRX.all=0x2026;
    SciaRegs.SCIFFCT.all=0x0;

    SciaRegs.SCIFFTX.bit.TXFIFORESET = 1;
    SciaRegs.SCIFFRX.bit.RXFIFORESET = 1;
}

//
// sciaRxFifoIsr - SCIA Receive FIFO ISR
//
interrupt void sciaRxFifoIsr(void)
{
    Uint16 i;

    for(i=0;i<6;i++)
    {
       SciReceivedChar[i]=SciaRegs.SCIRXBUF.all & 0x00FF;  // Read data
    }
    sci_pos = (Uint32)(SciReceivedChar[4] *65536) + (Uint32)(SciReceivedChar[3] *256) + (Uint32)(SciReceivedChar[2]);


    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
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
