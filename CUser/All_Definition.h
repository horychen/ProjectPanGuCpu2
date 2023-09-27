#define REAL float

#include "F28x_Project.h"
#include "F2837xD_Ipc_drivers.h"
#include "DAC_MAX5307.h"




//
// Globals
//

struct IPC_MEMORY_READ{
    /* read only (RO) shared memory @ GS1 */
    REAL dac_buffer[8];
    REAL test;

    // 双向变量
};
struct IPC_MEMORY_WRITE{
    /* read/write (RW) shared memory @ GS0  owned by CPU2 */
    REAL position_cmd_elec;
    REAL speed_cmd_elec;

    // 双向变量
};

extern struct IPC_MEMORY_WRITE Write;
extern struct IPC_MEMORY_READ Read;

//extern uint16_t isrfuncLoadStart;
//extern uint16_t isrfuncLoadEnd;
//extern uint16_t isrfuncRunStart;
//extern uint16_t isrfuncLoadSize;


//
// Function Prototypes
//
//__interrupt void cpu_timer0_isr(void);
//#pragma CODE_SECTION(cpu_timer0_isr,"isrfunc")

//void Shared_Ram_dataWrite_c2(void);
