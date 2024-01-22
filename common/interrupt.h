//###########################################################################
//
// FILE:   interrupt.h
//
// TITLE:  Stellaris style wrapper driver for C28x PIE Interrupt Controller.
//
//###########################################################################
// $TI Release: F2837xD Support Library v130 $
// $Release Date: Mon Oct 20 10:15:06 CDT 2014 $
//###########################################################################

#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************
    extern bool IntMasterEnable(void);
    extern bool IntMasterDisable(void);
    extern void IntRegister(uint32_t ui32Interrupt, void (*pfnHandler)(void));
    extern void IntUnregister(uint32_t ui32Interrupt);
    extern void IntEnable(uint32_t ui32Interrupt);
    extern void IntDisable(uint32_t ui32Interrupt);
    extern void IntIFRClear(uint16_t ui16Interrupts);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif // __INTERRUPT_H__


