//###########################################################################
//
// FILE:   debug.h
//
// TITLE:  Stellaris style debug header. Included for compatability.
//
//###########################################################################
// $TI Release: F2837xD Support Library v130 $
// $Release Date: Mon Oct 20 10:15:06 CDT 2014 $
//###########################################################################

#ifndef __DEBUG_H__
#define __DEBUG_H__

//*****************************************************************************
//
// Prototype for the function that is called when an invalid argument is passed
// to an API.  This is only used when doing a DEBUG build.
//
//*****************************************************************************
extern void __error__(char *pcFilename, unsigned long ulLine);

//*****************************************************************************
//
// The ASSERT macro, which does the actual assertion checking.  Typically, this
// will be for procedure arguments.
//
//*****************************************************************************


#ifdef DEBUG
#define ASSERT(expr) {                                      \
                         if(!(expr))                        \
                         {                                  \
                             __error__(__FILE__, __LINE__); \
                         }                                  \
                     }
#else
#define ASSERT(expr)
#endif

#endif // __DEBUG_H__


