/*
 * @file
 * Module name  : debug.h
 * 
 * changes:
 * 1241	12.11.2008 RMI	CPORT_DISPLAY for modem-access ==> DEBUG not defined (rmiSMS)
 * 1309 25.02.2009 RMI  don't include modbusappl.h to avoid compiling the whole system on each change, rmiMBA
 */
#ifndef DEBUG_APPL_H_
#define DEBUG_APPL_H_

#include "stdio.h"
//rmiMBA #include "modbusappl.h"

//rmiSMS,  12.11.08 #define DEBUG 

#ifdef DEBUG

//   #define DEBUG_USB

//   #define DEBUG_IOHANDLER

//   #define HMI_KEY_DEBUG

//   #define TEST_APPL_IO

//   #define DEBUG_HMI

//   #define DEBUG_STOP

// if this Symbol is defined, HMI Key commands and Macro Requests will be echoed
// #define DEBUG_HMI_PAGES

// if this Symbol is defined, the Display port will protocoll all task-checks with number of runs
// #define DEBUG_APPL_WATCHDOG   

// if this Symbol is defined, the Display port will protocoll all IO card init results
// #define DEBUG_APPL_IO_INIT

// if this Symbol is defined, the Display port will protocoll all IO card buffer failure results
//#define DEBUG_APPL_IO  

// if this Symbol is defined, the Display port will protocoll all BingBang requests
// #define DEBUG_BING_BANG  

// if this symbol is defined, a line will be protocolled for each state in MAIN.State
//#define DEBUG_MAIN

// if this symbol is defined, a line will be protocolled for each state cange in SUP.State
// #define DEBUG_SUP
//#define DEBUG_SUP_STATE

// if this symbol is defined, a line will be protocolled for each state cange in SAF.State
//#define DEBUG_SAF
//#define DEBUG_SAF_STATE

// if this symbol is defined, a line will be protocolled for each state cange in WAT.State
//#define DEBUG_WAT
//#define DEBUG_WAT_STATE

// if this symbol is defined, a line will be protocolled for each state cange in HVS.State
//#define DEBUG_HVS
//#define DEBUG_HVS_STATE

// if this symbol is defined, a line will be protocolled for each state cange in TUR.State
//#define DEBUG_TUR
//#define DEBUG_TUR_STATE
//#define DEBUG_TUR_POSITION
// if this symbol is defined, a line will be protocolled for each state cange in GEN.State
//#define DEBUG_GEN
//#define DEBUG_GEN_STATE 

// if this symbol is defined, a line will be protocolled for each state cange in GEN.State
//#define DEBUG_RTU
//#define DEBUG_RTU_STATE 


#define PRINT(a)          {printf(a);}
#define PRINT1(a)          {printf(a);}
#define PRINT2(a, b)       {printf(a, b);}
#define PRINT3(a, b, c)    {printf(a, b, c);}
#define PRINT4(a, b, c, e) {printf(a, b, c, e);}
#define DEBUG_REC printf("\n DEBUG RECORD: %s,%d",__FILE__, __LINE__)

#else // no DEBUG

#define PRINT4(a, b, c, e) {}
#define PRINT3(a, b, c)    {}
#define PRINT2(a, b)       {}
#define PRINT1(a)          {}
#define PRINT(a)           {}
#define DEBUG_REC          {}

#endif //DEBUG



#endif /*DEBUG_APPL_H_*/
