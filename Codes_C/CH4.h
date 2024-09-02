
/**
 * @file CH4.h
 * @ingroup Application
 * This is the handler for the CH4 option
 * of the REC gas engine control system .
 *
 * @remarks
 * @ void CH4_control() is called from 10Hz control Task
 * @author mvo
 * @date 03-jun-2008
 * 
 * changes:51
 * 1332 22.04.2010 GFH  deactivate CH4 value regulation if natural gas operation
 */



#ifndef CH4_H_
#define CH4_H_


#include "deif_types.h"
#include "appl_types.h"
#include "ELM.h"				//definition of  t_protection_state

extern void CH4_init(void);
extern void CH4_control_100ms(void);

// internal values of each protection
struct CH4_protection
{
	t_protection_state State;
    t_protection_state LastState;
    DU32               StateTimer;
    DS16               Limit;
    DBOOL              Exceeded;
};

// global Variables of CH4
typedef struct CH4struct
{
   // input
   DBOOL     DI_CalibrateCH4;    // TRUE means calibration takes place -> freeze CH4 value
   DS16      AI_I_CH4Value;      // raw value 5000=4mA, 25000 = 20mA
      
   // internal
   DBOOL     Option;
   DBOOL     OptionAndActive;

   DBOOL     WireBreakAI;

   DS16      CH4Value;           // in 0,1% CH4 (0...1000 = 0,0...100,0%)
   DS32      MaxPower_CH4;       // max. power due to CH4 value
   struct CH4_protection  Calibrating;
   struct CH4_protection  CH4ValueLow;
   struct CH4_protection  CH4ValueTooLow;
   DS16      MixerOffset;        // offset to mixer position in 0,1% of total
   
   DBOOL CH4ValueAvailable;
   
   //output 
   
} t_CH4;

extern t_CH4 CH4;

// constants

// raw value for 0% CH4
#define      CH4_VALUE_ZERO                5000L

// timeout for calibrating (in msec)
#define      CH4_CALIBRATING_TIMEOUT    1800000L
#define      CH4_CALIBRATING_REC_DELAY      100L
     
// time delays for CH4 value too low supervision, given in msec
#define      CH4_TOO_LOW_DELAY             1000L
#define      CH4_TOO_LOW_REC_DELAY       300000L

// hysteresis for CH4 value low supervision, in 0.1%
#define      CH4_TOO_LOW_HYST                20
 
// time delays for CH4 value low supervision, given in msec
#define      CH4_LOW_DELAY                 1000L
#define      CH4_LOW_REC_DELAY             2000L

#endif /*CH4_H_*/
