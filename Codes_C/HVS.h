/**
 * @file HVS.h
 * @ingroup Application
 * This is the handler for 22kV High Voltage switchboard cabinet
 * of Bergsfjord Hydro Power Plant
 *
 * @remarks
 * @ void HVS_control() is called from 10Hz control Task
 * @author aes
 * @date 05-sep-2006
 * 
 * changes:
 * 1306	18.12.2008 RMI  mains breaker support, rmiMB
 * 1310	27.03.2009 GFH  new internal variable HVS.T1EIsOffTimer and HVS.L1EIsOffTimer
 * 1313 08.05.2009 RMI	variables needed for "breaker doesnt close supervision", rmiIET
 * 1314 29.05.2009 GFH  now breaker operation counter come from PARA-struct (also NOVRAM)
 * 1326 09.12.2009 RMI  breaker disturbed only if not tripped by SCM, rmiBRTR
 * 1328 05.02.2010 GFH  operating counters at fixed NOVRAM area
 * 1328 09.02.2010 GFH  #define TRIP_BIT_MASK
 * 1333 01.07.2010 GFH	loadsharing - power management system
 * 1334 01.07.2010 RMI  ELM.h included, because of compiler errors when including HVS.h (t_protection_state)
 * 1334 09.07.2010 GFH	loadsharing - mains delomatic
 */
#ifndef HVS_H_
#define HVS_H_

#include "deif_types.h"
#include "appl_types.h"
#include "ELM.h"			// rmi: because of typedef enum{COLD, HOT, TRIP, RECOVER} t_protection_state; 

typedef struct
{
	DU32 GCB_switch_over_counter;
	DU32 MCB_switch_over_counter;
}t_nov_hvs;

extern t_nov_hvs hvs_nov;

extern void HVS_init(void);
// rmiIET xtern void HVS_control_100ms(void);
extern void HVS_control_20ms(void);
extern void HVS_control_1000ms(void);

extern DBOOL HVS_StateT1EIsOn(void);
extern DBOOL HVS_StateT1EOffAndReady(void);
extern DBOOL HVS_StateL1EIsOn(void);
extern DBOOL HVS_StateL1EOffAndReady(void);
extern DBOOL HVS_MCBPosDefaultOn(void);
extern DBOOL HVS_MCBPosDefaultOff(void);

extern void ResetHvsParameter(void);

// control Modes for HVS switch L1E (mains breaker)
typedef enum
{
   HVS_L1E_OFF,
   HVS_L1E_KEEP,
   HVS_L1E_ON
} t_HVS_L1E_mode;

// control Modes for HVS switch T1E (generator breaker)
typedef enum
{
   HVS_T1E_OFF,
   HVS_T1E_ON
} t_HVS_T1E_mode;


// states of HVS L1E (mains breaker)
typedef enum
{
	HVS_L1E_DETECT_STATE,
	HVS_L1E_OFF_AND_READY,
	HVS_L1E_OFF_BEFORE_ON,					// rmiMB
	HVS_L1E_WAIT_TO_CONNECT,				// rmiMB
	HVS_L1E_CONNECT,
	HVS_L1E_IS_ON,  
	HVS_L1E_DISCONNECT,
	HVS_L1E_WAIT_AFTER_DISCONNECT			// rmiMB
} t_HVS_L1E_state;

// states of HVS T1E (generator breaker)
typedef enum
{
	HVS_T1E_DETECT_STATE,
	HVS_T1E_OFF_AND_READY,
	HVS_T1E_OFF_BEFORE_ON,
	HVS_T1E_WAIT_TO_CONNECT,
	HVS_T1E_CONNECT,
	HVS_T1E_IS_ON,
	HVS_T1E_DISCONNECT,
	HVS_T1E_WAIT_AFTER_DISCONNECT
} t_HVS_T1E_state;

// breaker operations counter
/* not used anymore
struct operations_counter
{
	DU32            Value;
	DTIMESTAMP      TimestampOfReset;
};
*/

struct supervision_CloseBreaker
{
	t_protection_state State;
    t_protection_state LastState;
    DU32               StateTimer;
    DU8                Limit;
    DU8                Counter;
};

struct supervision_BreakerDisturbed
{
	t_protection_state State;
    t_protection_state LastState;
    DU32               StateTimer;
    DBOOL              Exceeded;
};

// global Variables of HVS
typedef struct HVSstruct
{
	// input signals
   //DBOOL           DI_GeneratorBreakerClosed;
   DBOOL           DI_GeneratorBreakerOpen;
   DBOOL           DI_GeneratorBreakerFault;
   
   DBOOL           DI_MainsBreakerOpen;
   DBOOL           DI_MainsBreakerFault;
   
   DBOOL           DI_ReleaseCloseBreaker;
   
   // internal values
   DBOOL           GeneratorBreakerTrippedBySCM;	        // rmiBRTR
   DBOOL           GeneratorBreakerClosed;
   DBOOL           GeneratorBreakerOpen;
   DBOOL           MainsBreakerTrippedBySCM;	            // rmiBRTR
   DBOOL           MainsBreakerClosed;
   DBOOL           MainsBreakerOpen;
   
   DBOOL           ReleaseCloseBreaker;
   
   DU32            T1EIsOffTimer;
   DU32            L1EIsOffTimer;
   DU32            T1E_tripped_30611_Timer;
   DU32            L1E_tripped_50621_Timer;
   
   DU8             T1EDoesntCloseCounter;					// rmiIET
   DU8             L1EDoesntCloseCounter;					// rmiIET
   
   //struct operations_counter     BreakerOperationsCounter;  
   //struct operations_counter     MainsBreakerOperationsCounter;  			// rmiMB
   
   DU32            BreakerOperationsCounter;
   DU32            MainsBreakerOperationsCounter;
     
   
   // output signals
   DBOOL           DO_CloseGeneratorBreaker;  //(ON closes)
   DBOOL           DO_TripGeneratorBreaker;   //(OFF trips)
   DBOOL           DO_OpenGeneratorBreaker;   //(ON opens)
   DBOOL           DO_BackupProtection;       //(ON opens backup breaker)

   DBOOL           DO_CloseMainsBreaker;  	//(ON closes)
   DBOOL           DO_TripMainsBreaker;   	//(ON trips)
   DBOOL           DO_OpenMainsBreaker;   	//(ON opens)
   
   DBOOL           DO_WishToCloseBreaker;
   
   DBOOL           DO_ReleaseBreaker;

   // rest of former program	
/* rmiMB	
   t_Digital_Input L1EEnabled;
   t_Digital_Input L1ENoFault;
   t_Digital_Input L1EOff;
   t_Digital_Input L1EOn;
   t_Digital_Input L1EGasAlarm;
*/
//   t_Digital_Input T1EEnabled;
//   t_Digital_Input T1ENoFault;
//   t_Digital_Input T1EOff;
//   t_Digital_Input T1EOn;
/* rmiMB
   t_Digital_Input T1EGasAlarm;
   t_Digital_Input St1ENotReady;
   t_Digital_Input TVLBNotReady;
   t_Digital_Input T1EEnergyPulses;	 
   
   t_Digital_Output ConnectL1E;
   t_Digital_Output DisconnectL1E;
*/
   //t_Digital_Output ConnectT1E;
   //t_Digital_Output DisconnectT1E;
/* rmiMB
   t_Logic_Signal TurnOffL1E;
   t_Logic_Signal TurnOnL1E;
*/
   
   t_HVS_L1E_state  stateL1E;
   t_HVS_T1E_state  stateT1E;
   t_HVS_L1E_mode   modeL1E;       
   t_HVS_T1E_mode   modeT1E; 
   t_HVS_L1E_mode   TestModeL1E;       
   t_HVS_T1E_mode   TestModeT1E;
   
   
   // stuctures needed for Breaker doesn't close supervision, rmiIET
   struct supervision_BreakerDisturbed  T1ETrippedBySCM;				// rmiBRTR
   struct supervision_BreakerDisturbed  T1EBreakerDisturbed;			// rmiBRTR
   struct supervision_CloseBreaker     	T1EDoesNotCloseCounter;
   struct supervision_CloseBreaker     	T1EDoesNotCloseSupervision; 
   
   struct supervision_BreakerDisturbed 	L1ETrippedBySCM;	 			// rmiBRTR
   struct supervision_BreakerDisturbed 	L1EBreakerDisturbed;			// rmiBRTR
   struct supervision_CloseBreaker     	L1EDoesNotCloseCounter;
   struct supervision_CloseBreaker     	L1EDoesNotCloseSupervision;

   DBOOL NovUpdateRequired;

} t_HVS;

extern t_HVS HVS;

#define HVS_SYNC_DISABLED		( PARA[ParRefInd[HVS_EXT_SYNC__PARREFIND]].Value )
#define HVS_SYNC_OFF			( HVS_SYNC_DISABLED OR ( PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value == NO_SCM ) )

// Timeouts

#define HVS_L1E_START_DELAY         15000L  //  before 22L1E is ready for on or off

#define HVS_L1E_DISCONNECT_MAX_TIME	 3000L // same as for Generator Breaker 1000L	//  timeout for disconnection of 22L1E
#define HVS_L1E_CONNECT_MAX_TIME	 1000L  //  timeout for connection of 22L1E
  // delay timer for off signal to compact mains breaker after off feedback, rmiMB
#define HVS_L1E_COMPACT_BREAKER_OFF_DELAY      3000L
  // stabilize time before closing mains breaker, rmiMB
#define HVS_L1E_WAIT_BEFORE_CLOSING_TIME        200L
  // keep off command high after opening open mains breaker (not option compact breaker), rmiMB 
#define HVS_L1E_WAIT_AFTER_OPENING_TIME         200L

// number of L1E-connects (limit), rmiIET
#define HVS_L1E_DOESNT_CLOSE_LIMIT 				3
#define HVS_L1E_DOESNT_CLOSE_DELAY_TIME			5000L
#define HVS_L1E_DOESNT_CLOSE_RECOVER_TIME		500L
#define HVS_T1E_DOESNT_CLOSE_LIMIT 				PARA[ParRefInd[GB_BREAKER_DOESNT_CLOSE_LIMIT__PARREFIND]].Value
#define HVS_T1E_DOESNT_CLOSE_DELAY_TIME			5000L
#define HVS_T1E_DOESNT_CLOSE_RECOVER_TIME		500L

// timeouts for breaker disturbed: 200ms, rmiBRTR
#define HVS_L1E_BREAKER_DISTURBED_DELAY_TIME	200L
#define HVS_T1E_BREAKER_DISTURBED_DELAY_TIME	200L

  // timeout for disconnection of generator breaker:
  // 03.10.2008 changed to 3s because of slow ABB Tmax 
#define HVS_T1E_DISCONNECT_MAX_TIME	           3000L
  // timeout for connection of generator breaker:	
#define HVS_T1E_CONNECT_MAX_TIME	            500L  
  // timeout for position failure ( mainly wire break): 1s
#define HVS_POSITION_FAILURE_DELAY             1000L
  // option compact breaker (= always off before on. Off time continues after opening for delay timer.)
#define HVS_T1E_COMPACT_BREAKER                    1
  // delay timer for off signal to compact breaker after off feedback:
#define HVS_T1E_COMPACT_BREAKER_OFF_DELAY      3000L
  // stabilize time before closing in static synchronization
#define HVS_T1E_WAIT_BEFORE_CLOSING_TIME        200L
  // keep off command high after opening open circuit breaker (not option compact breaker) 
#define HVS_T1E_WAIT_AFTER_OPENING_TIME         200L

// timeout generator breaker trip -> start back up protection  
#define HVS_GLS_TRIP_TIMEOUT					5000L

// bit mask for  masking out BIT_0 and BIT_1 for tripping generator breaker and mains breaker
#define TRIP_BIT_MASK                           0xFFFC

// rmiOPT extern DU8* HVS_L1E_state_text(t_HVS_L1E_state state,DU8 language);
// rmiOPT extern DU8* HVS_T1E_state_text(t_HVS_T1E_state state,DU8 language);
// rmiOPT extern DU8* HVS_L1E_mode_text(t_HVS_L1E_mode mode,DU8 language);
// rmiOPT extern DU8* HVS_T1E_mode_text(t_HVS_T1E_mode mode,DU8 language);

// MCB

#define HVS_MCB_POS_OFF         0L
#define HVS_MCB_POS_ON          1L
#define HVS_MCB_POS_DEFAULT     PARA[ParRefInd[HVS_MCB_DEFAULT_POSITION__PARREFIND]].Value

#endif /*HVS_H_*/
