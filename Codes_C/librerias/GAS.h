
/**
 * @file GAS.h
 * @ingroup Application
 * This is the handler for the gas line
 * of the REC gas engine control system .
 *
 * @remarks
 * @ void GAS_control() is called from 10Hz control Task
 * @author gfh
 * @date 01-jul-2008
 *  
 * changes:
 * 1309 13.02.2009 GFH  add new internal variable "GasPressure"
 * 1309 24.02.2009 GFH  move "GasPressure" and "GasTankLevel" to GPC struct
 * 1310 03.04.2009 GFH  support of "2 gas line operation" and debugging "standby operation"
 * 1410 22.03.2012 GFH  gas warning without stop
 */


#ifndef GAS_H_
#define GAS_H_


#include "deif_types.h"
#include "appl_types.h"

typedef struct GasCounter
{ DU32 CubicMeters;
  DU32 Milliliters;

  DU32 CubicMetersDay;
  DU32 CubicMetersWeek;
  DU32 CubicMetersMonth;
  DU32 CubicMetersYear;

  DTIMESTAMP TimestampOfReset;

} t_GasCounter;

typedef struct GasAlarm
{
	DBOOL StateDI;
	DBOOL SC1;
	DBOOL SC2;

} t_GasAlarm;

extern void GAS_counter_set( t_GasCounter* GasCounter, DTIMESTAMP timestamp, DU32 adjustment_cubic_meters);

extern void GAS_init(void);
extern void GAS_control_100ms(void);
extern void GAS_control_1000ms(void);

struct GAS_protection_variables
{
	t_protection_state State;
    t_protection_state LastState;
    DU32               StateTimer;
    DBOOL              Exceeded;
};

// control Modes for GAS
enum t_GAS_mode
{
   GAS_MODE_BLOCK,              // GAS blocked
   GAS_MODE_OFF,                // GAS off
   GAS_MODE_ON,                 // GAS on
   GAS_TEST_DEMANDED            // test mode demanded
};

// states of GAS
enum t_GAS_state
{
  GAS_BOOT,                    // during bootup
  GAS_VALVES_BLOCKED,
  GAS_VALVES_CLOSED,
  GAS_VALVES_OPEN_NOT_RUNNING,  // gas valves are open, but engine is not running
  GAS_VALVES_OPEN_RUNNING,      // gas valves are open and engine is running
  GAS_VALVES_SWITCHOVER_RUNNING,// both valves are open during switchover of gas type
  GAS_VALVES_UNDER_TEST         // Gas valves are in test mode
};


// global Variables of GAS
typedef struct GASstruct
{
   // input
   DS16  AI_I_GasFlow;			// rmi110411
   DS16  AI_I_GasFlow_B;

   DBOOL DI_GasAlarmWarning;
   DBOOL DI_GasAlarmWarningNoStop;
   DBOOL DI_GasAlarmFault;
   DBOOL DI_FaultGasAlarmDevice;
   DBOOL DI_GasPressureLimiterA;
   DBOOL DI_GasTemperatureLimiterA;
   DBOOL DI_GasPressureLimiterB;
   DBOOL DI_GasTemperatureLimiterB;
   DBOOL DI_DemandGasB;

   // internal
   t_GasCounter GasCounterA;
   t_GasCounter GasCounterB;

   enum t_GAS_state  state;
   enum t_GAS_mode   mode;
   
   DS16  GasFlow;				// gas flow, rmi110411 [0.01m³/h]
   DS16  GasFlow_B;				// [0.01m³/h], calculated from AI_I_GasFlow_B
   DBOOL GasAlarmWarning; 		// derived from DI_GasAlarmWarning
   DBOOL GasAlarmWarningNoStop;
   DBOOL WaitingForBackSynchronization; // marker for delayed selection of gas type
   
   DBOOL ValveA1TestDemand;
   DBOOL ValveA2TestDemand;

   DBOOL ValveB1TestDemand;
   DBOOL ValveB2TestDemand;
   
   DBOOL GasTypeBSelected;
   DBOOL GasTypeBActive;
   DBOOL Switchover;			// marker for switching over with engine running
   
   DS32  MaxPowerAuxA;           // max allowed electrical power when running with gas type A
   DS32  MaxPowerAuxB;           // max allowed electrical power when running with gas type A

   DBOOL BlockGasTypeA;
   DBOOL BlockGasTypeB;
   
   struct GAS_protection_variables     WarningNoStop;

   // Configurable gas alarm
   t_GasAlarm Alarm[3];
   
   //output 

   DBOOL DO_ValveA1;
   DBOOL DO_ValveA2;

   DBOOL DO_ValveB1;
   DBOOL DO_ValveB2;   
} t_GAS;


/* declaration of last Reset inside NOVRAM */
typedef struct
{
	t_GasCounter GAS_Counter;         // 32 bytes
	DS16 GAS_month_last_reset;     // + 2 Bytes
	DS16 GAS_year_last_reset;      // + 2 Bytes
}t_nov_gas;
extern t_nov_gas nov_gas_A;
extern t_nov_gas nov_gas_B;

extern t_GAS GAS;

extern void ResetGasAParameter(void);
extern void ResetGasBParameter(void);

// constants
// maximum time with open gas valves without engine running, in msec
#define GAS_MAX_TIME_GAS_VALVES_OPEN                      15000L

// delay time of gas pressure supervision
#define GAS_DELAY_TIMER_GAS_PRESSURE_SUPERVISION          500L

// gas warning (no stop) delay and recover delay [ms]
#define GAS_WARNING_NO_STOP_DELAY		1000L
#define GAS_WARNING_NO_STOP_REC_DELAY	10000L

// overlap time with gas valves A and B open, for gas switchover during engine running [ms]
#define GAS_VALVES_OVERLAP_TIME         1500L

#endif /*GAS_H_*/
