/**
 * @file MAIN_CONTROL.h
 * @ingroup Application
 * This is the main state handler
 * of Bergsfjord Hydro Power Plant
 *
 * @remarks
 * @author aes
 * @date 05-sep-2006
 *
 * changes:
 * 1305 12.12.2008 RMI	support second SCM-card, rmi2SCM
 * 1309 25.02.2009 RMI	appl_types.h necessary for t_Logic_Signal
 * 1310 05.03.2009 RMI	t_MAIN_Reduction StopEngine, rmiSTE
 * 1310 11.03.2009 GFH	new input variable "DI_AutomaticOperation"
 * 1310 26.03.2009 RMI  changed BitMask-handling (DebugBitMask2_10103), rmiTIC
 * 1310 03.04.2009 GFH  support of "2 gas line operation" and debugging "standby operation"
 * 1313 11.05.2009 RMI  version for IET: lambda control, synchronisation, ...
 * 1314 15.07.2009 RMI  new release
 * 1320 03.09.2009 RMI  protection functionality on SCM supported: 1320
 * 1321 15.10.2009 GFH  DO_Function for island operation
 * 1322 29.10.2009 RMI  IET-version
 * 1323 05.11.2009 GFH  new version
 * 1324 12.11.2009 RMI  PROJECT_NO (to identify customer versions)
 * 1326 21.12.2009 RMI  ignition diagnostic supervision, some DI's assignable
 * 1327 21.10.2010 RMI  TREND optimization (no switch/case-construct)
 * 1328 26.10.2010 GFH  new task MAIN_control_1000ms()
 * 1328 05.02.2010 GFH	MAINLOG at fixed NOVRAM area
 * 1330 23.02.2010 MVO  CLIENT_DEIF, CLIENT a.s.o removed
 * 1333 01.07.2010 GFH	loadsharing - power management system
 * 1334 09.07.2010 GFH	loadsharing - mains delomatic
 * 1335 27.07.2010 RMI  exhaust bypass option depending on heating flow and return
 * 1336 05.08.2010 RMI  mains recover time: max values for protection and power failure changed
 * 1341 21.09.2010 GFH  new software release 1.34.1
 * 1342 06.10.2010 RMI  release 1.34.2 (passwords not destroyed by login)
 * 1343 08.11.2010 GFH  option: cylinder temperature monitoring
 * 1343 08.11.2010 GFH  change #define to next software version 1.34.3
 * 1344 04.03.2011 RMI  parameter: stop if to many bypass operations
 * 1350 28.03.2011 RMI  option cylinder monitoring activated
 * 1360 14.04.2011 RMI  reactive power import / export: new parameter, DEIF=RhoenEnergie
 * 1361 04.05.2011 RMI  cooling pump (and preheater) off, when SC 30030 (G213: failure cooling water pump) caused by "fault cool.sys."
 * 1362 11.05.2011 RMI  gas flow unit changed m3/h -> Nm3/h
 * 1401 23.11.2011 MVO  engine cooling circuit pressure deassignable
 * 1402 28.11.2011 RMI  language swedish
 * 1404 23.12.2011 MVO  SC 50093, logic of gas level start/stop changed
 * 1405 04.01.2012 MVO  in PMS.c, PMS_SS_WaitForStart: start if ENG.running only in case of island operation.
 * 						in case of mains parallel operation we always wait for the timer (wish of IET)
 * 1405 04.01.2012 MVO  THROTTLE_POS_100_VOLTAGE__MIN_VALUE changed from 3.00V to 2.10V to support the new
 * 						cheap driver from Kleemann
 * 1410 22.03.2012 GFH  software-version 1.41.0
 * 1412 06.08.2012 DNI  software-version 1.41.2
 *  998 12.12.2012 GFH  preliminary version 998
 *  1420                 redundant breaker
 * 1421 22.05.2013 GFH  grid protection and control according to VDE AR-N 4105 - 2013
 * 1421 07.06.2013 MVO  version 1.42.1 generated
 * 1422 25.07.2013 GFH  ignition box HZM-Phlox
 * 1423 19.09.2013 MVO  phlox version generated after test with AKR 8 cylinders
 * 		05.12.2016 MVO  some unused constants removed
 *
 */


#ifndef MAIN_CONTROL_H_
#define MAIN_CONTROL_H_

//logica

#endif /*MAIN_CONTROL_H_*/

#include "deif_types.h"
#include "appl_types.h"
#include "applrev.h"

typedef enum
{
	MAIN_BOOT           = 0,
	MAIN_SYSTEM_START,
	MAIN_EMERGENCY_STOP,
	MAIN_UNDEFINED_BREAKER_POS,
	MAIN_BLACK_OPERATION,
	MAIN_MAINS_OPERATION,
	MAIN_EMERGENCY_BRAKING,
	MAIN_REARM_SAFETY_CHAIN,
	MAIN_SYSTEM_STOP,
	MAIN_SYSTEM_READY_FOR_START,
	MAIN_FAST_BRAKING,
	MAIN_STRT_PREPARE,
	MAIN_STARTING,
	MAIN_IGNITION,
	MAIN_OPEN_GAS_VALVES,
	MAIN_ACCELERATION,
	MAIN_LOW_IDLE_SPEED,
	MAIN_TRANSFORMER_DISCONNECTED,
	MAIN_WAIT_FOR_RELEASE_CLOSE_GCB,
	MAIN_CONNECT_T1E,					// rmiEPF
	MAIN_DISCONNECT_T1E_ISLAND,			// rmiEPF
	MAIN_SYNCHRON_CONNECT_T1E,
	MAIN_ISLAND_OPERATION,				// rmiEPF
	MAIN_LOADSHARING_RAMP_UP,
	MAIN_LOADSHARING,
	MAIN_LOADSHARING_RAMP_DOWN,
//	MAIN_CONNECT_L1E,					// rmiEPF
	MAIN_SYNCHRON_CONNECT_L1E,			// rmiEPF
//	MAIN_DISCONNECT_L1E,				// rmiEPF
	MAIN_DISCONNECT_L1E_TO_ISLAND,		// rmiEPF
	MAIN_GRID_PARALLEL_LIMITED_LOAD,
	MAIN_GRID_PARALLEL_FULL_LOAD,
	MAIN_DISCONNECT_T1E,
	MAIN_COOLDOWN,
	MAIN_TEST
} t_MAIN_state;

typedef enum
{
	SUBSTATE_NO_TEXT,
    MAIN_SUB_FT_ACTIVE,
	MAIN_SUB_SIMULATION,
	MAIN_SUB_GPT_ACTIVATED,
	MAIN_SUB_GPT_RUNNING,
	MAIN_SUB_GPT_TRIPPED,
	//MAIN_SUB_CLOSING_VALVES,
	//MAIN_SUB_REFILLING_DIESEL,
	//MAIN_SUB_REFILLING_POIL,
	//MAIN_SUB_SWITCH_TO_WATER,
	//MAIN_SUB_SWITCH_TO_ELECTRIC,
	//MAIN_SUB_OPENING_AIR_FLAPS,
	//MAIN_SUB_CLOSING_AIR_FLAPS,
	//MAIN_SUB_OPENING_EXH_FLAPS,
	//MAIN_SUB_CLOSING_EXH_FLAPS,
	MAIN_SUB_STOPPING,
	MAIN_SUB_FLUSHING,
	MAIN_SUB_FLUSHING_EXHAUST,
	MAIN_SUB_CRANKING,
	MAIN_SUB_CRANK_PAUSE,
	MAIN_SUB_ENGINE_RUNNING,
	MAIN_SUB_PMS_MANUAL_START_STOP,
	MAIN_SUB_ADJUSTING_VOLTAGE,
	//MAIN_SUB_ADJUSTING_VAR,
	MAIN_SUB_ADJUSTING_FREQUENCY,
	MAIN_SUB_ISLAND_OPERATION,									// rmiEPF
	MAIN_SUB_ADJUSTING_POWER,
	MAIN_SUB_POWER_REDUCTION,
	MAIN_SUB_DELOAD,
	MAIN_SUB_INT_PRELUBRICATION,
	MAIN_SUB_START_PRELUBRICATION,
	MAIN_SUB_POST_LUBRICATION,
	MAIN_SUB_MIX_CONFIG,
	MAIN_SUB_SCR_MANUAL,
	//MAIN_SUB_MOVING_TO_POIL,
	//MAIN_SUB_MOVING_TO_DIESEL,
	//MAIN_SUB_OILPUMP_ON
	MAIN_SUB_ISLAND_PARALLEL_DELOADING,
	MAIN_SUB_ISLAND_PARALLEL_OPERATION,
	MAIN_SUB_EZA_STOP,
	MAIN_SUB_EZA_LOADREDUCTION,
} t_MAIN_Substate;

typedef enum
{
	MAIN_GRID_PARALLEL_NORMAL_OPERATION,
	MAIN_GRID_PARALLEL_ADJUST_POWER,
	MAIN_GRID_PARALLEL_ADJUST_BLINDPOWER,
	MAIN_GRID_PARALLEL_SOFT_DISCONNECT_T1E,
	MAIN_GRID_PARALLEL_ADJUST_POWER_FOR_LOCAL_ISLAND_OPERATION, // not used
	MAIN_GRID_PARALLEL_SPEED_CONTROL

} t_MAIN_Regstate;

typedef enum
{
	MAIN_NO_REDUCTION,
	MAIN_COOLING_WATER,
	MAIN_EXHAUST_TEMP_A,
	MAIN_EXHAUST_TEMP_B,
	MAIN_OIL_TEMP,
	MAIN_RECEIVER_TEMP,
	MAIN_WARMING,
	MAIN_CH4,
	MAIN_MPI,
	MAIN_MISFIRE,
	MAIN_FREQUENCY,
	MAIN_VOLTAGE,
	MAIN_THROTTLE,
	MAIN_GAS_LEVEL,
	MAIN_HEAT_CONTROL,						// only stop, rmiSTE
	MAIN_MAINS_POWER_FAILURE,				// only stop, rmiSTE
	MAIN_MAXPOWER_GAS_A,
	MAIN_MAXPOWER_GAS_B,
	MAIN_MAXPOWER_GBV,
    MAIN_MAXPOWER_AKR,
    MAIN_MAXPOWER_MIX,
    MAIN_LR_30_PERCENT,
	MAIN_LR_60_PERCENT,
	MAIN_EXHAUST_CYLINDER_TEMP,
	MAIN_PMS,
	MAIN_BATTERY,
	MAIN_POWER_LIMITATION
} t_MAIN_Reduction;

typedef struct MAINstruct_IO
{
    DBOOL   DI_IslandParallel;
} t_MAIN_IO;
extern t_MAIN_IO MAIN_IO;

#define MAIN_ISLAND_PARALLEL            ((DI_FUNCT[IOA_DI_ISLAND_PARALLEL].Assigned == ASSIGNED) &&  MAIN_IO.DI_IslandParallel)
#define MAIN_ISLAND_PARALLEL_ACTIVE     ((MAIN.state == MAIN_ISLAND_OPERATION) &&  MAIN_ISLAND_PARALLEL)
#define MAIN_ISLAND_PARALLEL_NOT_ACTIVE ((MAIN.state == MAIN_ISLAND_OPERATION) && !MAIN_ISLAND_PARALLEL)
#define MAIN_GRID_PARALLEL              ((MAIN.state == MAIN_GRID_PARALLEL_LIMITED_LOAD) || (MAIN.state == MAIN_GRID_PARALLEL_FULL_LOAD))

extern DBOOL MAIN_IslandParallelActive(void);
extern DBOOL MAIN_IslandParallelNotActive(void);
extern DBOOL MAIN_GridParallel(void);

// software/hardware version numbers
typedef struct
{
	DU16    InterfaceVersion;
    DU16    HardwareVersion;
    DU16    SoftwareVersion;
    DU32    SoftwareRevision;
    DU16    CheckSumConfig;					//rmiREG
} t_version;

typedef struct MAINstruct
{
   t_version  Module2;
   t_version  Module3;
   t_version  Module4;
   t_version  Module5;
   t_version  Module6;
   t_version  Module7;
   t_version  Module8;
   t_version  Module9;



   // Outputs
   DBOOL    DO_ReadyForOperation;
   DBOOL    DO_IslandOperation;
   DBOOL    DO_Loadsharing;
   DBOOL    DO_AutomaticOperation;
   DBOOL    DO_LowIdleSpeed;

   // inputs
   DBOOL    DI_DigitalAutoDemand;
   DBOOL    DI_StartdemandRemote;
   DBOOL    DI_FastStop;
   DBOOL	DI_Acknowledge;
   DBOOL	DI_AutomaticOperation;

   // internal

   DBOOL    IO_TestDemand;
   DBOOL    IO_TestActive;
   DBOOL    Simulation;
   DU8      SimulationValue;
   DBOOL    ManualOperation;
   DBOOL    TestMode;
   DBOOL    RegularStop;
   DBOOL    BlockStart;
   DBOOL    LowIdleSpeed_Demand;

   DBOOL    startdemand;
   DBOOL    StartdemandLocal;
   DBOOL    StartdemandRemote;
   DBOOL    StartdemandRemoteAndAuto;
   DBOOL	AcknowledgeActive;
   DBOOL    GridParallelDelayed;            // indicates that we run parallel to the grid since a while
   DBOOL	EngineRunningNominalDelayed;	// indicates that we run at nominal speed since a while
   DBOOL	GasChangeOverInIdle;            // indicates that changeover of gas type is allowed when set to
   	   	   	   	   	   	   	   	   	   	    // idle changeover by parameter
   DBOOL    WishToCloseGCB;
   DBOOL	ExternalSynchronization;

   DBOOL    NovUpdateRequired;
   DBOOL	ManualPowerSetpointInitialized;
   DS32		ManualPowerSetpoint;
   DU32		DeloadTimeout;					// dynamically calculated time limit for the deload state
   DU32		AccelerationTimeout;			// dynamically calculated time limit for acceleration state

   DU16     SC_IndexToStopTrending;

   DBOOL    StopInIsland;

// temporary
   DU16		Client;

   DS32		MaxPowerDueToEngineProtections;

   DBOOL    PowerSetpointHasChanged;
   DU8      state;
   DU8      subState;
   DU8      regState;
   t_MAIN_Reduction reduction;
   t_MAIN_Reduction StopEngine;      // stop engine because of ....., rmiSTE
   t_MAIN_Regstate OldregState;
   t_Logic_Signal T1EisSynchron;     // flag to indicate, that the synchronisation is reached
   t_Logic_Signal L1EisSynchron;     // flag to indicate synchronisation is reached (for mains breaker), rmiMB
   DU32     DebugBitMask_10103;      // Debug Timeout data exchange, rmi2SCM
   DU32     DebugBitMask_10103_HMI;
     // for Debugging SC 10103:
     // =======================
     // Bit 0 = !CARD_OK SCM02 during init
     // Bit 1 = !CARD_OK IOM03 during init
     // Bit 2 = !CARD_OK IOM04 during init
     // Bit 3 = !CARD_OK IOM05 during init
     // Bit 4 = !CARD_OK IOM06 during init
     // Bit 5 = !CARD_OK IOM07 during init
     // Bit 6 = !CARD_OK IOM08 during init
     // Bit 7 = !CARD_OK SCM09 during init
     // ---------------------------------
     // Bit 8  = IOM4.2_3 data missing ctr >= 5 in 20ms task prio 1 DO + 4AO
     // Bit 9  = IOM4.2_3 data missing ctr >= 5 in 20ms task prio 1 DI+4AI
     // Bit 10 = IOM4.2_3 data missing ctr >= 5 in 100ms task prio6
     // Bit 11 = IOM4.2_3 data missing ctr >= 5 in 1sec task prio7
     // ---------------------------------
     // Bit 12 = IOM4.2_4 data missing ctr >= 5 in 20ms task prio 1 DO + 4AO
     // Bit 13 = IOM4.2_4 data missing ctr >= 5 in 20ms task prio 1 DI+4AI
     // Bit 14 = IOM4.2_4 data missing ctr >= 5 in 100ms task prio6
     // Bit 15 = IOM4.2_4 data missing ctr >= 5 in 1sec task prio7
     // ---------------------------------
     // Bit 16 = IOM4.2_5 data missing ctr >= 5 in 20ms task prio 1 DO + 4AO
     // Bit 17 = IOM4.2_5 data missing ctr >= 5 in 20ms task prio 1 DI+4AI
     // Bit 18 = IOM4.2_5 data missing ctr >= 5 in 100ms task prio6
     // Bit 19 = IOM4.2_5 data missing ctr >= 5 in 1sec task prio7
     // ---------------------------------
     // Bit 20 = IOM4.2_6 data missing ctr >= 5 in 20ms task prio 1 DO + 4AO
     // Bit 21 = IOM4.2_6 data missing ctr >= 5 in 20ms task prio 1 DI+4AI
     // Bit 22 = IOM4.2_6 data missing ctr >= 5 in 100ms task prio6
     // Bit 23 = IOM4.2_6 data missing ctr >= 5 in 1sec task prio7
     // ---------------------------------
     // Bit 24 = IOM4.2_7 data missing ctr >= 5 in 20ms task prio 1 DO + 4AO
     // Bit 25 = IOM4.2_7 data missing ctr >= 5 in 20ms task prio 1 DI+4AI
     // Bit 26 = IOM4.2_7 data missing ctr >= 5 in 100ms task prio6
     // Bit 27 = IOM4.2_7 data missing ctr >= 5 in 1sec task prio7
     // ---------------------------------
     // Bit 28 = IOM4.2_8 data missing ctr >= 5 in 20ms task prio 1 DO + 4AO
     // Bit 29 = IOM4.2_8 data missing ctr >= 5 in 20ms task prio 1 DI+4AI
     // Bit 30 = IOM4.2_8 data missing ctr >= 5 in 100ms task prio6
     // Bit 31 = IOM4.2_8 data missing ctr >= 5 in 1sec task prio7

   DU32     DebugBitMask2_10103;      // second Bitmask for SCM-cards, rmiTIC
   DU32     DebugBitMask2_10103_HMI;
     // for Debugging SC 10103:
     // =======================
     // Bit 0 = SCM2 Data missing counter >= 5 in 20ms task output prio1 (Enable/RouteThrough)
     // Bit 1 = not used yet
     // Bit 2 = not used yet
     // Bit 3 = not used yet
     // ---------------------------------
     // Bit 4  = SCM2 Data missing counter >=5 in 20ms task prio1
     // Bit 5  = SCM2 Data missing counter >=5 in 20ms task prio2
     // Bit 6  = SCM2 Data missing counter >=5 in 20ms task prio3
     // Bit 7  = SCM2 Data missing counter >=5 in 20ms task prio7
     // ---------------------------------
     // Bit 8  = SCM9 Data missing counter >= 5 in 20ms task output prio1 (Enable/RouteThrough)
     // Bit 9  = not used yet
     // Bit 10 = not used yet
     // Bit 11 = not used yet
     // ---------------------------------
     // Bit 12 = SCM9 Data missing counter >=5 in 20ms task prio1
     // Bit 13 = SCM9 Data missing counter >=5 in 20ms task prio2
     // Bit 14 = SCM9 Data missing counter >=5 in 20ms task prio3
     // Bit 15 = SCM9 Data missing counter >=5 in 20ms task prio7

   DBOOL MaintenanceMode;
} t_MAIN;

extern t_MAIN MAIN;

extern void ResetMainlog(void);

#define MAIN_STATE_LOG_NUMBER_OF_LINES  500  
#define MAIN_CYCLE_LOG_NUMBER_OF_LINES  500	// every hour ==> round about 20 days

typedef struct
{
   DU16 MAIN_stateLog_pointer;
	t_MAIN_StateLogLine MAIN_StateLog[MAIN_STATE_LOG_NUMBER_OF_LINES];
	DU16 MAIN_cycleLog_pointer;
	t_MAIN_CycleLogLine MAIN_CycleLog[MAIN_CYCLE_LOG_NUMBER_OF_LINES];
}t_nov_mainlog;
extern t_nov_mainlog mainlog;

extern t_MAIN_StateLogLine MS_GetLog (DU16 MSIndex);
extern t_MAIN_CycleLogLine MSC_GetLog (DU16 MSCIndex);

// *******************************************************
// ** SW version number **********************************
// *******************************************************
//
//#define SW_NUMBER					999 //61
//// next version to come: 62
//#define SW_VERSION                 (PROJECT_NO*1000L+SW_NUMBER)
//#define SW_DATE_OF_VERSION			"15.01.2020"
//#define SW_COMMENT					""

//// last software version before incompatibility
//// version of parameter file used for filetransfer must be at least SW_NUMBER_COMPATIBILITY_CHANGED to be accepted
//#define SW_NUMBER_COMPATIBILITY_CHANGED		1 // Parameters relative to nominal values

// to identify special customer software: definition of the project number:
// PDK MOVED to file build.conf
//#define PROJECT_NO				 0L
//#define PROJECT_NO				 436500L

//
// rules for SW version numbering:
// each release gets a new number
// first digit indicates big changes like totally new structure, e.g. 1.45.2 -> 2.00.0
// second and third digits indicates add-on functionality or new modules, e.g. 2.34.2 -> 2.35.0
// last digit changes with debugging or slight inner changes, e.g. 1.01.2 -> 1.01.3

// please log SW version here:
// WHO  version  date
// MVO  1.00.0   18.10.2007
// MVO  1.00.1   24.10.2007, finished 5.11.2007
// MVO  1.00.2   06.11.2007
// MVO  1.00.3   12.12.2007, delivered to Kraft and Surgalla
// MVO  1.00.4   17.12.2007, delivered to Gfrörer
// MVO  1.00.5   18.12.2007, delivered to Hauser 27.12.2007
// MVO  1.00.6   02.01.2008, delivered to Kraft 10.01.2008
// MVO  1.00.7   17.01.2008,
// MVO  1.00.8   23.01.2008, delivered to Gfroerer and Hauser 23.01.2008
// MVO  1.01.0   04.02.2008, delivered to Surgalla
// GFH  1.01.1   13.02.2008,
// MVO  1.10.0   12.08.2008, test gas software for IET project

// MVO  1.23.4
// MVO  1.23.5   02.09.2008, filtering of Receiver pressure
//                           software delivered to IET for test of mixer
// MVO  1.23.6   08.09.2008, Main.DebugBitMask_10103 put on diagnostics
//                           to analyze IO data exchange timeout on site
//                           acknowledge after software update in PAR.c
// MVO  1.23.7   11.09.2008, for tests in week 38/2008 IET, finished 19.09.2008
// MVO  1.23.8   20.09.2008, for week 40/2008 IET, first customer version
// MVO  1.23.9   24.10.2008, delivered to IET by email for test run
// MVO  1.24.0   28.10.2008, started in Villach for IET, changes
//                           delivered to IET by email 10.11.2008
// MVO  1.24.1   11.11.2008,
// RMI	1.30.0   14.11.2008, Modem connected to Display-Interface,
//							 IO-Card-addresses = module-number + 1	(because second SCM should be 3)
// MVO           19.11.2008, version delivered to IET for commission Konstanz/Germany
// MVO  1.30.1   20.11.2008, delivered to IET for commission Konstanz/Germany 21.11.2008
// MVO  1.30.2   21.11.2008, version for IET, delivered
// MVO  1.30.3   09.12.2008, version for IET, delivered 09.12.2008
// RMI  1.30.4   11.12.2008, version for IET, DKB_init to enable engine start
// MVO  1.30.5   16.12.2008, version for IET (MIX debugged), delivered 16.12.2008
// MVO  1.30.6   19.12.2008, bugfix for IET (startdemand), delivered 19.12.2008
// MVO  1.30.7   23.12.2008, speed limitation in MIX deactivated
// MVO  1.30.8   14.01.2009, preheating changed in WAT, used for test runs 27.01.2009 in Villach
//               30.01.2009, overtemperare heating circuit flow crrected with negativetemperatures
//                           SMS messsages added
// MVO  1.30.9   02.02.2009, version started 02.02.2009
// RMI  1.31.0   04.02.2009, island operation implemented
// RMI  1.31.1   17.04.2009, version number used for bugfix IET (TUR.TargetPhaseAngle)
// GFH  1.31.2   23.04.2009, modbus-support (ReceiverPressureFilteredValue-1000, 0.1 Hz)
// GFH  1.31.3   11.05.2009, nominal power < 100 kW, DEIF-version english-german, IET-version
// -------------------------------------------------------------------------------------------
// RMI  1.31.4   21.07.2009, text-definitions in bootflash ==> bfl_text needed !
// RMI  1.32.0   03.09.2009, protection function on SCM supported
// RMI  1.32.2   29.10.2009, version for IET
// RMI  1.32.4   24.11.2009, version for IET: solved I2t-probloem
// RMI  1.32.5   24.11.2009, version for IET: no ignition communication check
// RMI  1.32.6   21.12.2009, ignition diagnostic supervision, some DI's assignable
// RMI  1.32.7   21.01.2010, TREND optimization (no switch/case-construct)
// MVO  1.33.0   23.02.2010, IET version
// RMI  1.33.2   05.06.2010, IET version, slovenian, UTF8
// RMI  1.33.5   27.07.2010, IET, exhaust bypass option depending on heating flow and return (by MVO)
// RMI  1.33.6   05.08.2010, max values for mains recover time increased (necessary in Austria)
// RMI  1.34.4   04.03.2011, deactivated: parameter 26060, SC 70262, new: par 26075,  SC 70249
// RMI  1.35.0   28.03.2011, cylinder temperature supervision activated
// RMI  1.36.1   04.05.2011, cooling pump (and preheater) off, when SC 30030 set
// RMI  1.36.2   11.05.2011, gas flow unit changed m3/h -> Nm3/h
// MVO  1.42.4   06.11.2013, heat water flow and heat counter added
// MVO  1.42.5   22.11.2013, gas flow B added
// MVO  1.42.6   25.11.2013, speed to Modbus, number of par. messages increased for DEIF/Edel
// MVO  1.42.7   09.12.2013, back sync in gas type B, Modbus power demand also in PMS
// --------------------- DM400 std 436729 -------------------
// MVO  006	19.12.2014	release with separated port numbers
// GFH  007	22.01.2015	released for Ortadogu Turkey
// GFH  008	12.03.2015	released for Geisberger after LVRT-Tests
// ZZH  081 07.04.2015  MIC-4 CAN-Open supported
// MVO  009 23.04.2015  Phlox EMCY, error messages and alarm reset, released
// GFH  010 06.05.2015  Reactivate Moving to Idle-Position in Acceleration
// MVO  012             version for IET with increased settings for LR due to receiver temp
// MVO  013				released
// GFH  014	31.08.2015  version for IET PMS-CAN modified, MIC4 unblocked, CAI/TMP on Modbus
// MVO	015 09.11.2015	flushing air blower and flap, delivered to IET
// GFH	016 20.01.2016	for AKR testing IET
// GFH	017 25.02.2016	Kaltimex Bangladesh
// MVO  018 16.03.2016  Ortadogu Turkey
// GFH  019 07.04.2016  Activate SMS for Nimtofte
// GFH  020 30.06.2016  Activate LS-Transit for Copasa
// GFH  021 21.09.2016  PMS Correction (wrong array size) for IET
// GFH  022 04.10.2016  Modbus corrected for IET
// GFH  023 10.11.2016  CosPhi Regulation for AnKo
// MVO  024 30.01.2017	after cooling pump and separation of MK pump from preheating for IET
// MVO  025 02.03.2017	start blocking after undervoltage and inhibit of cooling water pressure for IET
// GFH  026 05.04.2017	Software for grid-protection test Korea
// GFH  027 29.05.2017	Software for Korea
// MVO  028 08.06.2017  IET version released (write access needed for modbus power setpoint)
// GFH  029 21.06.2017  Software for Korea: Commissioning MAN 3262
// GFH  030 03.08.2017  Software Activation
// GFH  031 10.08.2017  IET: Improvement for Gas Blending
// GFH  032 08.11.2017  IET: FFT-Values AKR on CAN
// GFH  033 22.11.2017  IET: Slovenian Translation
// GFH  034 12.12.2017  IET: AKR Separate SC's for Kanada
// GFH  035 19.01.2018  IET: GAS Max power Correction / U Trans Sec 690V / AKR FFT-Diagnose Mode Single/Continuous
// GFH  036 09.03.2018  Kaltimex Bangladesh
// GFH  037 09.04.2018  IET: Several changes
// GFH  038 25.04.2018  LEO: 2nd CLS / Modbus RS485
// MVO  039 ???
// GFH  040 24.05.2018  Shutdown after breaker open
// GFH  041 13.07.2018  FDN: Filter for gas level / El. Protection moved from 1000ms() to 20ms() / El. Simulation
// GFH  042 16.08.2018  IET: AKR Reset / Grid-Protection Sync-Release and U>(avg) on all delta and star voltages
// GFH  043 16.08.2018  IET: ELM Small Correction
// GFH  044 12.09.2018  IET: AKR Reset / HZM Acknowledge
// GFH  045 13.09.2018  IET: ELM Small Correction -> Power to display
// GFH  046 17.09.2018  IET: AKR Reset 5sec
// GFH  047 18.09.2018  IET: AKR Reset in Startprepare
// GFH  048 01.10.2018  IET: TLB gain / HZM Trending
// GFH  049 17.10.2018  IET: Grid-Protection Recover  / Preheating Temp. In/Out
// GFH  050 20.11.2018  IET: TLB regulation depending on el. power
// GFH  051 20.11.2018  IET: TLB regulation depending on el. power (Correction)
// GFH  052 04.12.2018  IET: Q Regulation by AI - Range expanded
// GFH  053 18.12.2018  DEIF: Cottingham
// GFH  054 16.01.2019  DEIF: Cottingham
// GFH  055 08.02.2019  DEIF: For DNI CAT Engine Mexico
// GFH  056 09.02.2019  DEIF: Cottingham
// GFH  057 20.02.2019  DEIF: For DNI CAT Engine Mexico
// GFH  058 26.03.2019  DEIF: For David Flemming UK - DO "time switch"
// GFH  059 28.03.2019  DEIF: For David Flemming UK - Default for DO "time switch" changed
// GFH  060 11.07.2019  IET: MAN 2676 6-Cylinder (Line)
// GFH  061 30.08.2019  IET: Support SCM even if external control of generator breaker and external U/Q regulation
// GFH  066 07.01.2020  DNI/India: DO "auxiliaries (engine stop)"
// GFH  068 09.01.2020  DNI/India: Filter for AI gas mixer position
// GFH  069 06.02.2020  IET: Write-Access IC92x
// TBA  070 18.02.2020  IET: IO_ASSIGNMENT_ALLOWED for terminals 301, 305, 356, 416, 553


//  parameter constants til MAIN module

// timeout for MAIN state STRT_PREPARE
#define MAIN_STRT_PREPARE_TIMEOUT          240000L // 4 min

// timeout for acceleration from 0 to 800 Rpm
#define MAIN_ACCELERATION_STATE_TIMEOUT    120000L // 2 min

// demand new power setpoint only if change in W is bigger than this parameter
#define MAIN_POWER_CHANGE_DEADBAND          1000L

// replaced by parameter
// timeout for idle running
//#define MAIN_IDLE_RUN_TIMEOUT             300000L // 5 min
#define MAIN_IDLE_RUN_TIMEOUT             ((DU32)PARA[ParRefInd[MAIN_IDLE_RUN_TIMEOUT__PARREFIND]].Value)

// timeout for synchronization
#define MAIN_SYNCHRON_CONNECT_L1E_TIMEOUT 300000L // 5 min for synchronize and connect L1E

// timeout for Deloading
#define MAIN_DELOAD_TIMEOUT               300000L

// cooldown timer in case of mains failure
#define COOL_DOWN_TIME_WITH_MAINS_FAILURE  30000L

// timeout for connection of T1E
#define MAIN_CONNECTT1E_TIMEOUT            10000L // 10 s

// replaced by parameter
// max time for engine warming
//#define MAIN_MAX_WARMING_TIME             600000L // 10 min
#define MAIN_MAX_WARMING_TIME             ((DU32)PARA[ParRefInd[POWER_WARMING_TIMEOUT__PARREFIND]].Value)

// cut out load in deload = Pnominal / fraction
#define MAIN_CUT_OUT_FRACTION_DELOAD		50L
// Cut out load in Island parallel ramp down = Pnom/Fraction
#define MAIN_CUT_OUT_FRACTION_ISLPAR    20L

// cut out load in loadsharing ramp down = Pnominal / fraction
#define MAIN_CUT_OUT_FRACTION_LOADSHARING	20L

// timeout for loadsharing ramp up [ms]
#define MAIN_LOADSHARING_RAMP_UP_TIMEOUT	450000L

// timeout for loadsharing ramp down [ms]
#define MAIN_LOADSHARING_RAMP_DOWN_TIMEOUT	450000L

// hysteresis for switching to load sharing ramps [W]
#define MAIN_ACCEPTABLE_POWER_DEVIATION      20000L

// Simulation
typedef enum
{
	MAIN_SIM_GEN_UL1L2 = 0,
	MAIN_SIM_GEN_UL2L3,
	MAIN_SIM_GEN_UL3L1,
	MAIN_SIM_GEN_UL1N,
	MAIN_SIM_GEN_UL2N,
	MAIN_SIM_GEN_UL3N,
	MAIN_SIM_GEN_F,
	MAIN_SIM_GEN_IL1,
	MAIN_SIM_GEN_IL2,
	MAIN_SIM_GEN_IL3,
	MAIN_SIM_GEN_P,
	MAIN_SIM_GEN_Q,
	MAIN_SIM_BB_UL1L2,
	MAIN_SIM_BB_UL2L3,
	MAIN_SIM_BB_UL3L1,
	MAIN_SIM_BB_UL1N,
	MAIN_SIM_BB_UL2N,
	MAIN_SIM_BB_UL3N,
	MAIN_SIM_BB_F,
	MAIN_SIM_GRID_UL1L2,
	MAIN_SIM_GRID_UL2L3,
	MAIN_SIM_GRID_UL3L1,
	MAIN_SIM_GRID_UL1N,
	MAIN_SIM_GRID_UL2N,
	MAIN_SIM_GRID_UL3N,
	MAIN_SIM_GRID_F,
	MAIN_SIM_GRID_IL1,
	MAIN_SIM_GRID_IL2,
	MAIN_SIM_GRID_IL3,
	MAIN_SIM_GRID_P,
	MAIN_SIM_GRID_Q,
} t_MAIN_SIM_Value;

extern void MAIN_control_init(void);
extern void MAIN_control_20ms(void);
extern void MAIN_control_100ms(void);
extern void MAIN_control_1000ms(void);
extern DU8* MAIN_state_text(DU8 state);
extern DU8* MAIN_subState_text(DU8 state);
extern DS32 MAIN_realpower_max_allowed(void);
extern DS32 MAIN_actual_realpower_setpoint(void);
extern DS32 MAIN_limit_ManualPowerSetpoint(DS32 adjustment);
extern void MAIN_Set_LowIdleSpeed(void);

#endif /*MAIN_CONTROL_H_*/
