/**
 * @file MAIN_CONTROL.c
 * @ingroup Application
 * 
 * This is the main state handler
 * of the DM4 REC gas engine plant controller
 *
 * remarks
 * void MAIN_control() is called from 50Hz control Task
 * author aes
 * date 04-okt-2006
 * modified mvo
 * 
 * 
 * 25-sep-2007 mvo: ReadTime() and SetTime() added to set system clock from BingBang service 0x0C
 * 1002	 07.11.2007	 FUE.ConsumptionTotalCounter instead of FUE.TotalFuelConsumption (for CycleLog)
 * 1005  19.12.2007  MVO  In the state MAIN_STARTING the call of GEN.mode is now depending on the state
 *                        of ENG to switch off excitation during preglow and make timer for U/f failure
 *                        work correctly.
 * 1007 22.01.2008 GFH  activate automatic restart in case of self acknowledged StopConditions
 * 1011 17.03.2008 GFH  include input/output assignments
 * 1011 14.05.2008 GFH  check BIT 4 in STOP.actualBitMask also in state : FastBreaking & EmergencyStop & EmergencyBreaking
 *                      switch off test mode in state EmergencyStop
 * 1237 11.09.2008 MVO	handling of quick stop handling improved
 * 1239 24.10.2008 MVO  load reduction due to CH4 added
 * 1240 28.10.2008 MVO  corrections for test run in Villach
 * 1300 17.11.2008 MVO  TUR.MAINRpmSet in ACCELERATION increased from 450 to 650 rpm. This was needed for engines with
 *                      low inertia, because they shoot up when igniting and were pulled down to zero by the old code.
 * 1301 20.11.2008 MVO  start value for speed ramp made parameterizable, power reduction due to misfiring added
 * 1302 02.12.2008 RMI  support DKB, rmiGASB
 * 1306 19.12.2008 MVO/RMI startdemand by digital input debugged
 * 1309 24.02.2009 GFH  load reduction due to DK.MaxPower and GPC.MaxPower
 * 1309 02.03.2009 GFH  set gas compressor demanded in state "ready for start" 
 * 1310 15.01.2009 RMI	new main states to support emergency power failure, rmiEPF
 * 1310 05.03.2009 RMI  startdemand due to MPF only if AUTO mode
 * 1310 11.03.2009 GFH	use internal variable "ISL.DemandIslandOperation" instead of input variable "ISL.DI_DemandIslandOperation"
 * 1310 11.03.2009 GFH  use modbus value MBA.bFastStop and MBA.bAutomaticOperation and MBA.bStartDemand if assigned
 * 1310 27.03.2009 GFH  modify main state machine for "standby power operation"
 * 1310 03.04.2009 GFH  support of "2 gas line operation" and debugging "standby operation"
 * 1314 23.06.2009 GFH  acknowledge by digital input debugged
 * 1314 06.07.2009 RMI  bugfix: reason for StopEngine in case of misfiring now correct (SC 50681 instead of 50679 checked)
 * 1315 25.08.2009 RMI  island_operation -> back synchronisation, only, if power recovery time
 * 3121 15.10.2009 GFH  DO_Function for island operation
 * 1326 15.12.2009 MVO  Position setpoint to TUR (TUR.MAINPosSet)
 * 						close throttle in DisconnectT1E
 * 1326 17.12.2009 MVO  set output MAIN.DO_ReadyForOperation according to the wishes of customer Carsten Köllner/IET:
 * 						ready for operation is only true in 4 states 
 * 1327 11.01.2010 RMI  DeloadCounter-reset to avoid SC in parallel-state when changing between normal and deloading
 *						in MAIN_GRID_PARALLEL_ADJUST_POWER, ..._ADJUST_BLINDPOWER, ..._NORMAL_OPERATION
 * 1327 19.01.2010 RMI	take care to get a cycle-log entry every hour, rmiCYCL
 * 1328 22.01.2010 RMI	acknowledge (STOP_ClearAllLevels) now in 100ms-task (not 20ms), rmiSPT
 * 1328 26.10.2010 GFH  new task MAIN_control_1000ms()
 *                      MAIN.PowerSetpointHasChanged = SetpointHasChanged(); from 100ms() to 1000ms()
 *                      reset MAIN.PowerSetpointHasChanged if evaluated
 * 1328 05.02.2010 GFH	MAINLOG at fixed NOVRAM area
 * 1328 05.02.2010 GFH  STOP_Set(STOPCONDITION_10201) relocated into last INIT (MAIN_CONTROL)
 * 1328 09.02.2010 GFH  new transit commands at parallel and island if breaker positions doesn't fit to actual state
 * 1328 09.02.2010 GFH  position regulation (5%) of throttle if generator breaker disconnects at parallel or island
 * 1328 12.02.2010 GFH  use ENG_IDLE_SPEED instead of "14900"
 * 1331 05.03.2010 GFH  check number of SCM modules in mains parallel operation before disconnect if mains breaker not closed
 * 1332 17.03.2010 RMI  SynchronConnectL1E: in case of MAIN_SYNCHRON_CONNECT_L1E_TIMEOUT disconnect to island                   
 * 1332 23.03.2010 RMI  MAIN.*1EisSynchron.Signal reset in SIG_EXIT of synchronization, rmi100323                 
 * 1332 26.03.2010 RMI  if realpower becomes less than 2% of nominal load (no more 10000) then DisconnectT1E, rmi100325                 
 *						DisconnectT1E, if MCB isn't closed, only if 2 SCM cards (when no island operation allowed)
 * 1333 01.07.2010 GFH	loadsharing - power management system
 * 1334 09.07.2010 GFH	loadsharing - mains delomatic
 * 1334 19.07.2010 GFH	leave back sync if busbar frequency not ok
 * 1336 31.07.2010 MVO  Set lower target speed for back synchronization in island and load sharing
 * 1337 09.08.2010 RMI  DisconnectT1E -> Cooldown (always if breaker open)
 * 1336 13.08.2010 GFH	loadsharing - power management system - open issues from visit 04.08.2010
 * 1336 13.08.2010 GFH	cooldown timer in case mains failure >= HOT
 * 1341 25.08.2010 GFH	3x par. PID regulators
 * 1341 25.08.2010 GFH	4th par. PID regulator
 * 1341 02.09.2010 GFH	ramp down in load sharing if regular stop
 * 1341 22.09.2010 GFH  configuration of SCM
 * 1343 08.11.2010 GFH  option: cylinder temperature monitoring
 * 1343 08.11.2010 GFH  reset start demand from soft button if engine stopping
 * 1343 08.11.2010 GFH  no transit from cooldown to acceleration
 * 1343 06.01.2011 GFH  change transit condition from Synchronize to DisconnectT1E from "ELM.BBVoltageIsZero_Flag" to "!ELM.BBVoltageIsOK_Flag"
 * 1351  11.04.2011 GFH	make MainsDeloInSynchronization static
 * 1410 22.03.2012 GFH  set droop mode in loadsharing
 * 1410 07.05.2012 GFH  activate throttle driver in start-prepare
 *      03.07.2012 MVO  IOA.h: now assignable in DEIF version: exhaust temperature after engine A,
 *      				and exhaust temperature after engine B
 *      26.08.2012 MVO  WAT.c: block cooling pump if low water pressure (50162R)
 * 1413 18.09.2012 MVO  IOA.c: GK valve setpoint made assignable
 * 1414 26.09.2012 MVO  TXT.c changes for HGS in stop condition texts 60026, 60040
 *                      STOPCONDITIONS and TXT.c new stop condition text for heating circuit flow monitor(70059)
 * 1421 22.03.2013 MVO  initial throttle position in acceleration changed from 7000 to 15000
 * 1421 22.03.2013 MVO  modbusappl: mixer positions in % added to counter values
 * 1421 22.03.2013 MVO  MIX.h, MIX.c, hmi_pages.c: big or small steps possible in EMIS Config using MIX.Fast
 * 1421 22.03.2013 MVO  hmi_pages.c: aux test pages allow valves to be set to 0% and 100% in one click
 *                      using buttons ^ and v
 * 1421 22.03.2013 MVO  hmi_appl_lib.c: button macros for ^ and v to move valves in test pages to 0% and 100%
 * 1421 22.03.2013 MVO  hmi_pages.c: tightness check feedbacks gas A and B visible on aux test pages
 * 1421 22.05.2013 GFH  grid protection and control according to VDE AR-N 4105 - 2013
 * 1421 07.06.2013 MVO  IOA.h: number of par messages MES_PDI_NBR increased for HGS from 17 to 30
 * 1422 24.07.2013 MVO  NK regulation value depending on NKK_VALVE_CONTROL_MODE__PARREFIND
 * 1422 16.09.2013 GFH  MIX: support of gas mixer with analogue control
 * 1422 16.09.2013 GFH  hmi_pages: support of gas mixer with analogue control
 * 1423 16.09.2013 MVO  RGB: initialization of variable WaitAfterStart
 * 1423 16.09.2013 MVO  hmi_pages: display of speed HZM.u32Phlox[SDO_220F_51] (/10rpm) corrected
 * 1423 18.09.2013 MVO  DK: state machine for throttle pos feedback out of range, with delay
 * 1423 18.09.2013 MVO  AKR: error message "AKR not configured" now with time delay
 * 1423 18.09.2013 MVO  display of AKR software version as ASCII letters
 * 1423 24.09.2013 MVO  hmi_appl_lib: indication of type of ignition communication support
 *                      on diagnostic page (PHLOX, CD200, none)
 * 1424 01.11.2013 MVO  new input heating circuit flow, display in m³/h, parameter
 *      				HKS: calculation of heat in heating circuit from water flow and temperatures
 *      				using a correction factor (for ethylene glykol or other things),
 *      				display on heating page
 *      				heat counter in NOVRAM and on Modbus, flow on Modbus, wire break
 * 1425 15.11.2013 MVO  in call of HKS_count bug corrected (factor 360 replaced by 100)
 *      21.11.2013 MVO  gas flow B added
 * 1426 25.11.2013 MVO  IOA.h: number of par. messages increased from 15 to 20
 * 1426 25.11.2013 MVO  modbusappl: engine speed added to modbus, address 4624, unit 0.1rpm
 * 1427 05.12.2013 MVO  TUR: TUR_read_setpoint_PowerGridParallel adapted to support power
 * 						setting by Modbus or analog also in power management
 * 1427 06.12.2031 MVO  GAS: If back synchronization is desired, no stopping due to
 *                      change of gas type anymore. Resetting of marker GAS.WaitingForBackSynchronization
 *                      in all SIG-ENTRYs where the mains breaker is closed, or the generator
 *                      breaker is open.
 * 1428 03.01.2014 MVO  modbusappl: heat of water system HKS.HK_Heat included into Modbus table, addr. 4625 and 4626
 * 1428 12.01.2014 MVO  MAIN.startdemandremote in case of PMS depending on PMS auto start/stop
 * 						version 1.42.8 released 12.01.2014
 * 		12.05.2014 MVO  waiting time for "ready for rearming" introduced into SAF to
 *						support new safety chain modules of Pilz and Siemens,
 *      17.11.2014 MVO  TXT.c: TEXT_STOPCOND_50166 changed for clarification from
 *      				´calibration timeout´ to ´calibration timeout CH4´
 *      17.11.2014 MVO  PAR, hmi_pages, NKK: internal regulation value changed to
 *      				valve position 0.0000 percent. Parameter 25025 expanded for
 *      				negative logic 20...0mA and 20...4mA
 *      10.12.2014 MVO	applmain.c: server with fixed port numbers:
 * 						   	modbus TCP on port 502
 *   						local operator bing bang on port 30000
 *   						remote operator bing bang on port 29000
 *		10.12.2014 MVO	Change of access level for "correction" parameters 17230-17235 from 2 to 3
 *						Correction Parameters will only be displayed to level 3 users.
 *		20.12.2014 MVO	emergency dry cooler with 7 stages, operation after operating hours
 *436729  12.03.2015  MVO  TXT.c: missing texts for stop conditions added in all languages:
 * 							50137, 50138, 50139, 50140,
 * 							50147, 50148, 50149, 50150
 *436729  15.04.2015  MVO  hmi_pages.c, TXT: comment line added (spark duration control off if setpoint = 0)
 *436729  15.04.2015  MVO  in IET version: power reduction due to PMS removed, PMS creates a setpoint, not a reduction now.
 *436729  23.04.2015  MVO  TXT, STOPCONDITIONS, hmi_pages, HZM: Decode EMCY telegram send by Heinzmann ignition,
 *436729  				   IGN speed measurement HZM.Speed and filtered value, monitoring of speed deviation,
 *436729  				   taking error codes and error status bits from EMCY telegram, creating stop conditions
 *436720  23.04.2015  MVO  HZM: restart of ignition to acknowledge fault/reset error state
 *		  18.06.2015  MVO  AKR.c, STOPCONDITIONS.c: power reduction if SC 60130 (AKR not configured), no stop
 *436729  22.07.2015  MVO  TRE, TXT: TecJet values added as curves 6 to 11 on page 2
 *436729  				   TXT, STOPCONDITIONS: TecJet warnings 70160-70197 added
 *436729  14.08.2015  MVO  IOA: free assignment of receiver temp also in IET version
 *436729  				   flushing air blower and flap, pre-chamber valve,
 *436729  				   manual mode after emergency stop, cold junction B,
 *436729
 *436729  16.02.2016  MVO  DKA.h: wait time for leakage check changed from 15s to 120s
 *		  18.02.2016  MVO  FAB.h, FAB.c: DO_Valve added
 *		  				   test mode for flush air valve added, texts and input added
 *436729  15.03.2016  MVO  due to problems with the position feedback of the pro act on site Ortdogu:
 *						   using the throttle position setpoint instead of the feeback for the calculation of the
 *						   TecJet flow setpoint in MIX.c
 *436729  16.03.2016  MVO  new par unit PERCENT_PER_K
 *436729  				   new parameters 22112 and 22113 adjustable start enrichment and selection of measurement point for this
 *436729  				   autoscaling of trending curves TecJet flow + new tick type
 *436729  05.10.2016  MVO  test version 989 for IET: adjustable ramp time for moving
 *		  				   tec jet from start flow to idle flow
 *436729  28.10.2016  MVO  test version 988 for IET with gas blending and gas counters
 *436729  16.11.2016  MVO  WAT.h: MAX_DIFF_PRES_TIMER corrected from 100ms to 3000ms
 *436729  25.11.2016  MVO  NKK: operating hours counter for emergency cooling pump/stage 1
 *		  				   counters of emergency cooling fans on modbus
 *436729  28.12.2016  MVO  ELM.c wire break evaluation of kW signal input only if GCB open
 *		  				   kW signal input scaling changed to see reverse power
 *436729  30.01.2017  MVO  corrections to after cooling pump and to separated preheating
 *436729  02.03.2017  MVO  cooling water circuit pressure supervision:
 *						   min supervision inhibited for 5s after starting the cooling water pump
 *						   max supervision inhibited for 5s after stopping the cooling water pump
 * 028    08.06.2017  MVO  TUR.c: take setpoint for power by modbus only if write access
 * 		  14.06.2017  MVO  shell for NOx sensor evaluation added: NOX.h, NOX.c
 * 098	  21.07.2017  MVO  NOx and O2 exhaust sensor values
 * 						   new module SCR for catalyst operation
 * 436729 18.09.2017  MVO  MVO_COMMITT_SCR entfernt
 */

#include "options.h"
#include "applrev.h"
#include "deif_types.h"
#include "appl_types.h"
#include "STOPCONDITIONS.h"
#include <bing_bang.h>
#include "statef.h"
#include "debug.h"
#include "hmi_appl_lib.h"
#include "AIR.h"
#include "AKR.h"
#include "ARC.h"
#include "AUD.h"
#include "CH4.h"
#include "COM.h"
#include "CYL.h"
#include "CLK.h"
#include "DK.h"
#include "DKA.h"
#include "DKB.h"				//rmiGASB
#include "ENG.h"
#include "EXH.h"
#include "FAB.h"
#include "FAN.h"
//#include "FUE.h"
#include "GAS.h"
#include "GBV.h"
#include "GEN.h"
#include "HKS.h"
#include "HVS.h"
#include "HZM.h"
#include "IC9.h"
#include "IGN.h"
#include "ISL.h"
#include "MES.h"
#include "MIC4.h"
#include "MIX.h"
#include "MPI.h"
#include "NKK.h"
#include "OIL.h"
#include "PLS.h"
//#include "PFH.h"
#include "REG.h"
#include "SAF.h"
#include "TEC.h"
#include "THR.h"
#include "TLB.h"
#include "TRE.h"
#include "TUR.h"			// rmi, 24.04.09 (former included in vdb.h)
#include "TXT.h"
#include "PAR.h"
#include "PID.h"
#include "PMS.h"
#include "RGB.h"
#include "SCR.h"
#include "SER.h"
#include "WAT.h"			// rmi, 24.04.09 (former included in vdb.h)
#include "file_transfer.h"
#include "GPC.h"
#include "modbusappl.h"
#include "tem_zs3.h"
#include "iohandler.h"
#include "NOV.h"

// declare NOVRAM memory for main state log lines and cyclic log lines
t_nov_mainlog mainlog;

// Position control in low idle
#define PAR_CUMMINS_OPTION				(PARA[ParRefInd[CUMMINS_OPTION__PARREFIND]].Value)
#define PAR_CUMMINS_THROTTLE_LOW_IDLE	(PARA[ParRefInd[CUMMINS_THROTTLE_LOW_IDLE__PARREFIND]].Value)

// declaration of the state transit and the state functions

static void transit( STATE myState );

static void	SystemStart(const DU8 sig);
static void	UndefinedBreakers(const DU8 sig);
static void	BlackOperation(const DU8 sig);
static void	MainsOperation(const DU8 sig);
static void EmergencyStop(const DU8 sig);
static void EmergencyBreaking(const DU8 sig);
static void RearmSafetyChain(const DU8 sig);
static void SystemStop(const DU8 sig);
static void SystemReadyForStart(const DU8 sig);
static void FastBraking(const DU8 sig);
static void StartPrepare(const DU8 sig);
static void Starting(const DU8 sig);
static void Ignition(const DU8 sig);
static void OpenGasValves(const DU8 sig);
static void Acceleration(const DU8 sig);
static void IdleRun(const DU8 sig);
static void WaitForReleaseCloseGCB(const DU8 sig);
static void Synchronize(const DU8 sig);
static void GridParallelOperationLimitedLoad(const DU8 sig);
static void GridParallelOperationFullLoad(const DU8 sig);
static void DisconnectT1E(const DU8 sig);
static void Cooldown(const DU8 sig);
static void Test(const DU8 sig);

//static void GeneratorOff(const DU8 sig);
//static void Demagnetize(const DU8 sig);
//static void Magnetization(const DU8 sig);
static void ConnectT1E(const DU8 sig);
static void IslandOperation(const DU8 sig);
static void LoadSharingRampUp(const DU8 sig);
static void LoadSharing(const DU8 sig);
static void LoadSharingRampDown(const DU8 sig);
//static void ConnectL1E(const DU8 sig);
static void SynchronConnectL1E(const DU8 sig);
//static void HighVoltageIslandOperation(const DU8 sig);
//static void DisconnectL1E(const DU8 sig);
//static void GridParallelOperationDiesel(const DU8 sig)
//static void GridParallelOperationPOil(const DU8 sig);
static void DisconnectL1EtoIsland(const DU8 sig);		// rmiEPF
static void DisconnectT1EIsland(const DU8 sig);			// rmiEPF

// Maincontrol structure including all public Variables of MAIN, see .h
t_MAIN_IO MAIN_IO;
t_MAIN MAIN;

// Local variables, not known outside this module
static STATE 	myState = 0;
//static STATE 	myLastState = 0;
static DU32 	myStateCnt = 0;
//static DU16 	MAIN_stateLogCnt;
static DBOOL 	PowerReductionActive;
static DU32 	DeloadCounter;
//static DU32 	DieselToPoilWaitTimer;
static DBOOL 	CloseThrottle = FALSE;				// close DK in case of MCB trip

DBOOL MAIN_IslandParallelActive(void)
{
	return MAIN_ISLAND_PARALLEL_ACTIVE;
}

DBOOL MAIN_IslandParallelNotActive(void)
{
	return MAIN_ISLAND_PARALLEL_NOT_ACTIVE;
}

DBOOL MAIN_GridParallel(void)
{
	return MAIN_GRID_PARALLEL;
}

static void MAIN_MainsDelomaticState(void)
{
	switch (PMS.CBPOS_OperationMode)
	{
		case PMS_CBPOS_UNDEFINED:
			// undefined breaker position
			transit(UndefinedBreakers);
		break;

		case PMS_CBPOS_BLACK_OPERATION:
			// black operation
			transit(BlackOperation);
		break;

		case PMS_CBPOS_MAINS_OPERATION:
			// mains operation
			transit(MainsOperation);
		break;

		case PMS_CBPOS_ISLAND_OPERATION:
			// island operation
			transit(IslandOperation);
		break;

		case PMS_CBPOS_PARALLEL_OPERATION:
			// parallel operation
			transit(GridParallelOperationFullLoad);
		break;

		default:
		break;
	}
}

static void MAIN_Set_MIX_mode(DU8 mode)
{
	if (MIX.Manual && !MIX_OPTION_TECJET) // mixer in configuration
	{
		MIX.mode[MixerInd1] = MIX_TEST_DEMANDED;
		MIX.mode[MixerInd2] = MIX_TEST_DEMANDED;
	}
	else if (ELM.ReleaseLoadForMixerControl)
	{
		// 31.07.2014 MVO: distinguish between lambda control and rest
		if (MIX.MixerControlRelease.State == TRIP)
		{
			// mixer control released
			MIX.mode[MixerInd1] = MIX_CTRL;
			MIX.mode[MixerInd2] = MIX_CTRL;
		}
		else
		{
			MIX.mode[MixerInd1] = mode;
			MIX.mode[MixerInd2] = mode;
		}
	}
	else
	{
		MIX.mode[MixerInd1] = mode;
		MIX.mode[MixerInd2] = mode;
	}
}

static void MAIN_Set_RGB_mode(void)
{
	if (RGB.ModeOff)
		RGB.mode = RGB_MODE_OFF;
	else
		RGB.mode = RGB_MODE_ON;
}

static void MAIN_Set_FAB_mode(void)
{
    if (FAB.FlushingSuccessful)
    	FAB.mode  = FAB_CLOSE_FLAP_STOP_BLOWER; // blower not needed
    else
    	FAB.mode  = FAB_OPEN_FLAP_RUN_BLOWER; // flush exhaust pipe by blower
}

extern void MAIN_Set_LowIdleSpeed(void)
{
	if (PARA[ParRefInd[LOW_IDLE_OPTION__PARREFIND]].Value == 0L) // Option = off
		MAIN.LowIdleSpeed_Demand = FALSE;
	else if (MAIN.state < MAIN_SYSTEM_STOP)
		MAIN.LowIdleSpeed_Demand = FALSE;
	else if (!MAIN.ManualOperation)
	{
		if (PARA[ParRefInd[LOW_IDLE_OPTION__PARREFIND]].Value == 1L) // Option = oil temperature
			MAIN.LowIdleSpeed_Demand = ENG.LubeOilTemp_Available && (ENG.LubeOilTemp.Value < (DS16)PARA[ParRefInd[LOW_IDLE_RELEASE_OIL_TEMP__PARREFIND]].Value);
		else // Option = time OR Option = oil temp. + time
			MAIN.LowIdleSpeed_Demand = (PARA[ParRefInd[LOW_IDLE_RELEASE_TIME__PARREFIND]].Value > 0L);
	}
	// else MAIN.LowIdleSpeed_Demand is set via soft-button in ManualMode
}

static void MAIN_StateLogLine_record(void)
{
	t_MAIN_StateLogLine MSlogline;


	   MSlogline.TimeStamp         = GetSystemTime();
	   MSlogline.MainState         = MAIN.state;
//	   MSlogline.StopActualCode    = STOP.actualCode;
	   MSlogline.StopActualLevel   = STOP.actualLevel;
//	   MSlogline.StopActualIndex   = STOP.actualIndex;
//	   MSlogline.StopActualBitMask = STOP.actualBitMask;
//	   MSlogline.CounterL1E        = ELM.L1E.counter; //from mains SCM
//	   MSlogline.T1EkWhProduction  = ELM.T1E.counter.kWhProduction;	//rmi: produced kWh
	   MSlogline.LineIsValid       = TRUE;

		mainlog.MAIN_stateLog_pointer++;
		if ( mainlog.MAIN_stateLog_pointer >= MAIN_STATE_LOG_NUMBER_OF_LINES )
			mainlog.MAIN_stateLog_pointer = 0;

		mainlog.MAIN_StateLog[mainlog.MAIN_stateLog_pointer] = MSlogline;
		MAIN.NovUpdateRequired = TRUE;

} //MAIN_StateLogLine_record

t_MAIN_StateLogLine MS_GetLog (DU16 MSIndex)
{
	if (MSIndex < MAIN_STATE_LOG_NUMBER_OF_LINES)
		return mainlog.MAIN_StateLog[MSIndex];
	else
		return mainlog.MAIN_StateLog[0];
}

t_MAIN_CycleLogLine MSC_GetLog (DU16 MSIndex)
{
	if (MSIndex < MAIN_CYCLE_LOG_NUMBER_OF_LINES)
		return mainlog.MAIN_CycleLog[MSIndex];
	else
		return mainlog.MAIN_CycleLog[0];
}

static void MAIN_CycleLogLine_record(void)
{
	t_MAIN_CycleLogLine MSlogline;
	   MSlogline.TimeStamp         = GetSystemTime();
	   MSlogline.MainState         = MAIN.state;
	   MSlogline.StopActualLevel   = STOP.actualLevel;
	   MSlogline.T1EkWhProduction  = ELM.T1E.counter.kWhProduction;		//produced kWh
	   //MSlogline.ConsumptionTotalCounter = FUE.ConsumptionTotalCounter;	//engine fuel-consumption
	   MSlogline.TotalRunningTime  = ENG.TotalRunningTime.hours;		//engine running time
	   MSlogline.LineIsValid       = TRUE;

	   mainlog.MAIN_cycleLog_pointer++;

	   if ( mainlog.MAIN_cycleLog_pointer >= MAIN_CYCLE_LOG_NUMBER_OF_LINES )
	   	mainlog.MAIN_cycleLog_pointer = 0;

	   mainlog.MAIN_CycleLog[mainlog.MAIN_cycleLog_pointer] = MSlogline;
	   MAIN.NovUpdateRequired = TRUE;
} //MAIN_CycleLogLine_record

// transit function which is called at any state change
static void transit( STATE newState )
{
	
	if ( newState != myState )
	{
		if ((newState == GridParallelOperationLimitedLoad) AND (PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT2))
			newState = GridParallelOperationFullLoad;

		#ifdef DEBUG_MAIN
		PRINT2("\n MAIN transit old state : %s",MAIN_state_text(MAIN.state));
		#endif
		
		// call the Exit function for the old State
		if ( myState != 0 ) 
		{
			myState(SIG_EXIT);
		}
		
		// assign the new state function
		myState = newState;
		
		// call the ENTRY function for the new state
		// and record a line in the MS log
		if ( myState != 0 ) 
		{
			myState(SIG_ENTRY);
			MAIN_StateLogLine_record();
		}
		myStateCnt = 0;

		#ifdef DEBUG_MAIN
		PRINT2("\n MAIN transit new state : %s",MAIN_state_text(MAIN.state));
		#endif
	}
}

// main control loop called all 20ms for the actual state
void MAIN_control_20ms(void)
{
	static DBOOL StartDemand_Old = ON;

	//DTIMESTAMP now;
	
	// increment the timecounter for this state
	// by adding 20ms for the time since last call
	if (myStateCnt < ( MAX_DU32 - 1000 )) 
		myStateCnt = myStateCnt + 20; 

    // all 1 sec check, if there shall be made a MS logline 
    // if YES then do it
    if ( myStateCnt % 1000 == 0 )
    {
    	//now, the log-functionality is shifted to the 100ms-loop
	    //now = GetSystemTime();
		//if ( now.H % 3600L == 0 ) MAIN_CycleLogLine_record();
		   
		#ifdef DEBUG_MAIN
		{ PRINT3("\n MAIN state : %s myStateCnt %12uL",MAIN_state_text(MAIN.state), myStateCnt);}
		#endif
    }

    // INTERNAL STATUS

    // manual mode
    MAIN.ManualOperation = STOP_is_Set(STOPCONDITION_30001);
    // test mode
    MAIN.TestMode = STOP_is_Set(STOPCONDITION_20099);
    // regular stop
    MAIN.RegularStop = !(STOP.actualBitMask & STOP_BIT_MASK_BIT1);
    // block start
    MAIN.BlockStart  = !(STOP.actualBitMask & STOP_BITMASK_BLOCK_START);

    // digital output to indicate "automatic operation"
    MAIN.DO_AutomaticOperation = !STOP_is_Set(STOPCONDITION_30001);

    // fast stop
	if ( ( (DI_FUNCT[FAST_STOP].Assigned == ASSIGNED) && (MAIN.DI_FastStop) )
		|| ( (MBA.WriteConfigurationDigital & MBA_CONFIG_FAST_STOP) && (MBA.bFastStop) )
	   )
    {
	    STOP_Set(STOPCONDITION_20001);
	    STOP_Tripped[STOPCONDITION_20001] = TRUE;
	    // blocked as long as input is set
    }
    else // fast stop is neither set via digital input nor via modbus -> can be acknowledged
        STOP_Tripped[STOPCONDITION_20001] = FALSE;
    
    
    // automatic operation is set via modbus
    if (MBA.WriteConfigurationDigital & MBA_CONFIG_AUTOMATIC_OPERATION)
    {
    	if (MBA.bAutomaticOperation)
    	{
           if (STOP_is_Set(STOPCONDITION_30001)) // = manual mode
           {
             // auto mode demanded
             STOP_Tripped[STOPCONDITION_30001] = FALSE;  // enable acknowledging
             STOP_Clear(STOPCONDITION_30001);
             // set GOV and AVR to auto mode
             TUR.GOVManual = FALSE;
             GEN.AVRManual = FALSE;
           }
    	}
       else
       {
           if (( MAIN.state != MAIN_GRID_PARALLEL_FULL_LOAD )
           	 && (MAIN.state != MAIN_GRID_PARALLEL_LIMITED_LOAD))
             {
               // manual mode demanded
               STOP_Set(STOPCONDITION_30001);
               STOP_Tripped[STOPCONDITION_30001] = TRUE; // block acknowledge function
             }
       }
    }
    // automatic operation is set via digital input
    else if (DI_FUNCT[AUTOMATIC_OPERATION].Assigned == ASSIGNED)
    {
    	if (MAIN.DI_AutomaticOperation)
    	{
           if (STOP_is_Set(STOPCONDITION_30001)) // = manual mode
           {
             // auto mode demanded
             STOP_Tripped[STOPCONDITION_30001] = FALSE;  // enable acknowledging
             STOP_Clear(STOPCONDITION_30001);
             // set GOV and AVR to auto mode
             TUR.GOVManual = FALSE;
             GEN.AVRManual = FALSE;
           }
    	}
       else
       {
           if (( MAIN.state != MAIN_GRID_PARALLEL_FULL_LOAD )
           	 && (MAIN.state != MAIN_GRID_PARALLEL_LIMITED_LOAD))
             {
               // manual mode demanded
               STOP_Set(STOPCONDITION_30001);
               STOP_Tripped[STOPCONDITION_30001] = TRUE; // block acknowledge function
             }
       }
    }
     
    // take care of acknowledge now in 100ms-Task (clear SC-Loop), rmiSPT

    if (PMS.EngineIDConfigured[ARC.nEngineId - 1])
    {
      if (PMS.AutoStartStop && !STOP_is_Set(STOPCONDITION_30001))
      {
    	  // PMS auto start/stop enabled and not in manual operation
      	  MAIN.StartdemandRemote = PMS.StartDemand;
      }
      else
      {
		// check if modbus startdemand remote is set
		if (MBA.WriteConfigurationDigital & MBA_CONFIG_START_DEMAND)
		{
			if (PARA[ParRefInd[MAIN_STARTDEMAND_ON_RISING_EDGE__PARREFIND]].Value) // start demand on rising edge
			{
				// reset start demand
				if ( (STOP.actualLevel < 3) || MAIN.RegularStop )
					MAIN.StartdemandRemote = OFF;
				else
				{
					if (!MAIN.StartdemandRemote)
					{
						// switch on at rising edge
						if (!StartDemand_Old && MBA.bStartDemand)
							MAIN.StartdemandRemote = ON;
					}
					else
					{
						// switch off at LOW-Signal
						if (!MBA.bStartDemand)
							MAIN.StartdemandRemote = OFF;
					}
				}

				StartDemand_Old = MBA.bStartDemand;
			}
			else // steady signal
			{
				MAIN.StartdemandRemote = MBA.bStartDemand;
			}
		}
		// check if digital input startdemand remote is set
		else if (DI_FUNCT[START_ENGINE].Assigned == ASSIGNED)
		{
			if (PARA[ParRefInd[MAIN_STARTDEMAND_ON_RISING_EDGE__PARREFIND]].Value) // start demand on rising edge
			{
				// reset start demand
				if ( (STOP.actualLevel < 3) || MAIN.RegularStop )
					MAIN.StartdemandRemote = OFF;
				else
				{
					if (!MAIN.StartdemandRemote)
					{
						// switch on at rising edge
						if (!StartDemand_Old && MAIN.DI_StartdemandRemote)
							MAIN.StartdemandRemote = ON;
					}
					else
					{
						// switch off at LOW-Signal
						if (!MAIN.DI_StartdemandRemote)
							MAIN.StartdemandRemote = OFF;
					}
				}

				StartDemand_Old = MAIN.DI_StartdemandRemote;
			}
			else // steady signal
			{
				// input is used, copy demand to internal variable
				MAIN.StartdemandRemote = MAIN.DI_StartdemandRemote;
			}
		}
      }

      // set/reset local start demand if AUTO
      if (!STOP_is_Set(STOPCONDITION_30001))
      	MAIN.StartdemandLocal = MAIN.StartdemandRemote;

    }
    else if (ELM.MainsFailure)
    {
      MAIN.StartdemandRemote = TRUE;
    }
    // check if modbus startdemand remote is set
    else if (MBA.WriteConfigurationDigital & MBA_CONFIG_START_DEMAND)
    {
    	if (PARA[ParRefInd[MAIN_STARTDEMAND_ON_RISING_EDGE__PARREFIND]].Value) // start demand on rising edge
    	{
    	    // reset start demand
    	    if ( (STOP.actualLevel < 3) || MAIN.RegularStop )
    	    	MAIN.StartdemandRemote = OFF;
    	    else
    	    {
				if (!MAIN.StartdemandRemote)
				{
					// switch on at rising edge
					if (!StartDemand_Old && MBA.bStartDemand)
						MAIN.StartdemandRemote = ON;
				}
				else
				{
					// switch off at LOW-Signal
					if (!MBA.bStartDemand)
						MAIN.StartdemandRemote = OFF;
				}
    	    }

    		StartDemand_Old = MBA.bStartDemand;
    	}
    	else // steady signal
    	{
    		MAIN.StartdemandRemote = MBA.bStartDemand;
    	}

      // set/reset local start demand if AUTO
      if (!STOP_is_Set(STOPCONDITION_30001))
      	MAIN.StartdemandLocal = MAIN.StartdemandRemote;
    }
    // check if digital input startdemand remote is set
    else if (DI_FUNCT[START_ENGINE].Assigned == ASSIGNED)
    {
    	if (PARA[ParRefInd[MAIN_STARTDEMAND_ON_RISING_EDGE__PARREFIND]].Value) // start demand on rising edge
    	{
    	    // reset start demand
    	    if ( (STOP.actualLevel < 3) || MAIN.RegularStop )
    	    	MAIN.StartdemandRemote = OFF;
    	    else
    	    {
				if (!MAIN.StartdemandRemote)
				{
					// switch on at rising edge
					if (!StartDemand_Old && MAIN.DI_StartdemandRemote)
						MAIN.StartdemandRemote = ON;
				}
				else
				{
					// switch off at LOW-Signal
					if (!MAIN.DI_StartdemandRemote)
						MAIN.StartdemandRemote = OFF;
				}
    	    }

    		StartDemand_Old = MAIN.DI_StartdemandRemote;
    	}
    	else // steady signal
    	{
    		// input is used, copy demand to internal variable
    		MAIN.StartdemandRemote = MAIN.DI_StartdemandRemote;
    	}

      // set/reset local start demand if AUTO
      if (!STOP_is_Set(STOPCONDITION_30001))
      	MAIN.StartdemandLocal = MAIN.StartdemandRemote;
    }
    else
        // startdemand remote is not assigned
        MAIN.StartdemandRemote = FALSE;
    
    // reset start demand from soft button
    if ( (STOP.actualLevel < 3) || !(STOP.actualBitMask & 0x0001) )
    	MAIN.StartdemandLocal = FALSE;
    
    // StartdemandRemoteAndAuto = startdemand is set and we are in AUTO mode    
    MAIN.StartdemandRemoteAndAuto = ( MAIN.StartdemandRemote && !STOP_is_Set(STOPCONDITION_30001) );
    
    // take care of manual mode or start button
    MAIN.startdemand = (MAIN.StartdemandRemoteAndAuto || MAIN.StartdemandLocal) AND MBA.bEZAStartRelease AND MBA.bEVUStartRelease;

    if ((!MAIN.DI_DigitalAutoDemand) && (DI_FUNCT[AUTO_DEMAND].Assigned == ASSIGNED))
    {
      STOP_Set( STOPCONDITION_50002 ); // Running blocked by switch AUTO-OFF on switchboard panel
      STOP_Tripped[STOPCONDITION_50002] = TRUE; // cannot be acknowledged         
    }  
    else
    {
      STOP_Tripped[STOPCONDITION_50002] = FALSE;
      STOP_Clear( STOPCONDITION_50002 );
    } 
          
    if (!MAIN.startdemand)  
    {
      STOP_Set( STOPCONDITION_50100 ); // Demand removed by user -> Show text
      STOP_Tripped[STOPCONDITION_50100] = TRUE;  // cannot be acknowledged
    }
    else
    {
      STOP_Tripped[STOPCONDITION_50100] = FALSE; // can be acknowledged again 
      STOP_Clear( STOPCONDITION_50100 );
    }

    if (ft_LoadBusy OR ft_SaveBusy)
        MAIN.subState = MAIN_SUB_FT_ACTIVE;
    else if (MAIN.Simulation)
    	MAIN.subState = MAIN_SUB_SIMULATION;
	// GridProtectionTest
    else if (ELM.GPTstate == ELM_GPT_STATE_ACTIVATED)
		MAIN.subState = MAIN_SUB_GPT_ACTIVATED;
	else if (ELM.GPTstate == ELM_GPT_STATE_RUNNING)
		MAIN.subState = MAIN_SUB_GPT_RUNNING;
	else if (ELM.GPTstate == ELM_GPT_STATE_TRIPPED)
		MAIN.subState = MAIN_SUB_GPT_TRIPPED;
	// End: GridProtectionTest
	else if ( STOP.actualLevel < 2)
    	MAIN.subState = SUBSTATE_NO_TEXT;
	else if (STOP_is_Set(STOPCONDITION_50010) OR STOP_is_Set(STOPCONDITION_50011))
		MAIN.subState = MAIN_SUB_EZA_STOP;
	else if (STOP_is_Set(STOPCONDITION_60010) OR STOP_is_Set(STOPCONDITION_60011))
		MAIN.subState = MAIN_SUB_EZA_LOADREDUCTION;
//    else if (OIL.state == OIL_PUMP_ON)
//      MAIN.subState = MAIN_SUB_OILPUMP_ON;
    else if (ENG.state == ENG_COOLDOWN_RUN)
      MAIN.subState = MAIN_SUB_ENGINE_RUNNING;
//    else if (ENG.state == ENG_COOLDOWN_NOT_RUN)
//      MAIN.subState = MAIN_SUB_CRANK_PAUSE;
    else if (MIX.Config)
      MAIN.subState = MAIN_SUB_MIX_CONFIG;
    else if (SCR.Inj.ManualMode)
        MAIN.subState = MAIN_SUB_SCR_MANUAL;
    else if (ENG.state == ENG_CRANK_PAUSE)
      MAIN.subState = MAIN_SUB_CRANK_PAUSE;
    else if (ENG.state == ENG_CRANK)
      MAIN.subState = MAIN_SUB_CRANKING;
    else if (ENG.state == ENG_START_CRANKING)
      MAIN.subState = MAIN_SUB_CRANKING;
    else if (ENG.state == ENG_FLUSHING)
      MAIN.subState = MAIN_SUB_FLUSHING;
    else if (FAB.state == FAB_FLAP_OPEN_BLOWER_RUNNING)
      MAIN.subState = MAIN_SUB_FLUSHING_EXHAUST;
    else if (ENG.state == ENG_STOPPING)
      MAIN.subState = MAIN_SUB_STOPPING;
    else if (PMS.EngineIDConfigured[ARC.nEngineId-1] && !PMS.AutoStartStop)
      MAIN.subState = MAIN_SUB_PMS_MANUAL_START_STOP;
//    else if (ENG.state == ENG_PREGLOW)
//      MAIN.subState = MAIN_SUB_PREGLOW;
//    else if (EXH.state == EXH_CLOSING_FLAP)
//      MAIN.subState = MAIN_SUB_CLOSING_EXH_FLAPS;
//    else if (EXH.state == EXH_OPENING_FLAP)
//      MAIN.subState = MAIN_SUB_OPENING_EXH_FLAPS;
//    else if (AIR.state == AIR_CLOSING_FLAPS)
//      MAIN.subState = MAIN_SUB_CLOSING_AIR_FLAPS;
//    else if (AIR.state == AIR_OPENING_FLAPS)
//      MAIN.subState = MAIN_SUB_OPENING_AIR_FLAPS;
//    else if (STH.state == STH_SWITCH_TO_ELECTRIC)
//      MAIN.subState = MAIN_SUB_SWITCH_TO_ELECTRIC;
//    else if (STH.state == STH_SWITCH_TO_WATER)
//      MAIN.subState = MAIN_SUB_SWITCH_TO_WATER;
//    else if (RSP.state == RSP_PUMP_ON)
//      MAIN.subState = MAIN_SUB_REFILLING_POIL;
//    else if (RSD.state == RSD_PUMP_ON)
//      MAIN.subState = MAIN_SUB_REFILLING_DIESEL;
//    else if (REG.state == REG_CLOSING_VALVES)
//      MAIN.subState = MAIN_SUB_CLOSING_VALVES;
//    else if (FUE.state == FUE_PAUSE_CHANGE_TO_DIESEL)
//      MAIN.subState = MAIN_SUB_MOVING_TO_DIESEL;
//    else if (FUE.state == FUE_MOVING_FLOW_VALVE_TO_DIESEL)
//      MAIN.subState = MAIN_SUB_MOVING_TO_DIESEL;
//    else if (FUE.state == FUE_MOVING_RETURN_VALVE_TO_DIESEL)
//      MAIN.subState = MAIN_SUB_MOVING_TO_DIESEL;
//    else if (FUE.state == FUE_MOVING_FLOW_VALVE_TO_POIL)
//      MAIN.subState = MAIN_SUB_MOVING_TO_POIL;
//    else if (FUE.state == FUE_MOVING_RETURN_VALVE_TO_POIL)
//      MAIN.subState = MAIN_SUB_MOVING_TO_POIL;
//    else if (FUE.state == FUE_PAUSE_CHANGE_TO_POIL)
//      MAIN.subState = MAIN_SUB_MOVING_TO_POIL;
    else if (MAIN.regState == MAIN_GRID_PARALLEL_SOFT_DISCONNECT_T1E)
      MAIN.subState = MAIN_SUB_DELOAD;
	else if ( MAIN_ISLAND_PARALLEL_ACTIVE )
	{
		if (!(STOP.actualBitMask & 0x0001) OR MAIN.StopInIsland OR !MAIN.startdemand)
			MAIN.subState = MAIN_SUB_ISLAND_PARALLEL_DELOADING;
		else
			MAIN.subState = MAIN_SUB_ISLAND_PARALLEL_OPERATION;
	}
    else if ( (PLS.state == PLS_INT_PUMP_ON) || (PLS.state == PLS_INT_PUMP_ON_PRESS) )
	  MAIN.subState = MAIN_SUB_INT_PRELUBRICATION;
	else if ( (PLS.state == PLS_START_PUMP_ON) || (PLS.state == PLS_START_PUMP_ON_PRESS) )
	  MAIN.subState = MAIN_SUB_START_PRELUBRICATION;
	else if ( (PLS.state == PLS_POST_PUMP_ON) || (PLS.state == PLS_POST_PUMP_ON_PRESS) )
	  MAIN.subState = MAIN_SUB_POST_LUBRICATION;
    else if (PowerReductionActive)
      MAIN.subState = MAIN_SUB_POWER_REDUCTION;
    else if ( TUR.state == TUR_REGULATE_RPM )
      MAIN.subState = MAIN_SUB_ADJUSTING_FREQUENCY;
    else if ( ( GEN.reg.state != GEN_REGSTATE_UISLAND_DONE )
    		&& (MAIN.state == MAIN_TRANSFORMER_DISCONNECTED)
    		&& (NOT GEN_REG_OFF) // NO_SCM
    		&& (GEN.mode == GEN_MODE_ON) )
	  MAIN.subState = MAIN_SUB_ADJUSTING_VOLTAGE;
	else if (MAIN.regState == MAIN_GRID_PARALLEL_ADJUST_POWER)
	  MAIN.subState = MAIN_SUB_ADJUSTING_POWER; 
	/*
	else if ( (MAIN.regState == MAIN_GRID_PARALLEL_ADJUST_BLINDPOWER)
	 	 	 && (PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value > NO_SCM) ) // NO_SCM
	  MAIN.subState = MAIN_SUB_ADJUSTING_VAR;
	*/
	else MAIN.subState = SUBSTATE_NO_TEXT;
  
	// call the actual state-function
	if (myState != 0) myState(SIG_DO);  		
}

	 
// SetPointHasChanged
// evaluate if new setpoint is significantly different from old setpoint
static DBOOL SetpointHasChanged(void)
{
	DS32  NewSetpoint;
	DBOOL Result;
	//NewSetpoint = MAIN_actual_realpower_setpoint();
	if (TLB.Regulation_is_ON AND (TLB.mode == TLB_POWER))
		NewSetpoint = MAIN_actual_realpower_setpoint() + TUR.OffsetForTurboBypassControl;
	else
		NewSetpoint = MAIN_actual_realpower_setpoint();
	
	if (TUR.Pset > NewSetpoint + MAIN_POWER_CHANGE_DEADBAND)
	  Result = TRUE;
	else if (TUR.Pset < NewSetpoint - MAIN_POWER_CHANGE_DEADBAND)
	  Result = TRUE;
	else
	  Result = FALSE; 
	return (Result);
}


// rmiCYCL #define SystemTime1msDefault 4294967		// rmi: copied from systemtime.c
void MAIN_control_100ms(void)
{
   static	uint32_t	last_log_seconds;			// save seconds of last log, rmiCYCL
// rmiCYCL   DS16 	MSec;

	// 100ms handler
	DTIMESTAMP now;
	now = GetSystemTime();

/*  relocate to 1000msTask
	// Evaluate if power setpoint has changed
	MAIN.PowerSetpointHasChanged = SetpointHasChanged(); 
*/
	// StateLogLine every hour in a special cycle-log
	if ( now.H % 3600L == 0 ) 
	{
		/* rmiCYCL
   		MSec = now.L / SystemTime1msDefault;
		if (MSec < 100)	
				MAIN_CycleLogLine_record();  
		*/
		if (now.H != last_log_seconds)			// rmiCYCL
		{										// rmiCYCL
			MAIN_CycleLogLine_record();			// rmiCYCL
		    last_log_seconds = now.H;			// rmiCYCL
		}										// rmiCYCL
			
	}
	
    // acknowledge from 20ms-Task shifted to this position, rmiSPT
    // acknowledge on rising edge of MAIN.DI_Acknowledge
    if ((MAIN.state >= MAIN_EMERGENCY_STOP)
    	&& (DI_FUNCT[ACKNOWLEDGE].Assigned == ASSIGNED)
    	&& (MAIN.DI_Acknowledge)
    	&& (!MAIN.AcknowledgeActive))
    {
	 	STOP_ClearAllLevels();			// rmiSPT
    	MAIN.AcknowledgeActive = TRUE;
    	HZM.ResetErrorStateDemanded = TRUE; // request to reset ignition error state
    	mic4.AcknowledgeAlarms = TRUE;
    }

    // check if digital input is not set anymore
    if (!MAIN.DI_Acknowledge)
    {
    	MAIN.AcknowledgeActive = FALSE;
    }
	
    // set manual adjustment possibility for mixer position
    if (ENG.state == ENG_START_CRANKING)
    	MIX.AdjustmentDuringStart_Possible = TRUE;
    // reset manual adjustment possibility for mixer position
	if ( ( MAIN.state > MAIN_TRANSFORMER_DISCONNECTED ) // synchronization and further
		|| (STOP.actualLevel < 3) 						// engine not allowed to run
		|| !(STOP.actualBitMask & 0x0001)				// regular stop
		|| !MAIN.startdemand	)						// engine switched off
	{
		MIX.AdjustmentDuringStart_Possible = FALSE;
		MIX.AdjustmentDuringStart_Activated = FALSE;
	}

	// ExternalSynchronization if NO_SCM
	MAIN.ExternalSynchronization = ( HVS_SYNC_OFF // Disabled or NO_SCM
			&& TUR.SpeedUpDownAssigned );

	// Update internal variable for "low idle speed"
	if (MAIN.state < MAIN_ACCELERATION)
		MAIN_Set_LowIdleSpeed();
	// Update DO for ext. GOV to run at low idle speed
	MAIN.DO_LowIdleSpeed = MAIN.LowIdleSpeed_Demand;

} // end MAIN_control_100ms

// main control loop called all 1000ms
void MAIN_control_1000ms(void)
{
	// operation blocked
	if (PARA[ParRefInd[QUICKSTOP_ACTIVE__PARREFIND]].Value < 2L)
		STOP_Tripped[STOPCONDITION_50001] = FALSE;

	// Evaluate if power setpoint has changed
	MAIN.PowerSetpointHasChanged = SetpointHasChanged();

	// save in file-system
	if (MAIN.NovUpdateRequired)
	{
		MAIN.NovUpdateRequired = FALSE;
		NOV_UpdateRequest[NOV_UPDATE_MAINLOG] = TRUE;
	}

	// set Deload timeout limit
	// nominal power/power ramp + 20%
	if (PARA[ParRefInd[POWER_RAMP_DOWN__PARREFIND]].Value >= 100L)
		MAIN.DeloadTimeout = (PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value * 12 / (PARA[ParRefInd[POWER_RAMP_DOWN__PARREFIND]].Value/100L));
	else
		MAIN.DeloadTimeout = MAIN_DELOAD_TIMEOUT;

	// calculate acceleration timeout
	{
		DU32 Timeout;

		if (PARA[ParRefInd[SPEED_RAMP_UP__PARREFIND]].Value/1000L > 0L)
			Timeout = (TUR.NominalSpeed - PARA[ParRefInd[STRT_VALUE_SPEED_RAMP__PARREFIND]].Value*1000L) / (PARA[ParRefInd[SPEED_RAMP_UP__PARREFIND]].Value/1000L);
		else
			Timeout = MAIN_ACCELERATION_STATE_TIMEOUT;

		// add 20% reserve
		Timeout += (Timeout/5);

		// limitation to minimum value
		if (Timeout < MAIN_ACCELERATION_STATE_TIMEOUT)
			Timeout = MAIN_ACCELERATION_STATE_TIMEOUT;

		MAIN.AccelerationTimeout = Timeout;
	}

} // end MAIN_control_1000ms


static DTIMESTAMP ReadTime(void)
{
   // used for reading a DTIMESTAMP from Bing
   // to replace the faulty ReadDTimeFromBing() from bing_bang.c	
   RTCLOCK Temp;
   DTIMESTAMP DTime;
   Temp.sDate     = ReadInt8FromBing();
   Temp.sMonth    = ReadInt8FromBing();
   Temp.sYear     = ReadInt8FromBing() + 2000;
   Temp.sHour     = ReadInt8FromBing();
   Temp.sMin      = ReadInt8FromBing();
   Temp.sSec      = ReadInt8FromBing();
   
   RTCLOCKToDTime_DM4(Temp, &DTime);

   DTime.L = (DU16)( (ReadInt8FromBing() << 8) + ReadInt8FromBing() );

   return (DTime);
}

static short SetTime( DU8 client, DU32 length )
{
   DTIMESTAMP DTime;
   if (client);

   // If length = 8 then get time from Bing telegram
   if ( length == 8 )
      DTime = ReadTime();
   
   // Set system time and store in time in NOVRam
   SetSystemTimeAndNOWRAM(DTime);
   // Return answer to request
   AddLenToBang(4);  // Telegram length
   AddInt16ToBang(12); // Acknoledge service Id 9
   AddInt16ToBang(0); // Service request accepted

   return 0;
}


void ResetMainlog(void)
{

}

// initialisation function of this module
void MAIN_control_init(void)
{
	#ifdef DEBUG_MAIN
	PRINT1("\n MAIN_control_init");
	#endif

    MAIN.DI_DigitalAutoDemand                           = FALSE; 
    MAIN.startdemand                                    = FALSE;    
    MAIN.StartdemandLocal                               = FALSE;    
    // Wozu???
   	MAIN.T1EisSynchron.Signal                           = FALSE;
   	MAIN.L1EisSynchron.Signal                           = FALSE; 	// rmiMB
   	
   	MAIN.DO_IslandOperation                             = FALSE;
   	MAIN.DO_Loadsharing                                 = FALSE;
   	MAIN.DO_ReadyForOperation                           = FALSE;
   	MAIN.EngineRunningNominalDelayed					= FALSE;
   	
   	MAIN.WishToCloseGCB                                 = FALSE;
   	MAIN.ExternalSynchronization						= FALSE;
   	
#if (CLIENT_VERSION == IET)
   	MAIN.ManualPowerSetpoint = TUR_setpoints_NOV.PowerGridParallel;
   	MAIN.ManualPowerSetpointInitialized = TRUE;
#endif
       MAIN.T1EisSynchron.Name_English                  = "GENERATOR synchron to Line"; 	
       MAIN.T1EisSynchron.TextIfFALSE_English           = "NOT SYNCHRON";   	
       MAIN.T1EisSynchron.TextIfTRUE_English            = "IS SYNCHRON";  	
       MAIN.T1EisSynchron.Name_Native                   = "Generator synkron med linje";  	
       MAIN.T1EisSynchron.TextIfFALSE_Native            = "IKKE SYNKRON";   	
       MAIN.T1EisSynchron.TextIfTRUE_Native             = "SYNKRON";   	
       MAIN.T1EisSynchron.colorIfFALSE                  = cl_LED_RED;   	
       MAIN.T1EisSynchron.colorIfTRUE                   = cl_LED_GREEN;   	
     // register date and time setting as bingbang service
     if ( BbRegisterServiceHandler( (ServiceHandler_t)SetTime, 0x0C ) != 0 )
      PRINT1("\nSet Time not added to Bing Bang handler!");
	 if (BbRegisterServiceHandler( (ServiceHandler_t)FileLogInfo, 0x0D) != 0)
      	PRINT1("\nLogInfo not supported in Bing Bang handler!");
	 if (BbRegisterServiceHandler( (ServiceHandler_t)FileLogUpdate, 0x0E) != 0)
      	PRINT1("\nLogUpdate not supported in Bing Bang handler!");
    
    // acknowledge all faults
	// after booting to avoid ghost stop conditions (left over from last software) being set
	STOP_ClearAllLevels();			// rmiSPT
    // always start with emergency stop after download or power up
    STOP_Set(STOPCONDITION_10201);

    // No Activation necessary for test software
    if (SW_NUMBER >= 900)
    {
    	NOV.SoftwareActivated = TRUE;
    }

    // Check if software is already activated
    if (!NOV.SoftwareActivated)
    {
    	STOP_Set(STOPCONDITION_0);
    	STOP_Tripped[STOPCONDITION_0] = TRUE;
    }

    MAIN.SC_IndexToStopTrending = STOP_Get_Index((DU32)PARA[ParRefInd[TRE_SC_INDEX_TO_STOP_TRENDING__PARREFIND]].Value);

    MAIN.Simulation = OFF;
    MAIN.MaintenanceMode = OFF;
	 
	MAIN.state            = MAIN_BOOT;
	myState = 0;
	// set actual state to System Start
	transit(SystemStart); 
}



///////////////////// MAIN State functions definitions 

static void	SystemStart(const DU8 sig)
{// Stoplevel 0, all outputs off
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_BLOCK;           // fan off, flaps closed
			AKR.mode      = AKR_MODE_OFF;
			DK.mode       = DK_MODE_OFF;         // throttle controller switched off
			ENG.mode      = ENG_OFF;             // engine shut down
			//EXH.mode      = EXH_CLOSE;           // close flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
			FAB.mode	  = FAB_BLOCK;			 // flushing air blower blocked
			GAS.mode      = GAS_MODE_BLOCK;      // gas valves blocked
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			GEN.mode      = GEN_MODE_OFF;        // AVR off
			GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
			HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_BLOCK;           // emergency cooler off
			OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_BLOCK;           // no prelubrication
			//PFH.mode      = PFH_BLOCK;           // plant oil filter heater off
			WAT.mode      = WAT_BLOCK;           // water circuit system off
            //RSD.mode      = RSD_BLOCK;           // Refilling off 
			//RSP.mode      = RSP_BLOCK;           // Refilling off
			SAF.mode      = SAF_TRIP;            // trip safety chain
			SCR.mode	  = SCR_BLOCK;			 // catalyst off
			//STH.mode      = STH_BLOCK;           // heating of poil blocked
			THR.mode	  = THR_BLOCK;			 // thermoreactor blocked
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_SHUTDOWN;        // governor stopped
			COM.mode      = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			MIX.mode[MixerInd1] = MIX_BLOCK;
			MIX.mode[MixerInd2] = MIX_BLOCK;
			PMS.mode      = PMS_BLOCK;
			
			PID.mode[PID_1] = PID_BLOCK;
			PID.mode[PID_2] = PID_BLOCK;
			PID.mode[PID_3] = PID_BLOCK;
			PID.mode[PID_4] = PID_BLOCK;
			PID.mode[PID_5] = PID_BLOCK;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_OFF;
				
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E
			
			// define actual MAIN.state
			MAIN.state                = MAIN_SYSTEM_START;
			MAIN.subState             = SUBSTATE_NO_TEXT;
			MAIN.regState             = MAIN_GRID_PARALLEL_NORMAL_OPERATION;
			PowerReductionActive      = FALSE;
			
			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
	        MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
			 
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
					
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			if (STOP.actualLevel == 1)
			{ 
				transit(EmergencyStop); 
				break;
			}
			
		    if (STOP.actualLevel > 1)
		    { 
		    	transit(RearmSafetyChain);
		    	break;
		    }
		    
		break; // end of regular block SIG_DO
	}
	
}

static void	UndefinedBreakers(const DU8 sig)
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered

			// define control for all other components
			AIR.mode      = AIR_BLOCK;           // fan off, flaps closed
			AKR.mode      = AKR_MODE_OFF;
            DK.mode       = DK_MODE_OFF;         // throttle controller switched off
            ENG.mode      = ENG_OFF;             // engine shut down
			//EXH.mode      = EXH_CLOSE;           // close flaps
            //FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            GAS.mode      = GAS_MODE_BLOCK;      // gas valves blocked
			GBV.mode      = GBV_MODE_BLOCK;
            GEN.mode      = GEN_MODE_OFF;        // AVR off
            GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
            HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_BLOCK;           // emergency cooler off
			OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_INTERVAL_REQ;    // interval prelubrication
			//PFH.mode      = PFH_BLOCK;           // plant oil filter heater off
			WAT.mode      = WAT_BLOCK;           // water circuit system off
			//RSD.mode      = RSD_BLOCK;           // Refilling off 
			//RSP.mode      = RSP_BLOCK;           // Refilling off
			SAF.mode      = SAF_TRIP;            // trip safety chain
			SCR.mode	  = SCR_BLOCK;			 // catalyst off
            //STH.mode      = STH_BLOCK;           // heating of poil blocked
			THR.mode	  = THR_BLOCK;			 // thermoreactor blocked
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_SHUTDOWN;        // governor stopped
			COM.mode      = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			MIX.mode[MixerInd1] = MIX_BLOCK;
			MIX.mode[MixerInd2] = MIX_BLOCK;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_OFF;
			
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E

			// define actual MAIN.state
			MAIN.state                = MAIN_UNDEFINED_BREAKER_POS;
			MAIN.subState             = SUBSTATE_NO_TEXT;
			MAIN.regState             = MAIN_GRID_PARALLEL_NORMAL_OPERATION;
			PowerReductionActive      = FALSE;
			
			// set state outputs
	        MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic
			if (PMS.WeAreMainsDelomatic)
			{
				switch (PMS.CBPOS_OperationMode)
				{
					case PMS_CBPOS_BLACK_OPERATION:
						// black operation
						transit(BlackOperation);
					break;

					case PMS_CBPOS_MAINS_OPERATION:
						// mains operation
						transit(MainsOperation);
					break;

					case PMS_CBPOS_ISLAND_OPERATION:
						// island operation
						transit(IslandOperation);
					break;

					case PMS_CBPOS_PARALLEL_OPERATION:
						// parallel operation
						transit(GridParallelOperationFullLoad);
					break;

					default:
					break;
				}
				break;
			}
			else // we are not the mains delomatic anymore !!!
			{ 
				transit(FastBraking);
			}

		break; // end of regular block SIG_DO
	}
}

static void	BlackOperation(const DU8 sig)
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered

			// define control for all other components
			AIR.mode      = AIR_BLOCK;           // fan off, flaps closed
			AKR.mode      = AKR_MODE_OFF;
            DK.mode       = DK_MODE_OFF;         // throttle controller switched off
            ENG.mode      = ENG_OFF;             // engine shut down
			//EXH.mode      = EXH_CLOSE;           // close flaps
            //FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            GAS.mode      = GAS_MODE_BLOCK;      // gas valves blocked
			GBV.mode      = GBV_MODE_BLOCK;
            GEN.mode      = GEN_MODE_OFF;        // AVR off
            GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
            HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_BLOCK;           // emergency cooler off
			OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_INTERVAL_REQ;    // interval prelubrication
			//PFH.mode      = PFH_BLOCK;           // plant oil filter heater off
			WAT.mode      = WAT_BLOCK;           // water circuit system off
			//RSD.mode      = RSD_BLOCK;           // Refilling off 
			//RSP.mode      = RSP_BLOCK;           // Refilling off
			SAF.mode      = SAF_TRIP;            // trip safety chain
			SCR.mode	  = SCR_BLOCK;			 // catalyst off
            //STH.mode      = STH_BLOCK;           // heating of poil blocked
			THR.mode	  = THR_BLOCK;			 // thermoreactor blocked
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_SHUTDOWN;        // governor stopped
			COM.mode      = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			MIX.mode[MixerInd1] = MIX_BLOCK;
			MIX.mode[MixerInd2] = MIX_BLOCK;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_OFF;
			
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E

			// define actual MAIN.state
			MAIN.state                = MAIN_BLACK_OPERATION;
			MAIN.subState             = SUBSTATE_NO_TEXT;
			MAIN.regState             = MAIN_GRID_PARALLEL_NORMAL_OPERATION;
			PowerReductionActive      = FALSE;
			
			// set state outputs
	        MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;

		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				switch (PMS.CBPOS_OperationMode)
				{
					case PMS_CBPOS_UNDEFINED:
						// undefined breaker position
						transit(UndefinedBreakers);
					break;

					case PMS_CBPOS_MAINS_OPERATION:
						// mains operation
						transit(MainsOperation);
					break;

					case PMS_CBPOS_ISLAND_OPERATION:
						// island operation
						transit(IslandOperation);
					break;

					case PMS_CBPOS_PARALLEL_OPERATION:
						// parallel operation
						transit(GridParallelOperationFullLoad);
					break;

					default:
					break;
				}
				break;
			}
			else // we are not the mains delomatic anymore !!!
			{ 
				transit(FastBraking);
			}

		break; // end of regular block SIG_DO
	}
}

static void	MainsOperation(const DU8 sig)
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered

			// define control for all other components
			AIR.mode      = AIR_BLOCK;           // fan off, flaps closed
			AKR.mode      = AKR_MODE_OFF;
            DK.mode       = DK_MODE_OFF;         // throttle controller switched off
            ENG.mode      = ENG_OFF;             // engine shut down
			//EXH.mode      = EXH_CLOSE;           // close flaps
            //FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            GAS.mode      = GAS_MODE_BLOCK;      // gas valves blocked
			GBV.mode      = GBV_MODE_BLOCK;
            GEN.mode      = GEN_MODE_OFF;        // AVR off
            GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
            HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_BLOCK;           // emergency cooler off
			OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_BLOCK;           // no prelubrication
			//PFH.mode      = PFH_BLOCK;           // plant oil filter heater off
			WAT.mode      = WAT_BLOCK;           // water circuit system off
			//RSD.mode      = RSD_BLOCK;           // Refilling off 
			//RSP.mode      = RSP_BLOCK;           // Refilling off
			SAF.mode      = SAF_TRIP;            // trip safety chain
			SCR.mode	  = SCR_BLOCK;			 // catalyst off
            //STH.mode      = STH_BLOCK;           // heating of poil blocked
			THR.mode	  = THR_BLOCK;			 // thermoreactor blocked
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_SHUTDOWN;        // governor stopped
			COM.mode      = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			MIX.mode[MixerInd1] = MIX_BLOCK;
			MIX.mode[MixerInd2] = MIX_BLOCK;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_OFF;
			
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E

			// define actual MAIN.state
			MAIN.state                = MAIN_MAINS_OPERATION;
			MAIN.subState             = SUBSTATE_NO_TEXT;
			MAIN.regState             = MAIN_GRID_PARALLEL_NORMAL_OPERATION;
			PowerReductionActive      = FALSE;
			
			// set state outputs
	        MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;

		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				switch (PMS.CBPOS_OperationMode)
				{
					case PMS_CBPOS_UNDEFINED:
						// undefined breaker position
						transit(UndefinedBreakers);
					break;

					case PMS_CBPOS_BLACK_OPERATION:
						// black operation
						transit(BlackOperation);
					break;

					case PMS_CBPOS_ISLAND_OPERATION:
						// island operation
						transit(IslandOperation);
					break;

					case PMS_CBPOS_PARALLEL_OPERATION:
						// parallel operation
						transit(GridParallelOperationFullLoad);
					break;

					default:
					break;
				}
				break;
			}
			else // we are not the mains delomatic anymore !!!
			{ 
				transit(FastBraking);
			}

		break; // end of regular block SIG_DO
	}
}

static void EmergencyStop(const DU8 sig)
{// requires Stoplevel >= 1
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_BLOCK;           // fan off, flaps closed
			AKR.mode      = AKR_MODE_OFF;
            DK.mode       = DK_MODE_OFF;         // throttle controller switched off
            ENG.mode      = ENG_OFF;             // engine shut down
			//EXH.mode      = EXH_CLOSE;           // close flaps
            //FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            FAB.mode	  = FAB_BLOCK;			 // flushing air blower blocked
            GAS.mode      = GAS_MODE_BLOCK;      // gas valves blocked
            GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
            GEN.mode      = GEN_MODE_OFF;        // AVR off
            GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
            HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_BLOCK;           // emergency cooler off
			OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_BLOCK;           // no prelubrication
			//PFH.mode      = PFH_BLOCK;           // plant oil filter heater off
			WAT.mode      = WAT_BLOCK;           // water circuit system off
			//RSD.mode      = RSD_BLOCK;           // Refilling off 
			//RSP.mode      = RSP_BLOCK;           // Refilling off
			SAF.mode      = SAF_TRIP;            // trip safety chain
			SCR.mode	  = SCR_BLOCK;			 // catalyst off
            //STH.mode      = STH_BLOCK;           // heating of poil blocked
			THR.mode	  = THR_BLOCK;			 // thermoreactor blocked
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_SHUTDOWN;        // governor stopped
			COM.mode      = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			MIX.mode[MixerInd1] = MIX_BLOCK;
			MIX.mode[MixerInd2] = MIX_BLOCK;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_OFF;
			
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E
			
			// switch off test mode
			STOP_Tripped[STOPCONDITION_20099] = FALSE; // can be acknowledged
			STOP_Clear(STOPCONDITION_20099); // test mode off
		
			// define actual MAIN.state
			MAIN.state                = MAIN_EMERGENCY_STOP;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;
			
			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;

			if (PARA[ParRefInd[CHANGE_AFTER_EMERG_STOP__PARREFIND]].Value)
			{
				// not from Modbus
			    if (!(MBA.WriteConfigurationDigital & MBA_CONFIG_AUTOMATIC_OPERATION))
			    // not from DI
			    if (DI_FUNCT[AUTOMATIC_OPERATION].Assigned != ASSIGNED)
			    {
					// change to manual mode
					STOP_Set(STOPCONDITION_30001);
					STOP_Tripped[STOPCONDITION_30001] = TRUE;
			    }
			}
				
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
					
		break; // end of SIG_EXIT

		case SIG_DO : // called every 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			if (MAIN.IO_TestDemand)
			{
				transit(Test);
				break;
			}

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// allow auto restart in case of self acknowledged StopConditions
			if ( (MAIN.startdemand) || (MAIN.StartdemandRemoteAndAuto) )
		      MAIN.startdemand = STOP.actualBitMask & 0x0010; // set or reset startdemand

			if (STOP.actualLevel < 1)
			{ 
				transit(SystemStart); 
				break;
			}
			
			// Don't leave Emergency-Stop if GridProtectionTest is active
			if (ELM.GPTstate > ELM_GPT_STATE_NOT_ACTIVE)
				break;

		    if ((STOP.actualLevel > 1) && (SAF_OPTION_SAF))
		    { 
		    	transit(RearmSafetyChain);
		    	break;
		    }

		    if ((STOP.actualLevel > 1) && !(SAF_OPTION_SAF))
		    {
		    	transit(SystemStop);
		    	break;
		    }
		    // if (STOP.actualLevel == 1) stay where you are.
		    
		break; // end of regular block SIG_DO
	}
	
}

static void EmergencyBreaking(const DU8 sig)
{// The safety chain has opened: An emergency stop has to be performed
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_BLOCK;           // fan off, flaps closed
			AKR.mode      = AKR_MODE_OFF;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
            ENG.mode      = ENG_OFF;             // engine shut down
			//EXH.mode keep as is, especially while braking
            //FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            FAB.mode	  = FAB_BLOCK;			 // flushing air blower blocked
            GAS.mode      = GAS_MODE_BLOCK;      // gas valves blocked
            GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
            GEN.mode      = GEN_MODE_OFF;        // AVR off
            GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
            HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
            NKK.mode      = NKK_BLOCK;           // emergency cooler off
            OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_BLOCK;           // no prelubrication
            //PFH.mode      = PFH_BLOCK;           // plant oil filter heater off
			WAT.mode      = WAT_BLOCK;           // water circuit system off
			//RSD.mode      = RSD_BLOCK;           // Refilling off 
			//RSP.mode      = RSP_BLOCK;           // Refilling off
			SAF.mode      = SAF_TRIP;            // trip safety chain
			SCR.mode	  = SCR_BLOCK;			 // catalyst off
			//STH.mode      = STH_BLOCK;           // heating of poil blocked
			THR.mode	  = THR_BLOCK;			 // thermoreactor blocked
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_SHUTDOWN;        // governor stopped
			COM.mode      = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			MIX.mode[MixerInd1] = MIX_BLOCK;
			MIX.mode[MixerInd2] = MIX_BLOCK;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_OFF;
				
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E 
			
			// define actual MAIN.state
			MAIN.state                = MAIN_EMERGENCY_BRAKING;
			
			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
					
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// allow auto restart in case of self acknowledged StopConditions
			if  ( (MAIN.startdemand) || (MAIN.StartdemandRemoteAndAuto) )
		      MAIN.startdemand = STOP.actualBitMask & 0x0010; // set or reset startdemand
			
			if (STOP.actualLevel < 1)
			{ 
			    transit(SystemStart); 
			    break;
			}
			
			// Check if engine speed is at standstill
			if (!TUR.isTurning.Signal)
			//if (ENG.S200EngineSpeed < ENG_STOPPED_SPEED) // ENG.S200EngineSpeedRaw is a DS16
			{
		        transit(EmergencyStop);
		        break;   
			}    
			
			// else engine is still running
			//Timeout -->
		    // Stopfailure
		break; // end of regular block SIG_DO
	}
	
}


static void RearmSafetyChain(const DU8 sig)
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_BLOCK;             // fan off, flaps closed
			AKR.mode      = AKR_MODE_OFF;
			DK.mode       = DK_MODE_OFF;         // throttle controller switched off
            ENG.mode      = ENG_OFF;             // engine shut down
			//EXH.mode      = EXH_CLOSE;           // close flaps
            //FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            //FAB.mode			keep as is
            GAS.mode      = GAS_MODE_BLOCK;      // gas valves blocked
            GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
            GEN.mode      = GEN_MODE_OFF;        // AVR off
            GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
            HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
            NKK.mode      = NKK_BLOCK;           // emergency cooler off
            OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_BLOCK;           // no prelubrication
            //PFH.mode      = PFH_BLOCK;           // plant oil filter heater off
            WAT.mode      = WAT_BLOCK;           // water circuit system off
			//RSD.mode      = RSD_BLOCK;           // Refilling off 
			//RSP.mode      = RSP_BLOCK;           // Refilling off
			SAF.mode      = SAF_REARM;           // rearm the safety chain for operation
			SCR.mode	  = SCR_BLOCK;			 // catalyst off
			//STH.mode      = STH_BLOCK;           // heating of poil blocked
			THR.mode	  = THR_BLOCK;			 // thermoreactor blocked
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_SHUTDOWN;        // governor stopped
			COM.mode      = COM_STOP;            // compressor off
			if (PARA[ParRefInd[IGN_OPTION__PARREFIND]].Value == 6L)
			{
				IGN.mode      = IGN_MODE_ON;        // ignition off
         		ZS3.OperatingStopRequested = TRUE;
			}
			else
				IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			MIX.mode[MixerInd1] = MIX_BLOCK;
			MIX.mode[MixerInd2] = MIX_BLOCK;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_OFF;
				
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E 
		
			// define actual MAIN.state
			MAIN.state                = MAIN_REARM_SAFETY_CHAIN;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
					
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
            if(!SAF_OPTION_SAF)
            {
                transit(EmergencyStop);
                break;
            }

			if (STOP.actualLevel < 1)
			{ 
			    transit(SystemStart); 
			    break;
			}
			
		    if (STOP.actualLevel == 1)
		    { 
		    	transit(EmergencyStop); 
		    	break;
		    }
		    
		    //else Level  >= 2
		    if (SAF.state == CHAIN_IS_ARMED)
		      transit(SystemStop);
		    
		break; // end of regular block SIG_DO
	}
	
}


static void SystemStop(const DU8 sig)
{
	// not AUTO or other reason to stay at level 2, DI G604 not set
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			//AIR.mode      = AIR_POSTRUN_DEMANDED;// off or postrun
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_OFF;         // throttle controller switched off
            ENG.mode      = ENG_OFF;             // engine shut down
			//EXH.mode      = EXH_CLOSE;           // close flaps
            //FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            GAS.mode      = GAS_MODE_OFF;        // gas valves closed
            GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
            GEN.mode      = GEN_MODE_OFF;        // AVR off
            GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
            HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
            NKK.mode      = NKK_ENABLE;          // emergency cooler active
            OIL.mode      = OIL_BLOCK;           // no oil pump
            PLS.mode      = PLS_INTERVAL_REQ;    // interval prelubrication
            //PFH.mode      = PFH_BLOCK;           // plant oil filter heater off
            WAT.mode      = WAT_POSTRUN_DEMANDED;// off or pump postrun
			//RSD.mode      = RSD_BLOCK;           // Refilling off 
			//RSP.mode      = RSP_BLOCK;           // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_BLOCK;			 // catalyst off
			//STH.mode      = STH_BLOCK;           // heating of poil blocked
			THR.mode	  = THR_HEAT_UP;		 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_SHUTDOWN;        // governor stopped
			COM.mode      = COM_STOP;            // compressor off
			if (PARA[ParRefInd[IGN_OPTION__PARREFIND]].Value == 6L)
			{
				IGN.mode      = IGN_MODE_ON;        // ignition on
         		ZS3.OperatingStopRequested = TRUE;
			}
			else
				IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode[MixerInd1] = MIX_BLOCK;
			//MIX.mode[MixerInd2] = MIX_BLOCK;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_ON;
			
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E 
				
			// define actual MAIN.state
			MAIN.state                = MAIN_SYSTEM_STOP; 

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
		    //MAIN.startdemand = FALSE;            // reset startdemand			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
			 		
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			if (FAN.FlushingDemand)
				AIR.mode = AIR_ENABLE;
			else
				AIR.mode = AIR_POSTRUN_DEMANDED;

			// set mixer mode depending on config and release load
			if (MIX_CALIBRATION_AFTER_ENGINE_STOP)
				MAIN_Set_MIX_mode(MIX_CALIBRATE);
			else
				MAIN_Set_MIX_mode(MIX_BLOCK);
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();

			// set mode for FAB
			MAIN_Set_FAB_mode();
			
			// allow auto restart in case of self acknowledged StopConditions
			if ( (MAIN.startdemand) || (MAIN.StartdemandRemoteAndAuto) )
		      MAIN.startdemand = STOP.actualBitMask & 0x0010; // set or reset startdemand		

			if (STOP.actualLevel < 2)
			{ 
				transit(EmergencyBreaking); 
				break;
			}
			//else: Level >=2
		    
		    // test demanded?
		    if (MAIN.TestMode)
		    { 
		    	transit(Test); 
		    	break;
		    }

		    // lube oil service active?
		    if (STOP_is_Set(STOPCONDITION_20093))
		    {
		    	transit(Test);
		    	break;
		    }
		    
		    if (STOP.actualLevel < 3)
			{ 
				// stay here 
				break;
			}
			
			// else level >= 3 and no test demanded

			if (MAIN.RegularStop)
			{
			    // stay here...
			    break;
			}

			if (MAIN.BlockStart)
			{
				// stay here...
				break;
			}

			// else -> goto system ready for start
			transit (SystemReadyForStart);

			
		break; // end of regular block SIG_DO
	}
	
}



static void SystemReadyForStart(const DU8 sig)
{
	// not AUTO or other reason to stay at level 2, but DI G604 set
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			//AIR.mode      = AIR_POSTRUN_DEMANDED;// off or postrun demanded
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_OFF;         // throttle controller switched off
			ENG.mode      = ENG_OFF;               // engine shut down
			//EXH.mode      = EXH_CLOSE;             // close flaps
            //FUE.mode      = FUE_DIESEL_DEMANDED;   // Diesel operation
            GAS.mode      = GAS_MODE_OFF;        // gas valves closed
            GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
            GEN.mode      = GEN_MODE_OFF;          // AVR off
            GEN.reg.mode  = GEN_REGMODE_UISLAND;   // type of Regulator nominal Voltage
            HVS.modeT1E   = HVS_T1E_OFF;           // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		   // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
            NKK.mode      = NKK_ENABLE;          // emergency cooler active
            OIL.mode      = OIL_BLOCK;             // no oil pump
            PLS.mode      = PLS_INTERVAL_REQ;    // interval prelubrication
            //PFH.mode      = PFH_ENABLE;            // plant oil filter heater regulation active
			WAT.mode      = WAT_POSTRUN_DEMANDED;  // pumps off or post run
			//RSD.mode      = RSD_ENABLE;            // Refilling off 
			//RSP.mode      = RSP_ENABLE;            // Refilling off
			SAF.mode      = SAF_GUARD;             // ready for TRIP and protection
			SCR.mode	  = SCR_BLOCK;
			//STH.mode      = STH_ELECTRIC_DEMANDED; // heating of poil electrically
			THR.mode	  = THR_HEAT_UP;			// thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_SHUTDOWN;          // governor stopped
			COM.mode      = COM_STOP;              // compressor off
			if (PARA[ParRefInd[IGN_OPTION__PARREFIND]].Value == 6L)
			{
				IGN.mode      = IGN_MODE_ON;        // ignition on
         		ZS3.OperatingStopRequested = TRUE;
			}
			else
				IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode[MixerInd1] = MIX_BLOCK;
			//MIX.mode[MixerInd2] = MIX_BLOCK;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_ON;
				
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E 
			
			// define actual MAIN.state
			MAIN.state                = MAIN_SYSTEM_READY_FOR_START; // = ready to start

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = TRUE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
					
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
			 		
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			if (FAN.FlushingDemand)
				AIR.mode = AIR_ENABLE;
			else
				AIR.mode = AIR_POSTRUN_DEMANDED;

			// set mixer mode depending on config and release load
			if (MIX_CALIBRATION_AFTER_ENGINE_STOP)
				MAIN_Set_MIX_mode(MIX_CALIBRATE);
			else
				MAIN_Set_MIX_mode(MIX_BLOCK);
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();

			// set mode for FAB
			MAIN_Set_FAB_mode();

			if (STOP.actualLevel < 2)
			{ 
				transit(EmergencyBreaking); 
				break;
			}
			
			// else Level is >= 2

		    if (STOP.actualLevel < 3)
		    {
		    	transit(FastBraking);
		    	break;
		    }

			// else level is >= 3
			
			// test demanded?
		    if (MAIN.TestMode)
		    { 
		        transit(Test); 
		        break;
		    }

		    // lube oil service active?
		    if (STOP_is_Set(STOPCONDITION_20093))
		    {
		    	transit(Test);
		    	break;
		    }
		    
		    // else level >= 3 and no test demanded
		    
			if ( MAIN.RegularStop )
			{
				transit(SystemStop);
				break;
			}

			// else level >= 3 and no test demanded and no regular stop

			if (MAIN.BlockStart)
			{
				transit(SystemStop);
				break;
			}
		    
			if (myStateCnt >= 1000L)
		    if (MAIN.startdemand) // start demanded 		            
		    { 
		        transit(StartPrepare);
		        break;
		    }
			
		    //else stay in Ready_for_Start
		    
		break; // end of regular block SIG_DO
	}
	
}

static void FastBraking(const DU8 sig)
{
#define MAIN_STOPPING_IGN_ON  PARA[ParRefInd[ENG_STOP_WITH_IGN__PARREFIND]].Value
#define MAIN_STOPPING_IGN_OFF (!MAIN_STOPPING_IGN_ON)

	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_ENABLE;          // Room air controller active
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_OFF;             // engine shut down
			//EXH.mode      = EXH_OPEN;            // open flaps
            //FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
			//FAB.mode		keep as is
            // close gas valves
            GAS.mode      = GAS_MODE_OFF;        // gas valves closed
            GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
            GEN.mode      = GEN_MODE_OFF;        // AVR off
            GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
            HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
            NKK.mode      = NKK_ENABLE;          // emergency cooler active
            OIL.mode      = OIL_BLOCK;           // no oil pump
            PLS.mode      = PLS_INTERVAL_REQ;    // interval prelubrication
            //PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
            WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_BLOCK;			 //
			//STH.mode      = STH_ELECTRIC_DEMANDED; // heating of poil electrically
			THR.mode	  = THR_HEAT_UP;			 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			if (MAIN_STOPPING_IGN_OFF)
				TUR.mode = TUR_SHUTDOWN;        // governor stopped
			// else MAIN_STOPPING_IGN_ON
				// keep as it is
			GEN.AVRManual = FALSE;               // set AVR to auto mode
            TUR.GOVManual = FALSE;               // set GOV to auto mode
            /* keep as it is...
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
            */
            if (MAIN_STOPPING_IGN_OFF)
            {
				if (PARA[ParRefInd[IGN_OPTION__PARREFIND]].Value == 6L)
				{
					IGN.mode      = IGN_MODE_ON;        // ignition on
					ZS3.OperatingStopRequested = TRUE;
				}
				else
					IGN.mode      = IGN_MODE_OFF;        // ignition off
            }
			// else MAIN_STOPPING_IGN_ON
				// keep as it is
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode[MixerInd1] = MIX_BLOCK;
			//MIX.mode[MixerInd2] = MIX_BLOCK;
			PMS.mode      = PMS_ENABLE; 
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			         
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E 

			// define actual MAIN.state
			MAIN.state                = MAIN_FAST_BRAKING;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

            // configuration of vector jump monitoring (not only if breaker closed)
            iohandler.VectorJumpOnlyIfBreakerOn = 0;
            iohandler.ConfigSCM = TRUE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
					
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			// set mixer mode depending on config and release load
			// keep as it is...
			//MAIN_Set_MIX_mode(MIX_BLOCK);
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();
			
			// allow auto restart in case of self acknowledged StopConditions
			if ( (MAIN.startdemand) || (MAIN.StartdemandRemoteAndAuto) )
		      MAIN.startdemand = STOP.actualBitMask & 0x0010; // set or reset startdemand
		    
		    if (STOP.actualLevel < 2)
		    { 
		    	transit(EmergencyBreaking); 
		    	break;
		    }
			
			// else STOP.actualLevel >= 2
			
			// engine must stop
			if ( !TUR.isTurning.Signal ) 
			//former expression ( ENG.S200EngineSpeed < ENG_STOPPED_SPEED ) 
			{ 
				transit(SystemStop); // hier muß Pumpennachlauf rein
				break;
		    }
		    
		    //else STOP.actualLevel >= 2 and engine still turning
			// this transition is not allowed for a gas engine
			/*
			if (    
			// engine may continue to run if still running and no regular stop and startdemand present
			        ( MAIN.startdemand )
			     && ( STOP.actualBitMask & 0x0001 )
			     && ( ENG.S200EngineSpeed > ENG_RUNNING_SPEED )
			   )
			{ 
				transit(Acceleration); 
				break;
			}*/
		    
		break; // end of regular block SIG_DO
	}
	
}

static void StartPrepare(const DU8 sig)
// open flaps, move everything into start position 
{
	DU32 TimeoutDelay;

	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			// Reset AKR
			if (AKR_RESET_IN_STARTPREPARE)
			{
				if (DO_FUNCT[IOA_DO_AKR_POWER_SUPPLY].Assigned == ASSIGNED)
					AKR.Reset = TRUE;
				else
					AKR.ChangeRequest_AKR14 = TRUE;
			}
			DK.mode       = DK_MODE_ON;         // throttle controller switched on
			ENG.mode      = ENG_OFF;             // engine shut down
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
			GAS.mode      = GAS_MODE_OFF;        // gas valves closed
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			GEN.mode      = GEN_MODE_OFF;        // AVR off
			GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
			HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_WARM_UP;			 // prepare catalyst system
			//STH.mode      = STH_ELECTRIC_DEMANDED; // heating of poil electrically
			THR.mode	  = THR_HEAT_UP;			 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			//TUR.mode      = TUR_SHUTDOWN;        // governor stopped
			TUR.mode      = TUR_TAKE_SETPOINT_POSITION; // reach and keep start value
			TUR.MAINPosSet= TUR.GovOutMax; // open throttle fully (purging)
            if (!GAS.GasTypeBActive) // gas type A
            {
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            }
            else // gas type B
            {
            	COM.mode  = COM_STOP;            // compressor off
            }
			if (PARA[ParRefInd[IGN_OPTION__PARREFIND]].Value == 6L)
			{
				IGN.mode      = IGN_MODE_ON;        // ignition on
         		ZS3.OperatingStopRequested = TRUE;
			}
			else
				IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode[MixerInd1] = MIX_CALIBRATE;
			//MIX.mode[MixerInd2] = MIX_CALIBRATE;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E 
			
			// define actual MAIN.state
			MAIN.state                = MAIN_STRT_PREPARE;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

            // configuration of vector jump monitoring (only if breaker closed)
            iohandler.VectorJumpOnlyIfBreakerOn = VJC_VECTORJUMP_ONLY_IF_BREAKER_ON;
            iohandler.ConfigSCM = TRUE;
			
			// set state outputs
			MAIN.DO_ReadyForOperation = TRUE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
						
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
					
		break; // end of SIG_EXIT

		case SIG_DO : // called every 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			// set mixer mode depending on config and release load
			MAIN_Set_MIX_mode(MIX_CALIBRATE);
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();

			// set mode for FAB
			MAIN_Set_FAB_mode();

			TimeoutDelay = MAIN_STRT_PREPARE_TIMEOUT;

			// Increase delay if mixer is very slow
			if ((AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
				&& (TimeoutDelay < ((DU32)PARA[ParRefInd[MIX_RUNNING_TIME_0_TO_100_PERCENT__PARREFIND]].Value * 3)))
				TimeoutDelay = (DU32)PARA[ParRefInd[MIX_RUNNING_TIME_0_TO_100_PERCENT__PARREFIND]].Value * 3;
			   
			   if ( myStateCnt > TimeoutDelay )
			   { STOP_Set( STOPCONDITION_20022 );}
			   
			   if ( STOP.actualLevel < 2)
			   { 
			   	 transit(EmergencyBreaking); 
			   	 break; 
			   }
			
			   // else STOP.actualLevel >= 2
			
		       if ( STOP.actualLevel == 2)
		       {  
		       	 transit(FastBraking); 
		       	 break; 
		       }
		      
		       // else STOP.actualLevel > 2
		      
		       // Are we in regular stop?
		       if ( !(STOP.actualBitMask & 0x0001) )
		       {
		         transit(FastBraking);
		         break;
		       }
		       
		       // else (STOP.actualLevel > 2) and no regular stop

				if (MAIN.BlockStart)
				{
					transit(FastBraking);
					break;
				}
		       
		       // no startdemand?
		       if ( !MAIN.startdemand )
		       { 
		         transit(FastBraking);
		         break;
		       }

		       if (myStateCnt < 1000L) break; // wait...
		   
			   // else (STOP.actualLevel > 2) and (MAIN.startdemand) and no regular stop
			   // gas type A
			   if (!GAS.GasTypeBActive)
			   {
				   if ( ( DKA.state == DKA_CHECK_OK )
				      &&( COM.state == COM_READY)
				      &&( (PLS.state == PLS_START_PUMP_ON_SUCCEEDED) || !PLS.Available)
					  &&( FAB.StartReleased )
				      &&((MIX.state[MixerInd1] == MIX_START_POSITION_REACHED ) || MIX.AdjustmentDuringStart_Activated )
				      &&((MIX.state[MixerInd2] == MIX_START_POSITION_REACHED ) || MIX.AdjustmentDuringStart_Activated )
				      &&( HZM.RUN_Mode || !HZM.Option )
				      &&(mic4.operational || !mic4.Option)
				      &&(ic92x.operational || !ic92x.Option)
				      &&((AKR.state >= AKR_WAITING) || !AKR.Option ) )
				   {
				   	 // Sub state machines are ready
					   transit (Starting);
					   break;
				   }
			   }
			   else // gas type B
			   {
				   if ( ( DKB.state == DKB_CHECK_OK )
					  &&( (PLS.state == PLS_START_PUMP_ON_SUCCEEDED) || !PLS.Available)
					  &&( FAB.StartReleased )
					  &&((MIX.state[MixerInd1] == MIX_START_POSITION_REACHED ) || MIX.AdjustmentDuringStart_Activated )
					  &&((MIX.state[MixerInd2] == MIX_START_POSITION_REACHED ) || MIX.AdjustmentDuringStart_Activated )
					  &&( HZM.RUN_Mode || !HZM.Option )
				      &&(mic4.operational || !mic4.Option)
				      &&(ic92x.operational || !ic92x.Option)
				      &&((AKR.state >= AKR_WAITING) || !AKR.Option ) )
				   {
				   	 // Sub state machines are ready
					   transit (Starting);
					   break;
				   }			   	
			   }
			
		break; // end of regular block SIG_DO
	}
}

static void Starting(const DU8 sig) 
{
	// turn on starter
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
            AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
            DK.mode       = DK_MODE_ON;          // throttle controller switched on
            ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            FAB.mode	  = FAB_CLOSE_FLAP_STOP_BLOWER;
			GAS.mode      = GAS_MODE_OFF;        // gas valves closed
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			GEN.mode      = GEN_MODE_OFF;        // AVR off
			GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
			HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_WARM_UP;			 // prepare catalyst system
			//STH.mode      = STH_ELECTRIC_DEMANDED; // heating of poil electrically
			THR.mode	  = THR_HEAT_UP;			 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_TAKE_SETPOINT_POSITION; // reach and keep start value
			TUR.MAINPosSet= TUR.GovOutMax; // open throttle fully (purging)
            GEN.AVRManual = FALSE;               // set AVR to auto mode
            TUR.GOVManual = FALSE;               // set GOV to auto mode
            if (!GAS.GasTypeBActive || !GAS.GasTypeBSelected) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			if (PARA[ParRefInd[IGN_OPTION__PARREFIND]].Value == 6L)
			{
				IGN.mode      = IGN_MODE_ON;        // ignition on
         		ZS3.OperatingStopRequested = TRUE;
			}
			else
				IGN.mode      = IGN_MODE_OFF;        // ignition off
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode[MixerInd1] = MIX_MOVE_TO_START_POSITION;
			//MIX.mode[MixerInd2] = MIX_MOVE_TO_START_POSITION;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
				
			//HVS.modeL1E    = HVS_L1E_KEEP;       // keep state of 22L1E 
		
			// define actual MAIN.state
			MAIN.state                = MAIN_STARTING;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = TRUE;

			// set state outputs
			MAIN.DO_ReadyForOperation = TRUE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called every 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			// set mixer mode depending on config and release load
			MAIN_Set_MIX_mode(MIX_MOVE_TO_START_POSITION);
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();

			if (STOP.actualLevel < 3)
			{ 
				transit(FastBraking); 
				break;
			}
			
		   // else STOP.actualLevel >= 3
           
           // Are we in regular stop?
	       if ( !(STOP.actualBitMask & 0x0001) )
	       {
	         // regular stop?
	         transit(FastBraking);
	         break;
	       }
	       
	       // else (STOP.actualLevel >=3) and no regular stop

			if (MAIN.BlockStart)
			{
				transit(FastBraking);
				break;
			}
	       
	       // no startdemand?
	       if (!MAIN.startdemand)
		   { 
		     transit(FastBraking);
		     break;
		   }
		       
		   // else (STOP.actualLevel >=3) and no regular stop and MAIN.startdemand
		   
		   // flushing finished?
		   if ( ENG.state == ENG_CRANK )
		   { 
		   	 //transit(Acceleration);
		   	 transit(Ignition);
		   	 break;
		   }
		    
		break; // end of regular block SIG_DO
	}
	
}

static void Ignition(const DU8 sig) 
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
            AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
            DK.mode       = DK_MODE_ON;          // throttle controller switched on
            ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            //FAB.mode		keep as is
			GAS.mode      = GAS_MODE_OFF;        // gas valves closed
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			GEN.mode      = GEN_MODE_OFF;         // AVR off
			GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
			HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_WARM_UP;			 // prepare catalyst system
			//STH.mode      = STH_ELECTRIC_DEMANDED; // heating of poil electrically
			THR.mode	  = THR_HEAT_UP;			 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_TAKE_SETPOINT_POSITION; // reach and keep start value
			if (PAR_CUMMINS_OPTION)
				TUR.MAINPosSet = TUR_MIN_POSITION_SETPOINT_AOUT
				+ (TUR_MAX_POSITION_SETPOINT_AOUT - TUR_MIN_POSITION_SETPOINT_AOUT)
				* PAR_CUMMINS_THROTTLE_LOW_IDLE / 1000L;
			else
				TUR.MAINPosSet = TUR.GovOutMax; // open throttle fully (purging)
            GEN.AVRManual = FALSE;               // set AVR to auto mode
            TUR.GOVManual = FALSE;               // set GOV to auto mode
            if (!GAS.GasTypeBActive || !GAS.GasTypeBSelected) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
            IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode[MixerInd1] = MIX_MOVE_TO_START_POSITION;
			//MIX.mode[MixerInd2] = MIX_MOVE_TO_START_POSITION;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
				
			//HVS.modeL1E    = HVS_L1E_KEEP;       // keep state of 22L1E 
		
			// define actual MAIN.state
			MAIN.state                = MAIN_IGNITION;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = TRUE;

			// set state outputs
			MAIN.DO_ReadyForOperation = TRUE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called every 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed			

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			// set mixer mode depending on config and release load
			MAIN_Set_MIX_mode(MIX_MOVE_TO_START_POSITION);
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();

			if (STOP.actualLevel < 3)
			{ 
				transit(FastBraking); 
				break;
			}
			
		   // else STOP.actualLevel >= 3
           
           // Are we in regular stop?
	       if ( !(STOP.actualBitMask & 0x0001) )
	       {
	         // regular stop?
	         transit(FastBraking);
	         break;
	       }
	       
	       // else (STOP.actualLevel >=3) and no regular stop

			if (MAIN.BlockStart)
			{
				transit(FastBraking);
				break;
			}
	       
	       // no startdemand?
	       if (!MAIN.startdemand)
		   { 
		     transit(FastBraking);
		     break;
		   }
		       
		   // else (STOP.actualLevel >=3) and no regular stop and MAIN.startdemand
		   
		   
		   // engine still turning starter?
		   if (!( (ENG.state == ENG_CRANK)
		   		|| (ENG.state == ENG_COOLDOWN_RUN)
		   		|| (ENG.state == ENG_RUNNING) ) )
		   {
		   	 transit(FastBraking);
		   	 break;
		   }
		   
		   // ignition ready?    
		   if (IGN.state == IGN_READY)   
		   { 
		   	 //transit(Acceleration);
		   	 transit(OpenGasValves);
		   	 break;
		   }
		    
		break; // end of regular block SIG_DO
	}
	
}

static void OpenGasValves(const DU8 sig) 
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
            AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
            DK.mode       = DK_MODE_ON;          // throttle controller switched on
            ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            //FAB.mode		keep as is
            FAB.FlushingSuccessful = FALSE;		 // when gas valves are open, flushing is needed with next start
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			// keep it... GEN.mode      = GEN_MODE_ON;         // AVR on
			GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
			HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_WARM_UP;			 // prepare catalyst system
			//STH.mode      = STH_ELECTRIC_DEMANDED; // heating of poil electrically
			THR.mode	  = THR_HEAT_UP;		 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TUR.mode      = TUR_TAKE_SETPOINT_POSITION; // reach and keep start value
			if (PAR_CUMMINS_OPTION)
				TUR.MAINPosSet = TUR_MIN_POSITION_SETPOINT_AOUT
				+ (TUR_MAX_POSITION_SETPOINT_AOUT - TUR_MIN_POSITION_SETPOINT_AOUT)
				* PAR_CUMMINS_THROTTLE_LOW_IDLE / 1000L;
			else
				TUR.MAINPosSet = TUR.GovOutMax; // open throttle fully (purging)
            GEN.AVRManual = FALSE;               // set AVR to auto mode
            TUR.GOVManual = FALSE;               // set GOV to auto mode
            if (!GAS.GasTypeBActive || !GAS.GasTypeBSelected) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
            IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode[MixerInd1] = MIX_MOVE_TO_START_POSITION;
			//MIX.mode[MixerInd2] = MIX_MOVE_TO_START_POSITION;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
				
			//HVS.modeL1E    = HVS_L1E_KEEP;       // keep state of 22L1E 
		
			// define actual MAIN.state
			MAIN.state                = MAIN_OPEN_GAS_VALVES;
			
			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = TRUE;

			// set state outputs
			MAIN.DO_ReadyForOperation = TRUE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called every 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			// set mixer mode depending on config and release load
			if (PARA[ParRefInd[MIX_IDLE_POS_IN_ACCELERATION__PARREFIND]].Value == 2L)
			{
				MAIN_Set_MIX_mode(MIX_MOVE_TO_IDLE_POSITION);
			}
			else
			{
				MAIN_Set_MIX_mode(MIX_MOVE_TO_START_POSITION);
			}
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();
			
			if (STOP.actualLevel < 3)
			{ 
				transit(FastBraking); 
				break;
			}
			
		   // else STOP.actualLevel >= 3
           
           // Are we in regular stop?
	       if ( !(STOP.actualBitMask & 0x0001) )
	       {
	         // regular stop?
	         transit(FastBraking);
	         break;
	       }
	       
	       // else (STOP.actualLevel >=3) and no regular stop

			if (MAIN.BlockStart)
			{
				transit(FastBraking);
				break;
			}
	       
	       // no startdemand?
	       if (!MAIN.startdemand)
		   { 
		     transit(FastBraking);
		     break;
		   }
		       
		   // else (STOP.actualLevel >=3) and no regular stop and MAIN.startdemand
    
		   // engine still turning starter?
		   if (!( (ENG.state == ENG_CRANK)
		   		|| (ENG.state == ENG_COOLDOWN_RUN)
		   		|| (ENG.state == ENG_RUNNING) ) )  
		   { 
		   	 transit(FastBraking);
		   	 break;
		   }
		   
		   // engine running?    
		   if (ENG.state == ENG_RUNNING)  
		   { 
			 TUR.MAINRpmSet = PARA[ParRefInd[STRT_VALUE_SPEED_RAMP__PARREFIND]].Value * 1000L;
		   	 transit(Acceleration);
		   	 break;
		   }
		    
		break; // end of regular block SIG_DO
	}
	
}


static void Acceleration(const DU8 sig)
{
	static DU32 LowIdleSpeedTimer = 0L;
	static DU32 AccelerationTimer = 0L;

	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			// keep it... GEN.mode      = GEN_MODE_ON;         // AVR off
			GEN.SetpointVoltage = GEN_NOMINAL_VOLTAGE; // setpoint = nominal
			GEN.reg.mode  = GEN_REGMODE_UISLAND; // type of Regulator nominal Voltage
			HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_BLOCK;           // no oil pump
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_WARM_UP;			 // prepare catalyst system
			//STH.mode      = STH_ELECTRIC_DEMANDED; //heating of poil electrically
			THR.mode	  = THR_HEAT_UP;		 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			//TUR.MAINRpmSet= PARA[ParRefInd[STRT_VALUE_SPEED_RAMP__PARREFIND]].Value * 1000L;
			//TUR.MAINRpmSet= 300000L + (ENG_RUNNING_SPEED * 100);
			//TUR.MAINRpmSet= 100000L + (ENG_RUNNING_SPEED * 100);
			if (PAR_CUMMINS_OPTION)
			{
				TUR.MAINPosSet = TUR_MIN_POSITION_SETPOINT_AOUT
						+ (TUR_MAX_POSITION_SETPOINT_AOUT - TUR_MIN_POSITION_SETPOINT_AOUT)
						* PAR_CUMMINS_THROTTLE_LOW_IDLE / 1000L;
				TUR.mode = TUR_TAKE_SETPOINT_POSITION;
			}
			else
				TUR.mode = TUR_TAKE_SETPOINT_RPM;  // start value
			
			GEN.AVRManual = FALSE;               // set AVR to auto mode
            TUR.GOVManual = FALSE;               // set GOV to auto mode
            if (!GAS.GasTypeBActive || !GAS.GasTypeBSelected) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
            IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode[MixerInd1] = MIX_MOVE_TO_START_POSITION;
			//MIX.mode[MixerInd2] = MIX_MOVE_TO_START_POSITION;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
            
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E 

			LowIdleSpeedTimer = 0L;
			AccelerationTimer = 0L;

			MAIN_Set_LowIdleSpeed();

		    // define actual MAIN.state
			if (MAIN.state == MAIN_OPEN_GAS_VALVES)
				MAIN.state = MAIN_ACCELERATION;
			else if (MAIN.LowIdleSpeed_Demand)
				MAIN.state = MAIN_LOW_IDLE_SPEED;
			else
				MAIN.state = MAIN_ACCELERATION;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = TRUE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			// set mixer mode depending on config and release load
			if (PARA[ParRefInd[MIX_IDLE_POS_IN_ACCELERATION__PARREFIND]].Value >= 1L)
			{
				//MIX.AdjustmentDuringStart = FALSE;
				MAIN_Set_MIX_mode(MIX_MOVE_TO_IDLE_POSITION);
			}
			else
			{
				MAIN_Set_MIX_mode(MIX_MOVE_TO_START_POSITION);
			}
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();

			if (PARA[ParRefInd[LOW_IDLE_OPTION__PARREFIND]].Value == 0L) // Option = off
				MAIN.LowIdleSpeed_Demand = FALSE;
			else if (!MAIN.ManualOperation)
			{
				if (PARA[ParRefInd[LOW_IDLE_OPTION__PARREFIND]].Value == 1L) // Option = oil temperature
				{
					if (!ENG.LubeOilTemp_Available || (ENG.LubeOilTemp.Value >= (DS16)PARA[ParRefInd[LOW_IDLE_RELEASE_OIL_TEMP__PARREFIND]].Value))
						MAIN.LowIdleSpeed_Demand = FALSE;
				}
				else if (PARA[ParRefInd[LOW_IDLE_OPTION__PARREFIND]].Value == 2L) // Option = time
				{
					if (LowIdleSpeedTimer > (DU32)PARA[ParRefInd[LOW_IDLE_RELEASE_TIME__PARREFIND]].Value)
						MAIN.LowIdleSpeed_Demand = FALSE;
				}
				else if (PARA[ParRefInd[LOW_IDLE_OPTION__PARREFIND]].Value == 3L) // Option = oil temp. + time
				{
					if (LowIdleSpeedTimer > (DU32)PARA[ParRefInd[LOW_IDLE_RELEASE_TIME__PARREFIND]].Value)
					if (!ENG.LubeOilTemp_Available || (ENG.LubeOilTemp.Value >= (DS16)PARA[ParRefInd[LOW_IDLE_RELEASE_OIL_TEMP__PARREFIND]].Value))
						MAIN.LowIdleSpeed_Demand = FALSE;
				}
				else // Option = NA...?!?
					MAIN.LowIdleSpeed_Demand = FALSE;
			}
			// else MAIN.LowIdleSpeed_Demand is set via soft-button in ManualMode

			// Change State depending on LowIdleSpeed-Demand
			if (MAIN.LowIdleSpeed_Demand)
			{
				AccelerationTimer = 0L;

				if (PAR_CUMMINS_OPTION)
				{
					TUR.MAINPosSet = TUR_MIN_POSITION_SETPOINT_AOUT
							+ (TUR_MAX_POSITION_SETPOINT_AOUT - TUR_MIN_POSITION_SETPOINT_AOUT)
							* PAR_CUMMINS_THROTTLE_LOW_IDLE / 1000L;
					TUR.mode = TUR_TAKE_SETPOINT_POSITION;

					// Take actual speed
					TUR.MAINRpmSet = ENG.S200EngineSpeed * 100;
				}
				else
				{
					TUR.mode = TUR_TAKE_SETPOINT_RPM;

					// ramp up to low idle speed
					if (TUR.MAINRpmSet < TUR.LowIdleSpeed)
					{
						TUR.MAINRpmSet += (DS32)PARA[ParRefInd[SPEED_RAMP_UP__PARREFIND]].Value/50;

						if (TUR.MAINRpmSet > TUR.LowIdleSpeed)
							TUR.MAINRpmSet = TUR.LowIdleSpeed;
					}
					// ramp down to low idle speed
					else if (TUR.MAINRpmSet > TUR.LowIdleSpeed)
					{
						TUR.MAINRpmSet -= (DS32)PARA[ParRefInd[SPEED_RAMP_UP__PARREFIND]].Value/50;

						if (TUR.MAINRpmSet < TUR.LowIdleSpeed)
							TUR.MAINRpmSet = TUR.LowIdleSpeed;
					}
				}

				if (MAIN.state != MAIN_LOW_IDLE_SPEED)
				{
					if ( PAR_CUMMINS_OPTION OR (TUR.MAINRpmSet >= TUR.LowIdleSpeed))
					{
						MAIN.state = MAIN_LOW_IDLE_SPEED;
						MAIN_StateLogLine_record();
					}
				}

				if ( PAR_CUMMINS_OPTION OR (TUR.MAINRpmSet == TUR.LowIdleSpeed))
				{
					LowIdleSpeedTimer += 20L;
				}

				if (LowIdleSpeedTimer > (DU32)PARA[ParRefInd[LOW_IDLE_TIMEOUT__PARREFIND]].Value)
					STOP_Set(STOPCONDITION_20019);
			}
			else
			{
				LowIdleSpeedTimer = 0L;
				AccelerationTimer += 20L;

				TUR.mode = TUR_TAKE_SETPOINT_RPM;

				if (MAIN.state != MAIN_ACCELERATION)
				{
					MAIN.state = MAIN_ACCELERATION;
					MAIN_StateLogLine_record();
				}

				// ramp up to nominal speed
				if (TUR.MAINRpmSet < TUR.NominalSpeed)
					TUR.MAINRpmSet += (DS32)PARA[ParRefInd[SPEED_RAMP_UP__PARREFIND]].Value/50;

				if (TUR.MAINRpmSet > TUR.NominalSpeed)
					TUR.MAINRpmSet = TUR.NominalSpeed;
			}

			if ( AccelerationTimer > MAIN.AccelerationTimeout )
			{ STOP_Set(STOPCONDITION_20020); }

			if (STOP.actualLevel < 3)
			{ 
			  transit(FastBraking); 
			  break;
			}
			
			// else actualLevel >= 3
			
			// Are we in regular stop?
	        if ( !(STOP.actualBitMask & 0x0001) )
	        {
	          transit(FastBraking);
	          break;
	        }
	        
	        // else (STOP.actualLevel >= 3) and no regular stop
	        
	        // no startdemand?
	        if (!MAIN.startdemand)
		    { 
		      transit(FastBraking);
		      break;
		    }
		
		    // else (STOP.actualLevel >=3) and no regular stop and startdemand
		
		    // engine dying for whatever reason: go back
		    if (ENG.state != ENG_RUNNING)
		    {
		      transit(FastBraking);
		      break;
		    }

		    // running speed lost (80% of parameter value)
		    if (myStateCnt >= 2000L)
		    if (ENG.S200EngineSpeed < (PARA[ParRefInd[ENGINE_RUNNING__PARREFIND]].Value/125))
		    {
	            transit(FastBraking);
	            break;
		    }
		       
	        // else actualLevel >=3 and not regular stop and startdemand and engine running

	        if (MAIN.LowIdleSpeed_Demand)
	        {
	        	GEN.mode = GEN_MODE_OFF;
	        }
	        else
	        {
			    // excitatio-on-speed reached
		        if (ENG.S200EngineSpeed >= (DS16)PARA[ParRefInd[GEN_AVR_ACTIVATION_SPEED__PARREFIND]].Value )
		        {
		        	GEN.mode = GEN_MODE_ON;
		        }

		        // idle speed reached or generator voltage in window?
		        // if (ENG.S200EngineSpeed > ENG_IDLE_SPEED )
		        // idle speed reached? 	NOMINAL [mrpm] / 100 = [1/10 rpm]
		        if (ENG.S200EngineSpeed >= (TUR.NominalSpeed / 100 * ENG_IDLE_SPEED_PERCENT / 1000 ))
		        {
		          transit(IdleRun);
		          break;
		        }
	        }
	        
		break; // end of regular block SIG_DO
	}
}




static void IdleRun(const DU8 sig) 
{
	static DS32 MAIN_RpmSet = 0;

	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
            AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
            DK.mode       = DK_MODE_ON;          // throttle controller switched on
            ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			// keep it... GEN.mode      = GEN_MODE_ON;         // AVR active
			GEN.SetpointVoltage = GEN_NOMINAL_VOLTAGE; // setpoint = nominal
			GEN.reg.mode  = GEN_REGMODE_UISLAND_TAKE_SETPOINT;
			HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // waste oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_WARM_UP;			 // prepare catalyst system
			//STH.mode      = STH_ELECTRIC_DEMANDED; // heating of poil electrically
			THR.mode	  = THR_HEAT_UP;		 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
            
            //TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
            TUR.AdditionalRpmSet = 0;
			// ramp to nominal speed
			if (TUR.MAINRpmSet < TUR.NominalSpeed)
				TUR.MAINRpmSet += (DS32)PARA[ParRefInd[SPEED_RAMP_UP__PARREFIND]].Value/50;
			if (TUR.MAINRpmSet > TUR.NominalSpeed)
				TUR.MAINRpmSet = TUR.NominalSpeed;
            MAIN_RpmSet = TUR.MAINRpmSet;
            
            TUR.mode      = TUR_TAKE_SETPOINT_RPM; // reach and keep speed
            if (!GAS.GasTypeBActive || !GAS.GasTypeBSelected) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
            IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			MIX.mode[MixerInd1] = MIX_MOVE_TO_IDLE_POSITION;
			MIX.mode[MixerInd2] = MIX_MOVE_TO_IDLE_POSITION;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			//HVS.modeL1E          = HVS_L1E_KEEP;       // keep state of 22L1E
					
			// define actual MAIN.state
			MAIN.state                = MAIN_TRANSFORMER_DISCONNECTED;	

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = TRUE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			//MAIN.WishToCloseGCB       = TRUE;
			//MAIN.EngineRunningNominalDelayed keep as is
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			if (myStateCnt > 500L) MAIN.EngineRunningNominalDelayed = TRUE;

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();

			// update the wish to close breaker
			//
			//
			// Wish to close is only helpful if the system is allowed to close
			// -> Check level
			//
			
			MAIN.WishToCloseGCB = ( ELM.BBVoltageIsZero_Flag					// black BB
									&& (STOP.actualLevel >= 5) );				// allowed to close breaker
			
			//if (STOP.actualLevel < 5){ transit(SoftBraking); break;}
			
		    /*if (   ( (STOP.actualLevel == 6) || (STOP.actualLevel == 7) )
		        && ( HVS.stateL1E == HVS_L1E_OFF_AND_READY )
		        && ( GEN.reg.state == GEN_REGSTATE_UISLAND_DONE )
		       )
		       { transit(ConnectT1E);break;}
		    */
		    
		    
		    
		    if ( myStateCnt > MAIN_IDLE_RUN_TIMEOUT )
			{ STOP_Set(STOPCONDITION_20019); }
			
			if (STOP.actualLevel < 3)
			{ 
			  transit(FastBraking); 
			  break;
			}
			
			// else actualLevel >= 3
			
			// Are we in regular stop?
	        if ( !(STOP.actualBitMask & 0x0001) && !(GAS.Switchover) )
	        {
	          // If gas switchover while engine running is demanded, perform this before stopping.
	          // Gas switchover is limited to GAS_VALVES_OVERLAP_TIME [ms].
	          transit(FastBraking);
	          break;
	        }
	        
	        // else (STOP.actualLevel >=3) and no regular stop
	        
	        // start not demanded?
	        if ( (!MAIN.startdemand) && (!GAS.Switchover) )
		    { 
	          // In case of gas switchover while engine running wait until switchover is done.
	          // Gas switchover is limited to GAS_VALVES_OVERLAP_TIME [ms].
		      transit(FastBraking);
		      break;
		    }
	        
	        // else (STOP.actualLevel >=3) and no regular stop and start demanded
	        
	        // engine dying for whatever reason: go back
	        //if ( (ENG.state != ENG_RUNNING) && (!GEN.VoltageInWindow) )
	        if ( (ENG.S200EngineSpeed < PARA[ParRefInd[ENGINE_RUNNING__PARREFIND]].Value/100) && (!GEN.VoltageInWindow) )
	        {
	        	// engine has stopped
	        	STOP_Set(STOPCONDITION_20096);

	          transit(FastBraking);
		      break;
	        }

			if (MAIN.LowIdleSpeed_Demand)
			{
			  transit(Acceleration);
			  break;
			}

			// ramp to nominal speed
			if (MAIN_RpmSet < TUR.NominalSpeed)
				MAIN_RpmSet += (DS32)PARA[ParRefInd[SPEED_RAMP_UP__PARREFIND]].Value/50;
			if (MAIN_RpmSet > TUR.NominalSpeed)
				MAIN_RpmSet = TUR.NominalSpeed;

			TUR.MAINRpmSet = MAIN_RpmSet;

			// when T1E is connected externally, perform a transit to Grid Parallel operation
		    if (( HVS.stateT1E == HVS_T1E_IS_ON )
		    	&& ( HVS.stateL1E == HVS_L1E_IS_ON )
		    	)
		    {
		      transit(GridParallelOperationLimitedLoad);
		      break;
		    }

			// when T1E is connected externally, perform a transit to island operation
		    if (( HVS.stateT1E == HVS_T1E_IS_ON )
		    	&& ( HVS.stateL1E == HVS_L1E_OFF_AND_READY )
		    	&& ( ISL.IslandOperationAllowed )
		    	)
		    {
		    	// if we are alone in the loadsharing line -> transit to island operation
		    	if (PMS_NoLSMemberConnected() || !PMS.EngineIDConfigured[ARC.nEngineId-1])
		    	{
			    	transit(IslandOperation);
			    	break;
		    	}
		    	else // we are not alone in the loadsharing line -> transit to loadsharing ramp up
		    	{
			    	transit(LoadSharingRampUp);
			    	break;
		    	}
		    }
/*	        
	        // ONE SCM-card
	        // actualLevel >=4 and U gen in window?
	        if ( (PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value <= 1)	// one SCM-card, rmiEPF
	        	&& (STOP.actualLevel > 3)
		        && (GEN.VoltageInWindow) 
		        && (HVS.stateT1E != HVS_T1E_IS_ON)
		        && (ELM.BBVoltageIsOK_Flag) )
		        {
		          transit(Synchronize);
		          break;
		        }

	        // TWO SCM-cards
	        if ( (PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value > 1)	// second SCM-card
				&& (GEN.VoltageInWindow)
				&& (HVS.stateT1E != HVS_T1E_IS_ON) )
				{
			    	if (ELM.MainsFailure || ISL.IslandOperationActive)				// mains failure OR island demanded
			    	{
						if ( (STOP.actualLevel > 4)
							&& ELM.BBVoltageIsZero_Flag								// BB black, rmiIET
							&& (GEN.reg.state == GEN_REGSTATE_UISLAND_DONE)
			        		&& (HVS.modeL1E == HVS_L1E_OFF)
			        		&& (HVS.stateL1E == HVS_L1E_OFF_AND_READY)
			        		&& (HVS.L1EIsOffTimer >= PARA[ParRefInd[EPF_SWITCH_ON_TIME__PARREFIND]].Value) )
			        	{
			        		transit(ConnectT1E);									// connect T1E to island operation
			        		break;			        		
			        	}			    		
			    	}
			    	else // no mains failure and no island demanded ==> change to synchronize
			    	{
			    		if ( (STOP.actualLevel > 3)
			    			&& (HVS.modeL1E == HVS_L1E_ON)
			    			&& (HVS.stateL1E == HVS_L1E_IS_ON) 
			    			&& (ELM.BBVoltageIsOK_Flag) )
			    		{
			        		transit(Synchronize);
			        		break;
			    		}
			    	}
				}
*/				
	        // actualLevel >=4 AND U gen in window AND generator breaker open?
	        if ( (STOP.actualLevel > 3)
		        && (GEN.VoltageInWindow) 
		        && (HVS.stateT1E != HVS_T1E_IS_ON)
		        && ((AKR.state == AKR_READY) || !AKR.Option)
		        )
		        {
		          if (HVS.stateL1E == HVS_L1E_IS_ON) // mains breaker is closed
		          {
		          	if (ELM.BBVoltageIsOK_Flag) // voltage on busbar
		    		{
		        		transit(Synchronize);
		        		break;
		    		}
		          }
		          else if ( (HVS.stateL1E == HVS_L1E_OFF_AND_READY) // mains breaker is open
		                    && ISL.IslandOperationAllowed )
		          {
		          	if (ELM.MainsFailure || ISL.IslandOperationActive)
		          	{
			          
			          	//if (!ELM.BBVoltageIsZero_Flag) // voltage on busbar
			          	if (ELM.BBVoltageIsOK_Flag)
			          	// 
			          	//
			          	//  MVO: not zero is not sufficient for synchronizing
			          	//       This should be Voltage is o.k.
			          	//
			          	//
			    		{
			        		transit(Synchronize);
			        		break;
			    		}
			          	else if (  (ELM.BBVoltageIsZero_Flag)
			          	        && (STOP.actualLevel > 4)
				          		&& (GEN.reg.state == GEN_REGSTATE_UISLAND_DONE)
				          		&& (HVS.L1EIsOffTimer >= (DU32)PARA[ParRefInd[EPF_SWITCH_ON_TIME__PARREFIND]].Value)
			          		)
			        	{
			        		// we have the release from all connected IDs and from DI if assigned
			        		if (HVS.ReleaseCloseBreaker)
			        		{
				        		transit(ConnectT1E); // connect T1E to island operation
				        		break;
			        		}
			        		// if there is no release atfer delay -> transit to "wait for release" and take back the wish to close breaker
			        		else if (PMS.WaitForReleaseCloseBreaker.State == TRIP) // no release
			        		{
				        		transit(WaitForReleaseCloseGCB); // wait for release
				        		break;
			        		}
			        		// else wait for (release / no release)		        		
			        	}
			        	// else voltage on busbar is not within limits -> do nothing
		          	}
		          }
		          // else mains breaker not in a stabil position -> do nothing 
		        }
	        
		    // continuously update TUR rpm setpoint TUR.MAINRpmSet
		    // Why ???
		    /*
		    if (   ( TUR.state == TUR_REGULATE_RPM ) || ( TUR.state == TUR_REGULATE_RPM_DONE ))
		       {
			      TUR.MAINRpmSet    = TUR.NominalSpeed;
			      TUR.mode          = TUR_TAKE_SETPOINT_RPM;            // reach and keep certain speed	
		       }
            */

            if (TUR.SpeedUpDownAssigned)
            {
                TUR.MAINRpmSet = MAIN_RpmSet + TUR.AdditionalRpmSet;		// change rpm setpoint (if SpeedUp or SpeedDown pressed)
            }

		    if ( TUR.state == TUR_REGULATE_RPM_START )
		       {
			      TUR.mode          = TUR_RPM;            // reach and keep certain speed	
		       }
		    
		    // excitatio-on-speed reached
	        if (ENG.S200EngineSpeed >= (DS16)PARA[ParRefInd[GEN_AVR_ACTIVATION_SPEED__PARREFIND]].Value )
	        {
	        	GEN.mode = GEN_MODE_ON;
	        }

	        if (GEN.mode == GEN_MODE_ON)
	        {
				// continuously update GEN.SetpointVoltage
				GEN.SetpointVoltage = GEN_NOMINAL_VOLTAGE;
				if ( GEN.reg.state == GEN_REGSTATE_UISLAND_START )
				{
					GEN.reg.mode = GEN_REGMODE_UISLAND;
				}
				// GEN.reg.state is "regulate" or "regulate done"
				else if (GEN.reg.SetpointVoltage != GEN_NOMINAL_VOLTAGE)
				{
					GEN.reg.mode = GEN_REGMODE_UISLAND_TAKE_SETPOINT;
				}
	        }

		       
		break; // end of regular block SIG_DO
	}
}

static void WaitForReleaseCloseGCB(const DU8 sig) 
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
            AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
            DK.mode       = DK_MODE_ON;          // throttle controller switched on
            ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			// keep it... GEN.mode      = GEN_MODE_ON;         // AVR active
			GEN.SetpointVoltage = GEN_NOMINAL_VOLTAGE; // setpoint = nominal
			GEN.reg.mode  = GEN_REGMODE_UISLAND_TAKE_SETPOINT;	
			HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // waste oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			//STH.mode      = STH_ELECTRIC_DEMANDED; // heating of poil electrically
			SCR.mode	  = SCR_WARM_UP;			 // prepare catalyst system
			THR.mode	  = THR_HEAT_UP;		 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
            
            TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
            
            TUR.mode      = TUR_TAKE_SETPOINT_RPM; // reach and keep speed
            if (!GAS.GasTypeBActive || !GAS.GasTypeBSelected) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
            IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			MIX.mode[MixerInd1] = MIX_MOVE_TO_IDLE_POSITION;
			MIX.mode[MixerInd2] = MIX_MOVE_TO_IDLE_POSITION;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			//HVS.modeL1E          = HVS_L1E_KEEP;       // keep state of 22L1E
					
			// define actual MAIN.state
			MAIN.state                = MAIN_WAIT_FOR_RELEASE_CLOSE_GCB;	

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = TRUE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();

			if (STOP.actualLevel < 3)
			{ 
			  transit(FastBraking); 
			  break;
			}
			
			// else actualLevel >= 3
			
			// Are we in regular stop?
	        if ( !(STOP.actualBitMask & 0x0001) )
	        {
	          // regular stop?
	          transit(FastBraking);
	          break;
	        }
	        
	        // else (STOP.actualLevel >=3) and no regular stop
	        
	        // start not demanded?
	        if (!MAIN.startdemand)
		    { 
		      transit(FastBraking);
		      break;
		    }
	        
	        // else (STOP.actualLevel >=3) and no regular stop and start demanded
	        
	        // engine dying for whatever reason: go back
	        //if ( (ENG.state != ENG_RUNNING) && (!GEN.VoltageInWindow) )
		    if ( (ENG.S200EngineSpeed < PARA[ParRefInd[ENGINE_RUNNING__PARREFIND]].Value/100) && (!GEN.VoltageInWindow) )
	        {
	        	// engine has stopped
	        	STOP_Set(STOPCONDITION_20096);

	          transit(FastBraking);
		      break;
	        }

			// when T1E is connected externally, perform a transit to Grid Parallel operation
		    if (( HVS.stateT1E == HVS_T1E_IS_ON )
		    	&& ( HVS.stateL1E == HVS_L1E_IS_ON )
		    	)
		    {
		      transit(GridParallelOperationLimitedLoad);
		      break;
		    }

			// when T1E is connected externally, perform a transit to island operation
		    if (( HVS.stateT1E == HVS_T1E_IS_ON )
		    	&& ( HVS.stateL1E == HVS_L1E_OFF_AND_READY )
		    	&& ( ISL.IslandOperationAllowed )
		    	)
		    {
		    	// if we are alone in the loadsharing line -> transit to island operation
		    	if (PMS_NoLSMemberConnected() || !PMS.EngineIDConfigured[ARC.nEngineId-1])
		    	{
			    	transit(IslandOperation);
			    	break;
		    	}
		    	else // we are not alone in the loadsharing line -> transit to loadsharing ramp up
		    	{
			    	transit(LoadSharingRampUp);
			    	break;
		    	}
		    }

	        // actualLevel >=4 AND U gen in window AND generator breaker open?
	        
	        
	        if (myStateCnt >= (DU32)(PARA[ParRefInd[ENGINE_ID__PARREFIND]].Value * PMS_WAIT_FOR_RELEASE_DELAY)) // try again
			{
	    		transit(IdleRun);
	    		break;
			}

		    if ( TUR.state == TUR_REGULATE_RPM_START )
		       {
			      TUR.mode          = TUR_RPM;            // reach and keep certain speed	
		       }

		    // excitatio-on-speed reached
	        if (ENG.S200EngineSpeed >= (DS16)PARA[ParRefInd[GEN_AVR_ACTIVATION_SPEED__PARREFIND]].Value )
	        {
	        	GEN.mode = GEN_MODE_ON;
	        }

		    if (GEN.mode == GEN_MODE_ON)
		    {
				// continuously update GEN.SetpointVoltage
				GEN.SetpointVoltage = GEN_NOMINAL_VOLTAGE;
				if ( GEN.reg.state == GEN_REGSTATE_UISLAND_START )
				{
					GEN.reg.mode = GEN_REGMODE_UISLAND;
				}
				// GEN.reg.state is "regulate" or "regulate done"
				else if (GEN.reg.SetpointVoltage != GEN_NOMINAL_VOLTAGE)
				{
					GEN.reg.mode = GEN_REGMODE_UISLAND_TAKE_SETPOINT;
				}
		    }
		    
		       
		break; // end of regular block SIG_DO
	}
}

/* rmiEPF */
// power failure and emergency power functionality => island
static void ConnectT1E(const DU8 sig)
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			
			SAF.mode                  = SAF_GUARD;          // ready for TRIP and protection
			
            AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
            DK.mode       = DK_MODE_ON;          // throttle controller switched on
            ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			GEN.mode      = GEN_MODE_ON;         // AVR active
			//GEN.SetpointVoltage       = TUR_read_setpoint_VoltageIsland();
			GEN.SetpointVoltage = GEN_NOMINAL_VOLTAGE; // setpoint = nominal
			GEN.reg.mode              = GEN_REGMODE_UISLAND_TAKE_SETPOINT;
			ELM.modeMB    = ELM_MB_OPEN;        // mains breaker open
            HVS.modeT1E   = HVS_T1E_ON;			// close generator breaker
            IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_ISLAND;     // island operation, rmiISL
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			MIX.mode[MixerInd1] = MIX_MOVE_TO_ISLAND_POSITION;
			MIX.mode[MixerInd2] = MIX_MOVE_TO_ISLAND_POSITION;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_ON;
			
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // waste oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			SCR.mode	  = SCR_WARM_UP;			 // prepare catalyst system
			THR.mode	  = THR_HEAT_UP;		 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
	
            TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
            // rmiIET  TUR.mode      = TUR_TAKE_SETPOINT_RPM;   // reach and keep certain speed
            	
			WAT.mode      = WAT_ENABLE;          // pumps on
	
			// define actual MAIN.state
			MAIN.state                = MAIN_CONNECT_T1E;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			// keep as it is
			//MAIN.DO_IslandOperation = ???;
			//MAIN.DO_Loadsharing       = ???;
			MAIN.WishToCloseGCB       = TRUE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
					
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// when connection takes more than 10 sec set STC timeout
			if ( myStateCnt > MAIN_CONNECTT1E_TIMEOUT )
			{
			   if ((!ELM.MainsFailure)								// NO mains failure
			        && (!ISL.IslandOperationActive) )				// AND no island demanded)
			   {
			   	  transit(DisconnectT1EIsland);						// mains power failure gone
			   	  break;											// ==> Disconnect / no time out
			   }
			   else
			      STOP_Set(STOPCONDITION_30021);				// (island-) connect time out, rmiEPF
			}
			
			// when any STC prohibits island operation, disconnect T1E
			// if (STOP.actualLevel < 6) {transit(DisconnectT1EIsland);break;}
			if (STOP.actualLevel < 5)
			{
				transit(DisconnectT1EIsland);
				break;
			}

			// Are we in regular stop?
	        if ( !(STOP.actualBitMask & 0x0001) )
	        {
	          transit(DisconnectT1EIsland);
	          break;
	        }

			// else no timeout and level not below 5 and no regular stop
			// voltage ok?
			if (!GEN.VoltageInWindow)
			{
				transit(DisconnectT1EIsland);
				break;
			}

			// else no timeout and level not below 5 and no regular stop and voltage ok
			// mains breaker open?
			if (HVS.stateL1E != HVS_L1E_OFF_AND_READY)
			{
				transit(DisconnectT1EIsland);
				break;
			}
			
			// when regulator is ready to takeover the actual setpoint, activate regulation
			if ( GEN.reg.state == GEN_REGSTATE_UISLAND_START )
			   GEN.reg.mode = GEN_REGMODE_UISLAND;

			// when the breaker is on, transit to local island operation
		    if (HVS.stateT1E == HVS_T1E_IS_ON)
		    {
		    	// if we are alone in the loadsharing line -> transit to island operation
		    	if (PMS_NoLSMemberConnected() || !PMS.EngineIDConfigured[ARC.nEngineId-1])
		    	{
			    	transit(IslandOperation);
			    	break;
		    	}
		    	else // we are not alone in the loadsharing line -> transit to loadsharing ramp up
		    	{
			    	transit(LoadSharingRampUp);
			    	break;		    		
		    	}
		    }

		break; // end of regular block SIG_DO
	}
	
} // end: ConnectT1E


static void Synchronize(const DU8 sig)
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			GEN.mode      = GEN_MODE_ON;         // AVR active
			GEN.reg.mode  = GEN_REGMODE_SYNC;    // type of Regulator for Synchronisation (U=U)
			HVS.modeT1E   = HVS_T1E_ON;          // connect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;           // power fail automatic for mains breaker
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no mixer control in this state
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_WARM_UP;		 // prepare catalyst system
			//STH.mode      = STH_WATER_DEMANDED;  // heating of poil by water
			THR.mode	  = THR_HEAT_UP;		 // thermoreactor heating enabled
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			//TUR.mode      = TUR_SYNC_T1;         // synchronize Voltages Gen.BusBar

			// start with nominal speed (in case of external synchronization)
			TUR.MAINRpmSet= TUR.NominalSpeed;
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			MIX.mode[MixerInd1] = MIX_MOVE_TO_IDLE_POSITION;
			MIX.mode[MixerInd2] = MIX_MOVE_TO_IDLE_POSITION;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_ON;
			
			//HVS.modeL1E   = HVS_L1E_ON;        // keep 22L1E connected 
			
            GEN.AVRManual = FALSE;               // set AVR to auto mode
            TUR.GOVManual = FALSE;               // set GOV to auto mode
			// define actual MAIN.state
			MAIN.state                = MAIN_SYNCHRON_CONNECT_T1E;
			MAIN.T1EisSynchron.Signal = FALSE;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			// keep as it is
			//MAIN.DO_IslandOperation = ???;
			//MAIN.DO_Loadsharing       = ???;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
			MAIN.T1EisSynchron.Signal = FALSE;		// reliable signal, rmi100323
		break; // end of SIG_EXIT

		case SIG_DO : // called every 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			// check external synchronization and set TUR.mode accordingly:
			// change the speed according to digitial inputs, if external synchronization
			if (MAIN.ExternalSynchronization)			// synchronization extern
			{
			   TUR.mode = TUR_SYNC_T1_EXTERN;          	// synchronization extern, take rpm-setpoint from main
               TUR.MAINRpmSet = TUR.NominalSpeed + TUR.AdditionalRpmSet;		// update RpmSet, dependend of SppeedUp, SpeedDown
			}
			else
			{
			   TUR.mode = TUR_SYNC_T1;        			// synchronize
			   TUR.MAINRpmSet = TUR.NominalSpeed;
			} // endif: external synchronization

			// set a STC in case of a timeout under synchronisation
			if ( myStateCnt > (DU32)PARA[ParRefInd[SYNC_TIMEOUT__PARREFIND]].Value )
			STOP_Set(STOPCONDITION_30020);
			
			if (STOP.actualLevel < 4)
			{ 
			  transit(DisconnectT1E); 
			  break;
			}
			
			// else actualLevel >= 4
			
			// Are we in regular stop?
	        if ( !(STOP.actualBitMask & 0x0001) )
	        {
	          transit(DisconnectT1E);
	          break;
	        }
			
			// else actual Level >= 4 and no regular stop
			
			// start demanded?
			if (!MAIN.startdemand)
		    { 
		      transit(DisconnectT1E);
		      break;
		    }

	        // engine dying for whatever reason: go back
	        //if ( (ENG.state != ENG_RUNNING) && (!GEN.VoltageInWindow) )
		    if ( (ENG.S200EngineSpeed < PARA[ParRefInd[ENGINE_RUNNING__PARREFIND]].Value/100) && (!GEN.VoltageInWindow) )
	        {
	        	// engine has stopped
	        	STOP_Set(STOPCONDITION_20096);

	          transit(DisconnectT1E);
		      break;
	        }
		    
		    // mains circuit breaker not closed
		    if ( (HVS.stateL1E != HVS_L1E_IS_ON)
		    	&& (!ISL.IslandOperationAllowed) ) // island operation not allowed
		    {
		    	transit(DisconnectT1E);
		    	break;
		    }  

			// when T1E is connected externally, perform a transit to Grid Parallel operation
		    if (( HVS.stateT1E == HVS_T1E_IS_ON ) 
		    	&& ( HVS.stateL1E == HVS_L1E_IS_ON )
		    	)
		    {
		      transit(GridParallelOperationLimitedLoad);
		      break;
		    }

			// when T1E is connected externally, perform a transit to island operation
		    if (( HVS.stateT1E == HVS_T1E_IS_ON )
		    	&& ( HVS.stateL1E == HVS_L1E_OFF_AND_READY )
		    	&& ( ISL.IslandOperationAllowed )
		    	)
		    {
		    	// if we are alone in the loadsharing line -> transit to island operation
		    	if (PMS_NoLSMemberConnected() || !PMS.EngineIDConfigured[ARC.nEngineId-1])
		    	{
			    	transit(IslandOperation);
			    	break;
		    	}
		    	else // we are not alone in the loadsharing line -> transit to loadsharing ramp up
		    	{
			    	transit(LoadSharingRampUp);
			    	break;		    		
		    	}
		    }

            // else actual Level >= 4 and no regular stop and start demanded and breaker still open

            // generator voltage still in window?
            if (!GEN.VoltageInWindow)
            {
            	//transit(IdleRun);
            	// now first open generator breaker before transit to idle
            	transit(DisconnectT1E);
            	break;
            }
            
            // else actual Level >= 4 and no regular stop and start demanded and breaker still open 
            // and gen voltage still in window
 
 		    if (!ELM.BBVoltageIsOK_Flag) // busbar voltage not ok
		    {
			   	transit(DisconnectT1E);
			   	break;
		    }           
            
	        if (STOP.actualLevel == 4)
	        { // stay in this state
	          break;
	        }
	        
            // else actual Level > 4 and no regular stop and start demanded and breaker still open 
            // and gen voltage still in window
           
			// command to connect breaker when voltages and frequencies are OK - actual phases and voltages cheched in HVS
			if (    ( GEN.state     == GEN_STATE_REGULATE )     // regulation is on
			     && ( GEN.reg.state == GEN_REGSTATE_SYNC_DONE ) // indicates that the voltages match
			     && ( TUR.state     == TUR_REGULATE_SYNCHRONISATION_FINISHED_T1 ) // frequencies match
			     // moved to HVS && ( ELM_syncronization_criteria_is_fullfilled() ) // phase conditions and voltages checked
			     && ( ELM.T1E.SynccheckDfDt.State == TRIP )     // df/dt of deltaf below max limit
			   )  
		    {
			   if (HVS.stateL1E == HVS_L1E_IS_ON) // mains breaker closed
			   {
				   if (ELM.BBVoltageIsOK_Flag) // busbar voltage is nominal
				   {
					   MAIN.T1EisSynchron.Signal = TRUE;
				   }
			   }
			   else if ( (HVS.stateL1E == HVS_L1E_OFF_AND_READY) // mains breaker open
			             && (ISL.IslandOperationAllowed) )
			   {
				   if (ELM.TransformerVoltageIs22kV50Hz_Flag) // generator voltage is nominal
				   {
					   MAIN.T1EisSynchron.Signal = TRUE;
				   }
			   }
		    }
		    else
		    {
		       MAIN.T1EisSynchron.Signal = FALSE;
		     
		    }

		break; // end of regular block SIG_DO
	}
	
}



/* rmiEPF */
static void IslandOperation(const DU8 sig)
{
	DS32 RampChange;
	static DS32 RampRest = 0L;
	static DU8 WaitAfterDeload = 0;

    static DBOOL  MainsDeloInSynchronization;
    static DBOOL  MainsDeloInSynchronizationBefore;
    
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered

            RampRest = 0L;
            MAIN.StopInIsland = FALSE;
			
			// define control for all other components
			
			SAF.mode            = SAF_GUARD;          // ready for TRIP and protection
			
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GBV.mode      = GBV_MODE_ENABLE;
			MIX.EvaluateConditionsToStartMixerControl = TRUE; // mixer control might be requested for the first time since starting
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			WAT.mode      = WAT_ENABLE;          // pumps on
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_ISLAND;     // island operation, rmiISL
			//MIX.mode      = MIX_MOVE_TO_ISLAND_POSITION;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_ON;
			SCR.mode	  = SCR_ENABLE;			 // catalyst system in operation
			//THR.mode	  = keep as is
			//TLB.mode      = TLB_GO_PARALLEL;     // move to predefined position
			//TLB.Regulation_is_ON = FALSE;
			TLB.Sset      = 15000L;

			TLB.mode = TLB_SPEED;
			
            TUR.mode                = TUR_TAKE_SETPOINT_RPM;
            //TUR.MAINPosSet        = Keep
            TUR.MAINRpmSet          = TUR.NominalSpeed;
			TUR.Pset                = 0;
            // Add droop offset
            if (MAIN_ISLAND_PARALLEL_NOT_ACTIVE)
            {
            	if (PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT0)
            		TUR.MAINRpmSet += TUR_CalculateDroopOffset(TUR.MAINRpmSet);
            }
            else if (MAIN_ISLAND_PARALLEL_ACTIVE)
            {
            	if (PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT1)
            		TUR.MAINRpmSet += TUR_CalculateDroopOffset(TUR.MAINRpmSet);
            }
            // else no droop

			//HVS.modeL1E         = HVS_L1E_OFF;        // keep 22L1E disconnected
			ELM.modeMB			= ELM_MB_OPEN;		  // mains breaker open, rmiEPF
			HVS.modeT1E         = HVS_T1E_ON;         // keep 22L1E connected

			GEN.mode            = GEN_MODE_ON;        // Generator magnetized to 690VAC
			//GEN.SetpointVoltage = TUR_read_setpoint_VoltageIsland();
			GEN.SetpointVoltage = GEN_NOMINAL_VOLTAGE; // setpoint = nominal
			GEN.reg.mode        = GEN_REGMODE_UISLAND_TAKE_SETPOINT; 
		
			// define actual MAIN.state
			MAIN.state          = MAIN_ISLAND_OPERATION; 

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = TRUE;
			MAIN.DO_Loadsharing       = TRUE;
			MAIN.WishToCloseGCB       = FALSE;

            // initialize variables
            MainsDeloInSynchronizationBefore = FALSE;
            MAIN.EngineRunningNominalDelayed					= TRUE;
            
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
            MAIN.StopInIsland = FALSE;
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				switch (PMS.CBPOS_OperationMode)
				{
					case PMS_CBPOS_UNDEFINED:
						// undefined breaker position
						transit(UndefinedBreakers);
					break;

					case PMS_CBPOS_BLACK_OPERATION:
						// black operation
						transit(BlackOperation);
					break;

					case PMS_CBPOS_MAINS_OPERATION:
						// mains operation
						transit(MainsOperation);
					break;

					case PMS_CBPOS_PARALLEL_OPERATION:
						// parallel operation
						transit(SynchronConnectL1E);
					break;

					default:
					
					    // problems on ArcNet
					    if (ARC.SafeMode)
						{
							break; // stay here
						}
					
					    // mains failure or island operation
					    if (ELM.MainsFailure || ISL.IslandOperationActive)
						{
							break; // stay here
						}
			
						if (myStateCnt < (DU32)(PARA[ParRefInd[EPF_MAINS_FAILURE_DELAY__PARREFIND]].Value + 1000L))
						{
						   break;  // stay here to wait for (delayed) mains power stop condition	
						}
						// ================================================================================
						
						// island-operation at least 1 second + value of mains-failure delay
						
						// back synchronisation?
						if( PARA[ParRefInd[EPF_RESYNCHRONISING__PARREFIND]].Value )
						{
							if (!(STOP.actualBitMask & STOP_BITMASK_ISLAND))	// isl-bit active, rmi100319
							   break;							// stay here and wait for auto acknowlwdge of mains protection SC's
							else								// rmi100319
							{
							    // busbar frequency ok? (f nom +/- 1Hz)
							    ELM.BBFrequencyOkForSync = (   (ELM.L1E.per.FavgBB >= (DU32)(GEN.NominalFrequency - 1000L) )
							                                && (ELM.L1E.per.FavgBB <= (DU32)(GEN.NominalFrequency + 1000L) ) );

								if (ELM.BBFrequencyOkForSync)
									transit(SynchronConnectL1E);		// only, if no SC with isl-bit is active
							}
							break;
						}
					break;
				}
				break;
			}

			// set mixer mode depending on config and release load
			MAIN_Set_MIX_mode(MIX_MOVE_TO_ISLAND_POSITION);

			if (THR.ReleaseLoad.State >= TRIP)
				THR.mode = THR_ENABLE;
			else
				THR.mode = THR_HEAT_UP;
/*
			if (TLB.Regulation_is_ON)
			{
				TLB.Regulation_is_ON = ELM.ReleaseLoadForTurboBypassControl
									&& (TLB.Regulation_OFF.State <  TRIP);
			}
			else
			{
				TLB.Regulation_is_ON = ELM.ReleaseLoadForTurboBypassControl
									&& (TLB.Regulation_ON.State >=  TRIP);
			}

			if (TLB.Regulation_is_ON)
			{
				TLB.mode = TLB_SPEED;
				//TLB.Pset = TUR.Reg.PowerSetPoint-TUR.OffsetForTurboBypassControl;// put new power setpoint to turbo bypass control

				// throttle controls speed
				TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
				TUR.mode      = TUR_TAKE_SETPOINT_RPM;   // reach and keep certain speed
			}
			else
			{
				TLB.mode = TLB_GO_PARALLEL;
				TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
				TUR.mode      = TUR_TAKE_SETPOINT_RPM;   // reach and keep certain speed
			}
*/
			
			// if local island operation is prohibited, disconnect T1E
			// rmiEPF if (STOP.actualLevel < 6)
			if ( ( STOP.actualLevel < 5 )
				|| (!ENG.Running) )
			{
				CloseThrottle = TRUE;		// close throttle to avoid over speed
				transit(DisconnectT1EIsland);
				break;
			}

		    // generator circuit breaker not closed
		    if (HVS.stateT1E != HVS_T1E_IS_ON)
		    {
			  // open breaker immediately
		      CloseThrottle = TRUE;		// close throttle to avoid over speed
		      transit(DisconnectT1EIsland);
		      break; 
		    }

            if ( ( PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value <= 1)
                 && (HVS.stateL1E == HVS_L1E_IS_ON) )
            {
            	transit(GridParallelOperationFullLoad);
		        break; 
		    }
		    
		    // mains circuit breaker is closed
		    if (HVS.stateL1E == HVS_L1E_IS_ON)
		    {
		      transit(SynchronConnectL1E);
		      break; 
		    }
			
			// else actualLevel >= 5
			
			// regular stop? 
            if (!(STOP.actualBitMask & 0x0001) OR MAIN.StopInIsland OR !MAIN.startdemand)
            {
            	WaitAfterDeload = 5;

				if (MAIN_ISLAND_PARALLEL_ACTIVE)
				{
                	RampChange = (TUR.NominalSpeed+RampRest)/300000L;
                	RampRest = (TUR.NominalSpeed+RampRest)%300000L;

					// Speed ramp down
					if (TUR.MAINRpmSet > TUR.NominalSpeed/10*9)
						TUR.MAINRpmSet -= RampChange;
					// Minimum speed setpoint reached (90% of nominal speed)
					else
					{
						transit(DisconnectT1EIsland);
						return;
					}

					// Check if speed(actual) < speed(cutout)
					if ( ENG.S200EngineSpeed < TUR.NominalSpeed/1000*9 )
					{
						transit(DisconnectT1EIsland);
						return;
					}

					// Check if P(actual) < P(cutout)
					if (ELM.T1E.sec.Psum < (PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value/MAIN_CUT_OUT_FRACTION_ISLPAR))
					{
						transit(DisconnectT1EIsland);
						return;
					}

					if (ELM.MainsFailure || ISL.IslandOperationActive)
						MAIN.StopInIsland = FALSE;

					// Stay here if OpenBreaker is demanded after speed/power ramp down
					break;

				}
				else // Open GCB directly
				{
					transit(DisconnectT1EIsland);
					return;
				}
            }
			else if (WaitAfterDeload > 0)
			{
				RampRest = 0L;
				WaitAfterDeload--;
				break;
			}

			// else actualLevel >= 5 and no regular stop
			
			// if we are not alone in the loadsharing line -> transit to loadsharing

		    if (PMS.EngineIDConfigured[ARC.nEngineId-1])
		    if (PMS_LoadSharingActive())
		    {
		    	transit(LoadSharing);
		    	break;
		    }
		    
            // set speed setpoint
            if (PMS.EngineIDConfigured[ARC.nEngineId - 1])  // engine in power menagement mode
            {
            	// safe old information 
            	MainsDeloInSynchronizationBefore  =  MainsDeloInSynchronization;
            	// evaluate new information 
                MainsDeloInSynchronization = ARC_BackSync;
                
                if (MainsDeloInSynchronization != MainsDeloInSynchronizationBefore)
                {
                	// back sync state of mains Delo has changed
                	if (MainsDeloInSynchronization)
                	{
                		// now in Back sync: lower speed
                		TUR.MAINRpmSet= TUR.NominalSpeed - 1500L;
                		TUR.mode      = TUR_TAKE_SETPOINT_RPM;   // reach and keep certain speed
                	}
                	else
                	{
                		// back sync has stopped - set normal speed again
                		TUR.MAINRpmSet= TUR.NominalSpeed;
                		TUR.mode      = TUR_TAKE_SETPOINT_RPM;   // reach and keep certain speed
                	}
                }
                // else do nothing
            }
            // else no power management -> no setpoint change
            else
            {
                // Update speed setpoint
    		    TUR.MAINRpmSet = TUR.NominalSpeed;
                // Add droop offset
                if (MAIN_ISLAND_PARALLEL_NOT_ACTIVE)
                {
                	if (PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT0)
                		TUR.MAINRpmSet += TUR_CalculateDroopOffset(TUR.MAINRpmSet);
                }
                else if (MAIN_ISLAND_PARALLEL_ACTIVE)
                {
                	if (PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT1)
                		TUR.MAINRpmSet += TUR_CalculateDroopOffset(TUR.MAINRpmSet);
                }
                // else no droop
            }

            // continuously update GEN.SetpointVoltage
            GEN.SetpointVoltage = GEN_NOMINAL_VOLTAGE;
		    if ( GEN.reg.state == GEN_REGSTATE_UISLAND_START )    
		    {
			    GEN.reg.mode = GEN_REGMODE_UISLAND;
		    }
		    // GEN.reg.state is "regulate" or "regulate done"
		    else if (GEN.reg.SetpointVoltage != GEN_NOMINAL_VOLTAGE)
		    {
			    GEN.reg.mode = GEN_REGMODE_UISLAND_TAKE_SETPOINT;
		    }

		    // mains failure or island operation
		    if (ELM.MainsFailure || ISL.IslandOperationActive)
			{
				break; // stay here
			}

			// else actualLevel >= 5 and no regular stop and no mains failure and no island operation



			// ================================================================================
			// rmi,25.08.09
			// when falling back from parallel load into island operation because of
			// a protection function (working on MCB) tripped, then the 
			// SC mains power failure will trip later
			// because of it's delay time (at least 500 ms)
			// -------------------------------------------------------------------------------
			// therefore we stay at least for this time (+ 1000ms) in island operation to avoid
			// o back synchronisation in case of power failure (in this second) and
			// o open the GCB in case of power failure (in this second, 
			//   in case of no back synchronisation is parameterized) 
			// ================================================================================
			// bugfix, 091211: MinValue substituted by Value
			if (myStateCnt < (DU32)(PARA[ParRefInd[EPF_MAINS_FAILURE_DELAY__PARREFIND]].Value + 1000L))
			{
			   break;  // stay here to wait for (delayed) mains power stop condition	
			}
			// ================================================================================
			
			// island-operation at least 1 second + value of mains-failure delay
			
			// back synchronisation?
			if( PARA[ParRefInd[EPF_RESYNCHRONISING__PARREFIND]].Value )
			{
				MAIN.StopInIsland = FALSE;

				if (!(STOP.actualBitMask & STOP_BITMASK_ISLAND))	// isl-bit active, rmi100319
				   break;							// stay here and wait for auto acknowlwdge of mains protection SC's
				else								// rmi100319
				   if ( !(PMS.EngineIDConfigured[ARC.nEngineId - 1]) )
				   transit(SynchronConnectL1E);		// only, if no SC with isl-bit is active
				                                    // and if not power management
				break;
			}
			// no back synchronisation
			else
			{
				MAIN.StopInIsland = TRUE;
			}
	}
	
} // end: IslandOperation

static void LoadSharingRampUp(const DU8 sig)
{
	//DBOOL Down;

	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			
			SAF.mode            = SAF_GUARD;          // ready for TRIP and protection
			
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GBV.mode      = GBV_MODE_ENABLE;
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			WAT.mode      = WAT_ENABLE;          // pumps on
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_ISLAND;     // island operation, rmiISL
			//MIX.mode      = MIX_MOVE_TO_ISLAND_POSITION;
			MIX.EvaluateConditionsToStartMixerControl = TRUE; // mixer control might be requested for the first time since starting
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_ON;
			SCR.mode	  = SCR_ENABLE;			 // catalyst system in operation
			//THR.mode	keep as is
			//TLB.mode      = TLB_GO_PARALLEL;     // move to predefined position
			TLB.Sset      = 15000L;
			TLB.mode = TLB_SPEED;
	
			TUR.Pset      = PMS.RealPowerSetpoint; // Setpoint for P from power management system
			TUR.mode      = TUR_TAKE_SETPOINT_POWER; // reach and keep power
			
			//HVS.modeL1E         = HVS_L1E_OFF;        // keep 22L1E disconnected
			ELM.modeMB			= ELM_MB_OPEN;		  // mains breaker open, rmiEPF
			HVS.modeT1E         = HVS_T1E_ON;         // keep 22L1E connected

			GEN.mode            = GEN_MODE_ON;        // Generator magnetized to 690VAC
			GEN.SetpointVAR     = PMS.BlindPowerSetpoint; // blindpower setpoint for loadsharing
			GEN.reg.mode        = GEN_REGMODE_VAR_TAKE_SETPOINT;

			// define actual MAIN.state
			MAIN.state          = MAIN_LOADSHARING_RAMP_UP; 

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = TRUE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;

		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			// set mixer mode depending on config and release load
			MAIN_Set_MIX_mode(MIX_MOVE_TO_ISLAND_POSITION);

			if (THR.ReleaseLoad.State >= TRIP)
				THR.mode = THR_ENABLE;
			else
				THR.mode = THR_HEAT_UP;

			if (TLB.Regulation_is_ON)
			{
				TLB.Regulation_is_ON = ELM.ReleaseLoadForTurboBypassControl
									&& (TLB.Regulation_OFF.State <  TRIP);
			}
			else
			{
				TLB.Regulation_is_ON = ELM.ReleaseLoadForTurboBypassControl
									&& (TLB.Regulation_ON.State >=  TRIP);
			}

			// state timeout
			if ( myStateCnt > MAIN_LOADSHARING_RAMP_UP_TIMEOUT )
			{
				STOP_Set(STOPCONDITION_50200); // 5R
			}
			
			if ( ( STOP.actualLevel < 5 )
				|| (!ENG.Running) )
			{
				CloseThrottle = TRUE;		// close throttle to avoid over speed
				transit(DisconnectT1EIsland);
				break;
			}

		    // generator circuit breaker not closed
		    if (HVS.stateT1E != HVS_T1E_IS_ON)
		    {
			  // open breaker immediately
		      CloseThrottle = TRUE;		// close throttle to avoid over speed
		      transit(DisconnectT1EIsland);
		      break; 
		    }

		    // mains circuit breaker is closed
		    if (HVS.stateL1E == HVS_L1E_IS_ON)
		    {
		      transit(SynchronConnectL1E);
		      break; 
		    }
			
			// else actualLevel >= 5 and engine running and breakers in right positions
			
			// regular stop? 
			if ( !(STOP.actualBitMask & 0x0001) )
			{
				transit(LoadSharingRampDown);
				break;
			}

			// if we are not demanded in the loadsharing line anymore -> transit to loadsharing ramp down
		    if (!MAIN.startdemand)
		    {
		      transit(LoadSharingRampDown);
		      break; 
		    }

			// else actualLevel >= 5 and no regular stop and start demanded

			// if we are alone in the loadsharing line -> transit to island operation
		    if (PMS_NoLSMemberConnected())
		    {
		    	transit(IslandOperation);
		    	break;
		    }

			// if desired load is reached -> transit to loadsharing
		    if (ELM.T1E.per.Psum >= PMS.RealPowerSetpoint)
		    {
		      transit(LoadSharing);
		      break;
		    }
/*
			// if desired load is reached -> transit to loadsharing
		    if (TUR.state == TUR_REGULATE_POWER_DONE)
		    {
		      transit(LoadSharing);
		      break; 
		    }
*/

			// else we are demanded in the loadsharing line -> stay here until desired load is reached

            // force VAR regulator to continuously takeover the setpoint
            if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
            {
       	         GEN.SetpointVAR  = PMS.BlindPowerSetpoint;
       	         GEN.reg.mode = GEN_REGMODE_VAR;
            }
    
            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
                 GEN.reg.mode = GEN_REGMODE_VAR;

            // force TUR regulation to continuously takeover actual setpoint for Power     
            if ( TUR.state == TUR_REGULATE_POWER_START )
            {
            	TUR.Pset      = PMS.RealPowerSetpoint;
	            TUR.mode      = TUR_POWER;          // reach and keep power	    
            }
            if (( TUR.state == TUR_REGULATE_POWER ) || ( TUR.state == TUR_REGULATE_POWER_DONE ))
            {

            	// always
            	TUR.Pset = PMS.RealPowerSetpoint;
/*
            	if ( PMS.RealPowerSetpointHasChanged )
	            {
					PMS.RealPowerSetpointHasChanged = FALSE;

					Down = (TUR.Pset < ELM.T1E.sec.Psum);

	            	// always
	            	TUR.Pset = PMS.RealPowerSetpoint;

					// only if direction is changing
					if (Down) // old direction was down
					{
						// new direction is up
						if (TUR.Pset > ELM.T1E.sec.Psum)
							TUR.mode = TUR_TAKE_SETPOINT_POWER; // new setpoint
					}
					else //  // old direction was up
					{
						// new direction is down
						if (TUR.Pset < ELM.T1E.sec.Psum)
							TUR.mode = TUR_TAKE_SETPOINT_POWER; // new setpoint
					}
	            }
*/
            }

		break; // end of regular block SIG_DO
	}
} // end: LoadSharingRampUp

static void LoadSharing(const DU8 sig)
{
	
	static DBOOL     MainsDeloInSynchronization;
    static DBOOL     MainsDeloInSynchronizationBefore;
	
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			
			SAF.mode            = SAF_GUARD;          // ready for TRIP and protection
			
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GBV.mode      = GBV_MODE_ENABLE;
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			WAT.mode      = WAT_ENABLE;          // pumps on
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_ISLAND;     // island operation, rmiISL
			//MIX.mode      = MIX_MOVE_TO_ISLAND_POSITION;
			MIX.EvaluateConditionsToStartMixerControl = TRUE; // mixer control might be requested for the first time since starting
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_ON;
	

			
			//HVS.modeL1E         = HVS_L1E_OFF;        // keep 22L1E disconnected
			ELM.modeMB			= ELM_MB_OPEN;		  // mains breaker open, rmiEPF
			HVS.modeT1E         = HVS_T1E_ON;         // keep 22L1E connected

			GEN.mode            = GEN_MODE_ON;        // Generator magnetized to 690VAC
			SCR.mode	  = SCR_ENABLE;			 // catalyst system in operation
			//THR.mode = keep as is
			//TLB.mode      = TLB_GO_PARALLEL;     // move to predefined position
			TLB.Sset      = 15000L;
			TLB.mode = TLB_SPEED;
			
			if (!ARC.SafeMode) // normal sequence
			{
				TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
				TUR.Pset      = PMS_realpower_setpoint();
				TUR.mode      = TUR_TAKE_SETPOINT_LS;

				GEN.SetpointVoltage = GEN_NOMINAL_VOLTAGE;
				GEN.SetpointVAR     = PMS.BlindPowerSetpoint; // blindpower setpoint for loadsharing
				GEN.reg.mode        = GEN_REGMODE_LS_TAKE_SETPOINT;
			}
			else // safe-sequence because of ArcNet problems: droop and voltage droop mode
			{
				TUR.MAINRpmSet= TUR_calculate_speed_setpoint_droop();// calculated setpoint depending on actual active pover
				TUR.mode      = TUR_TAKE_SETPOINT_RPM;

				GEN.SetpointVoltage = GEN_calculate_voltage_setpoint_droop();// calculated setpoint depending on actual reactive pover
				GEN.reg.mode        = GEN_REGMODE_UISLAND_TAKE_SETPOINT;
			}
			
			// define actual MAIN.state
			MAIN.state          = MAIN_LOADSHARING; 

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = TRUE;
			MAIN.DO_Loadsharing       = TRUE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
			
			// set droop mode
			TUR.DroopModeDemanded = TRUE;

		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
			
			// set droop mode
			TUR.DroopModeDemanded = FALSE;
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			// set mixer mode depending on config and release load
			MAIN_Set_MIX_mode(MIX_MOVE_TO_ISLAND_POSITION);

			if (THR.ReleaseLoad.State >= TRIP)
				THR.mode = THR_ENABLE;
			else
				THR.mode = THR_HEAT_UP;

			if (TLB.Regulation_is_ON)
			{
				TLB.Regulation_is_ON = ELM.ReleaseLoadForTurboBypassControl
									&& (TLB.Regulation_OFF.State <  TRIP);
			}
			else
			{
				TLB.Regulation_is_ON = ELM.ReleaseLoadForTurboBypassControl
									&& (TLB.Regulation_ON.State >=  TRIP);
			}

			if ( ( STOP.actualLevel < 5 )
				|| (!ENG.Running) )
			{
				CloseThrottle = TRUE;		// close throttle to avoid over speed
				transit(DisconnectT1EIsland);
				break;
			}

		    // generator circuit breaker not closed
		    if (HVS.stateT1E != HVS_T1E_IS_ON)
		    {
			  // open breaker immediately
		      CloseThrottle = TRUE;		// close throttle to avoid over speed
		      transit(DisconnectT1EIsland);
		      break; 
		    }

            if ( ( PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value <= 1)
                 && (HVS.stateL1E == HVS_L1E_IS_ON) )
            {
            	transit(GridParallelOperationFullLoad);
		        break; 
		    }
		    
		    // mains circuit breaker is closed
		    if (HVS.stateL1E == HVS_L1E_IS_ON)
		    {
		      transit(SynchronConnectL1E);
		      break; 
		    }
			
			// else actualLevel >= 5 and engine running and breakers in right positions
			
			// regular stop? 
			if ( !(STOP.actualBitMask & 0x0001) )
			{
				transit(LoadSharingRampDown);
				break;
			}

			// if we are not demanded in the loadsharing line anymore -> transit to loadsharing ramp down
		    if (!MAIN.startdemand)
		    {
		      transit(LoadSharingRampDown);
		      break; 
		    }

			// else actualLevel >= 5 and no regular stop and start demanded

			// if we are alone in the loadsharing line -> transit to island operation
		    if (PMS_NoLSMemberConnected())
		    {
		      transit(IslandOperation);
		      break; 
		    }

			// else we are demanded in the loadsharing line -> stay here

            // set speed setpoint
            if (PMS.EngineIDConfigured[ARC.nEngineId - 1])  // engine in power menagement mode
            {
            	// safe old information 
            	MainsDeloInSynchronizationBefore  =  MainsDeloInSynchronization;
            	// evaluate new information 
                MainsDeloInSynchronization = ARC_BackSync;
                
                if (MainsDeloInSynchronization != MainsDeloInSynchronizationBefore)
                {
                	// back sync state of mains Delo has changed
                	if (MainsDeloInSynchronization)
                	{
                		// now in Back sync: lower speed
                		TUR.MAINRpmSet= TUR.NominalSpeed - 1500L;
                		TUR.Pset      = PMS_realpower_setpoint();
			            TUR.mode      = TUR_TAKE_SETPOINT_LS;
                	}
                	else
                	{
                		// back sync has stopped - set normal speed again
                		TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
			            TUR.Pset      = PMS_realpower_setpoint();
			            TUR.mode      = TUR_TAKE_SETPOINT_LS;
                	}
                }
                // else no change, do nothing
            }
            // else no power management -> no setpoint change
            
            if (!ARC.SafeMode) // normal sequence
            {
	            // force VAR regulator to continuously takeover the setpoint
	            if (( GEN.reg.state == GEN_REGSTATE_LS )||( GEN.reg.state == GEN_REGSTATE_LS_DONE ))
	            {
	       	         GEN.SetpointVAR  = PMS.BlindPowerSetpoint;
	       	         GEN.reg.mode = GEN_REGMODE_LS;
	            }
	    
	            if ( GEN.reg.state == GEN_REGSTATE_LS_START )
	                 GEN.reg.mode = GEN_REGMODE_LS;
	
	            // force TUR regulation to continuously takeover actual setpoint for power and speed from PMS
	            if ( TUR.state == TUR_REGULATE_LS_START )
	            {
		            TUR.mode      = TUR_LS;         
	            }
            }
            else // safe-sequence because of ArcNet problems: droop and voltage droop mode
            {
	            // continuously update GEN.SetpointVoltage
	            GEN.SetpointVoltage = GEN_calculate_voltage_setpoint_droop();// calculated setpoint depending on actual reactive pover
			    if ( GEN.reg.state == GEN_REGSTATE_UISLAND_START )
			    {
				    GEN.reg.mode = GEN_REGMODE_UISLAND;
			    }
            
	            // continuously update TUR.MAINRpmSet
	            TUR.MAINRpmSet = TUR_calculate_speed_setpoint_droop();
	            if ( TUR.state == TUR_REGULATE_RPM_START )
	            {
		            TUR.mode = TUR_RPM;
	            }
            }
            
            TUR.Pset      = PMS_realpower_setpoint();
            

		break; // end of regular block SIG_DO
	}
} // end: LoadSharing

static void LoadSharingRampDown(const DU8 sig)
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			
			SAF.mode            = SAF_GUARD;          // ready for TRIP and protection
			
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GBV.mode      = GBV_MODE_ENABLE;
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			WAT.mode      = WAT_ENABLE;          // pumps on
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_ISLAND;     // island operation, rmiISL
			//MIX.mode      = MIX_MOVE_TO_ISLAND_POSITION;
			// keep MIX.EvaluateConditionsToStartMixerControl as is
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_ON;
			SCR.mode	  = SCR_ENABLE;			 // catalyst system in operation
			//TLB.mode      = TLB_GO_PARALLEL;     // move to predefined position
			//THR.mode = keep as is
			TLB.Sset      = 15000L;
			TLB.mode = TLB_SPEED;
	
			TUR.Pset      = 0L; // Setpoint for P from power management system
			TUR.mode      = TUR_TAKE_SETPOINT_POWER; // reach and keep power
			
			//HVS.modeL1E         = HVS_L1E_OFF;        // keep 22L1E disconnected
			ELM.modeMB			= ELM_MB_OPEN;		  // mains breaker open, rmiEPF
			HVS.modeT1E         = HVS_T1E_ON;         // keep 22L1E connected

			GEN.mode            = GEN_MODE_ON;        // Generator magnetized to 690VAC
			GEN.SetpointVAR     = PMS.BlindPowerSetpoint; // blindpower setpoint for loadsharing
 			GEN.reg.mode        = GEN_REGMODE_VAR_TAKE_SETPOINT;
		
			// define actual MAIN.state
			MAIN.state          = MAIN_LOADSHARING_RAMP_DOWN; 

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = TRUE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;

		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}

			// set mixer mode depending on config and release load
			MAIN_Set_MIX_mode(MIX_MOVE_TO_ISLAND_POSITION);

			if (THR.ReleaseLoad.State >= TRIP)
				THR.mode = THR_ENABLE;
			else
				THR.mode = THR_HEAT_UP;

			if (TLB.Regulation_is_ON)
			{
				TLB.Regulation_is_ON = ELM.ReleaseLoadForTurboBypassControl
									&& (TLB.Regulation_OFF.State <  TRIP);
			}
			else
			{
				TLB.Regulation_is_ON = ELM.ReleaseLoadForTurboBypassControl
									&& (TLB.Regulation_ON.State >=  TRIP);
			}

			// state timeout
			if ( myStateCnt > MAIN_LOADSHARING_RAMP_DOWN_TIMEOUT )
			{
				STOP_Set(STOPCONDITION_50201); // 5R
			}
			
			if ( ( STOP.actualLevel < 5 )
				|| (!ENG.Running) )
			{
				CloseThrottle = TRUE;		// close throttle to avoid over speed
				transit(DisconnectT1EIsland);
				break;
			}

		    // generator circuit breaker not closed
		    if (HVS.stateT1E != HVS_T1E_IS_ON)
		    {
			  // open breaker immediately
		      CloseThrottle = TRUE;		// close throttle to avoid over speed
		      transit(DisconnectT1EIsland);
		      break; 
		    }

		    // mains circuit breaker is closed
		    if (HVS.stateL1E == HVS_L1E_IS_ON)
		    {
		      transit(SynchronConnectL1E);
		      break; 
		    }
			
			// else actualLevel >= 5 and engine running and breakers in right positions
			
			// no regular stop and we are demanded in the loadsharing line again  -> transit to loadsharing ramp up
			if ( (STOP.actualBitMask & 0x0001) && MAIN.startdemand)
			{
				transit(LoadSharingRampUp);
				break;
			}

			// else actualLevel >= 5 and (regular stop or no start demand)
			// -> keep deloading

			// if we are alone in the loadsharing line -> transit to island operation
		    if (PMS_NoLSMemberConnected())
		    {
		      transit(IslandOperation);
		      break; 
		    }

		    // if desired cut out load is reached -> transit to DisconnectT1EIsland
		    if ( ELM.T1E.sec.Psum < (DS32)( PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value / MAIN_CUT_OUT_FRACTION_LOADSHARING) )
			{
		      transit(DisconnectT1EIsland);
		      break; 
			}

            // force VAR regulator to continuously takeover the setpoint
            if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
            {
       	         GEN.SetpointVAR  = PMS.BlindPowerSetpoint;
       	         GEN.reg.mode = GEN_REGMODE_VAR;
            }
    
            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
                 GEN.reg.mode = GEN_REGMODE_VAR;

            // force TUR regulation to continuously takeover 0kW    
            if ( TUR.state == TUR_REGULATE_POWER_START )
            {
            	
            	TUR.Pset      = 0L;                 // set power setpoint to 0 kW
	            TUR.mode      = TUR_POWER;          // reach and keep power	    
            }

		break; // end of regular block SIG_DO
	}
} // end: LoadSharingRampDown




/* rmiEPF */
static void SynchronConnectL1E(const DU8 sig)
{
	DU32 BusbarFrequency;
	
		switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			
			SAF.mode      = SAF_GUARD;          // ready for TRIP and protection
			
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GBV.mode      = GBV_MODE_ENABLE;
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			WAT.mode      = WAT_ENABLE;          // pumps on
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_ISLAND;     // island operation, rmiISL
			MIX.mode[MixerInd1] = MIX_MOVE_TO_ISLAND_POSITION;	// stay in island position, rmiEPF
			MIX.mode[MixerInd2] = MIX_MOVE_TO_ISLAND_POSITION;	// stay in island position
			// keep MIX.EvaluateConditionsToStartMixerControl as it is
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_ON;
			SCR.mode	  = SCR_ENABLE;			 // catalyst system in operation
			//THR.mode = keep as is
			TLB.mode      = TLB_GO_PARALLEL;     // move to predefined position
			
			TUR.mode      = TUR_SYNC_L1;        // regulate speed to synchronize voltages across L1
			ELM.modeMB	  = ELM_MB_CLOSE;		// close mains breaker, rmiEPF
			HVS.modeT1E   = HVS_T1E_ON;         // keep T1E connected
			
			GEN.mode      = GEN_MODE_ON;        // Generator magnetized to 690VAC
			GEN.reg.mode  = GEN_REGMODE_SYNC_L1E;   // type of Regulator for Synchronisation across L1E 
	
			// define actual MAIN.state
			MAIN.state    = MAIN_SYNCHRON_CONNECT_L1E;
			//MAIN.subState = SUBSTATE_START;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			// keep as it is
			//MAIN.DO_IslandOperation = ???;
			//MAIN.DO_Loadsharing       = ???;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
		
		break; // end of SIG_ENTRY
		case SIG_EXIT: // called, when this state is left
			MAIN.L1EisSynchron.Signal = FALSE;			// reliable signal, rmi100323
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// set a STC in case of a timeout under synchronisation
			if ( myStateCnt > MAIN_SYNCHRON_CONNECT_L1E_TIMEOUT )
			{
				STOP_Set(STOPCONDITION_30022);			// synchron connect timeout, rmiEPF
			}

		    if (PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value > 1)
		    	BusbarFrequency = ELM.L1E.per.FavgBB;
		    else
		    	BusbarFrequency = ELM.T1E.per.FavgBB;
		    
		    // busbar frequency ok? (f nom +/- 1Hz)
		    ELM.BBFrequencyOkForSync = (   (BusbarFrequency >= (DU32)(GEN.NominalFrequency - 1000L) )
		                                && (BusbarFrequency <= (DU32)(GEN.NominalFrequency + 1000L) ) );
			
			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				switch (PMS.CBPOS_OperationMode)
				{
					case PMS_CBPOS_UNDEFINED:
						// undefined breaker position
						transit(UndefinedBreakers);
					break;

					case PMS_CBPOS_BLACK_OPERATION:
						// black operation
						transit(DisconnectL1EtoIsland);
					break;

					case PMS_CBPOS_MAINS_OPERATION:
						// mains operation
						transit(MainsOperation);
					break;

					case PMS_CBPOS_PARALLEL_OPERATION:
						// parallel operation
						transit(GridParallelOperationFullLoad);
					break;

					default:
					break;
				}
				if (!(STOP.actualBitMask & STOP_BITMASK_ISLAND))  // isl-bit in actualBitMask
				{
				  transit(DisconnectL1EtoIsland); 
				  break;
				}
			    // island demanded?
			    if (ISL.IslandOperationActive)
			    {
			      transit(DisconnectL1EtoIsland);
			      break;
			    }
			    // mains failure?
			    if (ELM.MainsFailure)
			    {
			      transit(DisconnectL1EtoIsland);
			      break;
			    }

			    if (!ELM.BBFrequencyOkForSync)
			    {
			      transit(DisconnectL1EtoIsland);
			      break;
			    }

			    if (ARC.SafeMode)
			    {
			      transit(DisconnectL1EtoIsland);
			      break;
			    }
			    
			    // no back synchronizing?
			    if (PARA[ParRefInd[EPF_RESYNCHRONISING__PARREFIND]].Value == 0)
			    {
			      transit(DisconnectL1EtoIsland);
			      break;
			    }
			    
				// command to connect L1E when Voltage and Phase are OK
				if (    // moved to HVS ( ELM_L1E_syncronization_criteria_is_fullfilled()) &&
				     ( ELM.L1E.SynccheckDfDt.State == TRIP )     // df/dt of delta below max limit
				   )  
				{
					MAIN.L1EisSynchron.Signal = TRUE;
				}
				else
				{
					MAIN.L1EisSynchron.Signal = FALSE;
				}
				break;
			}

			if (THR.ReleaseLoad.State >= TRIP)
				THR.mode = THR_ENABLE;
			else
				THR.mode = THR_HEAT_UP;

			// do we want to stay in island operation, because isl-bit is set ?  rmi100317
			// e.g.: this is the case if we have a MCB closing failure (SC 50622/50624)
			if (!(STOP.actualBitMask & STOP_BITMASK_ISLAND))  // isl-bit in actualBitMask
			{
			  transit(DisconnectL1EtoIsland); 
			  break;
			}
			
			// STOP.actualLevel < 4?
			if (STOP.actualLevel < 4)
			{ 
			  transit(DisconnectL1EtoIsland); 
			  break;
			}

			// else actualLevel >= 4
			
		    // island demanded?
		    if (ISL.IslandOperationActive)
		    {
		      transit(DisconnectL1EtoIsland);
		      break;
		    }

			// else actualLevel >= 4 and no island demanded

		    // mains failure?
		    if (ELM.MainsFailure)
		    {
		      transit(DisconnectL1EtoIsland);
		      break;
		    }
				
			// else actualLevel >= 4 and no island demanded and no mains failure and no closing failure
		    
		    if (!ELM.BBFrequencyOkForSync)
		    {
		      transit(DisconnectL1EtoIsland);
		      break;
		    }

		    // no back synchronizing?
		    if (PARA[ParRefInd[EPF_RESYNCHRONISING__PARREFIND]].Value == 0)
		    {
		      transit(DisconnectL1EtoIsland);
		      break;
		    }
			
			// command to connect L1E when Voltage and Phase are OK
			if (    ( GEN.state     == GEN_STATE_REGULATE )
			     && ( GEN.reg.state == GEN_REGSTATE_SYNC_L1E_DONE )
			     && ( TUR.state     == TUR_REGULATE_SYNCHRONISATION_FINISHED_L1 )
			     // moved to HVS && ( ELM_L1E_syncronization_criteria_is_fullfilled())
			     && ( ELM.L1E.SynccheckDfDt.State == TRIP )     // df/dt of delta below max limit
			   )
			   {
			   	   MAIN.L1EisSynchron.Signal = TRUE;
			   }
			   else
			   {
			     MAIN.L1EisSynchron.Signal = FALSE;

			   }
			
			// see if a condition requires stop of synchron connection T1E
			// rmiEPF if (STOP.actualLevel < 8) {transit(DisconnectL1E);break;}
			
			
		    if ( (HVS.stateL1E == HVS_L1E_IS_ON)
		        && (PARA[ParRefInd[EPF_RESYNCHRONISING__PARREFIND]].Value == EPF_RESYNCHRONISING_OVERLAP)) //ÜSy
		    {
		    	transit(DisconnectT1E);
		    	break;
		    }

		    // if mains breaker closed perform parallel operation, rmiEPF
			// when L1E is connected externally, perform a transit to Grid Parallel operation
		    if (HVS.stateL1E == HVS_L1E_IS_ON)
		    {
		    	transit(GridParallelOperationLimitedLoad);
		    	break;
		    }

		break; // end of regular block SIG_DO
	}
	
} // end: SynchronConnectL1E




/* rmiEPF */
// T1E on, disconnect mains breaker
static void DisconnectL1EtoIsland(const DU8 sig)
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			
			SAF.mode                  = SAF_GUARD;          // ready for TRIP and protection
			
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			//GBV.mode      = GBV_MODE_XXX; keep as it is
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			WAT.mode      = WAT_ENABLE;          // pumps on
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_ISLAND;     // island operation, rmiISL
			MIX.mode[MixerInd1] = MIX_MOVE_TO_ISLAND_POSITION;
			MIX.mode[MixerInd2] = MIX_MOVE_TO_ISLAND_POSITION;
			//keep MIX.EvaluateConditionsToStartMixerControl as it is
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_ON;
			SCR.mode	  = SCR_ENABLE;			 // catalyst system in operation
			//THR.mode = keep as is
			//TLB.mode      = ???
	
            TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
            TUR.mode      = TUR_TAKE_SETPOINT_RPM;   // reach and keep certain speed
			
			//HVS.modeL1E               = HVS_L1E_OFF;        // disconnect 22L1E
			ELM.modeMB				  = ELM_MB_OPEN;		// disconnect mains breaker, rmiEPF
			HVS.modeT1E               = HVS_T1E_ON;         // keep T1E connected

			GEN.SetpointVoltage       = ELM.T1E.sec.UdAvg;
			GEN.reg.mode              = GEN_REGMODE_UISLAND_TAKE_SETPOINT;
			GEN.mode                  = GEN_MODE_ON; 
			
			// define actual MAIN.state
			MAIN.state                = MAIN_DISCONNECT_L1E_TO_ISLAND;
			//MAIN.subState             = SUBSTATE_START;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			// keep as it is
			//MAIN.DO_IslandOperation = ???;
			//MAIN.DO_Loadsharing       = ???;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
					
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed;

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// STOP.actualLevel < 4?
			if (STOP.actualLevel < 4)
			{ 
			  transit(DisconnectT1E); 
			  break;
			}

			// else actualLevel >= 4

			if (THR.ReleaseLoad.State >= TRIP)
				THR.mode = THR_ENABLE;
			else
				THR.mode = THR_HEAT_UP;

			// timeout open mains breaker?
			if( STOP_is_Set(STOPCONDITION_50623) )
			{
			   transit(DisconnectT1E);
			   break;
			}
			
			// else actualLevel >= 4  and no timeout open mains breaker
            
            // If there is no second SCM board the mains breaker will not open anyway.
            if ( ( PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value <= 1)
                 && (HVS.stateL1E == HVS_L1E_IS_ON) )
            {
            	transit(GridParallelOperationLimitedLoad);
		        break; 
		    }
		    
			// tell AVR to takeover setpoint
			if ( GEN.reg.state == GEN_REGSTATE_UISLAND_START )
			    GEN.reg.mode = GEN_REGMODE_UISLAND;
			
			
			// when L1E is OFF and the voltage has reached island level, transit to Local Island operation
		    //if (( HVS.stateL1E == HVS_L1E_OFF_AND_READY ) || ( HVS.stateL1E == HVS_L1E_DETECT_STATE ))
		    // if mains breaker disconnected, rmiEPF
		    if ( ( HVS.stateL1E == HVS_L1E_WAIT_AFTER_DISCONNECT ) || ( HVS.stateL1E == HVS_L1E_OFF_AND_READY ) )
		       {
		       	   // if we are alone in the loadsharing line -> transit to island operation
		       	   if (PMS_NoLSMemberConnected() || !PMS.EngineIDConfigured[ARC.nEngineId-1])
		       	   {
			       	   transit(IslandOperation);
			       	   break;
		       	   }
		       	   else // we are not alone in the loadsharing line and PMS decides to share load -> transit to loadsharing
		       	   {
		       		   // -> LS ramp up/down?
		       		   // if power far below load sharing setpoint go to load sharing ramp up
		       		   if (ELM.T1E.per.Psum < PMS.RealPowerSetpoint - MAIN_ACCEPTABLE_POWER_DEVIATION)
		       		   {
		       			   transit(LoadSharingRampUp);
		       			   break;
		       		   }
		       		   else if(ELM.T1E.per.Psum > PMS.RealPowerSetpoint + MAIN_ACCEPTABLE_POWER_DEVIATION)
		       		   // if power far above load sharing setpoint go to load sharing ramp down
					   {
						   transit(LoadSharingRampDown);
					       break;
					   }
					   else
					   {
						   // if power similar to setpoint go directly to load sharing
						   transit(LoadSharing);
						   break;
					   }
		       	   }
		       }
           
           
		break; // end of regular block SIG_DO
	}
	
} // end: DisconnectL1EtoIsland


DS32 MAIN_realpower_max_allowed(void)
{
	DS32 PowerMax;
	DS32 PowerMaxEngine;
	// calculate power reduction max power due to
	// engine cooling water temperature T201
	// too high cylinder temperature
	// oil temperature T208
	// Check the reason why engine was stopped, rmiSTE
	PowerMax = PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value;
	PowerMaxEngine = PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value;
	MAIN.reduction = MAIN_NO_REDUCTION;
	MAIN.StopEngine = MAIN_NO_REDUCTION;		// rmiSTE
	if (!MIX.Config) // no load reduction if mixer is in configuration
	{
		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		// FIRST CALCULATE MAX POWER DUE TO ENGINE PROTECTIONS
		/////////////////////////////////////////////////////////////////////////////////////////////////////////

		// power reduction engine cooling water temperature T202
		if (PowerMax > ENG.MaxPower_T202)
		{
			PowerMax = ENG.MaxPower_T202;
			MAIN.reduction = MAIN_COOLING_WATER;
		}
		
#if (OPTION_CYLINDER_MONITORING == TRUE)
		// power reduction too high exhaust cylinder temperature
		if (PowerMax > CYL.MaxPower)
		{
			PowerMax = CYL.MaxPower;
			MAIN.reduction = MAIN_EXHAUST_CYLINDER_TEMP;
		}
#endif // OPTION_CYLINDER_MONITORING

		// power reduction too high exhaust temperature A
		if (PowerMax > EXH.MaxPowerA)
		{
			PowerMax = EXH.MaxPowerA;
			MAIN.reduction = MAIN_EXHAUST_TEMP_A;
		}
		// StopEngine, because of too high exhaust temperature A, rmiSTE
		//if (STOP_is_Set(STOPCONDITION_50154))
		//    MAIN.StopEngine = MAIN_EXHAUST_TEMP_A;

		// power reduction too high exhaust temperature B
		if (PowerMax > EXH.MaxPowerB)
		{
			PowerMax = EXH.MaxPowerB;
			MAIN.reduction = MAIN_EXHAUST_TEMP_B;
		}
		// StopEngine, because of too high exhaust temperature B, rmiSTE
		//if (STOP_is_Set(STOPCONDITION_50155))
		//    MAIN.StopEngine = MAIN_EXHAUST_TEMP_B;

		// power reduction oil temperature T208
		if (PowerMax > ENG.MaxPower)
		{
			PowerMax = ENG.MaxPower;
			MAIN.reduction = MAIN_OIL_TEMP;
		}

		// power reduction too high receiver temperature
		if (PowerMax > MIX.MaxPower_Temp)
		{
			PowerMax = MIX.MaxPower_Temp;
			MAIN.reduction = MAIN_RECEIVER_TEMP;
		}

		// power reduction due to misfirings
		if (PowerMax > ELM.MaxPower_Misfiring)
		{
			PowerMax = ELM.MaxPower_Misfiring;
			MAIN.reduction = MAIN_MISFIRE;
		}

		// StopEngine, because of misfirings (corrected MVO 10.09.2009)
		if (STOP_is_Set(STOPCONDITION_50679))
			MAIN.StopEngine = MAIN_MISFIRE;

		// power reduction due to throttle position
		if (PowerMax > DK.MaxPower)
		{
			PowerMax = DK.MaxPower;
			MAIN.reduction = MAIN_THROTTLE;
		}

		// Power reduction because of max power from AKR
		if (PowerMax > AKR.MaxPower)
		{
			PowerMax = AKR.MaxPower;
			MAIN.reduction = MAIN_MAXPOWER_AKR;
		}

		// Power reduction because of max power from MIX
		if (PowerMax > MIX.MaxPower)
		{
			PowerMax = MIX.MaxPower;
			MAIN.reduction = MAIN_MAXPOWER_MIX;
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		// REMEMBER MAX POWER DUE TO ENGINE PROTECTIONS
		/////////////////////////////////////////////////////////////////////////////////////////////////////////


		// Used as Shutdown-Condition during power ramp is stopped by GBV
		PowerMaxEngine = PowerMax;

		/////////////////////////////////////////////////////////////////////////////////////////////////////////
		// NOW CALCULATE MAX POWER DUE TO REGULATIONS
		/////////////////////////////////////////////////////////////////////////////////////////////////////////

	   if (PMS.EngineIDConfigured[ARC.nEngineId-1])
	   {
#if (CLIENT_VERSION != IET)
		   // reduction by PMS
		   // In IET version PMS is a setpoint, not a reduction
		   if (PowerMax > PMS.OwnPowerSetPoint)
		   {
		   	   PowerMax = PMS.OwnPowerSetPoint;
		       MAIN.reduction = MAIN_PMS;
		   }
#endif
		   // stop by PMS
		   if (  (!PMS.StartDemand)
		      && (STOP.actualLevel >= 5)
		      && (STOP.actualBitMask & 0x0001) )
		   {
		   	   MAIN.StopEngine = MAIN_PMS;
		   }
	   }


	    if (!PMS.EngineIDConfigured[ARC.nEngineId-1] // no PMS function
	    	|| (PARA[ParRefInd[PMS_REG_CH4__PARREFIND]].Value == 0L)) // no regulation from PMS
	    {
		    // power reduction due to low CH4 value
		    if (PowerMax > CH4.MaxPower_CH4)
		    {
		    	PowerMax = CH4.MaxPower_CH4;
		    	MAIN.reduction = MAIN_CH4;
		    }    
	    }

	    // StopEngine, because of CH4-stoplimit, rmiSTE
		if (STOP_is_Set(STOPCONDITION_50167))
		    MAIN.StopEngine = MAIN_CH4;
	    
	    if (!PMS.EngineIDConfigured[ARC.nEngineId-1] // no PMS function
	    	|| (PARA[ParRefInd[PMS_REG_MAINS_POWER__PARREFIND]].Value == 0L)) // no regulation from PMS
	    {
		    // power reduction due to mains power
		    if (PowerMax > MPI.MaxPower)
		    {
		    	PowerMax = MPI.MaxPower;
		    	MAIN.reduction = MAIN_MPI;
		    }
	    }
	    
	    // StopEngine, because of mains power, rmiSTE
		    if (STOP_is_Set(STOPCONDITION_50170))
		        MAIN.StopEngine = MAIN_MPI;

	    // power reduction due to high frequency
	    if (PowerMax > ELM.MaxPower_Frequency)
	    {
	    	PowerMax = ELM.MaxPower_Frequency;
	    	MAIN.reduction = MAIN_FREQUENCY;
	    }

		    // power reduction due to high voltage
		    if (PowerMax > ELM.MaxPower_Voltage)
		    {
		    	PowerMax = ELM.MaxPower_Voltage;
		    	MAIN.reduction = MAIN_VOLTAGE;
		    }

	    if (!PMS.EngineIDConfigured[ARC.nEngineId-1] // no PMS function
	    	|| (PARA[ParRefInd[PMS_REG_GAS_LEVEL__PARREFIND]].Value == 0L)) // no regulation from PMS
	    {
		    // power reduction due to gas level
		    if (PowerMax > GPC.MaxPower)
		    {
		    	PowerMax = GPC.MaxPower;
		    	MAIN.reduction = MAIN_GAS_LEVEL;
		    }
	    }
	    
	    // power reduction due to digital input 30%
	    if (PowerMax > ENG.MaxPower_DI_30Percent)
	    {
	    	PowerMax = ENG.MaxPower_DI_30Percent;
	    	MAIN.reduction = MAIN_LR_30_PERCENT;
	    }

	    // power reduction due to digital input 60%
	    if (PowerMax > ENG.MaxPower_DI_60Percent)
	    {
	    	PowerMax = ENG.MaxPower_DI_60Percent;
	    	MAIN.reduction = MAIN_LR_60_PERCENT;
	    }
	    
	    // StopEngine, because of gas level, rmiSTE
		if (STOP_is_Set(STOPCONDITION_50093))
		    MAIN.StopEngine = MAIN_GAS_LEVEL;
	    
	    // power reduction limited load
	    if ( (MAIN.state == MAIN_GRID_PARALLEL_LIMITED_LOAD)
	    	&& (PowerMax > (PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value/1000L*PARA[ParRefInd[POWER_WARMING_LOAD__PARREFIND]].Value)) )
	    {
	    	PowerMax = (PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value/1000L*PARA[ParRefInd[POWER_WARMING_LOAD__PARREFIND]].Value);
	    	MAIN.reduction = MAIN_WARMING;
	    }
	    
	    // StopEngine, because of heat control, rmiSTE
	    if (STOP_is_Set(STOPCONDITION_50681))
	        MAIN.StopEngine = MAIN_HEAT_CONTROL;
	    
	    // StopEngine, because of mains power failure, rmiSTE
	    if (   (STOP_is_Set(STOPCONDITION_30650))		// under frequency
	        || (STOP_is_Set(STOPCONDITION_30651))		// under voltage
	        || (STOP_is_Set(STOPCONDITION_30652))		// under voltage star
	        || (STOP_is_Set(STOPCONDITION_30653))		// over frequency
	        || (STOP_is_Set(STOPCONDITION_30654))		// over voltage
	        || (STOP_is_Set(STOPCONDITION_30655))		// over voltage star
	       )
	       MAIN.StopEngine = MAIN_MAINS_POWER_FAILURE;
	   
	   // Power reduction because of max power for gas type A
	   if (PowerMax > GAS.MaxPowerAuxA)
	   {
	   	   PowerMax = GAS.MaxPowerAuxA;
	       MAIN.reduction = MAIN_MAXPOWER_GAS_A;
	   }
	   
	   // Power reduction because of max power for gas type B
	   if (PowerMax > GAS.MaxPowerAuxB)
	   {
	   	   PowerMax = GAS.MaxPowerAuxB;
	       MAIN.reduction = MAIN_MAXPOWER_GAS_B;
	   }

	   // Power reduction because of max power for gas blending
	   if (PowerMax > GBV.MaxPower)
	   {
	   	   PowerMax = GBV.MaxPower;
	       MAIN.reduction = MAIN_MAXPOWER_GBV;
	   }

	   // Battery undervoltage
	   if (!ENG.Running && (PARA[ParRefInd[AUD_BATT_VOLT_TIMER__PARREFIND]].Value > 0L))
		   MAIN.StopEngine = MAIN_BATTERY;

	   // Power reduction because of power limitation by analog input
	   if (PowerMax > TUR.PowerLimit_Filtered)
	   {
	   	   PowerMax = TUR.PowerLimit_Filtered;
	       MAIN.reduction = MAIN_POWER_LIMITATION;
	   }
	}

	MAIN.MaxPowerDueToEngineProtections = PowerMaxEngine;

    return (PowerMax);
}

#if (CLIENT_VERSION == IET)
// limit manual adjustment of power setpoint DS32 [W]
DS32 MAIN_limit_ManualPowerSetpoint(DS32 adjustment)
{
	DS32 Limitation_Due_To_PMS;
	DS32 ReturnValue;

	if (PMS.EngineIDConfigured[ARC.nEngineId-1]) // take power setpoint from PMS
	{
		// set power to PMS demand
		Limitation_Due_To_PMS = PMS.OwnPowerSetPoint;
		//Power = Limitation_Due_To_PMS;
	}
	else
	{
		// no PMS -> no limitation due to PMS
		Limitation_Due_To_PMS = PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value;
	}

	if (adjustment > Limitation_Due_To_PMS)
		adjustment = Limitation_Due_To_PMS;
	if (adjustment < 0L)
		adjustment = 0L;

	ReturnValue = adjustment;
	return (ReturnValue);
}
#endif

DS32 MAIN_actual_realpower_setpoint(void)
{
	DS32 Power;
    DS32 PowerMax;
	// here the actual realpower setpoint is calculated as minimum of 2 values:
	// internal parameter (power setpoint) 
	// and max allowed power from power reduction
	
	// unit is DS32 W
	Power = TUR_read_setpoint_PowerGridParallel(); // setpoint in W
    PowerMax = MAIN_realpower_max_allowed(); // max allowed power in W

    if (Power > PowerMax)
      Power = PowerMax;
    
	return( Power );
}

//CONTINUE ACA ***************************************
static void GridParallelOperationLimitedLoad(const DU8 sig)
{
	DBOOL Down;
	
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_ENABLE;
			GEN.mode      = GEN_MODE_ON;         // AVR active
			GEN.reg.mode  = GEN_REGMODE_VAR_TAKE_SETPOINT;// type of Regulator for mains parallel operation var control
			HVS.modeT1E   = HVS_T1E_ON;          // connect generator breaker
			ELM.modeMB	  = ELM_MB_CLOSE;		 // keep mains breaker closed, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = TRUE; // mixer control might be requested for the first time since starting
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_ENABLE;			 // catalyst system in operation
			//STH.mode      = STH_WATER_DEMANDED;  // heating of poil by water
			//THR.mode	  = keep as is
			TLB.mode      = TLB_GO_PARALLEL;     // move to predefined position
			TUR.Pset      = MAIN_actual_realpower_setpoint(); // Setpoint for P is a function to return the actual minimum setpoint
			TUR.mode      = TUR_TAKE_SETPOINT_POWER; // reach and keep power
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode      = in SIG_DO;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_ON;
			
			//HVS.modeL1E   = HVS_L1E_ON;         // keep 22L1E connected

			// change this to parameter for cos phi setpoint !!!
			GEN.SetpointVAR           = GEN_get_setpoint_var(); 

			// define actual MAIN.state
			MAIN.state                = MAIN_GRID_PARALLEL_LIMITED_LOAD;
			MAIN.regState             = MAIN_GRID_PARALLEL_ADJUST_POWER;
			DeloadCounter             = 0;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
	        		
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
			MAIN.regState = MAIN_GRID_PARALLEL_NORMAL_OPERATION;
			MAIN.GridParallelDelayed = FALSE;
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously

            if (myStateCnt > 500L)
                MAIN.GridParallelDelayed = TRUE;

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// set mixer mode depending on config and release load
			MAIN_Set_MIX_mode(MIX_MOVE_TO_PARALLEL_POSITION);

			if (THR.ReleaseLoad.State >= TRIP)
				THR.mode = THR_ENABLE;
			else
				THR.mode = THR_HEAT_UP;

			// timeout warming
			if (myStateCnt > MAIN_MAX_WARMING_TIME)
			{
				// if warning already timed out, but stop condition is acknowledged: reset StateCnt
			  	if ( !STOP_is_Set(STOPCONDITION_70215) ) 
			  	{
			  	// has been acknowledged or not yet been set
			  	    if ( myStateCnt > MAIN_MAX_WARMING_TIME + 1000L )
			  	    // has been acknowledged -> reset counter
			  	        myStateCnt = 20L;
			  	    else
			  	    // has not been set yet: set!
			            STOP_Set(STOPCONDITION_70215);
			  	}
			}
			
			if ( ( STOP.actualLevel < 5 )
				|| (!ENG.Running) )
			{
			  // open breaker immediately
		      CloseThrottle = TRUE;		// close throttle to avoid over speed
		      transit(DisconnectT1E);
		      break; 
			}

		    // generator circuit breaker not closed
		    if (HVS.stateT1E != HVS_T1E_IS_ON)
		    {
			  // open breaker immediately
		      CloseThrottle = TRUE;		// close throttle to avoid over speed
		      transit(DisconnectT1E);
		      break; 
		    }

		    // mains circuit breaker not closed
		    if ( (HVS.stateL1E != HVS_L1E_IS_ON)
		    	&& (!ISL.IslandOperationAllowed)				// island operation not allowed
		    	//&& (PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value > 1)
		       )
		    {
			  // open generator breaker immediately, 
		      CloseThrottle = TRUE;		// close throttle to avoid over speed
		      transit(DisconnectT1E);
		      break; 
		    }

		    // mains circuit breaker not closed, but island operation allowed (implicit 2 SCM-cards)
		    if ( (HVS.stateL1E != HVS_L1E_IS_ON)
		    	&& (ISL.IslandOperationAllowed)				// island operation allowed
		    	//&& (PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value > 1)
		       )
		    {
			  // open mains breaker immediately
		      transit(DisconnectL1EtoIsland);
		      break; 
		    }

			// else actualLevel >= 5
			// Are we in regular stop?
	        if ( !(STOP.actualBitMask & 0x0001) )
	        {
	          MAIN.regState = MAIN_GRID_PARALLEL_SOFT_DISCONNECT_T1E;
	        }
	        
	        // demand removed at operator´s panel?
	        if ( !MAIN.startdemand )
	        {
	          MAIN.regState = MAIN_GRID_PARALLEL_SOFT_DISCONNECT_T1E;
	        }

		    // engine warming
		    if (ENG.WarmingDone)
		    {
		    	transit(GridParallelOperationFullLoad);
		    	break;
		    }
		    
		    if (PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT2)
		    {
		    	transit(GridParallelOperationFullLoad);
		    	break;
		    }

		    // level >=4 and no regular stop and 2xSCM and no timeout open mains breaker
		    // and (mains failure or island operation or mains breaker open)
	        if ( (STOP.actualLevel > 3)
	        	 && ( STOP.actualBitMask & 0x0001 )																	// no regular stop
	        	 && (ISL.IslandOperationAllowed)//&& ( PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value > 1 )										// second SCM-card
	        	 && ( !STOP_is_Set(STOPCONDITION_50623) )															// no timeout open mains breaker
			     && ( (ELM.MainsFailure )																			// AND (mains failure
				    || (ISL.IslandOperationActive )																	//      OR island demanded
				    || (HVS.stateL1E == HVS_L1E_OFF_AND_READY)														//      OR mains breaker opened)
				    )
			   )
		    {
		    	transit(DisconnectL1EtoIsland);
		    	break;
		    }
		    
		    switch (MAIN.regState)
		    {
		        case MAIN_GRID_PARALLEL_ADJUST_POWER:
		        
					DeloadCounter             = 0;		// rmi100111
		    
		             // force VAR regulator to continuously takeover the setpoint
		             if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
		             {
		       	         GEN.SetpointVAR  = GEN_get_setpoint_var();
		       	         GEN.reg.mode = GEN_REGMODE_VAR;
		             }
		    
		            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
		                 GEN.reg.mode = GEN_REGMODE_VAR;
		                 
		            // force TUR regulation to continuously takeover actual setpoint for Power     
		            if ( TUR.state == TUR_REGULATE_POWER_START )
		            {
		            	// Take the setpoint for P as the actual minimum setpoint
		            	TUR.Pset      = MAIN_actual_realpower_setpoint(); 
			            TUR.mode      = TUR_POWER;          // reach and keep power	    
		            }
		            
		            if (( TUR.state == TUR_REGULATE_POWER ) || ( TUR.state == TUR_REGULATE_POWER_DONE ))
		            {
			            if ( MAIN.PowerSetpointHasChanged )
						{
							MAIN.PowerSetpointHasChanged = FALSE;

							Down = (TUR.Pset < ELM.T1E.sec.Psum);

							// always
							TUR.Pset = MAIN_actual_realpower_setpoint();

							// only if direction is changing
							if (Down) // old direction was down
							{
								// new direction is up
								if (TUR.Pset > ELM.T1E.sec.Psum)
									TUR.mode = TUR_TAKE_SETPOINT_POWER; // new setpoint
							}
							else //  // old direction was up
							{
								// new direction is down
								if (TUR.Pset < ELM.T1E.sec.Psum)
									TUR.mode = TUR_TAKE_SETPOINT_POWER; // new setpoint
							}
						}
		            }
		             
		            // when desired power is reached, substate = adjust blindpower 
		            if ( TUR.state == TUR_REGULATE_POWER_DONE )
		              MAIN.regState = MAIN_GRID_PARALLEL_ADJUST_BLINDPOWER; 
		                 
		        break;
		        
		        
		        case MAIN_GRID_PARALLEL_ADJUST_BLINDPOWER:
		        
					DeloadCounter             = 0;		// rmi100111
		    
		            // force VAR regulator to continuously takeover the setpoint
		            if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
		            {
		       	         GEN.SetpointVAR  = GEN_get_setpoint_var();
		       	         GEN.reg.mode = GEN_REGMODE_VAR;
		            }
		    
		            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
		                 GEN.reg.mode = GEN_REGMODE_VAR;
		                 
		            // force TUR regulation to continuously takeover actual setpoint for Power     
		            if ( TUR.state == TUR_REGULATE_POWER_START )
		            {
		            	TUR.Pset      = MAIN_actual_realpower_setpoint();
			            TUR.mode      = TUR_POWER;          // reach and keep power	    
		            }
		            
		            if (( TUR.state == TUR_REGULATE_POWER ) || ( TUR.state == TUR_REGULATE_POWER_DONE ))
		            {
			            if ( MAIN.PowerSetpointHasChanged )
						{
							MAIN.PowerSetpointHasChanged = FALSE;

							Down = (TUR.Pset < ELM.T1E.sec.Psum);

							// always
							TUR.Pset = MAIN_actual_realpower_setpoint();

							// only if direction is changing
							if (Down) // old direction was down
							{
								// new direction is up
								if (TUR.Pset > ELM.T1E.sec.Psum)
									TUR.mode = TUR_TAKE_SETPOINT_POWER; // new setpoint
							}
							else //  // old direction was up
							{
								// new direction is down
								if (TUR.Pset < ELM.T1E.sec.Psum)
									TUR.mode = TUR_TAKE_SETPOINT_POWER; // new setpoint
							}
						}
		            }
		            
		            if ( GEN.reg.state == GEN_REGSTATE_VAR_DONE )
		              MAIN.regState = MAIN_GRID_PARALLEL_NORMAL_OPERATION; 
		                 
		        break;
			        
		        case MAIN_GRID_PARALLEL_NORMAL_OPERATION:
		        
					 DeloadCounter             = 0;		// rmi100111
		    
		             // force VAR regulator to continuously takeover the setpoint
		             if ( GEN.SetpointVAR != GEN_get_setpoint_var() );
		             {
		       	         MAIN.regState = MAIN_GRID_PARALLEL_ADJUST_BLINDPOWER; 
		             }
		                 
		             // force TUR regulation to continuously takeover actual setpoint for Power     
		             if ( MAIN.PowerSetpointHasChanged )
		             {
		            	MAIN.regState = MAIN_GRID_PARALLEL_ADJUST_POWER;
		            	//MAIN.PowerSetpointHasChanged = FALSE;
		             }
		                 
		        break;
	        
		        case MAIN_GRID_PARALLEL_SOFT_DISCONNECT_T1E :
		        
		            // if no longer in regular stop and MAIN.startdemand then go back to normal power regulation
		            if ( (STOP.actualBitMask & 0x0001) && MAIN.startdemand )
		              MAIN.regState = MAIN_GRID_PARALLEL_NORMAL_OPERATION;
		           
		            // force TUR regulation to continuously takeover 0kW    
		            if ( TUR.state == TUR_REGULATE_POWER_START )
		            {
		            	
		            	TUR.Pset      = 0;                  // set power setpoint to 0 kW
			            TUR.mode      = TUR_POWER;          // reach and keep power	    
		            }
		            
		            if ( MAIN.OldregState != MAIN.regState )
		            {
			            TUR.mode      = TUR_TAKE_SETPOINT_POWER;          // new setpoint	    
		            }
		        
		        
		             // force VAR regulator to continuously takeover the setpoint for 0 kVAR
		             if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
		             {
		       	         GEN.SetpointVAR  = GEN_get_setpoint_var();
		       	         GEN.reg.mode = GEN_REGMODE_VAR;
		             }
		    
		            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
		                 GEN.reg.mode = GEN_REGMODE_VAR;
		                 
	        
		            // as soon as actual realpower becomes less than 2% of nominal, transit to disconnect T1E
		            if ( ELM.T1E.sec.Psum < (DS32)( PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value / MAIN_CUT_OUT_FRACTION_DELOAD) )
		            {
		            	 DeloadCounter = 0;
		            	 transit(DisconnectT1E);
		            	 break;
		            } 
		            else
		            {
		            	if (!TUR.Reg.PowerRampStopped)
		            		DeloadCounter += 20L;

		            	//if (DeloadCounter > MAIN_DELOAD_TIMEOUT)
		            	if (DeloadCounter > MAIN.DeloadTimeout)
		            	  STOP_Set(STOPCONDITION_30035);
		            }
		            
		        
		        break;
		        
		        case MAIN_GRID_PARALLEL_ADJUST_POWER_FOR_LOCAL_ISLAND_OPERATION :
		            /*
		            // force TUR regulation to continuously takeover -3kW as setpoint    
		            if ( TUR.state == TUR_REGULATE_POWER_START )
		            {
		            	TUR.Pset      = (ELM.T1E.per.Psum - ELM.L1E.per.Psum);             // set power setpoint to difference between generator and grid
			            TUR.mode      = TUR_POWER;          // reach and keep certain speed	    
		            }
		            if (( TUR.state == TUR_REGULATE_POWER )||( TUR.state == TUR_REGULATE_POWER_DONE ))
		            {
			            TUR.mode      = TUR_TAKE_SETPOINT_POWER;          // reach and keep certain speed	    
		            }
		        
		             // force VAR regulator to continuously takeover the setpoint for 0 kVAR
		             if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
		             {
		       	         GEN.SetpointVAR  = GEN_get_setpoint_var();
		       	         GEN.reg.mode     = GEN_REGMODE_VAR;
		             }
		    
		            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
		                 GEN.reg.mode = GEN_REGMODE_VAR;
		                 
	        
		            // as soon as actual gridpower becomes less than 10kW, transit to disconnect T1E
		            if (( ELM.L1E.sec.Psum < 10000 )&&( ELM.L1E.sec.Psum > -10000 ))
		            {
		            	 transit(DisconnectL1E);
		            	 break;
		            } 
		            */
		        break;
		        
                default : MAIN.regState = MAIN_GRID_PARALLEL_ADJUST_POWER;
		        
		    }
		MAIN.OldregState = MAIN.regState;
		break; // end of regular block SIG_DO
	}
	
}

static void GridParallelOperationFullLoad(const DU8 sig)
{
	DS32 RampChange;
	static DS32 RampRest = 0L;
	
	DBOOL Down;

	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			RampRest = 0L;

			// define control for all other components
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_ENABLE;
			GEN.mode      = GEN_MODE_ON;         // AVR active
			GEN.reg.mode  = GEN_REGMODE_VAR_TAKE_SETPOINT;// type of Regulator for mains parallel operation var control
			HVS.modeT1E   = HVS_T1E_ON;          // connect generator breaker
			ELM.modeMB	  = ELM_MB_CLOSE;		 // keep mains breaker closed, rmiEPF
			// keep MIX.EvaluateConditionsToStartMixerControl as it is
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_ENABLE;			 // catalyst system in operation
			//STH.mode      = STH_WATER_DEMANDED;  // heating of poil by water
			//THR.mode	  = keep as is
			//TLB.mode      = TLB_GO_PARALLEL;     // move to predefined position
			//TLB.Regulation_is_ON = FALSE;
			//TLB.Pset      = TUR.Reg.PowerSetPoint; // Setpoint for P is a function to return the actual minimum setpoint

            // Speed-Control active with Droop
			if (PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT2)
            {
                TUR.mode                = TUR_TAKE_SETPOINT_RPM;
                //TUR.MAINPosSet        = Keep
                TUR.MAINRpmSet          = TUR.NominalSpeed + TUR_CalculateDroopOffset(TUR.NominalSpeed);
    			TUR.Pset                = 0;
            }
			else
			{
    			TUR.mode                = TUR_TAKE_SETPOINT_POWER;
                //TUR.MAINPosSet        = Keep
                //TUR.MAINRpmSet        = Keep
    			TUR.Pset                = MAIN_actual_realpower_setpoint();
			}

            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;  
				          // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode      = in SIG_DO;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			RGB.mode      = RGB_MODE_ON;
			
			//HVS.modeL1E   = HVS_L1E_ON;         // keep 22L1E connected

			// change this to parameter for cos phi setpoint !!!
			GEN.SetpointVAR           = GEN_get_setpoint_var(); 

			// define actual MAIN.state
			MAIN.state                = MAIN_GRID_PARALLEL_FULL_LOAD;
        	if (PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT2)
        		MAIN.regState = MAIN_GRID_PARALLEL_SPEED_CONTROL;
        	else
        		MAIN.regState = MAIN_GRID_PARALLEL_ADJUST_POWER;
			DeloadCounter             = 0;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
	        		
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
			MAIN.regState = MAIN_GRID_PARALLEL_NORMAL_OPERATION;
			MAIN.GridParallelDelayed = FALSE;
			//TLB.mode      = TLB_OPEN;            // open turbo bypass
		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously

            if (myStateCnt > 500L)
                MAIN.GridParallelDelayed = TRUE;

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN.subState             = SUBSTATE_NO_TEXT;
				MAIN.regState             = MAIN_GRID_PARALLEL_NORMAL_OPERATION;

				switch (PMS.CBPOS_OperationMode)
				{
					case PMS_CBPOS_UNDEFINED:
						// undefined breaker position
						transit(UndefinedBreakers);
					break;

					case PMS_CBPOS_BLACK_OPERATION:
						// black operation
						transit(DisconnectL1EtoIsland);
					break;

					case PMS_CBPOS_MAINS_OPERATION:
						// mains operation
						transit(MainsOperation);
					break;

					case PMS_CBPOS_ISLAND_OPERATION:
						// island operation
						transit(DisconnectL1EtoIsland);
					break;

					default:
					
					    // problems on ArcNet
					    if (ARC.SafeMode)
						{
							break; // stay here
						}
						
	        	        if ( !STOP_is_Set(STOPCONDITION_50623)															// no timeout open mains breaker
			                && ( ELM.MainsFailure || ISL.IslandOperationActive) )
	        	        {
	        		        transit(DisconnectL1EtoIsland);
	        		        break;
	        	        }
                    break;
				}
				break;
			}

/*			
			// if Diesel empty in Diesel operation: regular stop
			if (STOP_is_Set(STOPCONDITION_70031))
			  STOP_Set(STOPCONDITION_70020);
*/			
                
			// set mixer mode depending on config and release load
			MAIN_Set_MIX_mode(MIX_MOVE_TO_PARALLEL_POSITION);

			if (THR.ReleaseLoad.State >= TRIP)
				THR.mode = THR_ENABLE;
			else
				THR.mode = THR_HEAT_UP;

			if (PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT2)
			{
				TLB.mode = TLB_SPEED;
			}
			else
			{
				if (TLB.Regulation_is_ON)
				{
					TLB.Regulation_is_ON = ELM.ReleaseLoadForTurboBypassControl
										&& (TLB.Regulation_OFF.State <  TRIP);
				}
				else
				{
					TLB.Regulation_is_ON = ELM.ReleaseLoadForTurboBypassControl
										&& (TLB.Regulation_ON.State >=  TRIP);
				}

				if (TLB.Regulation_is_ON)
				{
					TLB.mode = TLB_POWER;
					TLB.Pset = TUR.Reg.PowerSetPoint-TUR.OffsetForTurboBypassControl;// put new power setpoint to turbo bypass control
				}
				else
					TLB.mode = TLB_GO_PARALLEL;
			}

			if ( ( STOP.actualLevel < 5 )
				|| (!ENG.Running) )
			{
			  // open breaker immediately
			  CloseThrottle = TRUE;		// close throttle to avoid over speed, mvo,rmi091216
		      transit(DisconnectT1E);
		      break; 
			}

		    // generator circuit breaker not closed
		    if (HVS.stateT1E != HVS_T1E_IS_ON)
		    {
			  // open breaker immediately
		      CloseThrottle = TRUE;		// close throttle to avoid over speed
		      transit(DisconnectT1E);
		      break; 
		    }

		    // mains circuit breaker not closed, but 2 SCM and no island operation allowed
		    if ( (HVS.stateL1E != HVS_L1E_IS_ON)
		         && (!ISL.IslandOperationAllowed)								// island operation not allowed
		    	//&& (PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value > 1)
		       )
		    {
			  // open generator breaker immediately
		      CloseThrottle = TRUE;		// close throttle to avoid over speed
		      transit(DisconnectT1E);
		      break; 
		    }

		    // mains circuit breaker not closed, but island operation allowed (implicit 2 SCM-cards)
		    if ( (HVS.stateL1E != HVS_L1E_IS_ON)
		    	&& (ISL.IslandOperationAllowed)				// island operation allowed
		    	//&& (PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value > 1)
		       )
		    {
			  // open mains breaker immediately
		      transit(DisconnectL1EtoIsland);
		      break; 
		    }

			// else actualLevel >= 5
			// Are we in regular stop?
	        if ( !(STOP.actualBitMask & 0x0001) )
	        {
	          MAIN.regState = MAIN_GRID_PARALLEL_SOFT_DISCONNECT_T1E;
	        }
	        
	        // demand removed at operator´s panel?
	        if ( !MAIN.startdemand )
	        {
	          MAIN.regState = MAIN_GRID_PARALLEL_SOFT_DISCONNECT_T1E;
	        }

		    // level >=4 and no regular stop and 2xSCM and no timeout open mains breaker
		    // and (mains failure or island operation or mains breaker open)
	        if ( (STOP.actualLevel > 3)
	        	 && ( STOP.actualBitMask & 0x0001 )																	// no regular stop
	        	 && (ISL.IslandOperationAllowed)//&& ( PARA[ParRefInd[NBR_OF_SC_MODULES__PARREFIND]].Value > 1 )										// second SCM-card
	        	 && ( !STOP_is_Set(STOPCONDITION_50623) )															// no timeout open mains breaker
			     && ( (ELM.MainsFailure )																			// AND (mains failure
				    || (ISL.IslandOperationActive )																	//      OR island demanded
				    || (HVS.stateL1E == HVS_L1E_OFF_AND_READY)														//      OR mains breaker opened)
				    )
			   )
		    {
		    	transit(DisconnectL1EtoIsland);
		    	break;
		    }

        	if ((PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT2) AND (MAIN.regState != MAIN_GRID_PARALLEL_SPEED_CONTROL))
        	{
        		MAIN.regState = MAIN_GRID_PARALLEL_SPEED_CONTROL;
                TUR.mode = TUR_TAKE_SETPOINT_RPM;
    			TUR.Pset = 0;
        	}

		    switch (MAIN.regState)
		    {
		        case MAIN_GRID_PARALLEL_ADJUST_POWER:
		        
					DeloadCounter             = 0;		// rmi100111
		    
		            // force VAR regulator to continuously takeover the setpoint
		            if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
		            {
		       	         GEN.SetpointVAR  = GEN_get_setpoint_var();
		       	         GEN.reg.mode = GEN_REGMODE_VAR;
		            }
		    
		            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
		                 GEN.reg.mode = GEN_REGMODE_VAR;

		            if ( MAIN.PowerSetpointHasChanged )
					{
						MAIN.PowerSetpointHasChanged = FALSE;

						Down = (TUR.Pset < ELM.T1E.sec.Psum);

						// update power-setpoint
						if (TLB.Regulation_is_ON)
							TUR.Pset      = MAIN_actual_realpower_setpoint() + TUR.OffsetForTurboBypassControl;
						else
							TUR.Pset      = MAIN_actual_realpower_setpoint();

						// only if direction is changing
						if (Down) // old direction was down
						{
							// new direction is up
							if (TUR.Pset > ELM.T1E.sec.Psum)
								TUR.mode = TUR_TAKE_SETPOINT_POWER; // new setpoint
						}
						else //  // old direction was up
						{
							// new direction is down
							if (TUR.Pset < ELM.T1E.sec.Psum)
								TUR.mode = TUR_TAKE_SETPOINT_POWER; // new setpoint
						}
					}
		                 
		            // force TUR regulation to continuously takeover actual setpoint for Power     
		            if ( TUR.state == TUR_REGULATE_POWER_START )
		            {
		            	// Take the setpoint for P as the actual minimum setpoint
		            	if (TLB.Regulation_is_ON)
		            	    TUR.Pset      = MAIN_actual_realpower_setpoint() + TUR.OffsetForTurboBypassControl;
		            	else
		            		TUR.Pset      = MAIN_actual_realpower_setpoint();
			            TUR.mode      = TUR_POWER;          // reach and keep power
		            }
		             
		            // when desired power is reached, substate = adjust blindpower 
		            if ( TUR.state == TUR_REGULATE_POWER_DONE )
		              MAIN.regState = MAIN_GRID_PARALLEL_ADJUST_BLINDPOWER; 
		                 
		        break;
		        
		        
		        case MAIN_GRID_PARALLEL_ADJUST_BLINDPOWER:
		        
					DeloadCounter             = 0;		// rmi100111
		    
		            // force VAR regulator to continuously takeover the setpoint
		            if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
		            {
		       	         GEN.SetpointVAR  = GEN_get_setpoint_var();
		       	         GEN.reg.mode = GEN_REGMODE_VAR;
		            }
		    
		            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
		                 GEN.reg.mode = GEN_REGMODE_VAR;

		            if ( MAIN.PowerSetpointHasChanged )
					{
						MAIN.PowerSetpointHasChanged = FALSE;

						Down = (TUR.Pset < ELM.T1E.sec.Psum);

						// update power-setpoint
						if (TLB.Regulation_is_ON)
							TUR.Pset      = MAIN_actual_realpower_setpoint() + TUR.OffsetForTurboBypassControl;
						else
							TUR.Pset      = MAIN_actual_realpower_setpoint();

						// only if direction is changing
						if (Down) // old direction was down
						{
							// new direction is up
							if (TUR.Pset > ELM.T1E.sec.Psum)
								TUR.mode = TUR_TAKE_SETPOINT_POWER; // new setpoint
						}
						else //  // old direction was up
						{
							// new direction is down
							if (TUR.Pset < ELM.T1E.sec.Psum)
								TUR.mode = TUR_TAKE_SETPOINT_POWER; // new setpoint
						}
					}
		                 
		            // force TUR regulation to continuously takeover actual setpoint for Power     
		            if ( TUR.state == TUR_REGULATE_POWER_START )
		            {
		            	if (TLB.Regulation_is_ON)
		            	    TUR.Pset      = MAIN_actual_realpower_setpoint() + TUR.OffsetForTurboBypassControl;
		            	else
		            		TUR.Pset      = MAIN_actual_realpower_setpoint();
			            TUR.mode      = TUR_POWER;          // reach and keep power
		            }
		            
		            if ( GEN.reg.state == GEN_REGSTATE_VAR_DONE )
		              MAIN.regState = MAIN_GRID_PARALLEL_NORMAL_OPERATION; 
		                 
		        break;
			        
		        case MAIN_GRID_PARALLEL_NORMAL_OPERATION:
		        
					 DeloadCounter             = 0;		// rmi100111
		    
		             // force VAR regulator to continuously takeover the setpoint
		             if ( GEN.SetpointVAR != GEN_get_setpoint_var() );
		             {
		       	         MAIN.regState = MAIN_GRID_PARALLEL_ADJUST_BLINDPOWER; 
		             }
		                 
		             // force TUR regulation to continuously takeover actual setpoint for Power     
		             if ( MAIN.PowerSetpointHasChanged )
		             {
		            	MAIN.regState = MAIN_GRID_PARALLEL_ADJUST_POWER;
		            	//MAIN.PowerSetpointHasChanged = FALSE;
		             }
		                 
		        break;
	        
		        case MAIN_GRID_PARALLEL_SOFT_DISCONNECT_T1E :
		        
		            // if no longer in regular stop and MAIN.startdemand then go back to normal power regulation
		            if ( (STOP.actualBitMask & 0x0001) && MAIN.startdemand )
		              MAIN.regState = MAIN_GRID_PARALLEL_NORMAL_OPERATION;
		           
		            // force TUR regulation to continuously takeover 0kW    
		            if ( TUR.state == TUR_REGULATE_POWER_START )
		            {
		            	
		            	TUR.Pset      = 0;                  // set power setpoint to 0 kW
			            TUR.mode      = TUR_POWER;          // reach and keep power	    
		            }
		            
		            if ( MAIN.OldregState != MAIN.regState )
		            {
			            TUR.mode      = TUR_TAKE_SETPOINT_POWER;          // new setpoint	    
		            }
		        
		        
		             // force VAR regulator to continuously takeover the setpoint for 0 kVAR
		             if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
		             {
		       	         GEN.SetpointVAR  = GEN_get_setpoint_var();
		       	         GEN.reg.mode = GEN_REGMODE_VAR;
		             }
		    
		            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
		                 GEN.reg.mode = GEN_REGMODE_VAR;
		                 
	        
		            // rmi100325 if ( ELM.T1E.sec.Psum < 10000L )
		            // as soon as actual realpower becomes less than 2%, transit to disconnect T1E
		            if ( ELM.T1E.sec.Psum < (DS32)( PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value / MAIN_CUT_OUT_FRACTION_DELOAD) )
		            {
		            	 DeloadCounter = 0;
		            	 transit(DisconnectT1E);
		            	 break;
		            } 
		            else
		            {
		            	if (!TUR.Reg.PowerRampStopped)
		            		DeloadCounter += 20L;

		            	//if (DeloadCounter > MAIN_DELOAD_TIMEOUT)
		            	if (DeloadCounter > MAIN.DeloadTimeout)
		            	  STOP_Set(STOPCONDITION_30035);
		            }
		            
		        
		        break;
		        
		        case MAIN_GRID_PARALLEL_ADJUST_POWER_FOR_LOCAL_ISLAND_OPERATION :
		            /*
		            // force TUR regulation to continuously takeover -3kW as setpoint    
		            if ( TUR.state == TUR_REGULATE_POWER_START )
		            {
		            	TUR.Pset      = (ELM.T1E.per.Psum - ELM.L1E.per.Psum);             // set power setpoint to difference between generator and grid
			            TUR.mode      = TUR_POWER;          // reach and keep certain speed	    
		            }
		            if (( TUR.state == TUR_REGULATE_POWER )||( TUR.state == TUR_REGULATE_POWER_DONE ))
		            {
			            TUR.mode      = TUR_TAKE_SETPOINT_POWER;          // reach and keep certain speed	    
		            }
		        
		             // force VAR regulator to continuously takeover the setpoint for 0 kVAR
		             if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
		             {
		       	         GEN.SetpointVAR  = GEN_get_setpoint_var();
		       	         GEN.reg.mode     = GEN_REGMODE_VAR;
		             }
		    
		            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
		                 GEN.reg.mode = GEN_REGMODE_VAR;
		                 
	        
		            // as soon as actual gridpower becomes less than 10kW, transit to disconnect T1E
		            if (( ELM.L1E.sec.Psum < 10000 )&&( ELM.L1E.sec.Psum > -10000 ))
		            {
		            	 transit(DisconnectL1E);
		            	 break;
		            } 
		            */
		        break;
		        
		        case MAIN_GRID_PARALLEL_SPEED_CONTROL:

		        	if (!(PARA[ParRefInd[SPEED_REG_DROOP_MODE__PARREFIND]].Value & BIT2))
		        	{
		        		MAIN.regState = MAIN_GRID_PARALLEL_NORMAL_OPERATION;
		    			TUR.mode = TUR_TAKE_SETPOINT_POWER;
		    			TUR.Pset = MAIN_actual_realpower_setpoint();

		    			RampRest = 0;
		        	}
		        	else
		        	{
			             // force VAR regulator to continuously takeover the setpoint
			             if (( GEN.reg.state == GEN_REGSTATE_VAR )||( GEN.reg.state == GEN_REGSTATE_VAR_DONE ))
			             {
			       	         GEN.SetpointVAR  = GEN_get_setpoint_var();
			       	         GEN.reg.mode = GEN_REGMODE_VAR;
			             }

			            if ( GEN.reg.state == GEN_REGSTATE_VAR_START )
			                 GEN.reg.mode = GEN_REGMODE_VAR;

			        	// Deloading
			        	if ((!(STOP.actualBitMask & 0x0001)) OR (!MAIN.startdemand))
			        	{
			            	RampChange = (TUR.NominalSpeed+RampRest)/300000L;
			            	RampRest = (TUR.NominalSpeed+RampRest)%300000L;

							// Speed ramp down
							if (TUR.MAINRpmSet > TUR.NominalSpeed/10*9)
								TUR.MAINRpmSet -= RampChange;
							// Minimum speed setpoint reached (90% of nominal speed)
							else
							{
								transit(DisconnectT1E);
								return;
							}

							// Check if speed(actual) < speed(cutout)
							if ( ENG.S200EngineSpeed < TUR.NominalSpeed/1000*9 )
							{
								transit(DisconnectT1E);
								return;
							}

							// Check if P(actual) < P(cutout)
							if (ELM.T1E.sec.Psum < (PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value/MAIN_CUT_OUT_FRACTION_DELOAD))
							{
								transit(DisconnectT1E);
								return;
							}
			        	}
			        	else
			        	{
			        		// Update speed setpoint
			                TUR.MAINRpmSet = TUR.NominalSpeed + TUR_CalculateDroopOffset(TUR.NominalSpeed);
			        	}
		        	}

		        	break;

                default : MAIN.regState = MAIN_GRID_PARALLEL_ADJUST_POWER;
		        
		    }
		MAIN.OldregState = MAIN.regState;
		break; // end of regular block SIG_DO
	}
	
}

// L1E (mains breaker) closed
static void DisconnectT1E(const DU8 sig)
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
            //FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
            GAS.mode      = GAS_MODE_ON;         // open gas valves
            //GBV.mode      = GBV_MODE_XXX; keep as it is
            GEN.mode      = GEN_MODE_ON;         // AVR active
            GEN.reg.mode  = GEN_REGMODE_UISLAND_TAKE_SETPOINT;
            HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_CLOSE;		 // keep mains breaker closed, rmiEPF
			// keep MIX.EvaluateConditionsToStartMixerControl as it is
            NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_WARM_UP;		 //
			//STH.mode      = STH_WATER_DEMANDED;  // heating of poil by water
			THR.mode	  = THR_BLOCK;			 // thermoreactor blocked
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TLB.Regulation_is_ON = FALSE;
			TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
			if (CloseThrottle)                   // close throttle to avoid over speed (because of trip), mvo/rmi091216
			{
			   CloseThrottle = FALSE;            // reset CloseThrottle
               TUR.MAINPosSet= TUR.GovOutMin + (TUR.GovOutMax-TUR.GovOutMin)/20;
               TUR.mode      = TUR_TAKE_SETPOINT_POSITION;
			}
			else                                 // no one forced to close the throttle
			{
               TUR.mode      = TUR_TAKE_SETPOINT_RPM; // reach and keep speed
			}	// endif: someone forced to close the throttle
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode      = keep last mode;
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
			//HVS.modeL1E     = HVS_L1E_KEEP;       // keep state of 22L1E
			//GEN.SetpointVoltage       = TUR_read_setpoint_VoltageIsland();
			 
			// define actual MAIN.state
			MAIN.state                = MAIN_DISCONNECT_T1E;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			// keep as it is
			//MAIN.DO_IslandOperation = ???;
			//MAIN.DO_Loadsharing       = ???;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
					
		break; // end of SIG_EXIT

		case SIG_DO : // called every 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();
			
		// emergency stop button
		if (STOP_is_Set(STOPCONDITION_10201))
		{
			transit(FastBraking);
			break;
		}

		// else no emergency stop button
		
		// when T1E is disconnected transit to Cooldown		   
		if (       (HVS.stateT1E == HVS_T1E_DETECT_STATE) 
		        || (HVS.stateT1E == HVS_T1E_OFF_AND_READY)
		    	|| ( HVS.stateT1E ==  HVS_T1E_WAIT_AFTER_DISCONNECT ) )
	    {
	    	TUR.mode      = TUR_TAKE_SETPOINT_RPM;
	    	//TUR.mode = TUR_RPM;
	    	
	    	
	    	transit (Cooldown);
	    	break;
	    	
	    	// don't transit to IdleRun to avoid engine stop in case of GCB close failure
	    	// in PMS operation, 													
	    	// PMS-Startdemand is set to FALSE, if level < 5 (PMS.c),				rmi100809
	    	/*
	    	// regular stop or no startdemand?
	    	if ( !(STOP.actualBitMask & 0x0001) || !MAIN.startdemand )
	    	{
		    	transit(Cooldown);
		    	break;
	    	}
		    else if ((ELM.MainsFailure)							// mains failure
			    || (ISL.IslandOperationActive)					// island operation
			    || (!GEN.VoltageInWindow)						// generator voltage
		       )
		    {
	        	transit(IdleRun);
	        	break;
		    }
		    else // default: leave this state if generator breaker has opened
		    {
		    	transit(Cooldown);
		    	break;
		    }
		    */
	    }

        // else breaker still closed



		/* old condition with problems  
		    if ( (HVS.stateT1E <= HVS_T1E_OFF_AND_READY)
		    	|| ( HVS.stateT1E ==  HVS_T1E_WAIT_AFTER_DISCONNECT ) )
		    {
		    	TUR.mode = TUR_RPM;
		    	transit(Cooldown);
		    	break;
		    }
		    */
	    //else stay here forever, SC 30613 is set from HVS after timeout
	    
	    
	    // !!! Gefunden 29.08.2008 MVO
	    // Störung Überdrehzahl als Folge von Asymmetrie
	    // Level == 2
	    // HVS.State == 0
	    // MAIN.State == Schalter öffnen
	    // Schalter ist offen.
	    // Motor stellt aber nicht ab.
		
		break; // end of regular block SIG_DO
	}
	
}

/* rmiEPF */
// L1E (mains breaker) open, island operation: disconnect T1E
static void DisconnectT1EIsland(const DU8 sig)
{
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			
			AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_MODE_ON;          // throttle controller switched on
			ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			//GBV.mode      = GBV_MODE_XXX; keep as it is
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			WAT.mode      = WAT_ENABLE;          // pumps on
            if (!GAS.GasTypeBActive) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL
			//MIX.mode      = keep last mode;	 // 
			// keep MIX.EvaluateConditionsToStartMixerControl as it is
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;
			
            GEN.mode      = GEN_MODE_ON;         // AVR active
            GEN.reg.mode  = GEN_REGMODE_UISLAND_TAKE_SETPOINT;
            HVS.modeT1E   = HVS_T1E_OFF;         // open generator breaker
			ELM.modeMB	  = ELM_MB_OPEN;		 // keep mains breaker disconnected, rmiEPF	
			SCR.mode	  = SCR_WARM_UP;		 //
			//THR.mode	  = keep as is
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			TLB.Regulation_is_ON = FALSE;
			TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
			if (CloseThrottle)                   // close throttle to avoid over speed (because of trip), mvo/rmi091216
			{
			   CloseThrottle = FALSE;            // reset CloseThrottle
               TUR.MAINPosSet= TUR.GovOutMin + (TUR.GovOutMax-TUR.GovOutMin)/20;
               TUR.mode      = TUR_TAKE_SETPOINT_POSITION;
			}
			else                                 // no one forced to close the throttle
			{
               TUR.mode      = TUR_TAKE_SETPOINT_RPM; // reach and keep speed
			}	// endif: someone forced to close the throttle
			
			//HVS.modeL1E     = HVS_L1E_KEEP;       // keep state of 22L1E
			//GEN.SetpointVoltage       = TUR_read_setpoint_VoltageIsland();
			 
			// define actual MAIN.state
			MAIN.state                = MAIN_DISCONNECT_T1E_ISLAND;

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			// keep as it is
			//MAIN.DO_IslandOperation = ???;
			//MAIN.DO_Loadsharing       = ???;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
					
		break; // end of SIG_EXIT

		case SIG_DO : // called every 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();
			
		// STOP.actualLevel < 3?
		if (STOP.actualLevel < 3)
		{ 
		  transit(FastBraking); 
		  break;
		}
		
		// else actualLevel >= 3

		// timeout for opening generator breaker
		if (STOP_is_Set(STOPCONDITION_30613))
		{
		    	transit(FastBraking);
		    	break;
		}

		// else actualLevel >= 3 and timer not yet elapsed
		
		if (THR.ReleaseLoad.State >= TRIP)
			THR.mode = THR_ENABLE;
		else
			THR.mode = THR_HEAT_UP;

		// voltage ok?
		if (!GEN.VoltageInWindow)
		{
		    	transit(IdleRun);
		    	break;
		}		

		// else when T1E is disconnected transit to Cooldown
		  
		if (       (HVS.stateT1E == HVS_T1E_DETECT_STATE) 
		        || (HVS.stateT1E == HVS_T1E_OFF_AND_READY)
		    	|| (HVS.stateT1E ==  HVS_T1E_WAIT_AFTER_DISCONNECT) )
		{
		    	TUR.mode = TUR_RPM;
		    	transit(Cooldown);
		    	break;
		}
	    //else stay here
		
		break; // end of regular block SIG_DO
	}
	
} // end: DisconnectT1EIsland


static void Cooldown(const DU8 sig) 
{
	static DU32 CooldownTimerForMainsFailure;
	
	switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
            AIR.mode      = AIR_ENABLE;          // Open flaps, turn room ventilation on
			AKR.mode      = AKR_MODE_ON;
            DK.mode       = DK_MODE_ON;          // throttle controller switched on
            ENG.mode      = ENG_START_DEMANDED;  // engine shall run
			//EXH.mode      = EXH_OPEN;            // open flaps
			//FUE.mode      = FUE_DIESEL_DEMANDED; // Diesel operation
			GAS.mode      = GAS_MODE_ON;         // open gas valves
			GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_BLOCK;
			GEN.mode      = GEN_MODE_ON;         // AVR active
			GEN.reg.mode  = GEN_REGMODE_UISLAND_TAKE_SETPOINT;	
			HVS.modeT1E   = HVS_T1E_OFF;         // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		 // power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no active control of mixer in this state
			NKK.mode      = NKK_ENABLE;          // emergency cooler active
			OIL.mode      = OIL_ENABLE;          // oil system on
			PLS.mode      = PLS_ENG_OPERATION_REQ; // start prelubrication
			//PFH.mode      = PFH_ENABLE;          // plant oil filter heater regulation active
			WAT.mode      = WAT_ENABLE;          // pumps on
			//RSD.mode      = RSD_ENABLE;          // Refilling off 
			//RSP.mode      = RSP_ENABLE;          // Refilling off
			SAF.mode      = SAF_GUARD;           // ready for TRIP and protection
			SCR.mode	  = SCR_WARM_UP;			 // catalyst system standby
			//STH.mode      = STH_WATER_DEMANDED;  // heating of poil by water
			THR.mode	  = THR_HEAT_UP;		 // thermoreactor blocked
			TLB.mode      = TLB_OPEN;            // open turbo bypass
			if (PAR_CUMMINS_OPTION AND PARA[ParRefInd[LOW_IDLE_OPTION__PARREFIND]].Value)
			{
				TUR.MAINPosSet = TUR_MIN_POSITION_SETPOINT_AOUT
						+ (TUR_MAX_POSITION_SETPOINT_AOUT - TUR_MIN_POSITION_SETPOINT_AOUT)
						* PAR_CUMMINS_THROTTLE_LOW_IDLE / 1000L;
				TUR.mode = TUR_TAKE_SETPOINT_POSITION;
			}
			else
			{
				TUR.MAINRpmSet= TUR.NominalSpeed;     // control to nominal speed
				TUR.mode      = TUR_TAKE_SETPOINT_RPM; // reach and keep speed
			}
            if (!GAS.GasTypeBActive || !GAS.GasTypeBSelected) // gas type A
            	COM.mode  = COM_DEMANDED;        // compressor demanded
            else // gas type B
            	COM.mode  = COM_STOP;            // compressor off
			IGN.mode      = IGN_MODE_ON;         // ignition on
         	ZS3.OperatingStartRequested = TRUE;
			ISL.mode      = ISL_MODE_PARALLEL;   // parallel operation (all stages on), rmiISL

			if (PAR_CUMMINS_OPTION AND PARA[ParRefInd[LOW_IDLE_OPTION__PARREFIND]].Value)
			{
				MIX.mode[MixerInd1] = MIX_MOVE_TO_START_POSITION;
				MIX.mode[MixerInd2] = MIX_MOVE_TO_START_POSITION;
			}
			else
			{
				MIX.mode[MixerInd1] = MIX_MOVE_TO_IDLE_POSITION;
				MIX.mode[MixerInd2] = MIX_MOVE_TO_IDLE_POSITION;
			}
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_BLOCK;

			//HVS.modeL1E          = HVS_L1E_KEEP;       // keep state of 22L1E
			//GEN.SetpointVoltage  = TUR_read_setpoint_VoltageIsland();			
			
			// define actual MAIN.state
			MAIN.state                = MAIN_COOLDOWN;	

			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = TRUE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= TRUE;
			
			CooldownTimerForMainsFailure = 0L;
			
		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left
		break; // end of SIG_EXIT

		case SIG_DO : // called every 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			// we are the mains delomatic !!!
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();

			// cooldown time in case mains failure
			// in this case we want to run for a defined time
			// not less and not longer - independent of the parameter for normal cooling time
			// trigger for the beginning of this special cooling time is mains failure >= HOT 
			if ( ( ELM.L1E.MainsPowerFailure.State >= HOT ) || PMS.MainsFailureHot )
			{
				CooldownTimerForMainsFailure += 20L;
				if ( CooldownTimerForMainsFailure > COOL_DOWN_TIME_WITH_MAINS_FAILURE )
				{
			      transit(FastBraking); 
				  break;
				}
			}
			else
			{
				CooldownTimerForMainsFailure = 0L;
			    // check if normal cooldown timer is elapsed
			    if ( myStateCnt > (DU32)PARA[ParRefInd[COOL_DOWN_TIME__PARREFIND]].Value )
				{ 
				  transit(FastBraking); 
				  break;
				}
			}

			// else Cooldown timer not yet elapsed
			
			// STOP.actualLevel < 3?
			if (STOP.actualLevel < 3)
			{ 
			  transit(FastBraking); 
			  break;
			}
			
			// else actualLevel >= 3 and Cooldown timer still running

			// Update position setpoint
			if (PAR_CUMMINS_OPTION AND PARA[ParRefInd[LOW_IDLE_OPTION__PARREFIND]].Value)
			{
				TUR.MAINPosSet = TUR_MIN_POSITION_SETPOINT_AOUT
						+ (TUR_MAX_POSITION_SETPOINT_AOUT - TUR_MIN_POSITION_SETPOINT_AOUT)
						* PAR_CUMMINS_THROTTLE_LOW_IDLE / 1000L;
			}
			
			// regulate RPM
		    if ( TUR.state == TUR_REGULATE_RPM_START )
		       {
			      TUR.mode          = TUR_RPM;            // reach and keep certain speed	
		       }

	        if (PAR_CUMMINS_OPTION AND PARA[ParRefInd[LOW_IDLE_OPTION__PARREFIND]].Value)
	        {
	        	GEN.mode = GEN_MODE_OFF;
	        }
	        else
	        {
				// continuously update GEN.SetpointVoltage
				if (   ( GEN.reg.SetpointVoltage != GEN_NOMINAL_VOLTAGE)
					 &&(( GEN.reg.state == GEN_REGSTATE_UISLAND ) || ( GEN.reg.state == GEN_REGSTATE_UISLAND_DONE )))
				   {
					GEN.reg.mode              = GEN_REGMODE_UISLAND_TAKE_SETPOINT;
				   }
				if ( GEN.reg.state == GEN_REGSTATE_UISLAND_START )
				   {
					GEN.SetpointVoltage       = GEN_NOMINAL_VOLTAGE;
					GEN.reg.mode              = GEN_REGMODE_UISLAND;
				   }
	        }
			
		    // Speed regulation does not work
		    if (STOP_is_Set(STOPCONDITION_50033))
		    {
		    	transit(FastBraking);
		    	break;
		    }

			// engine dying for whatever reason: switch it off
	        //if ( (ENG.state != ENG_RUNNING) && (!GEN.VoltageInWindow) )
	        if ( (ENG.S200EngineSpeed < PARA[ParRefInd[ENGINE_RUNNING__PARREFIND]].Value/100) && (!GEN.VoltageInWindow) )
	        {
	        	// engine has stopped
	        	STOP_Set(STOPCONDITION_20096);

	          transit(FastBraking);
		      break;
	        }
	        // if no mains failure
			// are we still in regular stop or is stopping requested?
	        if ( //!ELM.MainsFailure &&
	             // 04.08.2010
	        	      !(STOP.actualBitMask & 0x0001)			// regular stop
	        		|| (!MAIN.startdemand)						// no start demand
	        		|| (STOP_is_Set(STOPCONDITION_30612) ) )	// gen breaker close failure - wait for next try
	        {
	          // then stay here
	          break;
	        }
	        
	        // else actualLevel >= 3 and Cooldown timer still running and start demanded and no regular stop and
	        // (engine running || gen voltage in window)

	        // run demanded
			if (PAR_CUMMINS_OPTION)
			{
				transit(Acceleration);
				break;
			}
			else
			{
				transit(IdleRun);
				break;
			}
		       
		break; // end of regular block SIG_DO
	}
}


static void Test(const DU8 sig)
{
	// Switch all modules to test mode
	// Can only be called from System Stop.
switch(sig)
	{
		case SIG_ENTRY: // called when this state is entered
			
			// define control for all other components
			AIR.mode      = AIR_TEST_DEMANDED;  // test 
			AKR.mode      = AKR_MODE_ON;
			DK.mode       = DK_TEST_DEMANDED;   // throttle controller under test
			ENG.mode      = ENG_TEST_DEMANDED;  // test
			//EXH.mode      = EXH_TEST;           // test
            //FUE.mode      = FUE_TEST_DEMANDED;  // test
			FAB.mode	  = FAB_TEST_DEMANDED;
            GAS.mode      = GAS_TEST_DEMANDED;  // test gas valves
            GAS.WaitingForBackSynchronization = FALSE; // reset marker
			GBV.mode      = GBV_MODE_TEST;
            GEN.mode      = GEN_MODE_TEST;      // AVR output under test 
            GEN.reg.mode  = GEN_REGMODE_UISLAND;// type of Regulator nominal Voltage
            HVS.modeT1E   = HVS_T1E_OFF;        // disconnect generator breaker
			ELM.modeMB	  = ELM_MB_AUTO;		// power fail automatic for mains breaker, rmiEPF
			MIX.EvaluateConditionsToStartMixerControl = FALSE; // no active mixer control in this state
            NKK.mode      = NKK_TEST_DEMANDED;  // emergency cooler under test
            OIL.mode      = OIL_TEST_DEMANDED;  // test
            PLS.mode      = PLS_TEST_DEMANDED;  // test prelubrication
            //PFH.mode      = PFH_TEST_DEMANDED;  // test      
            WAT.mode      = WAT_TEST;           // test
            //RSD.mode      = RSD_TEST_DEMANDED;  // test
            //RSP.mode      = RSP_TEST_DEMANDED;  // test
			SAF.mode      = SAF_GUARD;          // ready for TRIP and protection
			SCR.mode	  = SCR_TEST_DEMANDED;			 // catalyst system in operation
			//STH.mode      = STH_TEST_DEMANDED;  // test
			THR.mode	  = THR_TEST_DEMANDED;	// thermoreactor under test
			TLB.mode      = TLB_TEST_DEMANDED;  // close turbo bypass
			TUR.mode      = TUR_TEST;           // governor output under test
			COM.mode      = COM_TEST_DEMANDED;  // test demanded
			IGN.mode      = IGN_TEST_DEMANDED;  // test demanded
			//ISL.mode      = ISL_MODE_TEST;   // test operation, rmiISL
			MIX.mode[MixerInd1] = MIX_TEST_DEMANDED;  // test demanded
			MIX.mode[MixerInd2] = MIX_TEST_DEMANDED;  // test demanded
			PMS.mode      = PMS_ENABLE;
			
			PID.mode[PID_1] = PID_CONTROL;
			PID.mode[PID_2] = PID_CONTROL;
			PID.mode[PID_3] = PID_CONTROL;
			PID.mode[PID_4] = PID_CONTROL;
			PID.mode[PID_5] = PID_CONTROL;

			SER.mode = SER_MODE_ON;
				
			//HVS.modeL1E   = HVS_L1E_KEEP;       // keep state of 22L1E 
		
			// define actual MAIN.state
			MAIN.state                = MAIN_TEST;
			
			// Is a gas type changeover allowed in this state if set to no load changeover?
			MAIN.GasChangeOverInIdle  = FALSE;

			// set state outputs
			MAIN.DO_ReadyForOperation = FALSE;
			MAIN.DO_IslandOperation   = FALSE;
			MAIN.DO_Loadsharing       = FALSE;
			MAIN.WishToCloseGCB       = FALSE;
			MAIN.EngineRunningNominalDelayed					= FALSE;
#if (CLIENT_VERSION != IET)
			MAIN.IO_TestActive = MAIN.IO_TestDemand;
#endif
			if (MAIN.IO_TestActive)
				IOA_SetPointerFor_IO_Test();

		break; // end of SIG_ENTRY

		case SIG_EXIT: // called, when this state is left

			if (MAIN.IO_TestActive)
			{
				set_IOA_all_adress_pointer(HMI_USB_CLIENT);
			}

			MAIN.IO_TestDemand = OFF;
			MAIN.IO_TestActive = OFF;

		break; // end of SIG_EXIT

		case SIG_DO : // called all 20ms when this state is running continuously
		default:      // here all conditions for a state change are listed

			if (MAIN.IO_TestDemand)
			{
				break; // stay here...
			}

			// we are the mains delomatic
			if (PMS.WeAreMainsDelomatic)
			{
				MAIN_MainsDelomaticState();
				break;
			}
			
			// set RGB.mode depending on alarms
			MAIN_Set_RGB_mode();

			if (STOP.actualLevel < 1)
			{ 
				transit(SystemStart); 
				break;
			}
			
		    if (STOP.actualLevel == 1)
		    { 
/*
				// Emergency stop button
				if (STOP_is_Set(STOPCONDITION_10201))
				{
*/
			    	transit(EmergencyStop);
			    	break;
/*
				}
				// Lube oil service not active
				else if (!STOP_is_Set(STOPCONDITION_20093))
				{
			    	transit(EmergencyStop);
			    	break;
				}
				// else stay...
*/
		    }

		    if (   !STOP_is_Set(STOPCONDITION_20099)   // no test mode anymore?
		    	&& !STOP_is_Set(STOPCONDITION_20093) ) // no lube oil service active?
		    {
		    	//if ((!MAIN.DI_DigitalAutoDemand) && (DI_FUNCT[AUTO_DEMAND].Assigned == ASSIGNED))
		    	// level < 3 or regular stop
		    	if ((STOP.actualLevel < 3) || !(STOP.actualBitMask & 0x0001))
		    	{
		    	  transit(SystemStop);
		    	  break;
		    	}
		    	else
		    	{
		    	  transit(SystemReadyForStart);
		    	  break;
		    	}
		    }

		    // switch off breaker if voltage detected
		    if (!(ELM.TransformerVoltageIsZero_Flag && ELM.BBVoltageIsZero_Flag) || STOP_is_Set(STOPCONDITION_30613))
		    {
		    	HVS.modeT1E = HVS_T1E_OFF;
		    	HVS.DO_TripGeneratorBreaker = FALSE;
                ForceOpenTripRelay(ELM_GPM51_MODULE_INDEX_GCB, GPM51_FORCE_TRIP_RELAY_FUNC1);
		    }

		break; // end of regular block SIG_DO
	}
	
}


DU8* MAIN_state_text(DU8 state)
{
   //DU8* text_english;
   //DU8* text_native;
   static	DU8* text;
   
   switch ( state )
   {
  	
   	  case MAIN_BOOT:
   	 //      text_english = "SYSTEM BOOT";
	 //      text_native  = "SYSTEM BOOT";
	       text         = globaltext [TEXT_MAIN_BOOT]; //"SYSTEM BOOT"
           break;
           
   	  case MAIN_SYSTEM_START:
   	  //     text_english = "SYSTEM POWER UP";
	  //     text_native  = "SYSTEM OPSTART";
	       text         = globaltext [TEXT_MAIN_SYSTEM_START]; //"SYSTEM POWER UP"
           break;
           
   	  case MAIN_EMERGENCY_STOP:
   	  //     text_english = "EMERGENCY STOP";
	  //     text_native  = "NØDSTOPP";
	       text         = globaltext [TEXT_MAIN_EMERGENCY_STOP]; //"EMERGENCY STOP"
           break;
           
   	  case MAIN_UNDEFINED_BREAKER_POS:
	       text         = globaltext [TEXT_MAIN_UNDEFINED_BREAKER_POSITION]; //"UNDEFINED BREAKER POSITION"
           break;
           
   	  case MAIN_BLACK_OPERATION:
	       text         = globaltext [TEXT_MAIN_BLACK_OPERATION]; //"BLACK OPERATION"
           break;
           
   	  case MAIN_MAINS_OPERATION:
	       text         = globaltext [TEXT_MAIN_MAINS_OPERATION]; //"MAINS OPERATION"
           break;
         
   	  case MAIN_EMERGENCY_BRAKING:
   	  //     text_english = "EMERGENCY BRAKING";
	  //     text_native  = "NØDBREMSNING";
           text         = globaltext [TEXT_MAIN_EMERGENCY_BRAKING]; //"EMERGENCY BRAKING"
           break;
         
   	  case MAIN_REARM_SAFETY_CHAIN:
   	  //     text_english = "REARM SAFETY CHAIN";
	  //     text_native  = "REARM SIKKERHETSKRETS";
	       text         = globaltext [TEXT_MAIN_REARM_SAFETY_CHAIN]; //"REARM SAFETY CHAIN"
           break;
         
   	  case MAIN_SYSTEM_STOP:
   	  //     text_english = "SYSTEM STOP";
	  //     text_native  = "SYSTEM STOPP";
	       text         = globaltext [TEXT_MAIN_SYSTEM_STOP]; //"START BLOCKED"
           break;
          
      case MAIN_SYSTEM_READY_FOR_START:
           text         = globaltext [TEXT_MAIN_SYSTEM_READY_FOR_START]; // READY TO START
           break;
           
   	  case MAIN_FAST_BRAKING:
   	  //     text_english = "CLOSE WATERWAY AND BRAKE TO STOP";
	  //     text_native  = "LUKK VANNVEJ OG BREMS TIL STOPP";
	       text         = globaltext [TEXT_MAIN_FAST_BRAKING]; //"QUICKSTOP"
           break;
          
   	  case MAIN_STRT_PREPARE:
   	  //     text_english = "FILL TURBINE WITH WATER";
	  //     text_native  = "FYLLER TURBIN MED VANN";
	       text         = globaltext [TEXT_MAIN_STRT_PREPARE]; //"STARTING"
           break;

 	  case MAIN_STARTING:
   	  //     text_english = "TURBINE STOP";
	  //     text_native  = "TURBIN STOPP";
           text         = globaltext [TEXT_MAIN_STARTING]; //"STOPPING"
           break;

 	  case MAIN_IGNITION:
           text         = globaltext [TEXT_MAIN_IGNITION]; //"IGNITION"
           break;

 	  case MAIN_OPEN_GAS_VALVES:
           text         = globaltext [TEXT_MAIN_OPEN_GAS_VALVES]; //"OPEN GAS VALVES"
           break;
                      
   	  case MAIN_ACCELERATION:
   	  //     text_english = "ACCELERATION";
	  //     text_native  = "ACCELERASJON";
	       text         = globaltext [TEXT_MAIN_ACCELERATION]; //"ACCELERATION"
           break;

   	  case MAIN_LOW_IDLE_SPEED:
           text         = globaltext [TEXT_MAIN_LOW_IDLE_SPEED];
           break;
           
   	  case MAIN_TRANSFORMER_DISCONNECTED:
   	  //     text_english = "MAIN TRANSFORMER DISCONNECTED";
	  //     text_native  = "1300kVA TRAFO AF";
	       text         = globaltext [TEXT_MAIN_TRANSFORMER_DISCONNECTED]; //"IDLE RUNNING"
           break;
           
      case MAIN_COOLDOWN:
           text         = globaltext [TEXT_MAIN_COOLDOWN]; //"COOLDOWN"
           break;
           
   	  case MAIN_DISCONNECT_T1E:
   	  //     text_english = "DISCONNECT T1E";
	  //     text_native  = "FRAKOBBLING T1E";
	       text         = globaltext [TEXT_MAIN_DISCONNECT_T1E]; //"OPEN BREAKER"
           break;

   	  case MAIN_SYNCHRON_CONNECT_T1E:
      //   text_english = "SYNCHRON CONNECT T1E";
	  //   text_native  = "SYNKRON INNKOBBLING T1E";
           text         = globaltext [TEXT_MAIN_SYNCHRON_CONNECT_T1E]; //"SYNCHRONIZING"
           break;

      case MAIN_GRID_PARALLEL_LIMITED_LOAD:
           text         = globaltext [TEXT_MAIN_GRID_PARALLEL_LIMITED_LOAD]; //"LIMITED LOAD"
           break;

      case MAIN_GRID_PARALLEL_FULL_LOAD:
           text         = globaltext [TEXT_MAIN_GRID_PARALLEL_FULL_LOAD]; //"GRID PARALLEL OPERATION"
           break;

      case MAIN_TEST:
           text         = globaltext [TEXT_MAIN_TEST]; //TEST
           break;

      case MAIN_CONNECT_T1E:
           text         = globaltext [TEXT_MAIN_CONNECT_T1E]; 				//rmiEPF
           break;

      case MAIN_WAIT_FOR_RELEASE_CLOSE_GCB:
           text         = globaltext [TEXT_MAIN_WAIT_FOR_BLACK_CLOSING]; // "WAIT FOR BLACK CLOSING"
           break;

      case MAIN_DISCONNECT_T1E_ISLAND:
           text         = globaltext [TEXT_MAIN_DISCONNECT_T1E_ISLAND]; 	//rmiEPF
           break;

      case MAIN_ISLAND_OPERATION:
           text         = globaltext [TEXT_MAIN_ISLAND_OPERATION]; 			//rmiEPF
           break;

      case MAIN_SYNCHRON_CONNECT_L1E:
           text         = globaltext [TEXT_MAIN_SYNCHRON_CONNECT_L1E]; 		//rmiEPF
           break;

      case MAIN_DISCONNECT_L1E_TO_ISLAND:
           text         = globaltext [TEXT_MAIN_DISCONNECT_L1E_TO_ISLAND]; 	//rmiEPF
           break;

      case MAIN_LOADSHARING_RAMP_UP:
           text         = globaltext [TEXT_MAIN_LOADSHARING_RAMP_UP];
           break;

      case MAIN_LOADSHARING:
           text         = globaltext [TEXT_MAIN_LOADSHARING];
           break;

      case MAIN_LOADSHARING_RAMP_DOWN:
           text         = globaltext [TEXT_MAIN_LOADSHARING_RAMP_DOWN];
           break;
           
      default :
   	  // text_english = "! MAIN STATE is undefined !";
	  //     text_native  = "! MAIN STATE is undefined !";
	       text         = "! MAIN STATE is undefined !";
           break;
   }   
   /*if ( language == 0 ) return(text_english);
   else return(text_native);*/
   // Always return text
   return(text);
}			
			
DU8* MAIN_subState_text(DU8 state)
{
   DU8* text;
   static DU8 text2[100];
   
   switch ( state )
   {
   	  case SUBSTATE_NO_TEXT:
   	       text = " ";
   	       break;

   	  case MAIN_SUB_FT_ACTIVE:
   	       sprintf(text2, "%s (%d)", globaltext[TEXT_MAIN_SUB_FT_ACTIVE], ft_SaveConter);
   	       return (text2);
   	       break;

	  case MAIN_SUB_SIMULATION:
		   text = "SIMULATION";
		   break;

   	  case MAIN_SUB_GPT_ACTIVATED:
   	       text = globaltext[TEXT_MAIN_SUB_GPT_ACTIVATED];
   	       break;

   	  case MAIN_SUB_GPT_RUNNING:
   		text = globaltext[TEXT_MAIN_SUB_GPT_RUNNING];
   	       break;

   	  case MAIN_SUB_GPT_TRIPPED:
   		text = globaltext[TEXT_MAIN_SUB_GPT_TRIPPED];
   	       break;
/*   	       
	  case MAIN_SUB_CLOSING_VALVES:
	       text = globaltext[TEXT_MAIN_SUB_CLOSING_VALVES];   // closing water valves, Wasserventile werden geschlossen, Vandvalves lukkes
   	       break;
   	       
	  case MAIN_SUB_REFILLING_DIESEL:
	       text = globaltext[TEXT_MAIN_SUB_REFILLING_DIESEL]; // refilling Diesel
   	       break;
   	       
	  case MAIN_SUB_REFILLING_POIL:
	       text = globaltext[TEXT_MAIN_SUB_REFILLING_POIL];   // refilling plant oil
   	       break;
   	       
	  case MAIN_SUB_SWITCH_TO_WATER:
	  	   if ((DBOOL)PARA[ParRefInd[POIL_TEMP_EL_HEATER_OPTION_VALVE__PARREFIND]].Value)
	       		text = globaltext[TEXT_MAIN_SUB_SWITCH_TO_WATER];  // storage tank heating moving to water operation
	       else
   	       text = " ";
   	       break;
   	       
	  case MAIN_SUB_SWITCH_TO_ELECTRIC:
	  	   if ((DBOOL)PARA[ParRefInd[POIL_TEMP_EL_HEATER_OPTION_VALVE__PARREFIND]].Value)
	       		text = globaltext[TEXT_MAIN_SUB_SWITCH_TO_ELECTRIC];// storage tank heating moving to electric operation
	       else
   	       text = " ";
   	       break;
   	       
	  case MAIN_SUB_OPENING_AIR_FLAPS:
	       text = globaltext[TEXT_MAIN_SUB_OPENING_AIR_FLAPS];// opening air flaps
   	       break;
   	       
	  case MAIN_SUB_CLOSING_AIR_FLAPS:
	       text = globaltext[TEXT_MAIN_SUB_CLOSING_AIR_FLAPS];// closing air flaps
   	       break;
   	       
	  case MAIN_SUB_OPENING_EXH_FLAPS:
	       text = globaltext[TEXT_MAIN_SUB_OPENING_EXH_FLAPS];// opening exhaust flaps
   	       break;
   	       
	  case MAIN_SUB_CLOSING_EXH_FLAPS:
	       text = globaltext[TEXT_MAIN_SUB_CLOSING_EXH_FLAPS];// closing exhaust flaps
   	       break;
*/   	       
	  case MAIN_SUB_MIX_CONFIG:
	       text = globaltext[TEXT_MAIN_SUB_EMIS_CONFIG];        // mixer in configuration
   	       break;

	  case MAIN_SUB_SCR_MANUAL:
	       text = globaltext[TEXT_MAIN_SUB_SCR_MANUAL];        // catalyst in manual mode
   	       break;

	  case MAIN_SUB_FLUSHING:
	       text = globaltext[TEXT_MAIN_SUB_FLUSHING];          // flushing
   	       break;
	       
	  case MAIN_SUB_FLUSHING_EXHAUST:
		   text = globaltext[TEXT_MAIN_SUB_FLUSHING_EXHAUST];	   // flushing exhaust pipe with blower
		   break;

	  case MAIN_SUB_STOPPING:
	       text = globaltext[TEXT_MAIN_SUB_STOPPING];         // stop time
   	       break;
   	       
	  case MAIN_SUB_CRANKING:
	       text = globaltext[TEXT_MAIN_SUB_CRANKING];         // starter on
   	       break;
   	       
	  case MAIN_SUB_CRANK_PAUSE:
	       text = globaltext[TEXT_MAIN_SUB_CRANK_PAUSE];      // starter pause
   	       break;
   	       
	  case MAIN_SUB_PMS_MANUAL_START_STOP:
	       text = globaltext[TEXT_MAIN_SUB_PMS_MANUAL_START_STOP];
   	       break;

	  case MAIN_SUB_ENGINE_RUNNING:
	       text = globaltext[TEXT_MAIN_SUB_ENGINE_RUNNING];   // Engine running
   	       break;
   	       
	  case MAIN_SUB_ADJUSTING_VOLTAGE:
	       text = globaltext[TEXT_MAIN_SUB_ADJUSTING_VOLTAGE]; // adjusting voltage, Spannung wird nachgeregelt, juster spænding
   	       break;
   	       
   	       /*
	  case MAIN_SUB_ADJUSTING_VAR:
	       text = globaltext[TEXT_MAIN_SUB_ADJUSTING_VAR]; // adjusting cos phi, Cos phi wird nachgeregelt, justering cos phi
   	       break;
   	       */
   	       
	  case MAIN_SUB_ADJUSTING_FREQUENCY:
	       text = globaltext[TEXT_MAIN_SUB_ADJUSTING_FREQUENCY];// adjusting frequency
   	       break;
   	       
	  case MAIN_SUB_ADJUSTING_POWER:
	       text = globaltext[TEXT_MAIN_SUB_ADJUSTING_POWER]; // adjusting real power, Leistung wird nachgeregelt, juster effekt 
   	       break;
   	       
	  case MAIN_SUB_POWER_REDUCTION:
	       text = globaltext[TEXT_MAIN_SUB_POWER_REDUCTION]; // load reduced operation, Leistungsreduktion, drift med reduseret effekt
   	       break;
   	       
   	  case MAIN_SUB_DELOAD:
   	       text = globaltext[TEXT_MAIN_SUB_DELOAD];          //deloading for cut out, Entlasten, nedkør effekt
   	       break;
// prelubrication
	  case MAIN_SUB_INT_PRELUBRICATION:
		   text = globaltext[TEXT_MAIN_SUB_INT_PRELUBRICATION]; // interval prelubrication
		   break;

	  case MAIN_SUB_START_PRELUBRICATION:
		   text = globaltext[TEXT_MAIN_SUB_START_PRELUBRICATION]; // start prelubrication
		   break;

	  case MAIN_SUB_POST_LUBRICATION:
		   text = globaltext[TEXT_MAIN_SUB_POST_LUBRICATION]; // post lubrication
		   break;
/*   	       
	  case MAIN_SUB_MOVING_TO_POIL:
	       text = globaltext[TEXT_MAIN_SUB_MOVING_TO_POIL];  // Switchover to plant oil operation
   	       break;
   	       
	  case MAIN_SUB_MOVING_TO_DIESEL:
	       text = globaltext[TEXT_MAIN_SUB_MOVING_TO_DIESEL];// Switchover to Diesel operation, Umschaltung Dieselbetrieb, 
   	       break;
   	       
	  case MAIN_SUB_OILPUMP_ON:
	       text = globaltext[TEXT_MAIN_SUB_OILPUMP_ON]; // waste oil pump running, Altölpumpe läuft, gamel olie pump kør
   	       break;
*/   	       
   	  case MAIN_SUB_ISLAND_PARALLEL_DELOADING:
   		text = globaltext[TEXT_MAIN_SUB_ISLAND_PARALLEL_DELOADING];
   	       break;

   	  case MAIN_SUB_ISLAND_PARALLEL_OPERATION:
   		text = globaltext[TEXT_MAIN_SUB_ISLAND_PARALLEL_OPERATION];
   	       break;

   	  case MAIN_SUB_EZA_STOP:
   		text = globaltext[TEXT_MAIN_SUB_EZA_STOP];
   	       break;

   	  case MAIN_SUB_EZA_LOADREDUCTION:
   		text = globaltext[TEXT_MAIN_SUB_EZA_LOADREDUCTION];
   	       break;

      default :
   	       text = "! SUB STATE is undefined !";
           break;
   }   
   return(text);
}


   	  
 			           
           		
