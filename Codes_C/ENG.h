
/**
 * @file ENG.h
 * @ingroup Application
 * This is the handler for the engine data
 * of the Kraft PHKW project
 *
 * @remarks
 * @ void ENG_control_100ms() is called from 10Hz control Task
 * @author mvo
 * @date 09-may-2007
 *
 * Changes:
 * 04.01.2008  MVO  1006  definition of engine types added
 * 06.03.2008  RMI  1011  ELM.h must be included, because definition of it used here
 * 1011  03.04.2008  GFH  add defines for selection of rpm-input
 * 1240  29.10.2008  MVO  ENG_WARMING_HYST increased from 5K to 15K
 * 1300  19.11.2008  MVO  ENG_WARMING_HYST increased from 15K to 20K
 * 1310  11.03.2009  GFH  new internal variable "ENG.KnockingSignal"
 * 1313  28.04.2009  RMI  DO_Engine_Running, rmiERU
 * 1313  04.05.2009  RMI  ENG_RUNNING_SPEED replaced by parameter, rmiIET
 * 1326  09.12.2009  RMI  flushing time replaced by parameter
 * 1328  05.02.2010  GFH  operating counters at fixed NOVRAM area
 * 1328  12.02.2010  GFH  #define ENG_IDLE_SPEED 14900
 * 1343 08.11.2010 GFH  option: cylinder temperature monitoring
 * 1343 08.11.2010 GFH  start attempt counter - only one in manual mode
 */

#ifndef ENG_H_
#define ENG_H_


#include "deif_types.h"
#include "appl_types.h"
#include "ELM.h"				//definition of  t_protection_state

extern void ENG_init(void);
extern void ENG_control_100ms(void);
extern void ENG_control_1000ms(void);

extern void ENG_Set_FlushingTime(void);
extern DBOOL ENG_GetEngRunning(void);
extern DBOOL ENG_GetRefillingStateColdDI(void);
extern DBOOL ENG_GetRefillingStateColdAI(void);

// control modes for ENG
enum t_ENG_mode
{
   ENG_OFF,
   ENG_START_DEMANDED,
   ENG_TEST_DEMANDED
};

// states of ENG
enum t_ENG_state
{
   ENG_BOOT,
   ENG_ALL_OFF,
   ENG_PRECRANK,
   ENG_START_CRANKING,
   ENG_FLUSHING,
   ENG_CRANK,
   ENG_COOLDOWN_RUN,
   ENG_CRANK_PAUSE,
   ENG_RUNNING,
   ENG_STOPPING,
   ENG_UNDER_TEST
};

// operating hours
typedef struct
{
	DU32 hours;
	DU32 milliseconds;

	DTIMESTAMP TimestampOfReset;
}operating_hours;

typedef enum{NOT_ACTIVE, CHECK_LOW_IDLE, LOW_IDLE_ACTIVE, CHECK_NORMAL, NORMAL_ACTIVE} t_activation_state;

struct activation_vars
{
	t_activation_state State;
	t_activation_state LastState;
    DU32               StateTimer;
};

// internal values of each protection
struct protection_vars
{
	t_protection_state State;
    t_protection_state LastState;
    DU32               StateTimer;
    DS16               Limit;
    DBOOL              Exceeded;
};

// internal values of each protection
struct protection_DI
{
	t_protection_state State;
    t_protection_state LastState;
    DU32               StateTimer;
    DBOOL              Exceeded;
};

// protection functions for each DS16 analog value
struct protection_variables_ChargeAir
{
	struct protection_vars     TooLow;
	struct protection_vars     TooHigh;
};

// protection functions for each DS16 analog value, only too high
struct protection_variables_ChargeAirT
{
	struct protection_vars     TooHigh;
};

// protection functions for each DS16 analog value, only Max
struct protection_variables_EngTempInlet
{
	struct protection_vars     Max;
};

// protection functions for each DS16 2x too high
struct protection_variables_LubeOil
{
	struct protection_vars     LoadReduction;
	struct protection_vars     Max;
};

// protection functions for load reduction on digital input
struct protection_variables_DI
{
	struct protection_DI     LR_30Percent;
	struct protection_DI     LR_60Percent;
};

struct misfire_vars
{
	DBOOL Active;
	DBOOL Fault;
	DBOOL Cylinder[12+1];
	DS32  RecoverTime[12+1];
	DU32  Counter[12+1];
	DU8   Status;
	DU8   StatusOld;
};

// global Variables of ENG
typedef struct ENGstruct
{
   // inputs
   struct Temp_Input CoolingWaterEngineEntrance;
   struct Temp_Input CoolingWaterEngineExit;
   struct Temp_Input LubeOilTemp;

   DS16  S200EngineSpeedRaw;
   DS16  S200EngineSpeed;                // in [0.1rpm]

   //DS16  AI_I_CoolingWaterPressure;	// rmi: now in WAT
   DS16  AI_I_LubeOilLevel;

   DS16  T203InletAir;
   DS16  T204ChargeAir;

   DS16  AI_I_LubeOilRaw;
   DS16  AI_I_CrankCasePressure;

   DS16  AI_I_LubeOilBSideRaw;	// 0...25000 = 0...20mA

   DS16  AI_I_CoolingWaterTempOut;	// 4...20mA <=> -18...149°C
   DS16  AI_I_LubeOilTemp;			// 4...20mA <=> -18...149°C

   DS16  P207ChargeAirPressureRaw;

   DBOOL DI_LubeOilLevelMin;
   DBOOL DI_LubeOilLevelMax;

   DBOOL DI_LoadReduction30Percent;
   DBOOL DI_LoadReduction60Percent;

   DBOOL DI_PowerStartUnitReady;

   DBOOL DI_KleemannMisfireStatus[5];

   DBOOL DI_KleemannNOxActive;
   DBOOL DI_KleemannNOxWarning;
   DBOOL DI_KleemannNOxAlarm;
   DBOOL DI_KleemannNOxFault;

   // internal
   enum t_ENG_mode  mode;
   enum t_ENG_state state;
   DBOOL IgnitionTestDemand;
   DBOOL StarterTestDemand;
   DBOOL PowerStartUnitTestDemand;
   DBOOL PreglowTestDemand;   // not used in hmi test page
   DBOOL NoCrankingSpeed;					// rmi: used to set SC: no cranking speed detected
   DBOOL Running;
   DU32  EngRunningCounter;
   DBOOL OilPressureMinSupervision_active;
   operating_hours TotalRunningTime;
   DU8   MaxNumberOfStartAttempts;
   DU8   StartAttemptsCounter;
   DU32  StartAttemptsCounterTotal;
   DS16  P205LubeOil;
   DS16  P205LubeOil_Filtered;
   DS16  LubeOilBSide;				// calculated from AI_I_LubeOilBSideRaw [0.01bar]
   DS16  LubeOilBSide_Filtered; 	// [0.01bar]
   DS16  OilPressureMinLimit;
   DS16  P207ChargeAirPressure;
   DS16  T201_Filtered;
   DS16  T201_Gradient;
   DBOOL WarmingDone;
   DS16  LubeOilLevel;
   DS16  CrankCasePressure;

   DU32  FlushingTime;

   DBOOL CoolingWaterEngineEntrance_Available;
   DBOOL CoolingWaterEngineExit_Available;
   DBOOL LubeOilTemp_Available;

   DBOOL KlemmanNOxResetDemand;
   DS32  KlemmanNOxResetTime;

   DTIMESTAMP EM_ZAK_2_DateTimeOfLastReset;
   DU32       EM_ZAK_2_OperatingHoursSinceLastReset;

   struct protection_variables_ChargeAir   	ChargeAir;
   struct protection_variables_ChargeAirT  	T204ChargeAirSupervision;
   struct protection_variables_LubeOil     	LubeOilSupervision;
   struct protection_variables_LubeOil     	T202Supervision;
   struct protection_variables_EngTempInlet EngTempInletSupervision;	//rmi, only Max
   struct protection_variables_DI           DI_Supervision;
   struct protection_vars                   LubeOilRefilling_DI;
   struct protection_vars                   LubeOilRefilling_AI;
   struct protection_vars                   PickupSignalMissing;
   struct protection_vars					CrankingSpeedSupervision;	//rmi
   struct protection_vars   				MainsStartDevNotReady;
   struct activation_vars   				OilPressureMinSupervision_Activation;

   struct misfire_vars Misfire;

   DBOOL LubeOilLevelMin_DI;

   DS32  MaxPower;       // power reduction due to hot oil temperature
   DS32  MaxPower_T202;  // power reduction due to hot cooling water engine exit
   DS32  MaxPower_DI_30Percent;
   DS32  MaxPower_DI_60Percent;

   DBOOL TimeSwitchManual;

   DBOOL TimeSwitchStart_HMI_Demand;
   DBOOL TimeSwitchStop_HMI_Demand;

   DBOOL NovUpdateRequired;

   // Outputs
   DBOOL DO_Starter;
   DBOOL DO_Engine_Running;							// rmiERU
   //DBOOL Ignition;
   DBOOL DO_PowerStartUnit;
   DBOOL DO_TimeSwitch;
   DBOOL DO_KleemannNOxReset;

   DBOOL Preglow;             // not assigned to an output

} t_ENG;



/* declaration of operating hours counter inside NOVRAM */
typedef struct
{
	operating_hours operating_time;
	DU32 start_attempts_total;
}t_nov_eng;

extern t_nov_eng eng_nov;
extern t_ENG ENG;

extern void ResetNovEng(void);

// Factor to calculate Recover_Level
#define   ENG_RECOVER_HYST                   200

// Speed values refering to DS16 ENG.S200EngineSpeedRaw
// given by the analog input
// Scaling of IOM4.2: -25000...+25000 = -20mA...+20mA
// Phoenix converter set to  4mA = 0rpm    = + 5000
// Phoenix converter set to 20mA = 2000rpm = +25000
// So value is 5000 + (rpm * 10).
      // stopped means less then 40rpm
//#define   ENG_STOPPED_SPEED                  5400
#define   ENG_STOPPED_SPEED                   400

      // delay for pickup signal missing (in ms)
#define   ENG_PICKUP_SIGNAL_MISSING_DELAY     500L

// recover time of NoCrankingSpeed: 5 minutes, rmi
#define   ENG_NO_CRANKING_SPEED_RECOVER_TIME   300000L

      // starter must have minimum 80 rpm
//#define   ENG_STARTER_SPEED                  5800
#define   ENG_STARTER_SPEED                   800
      // running is considered 350 rpm and up
//#define   ENG_RUNNING_SPEED                  8500
//rmiIET,04.05.09, 50kW-engine: engine running speed exceeded when flushing  #define   ENG_RUNNING_SPEED                  3500
      // idle speed is where you can turn on the frequency regulation 1300 rpm
//#define   ENG_IDLE_SPEED                    18000
//#define   ENG_IDLE_SPEED                    14900
#define	  ENG_IDLE_SPEED_PERCENT			993			// [1/10 %] 99.3 % of nominal speed

      // timer for preglowing time in ms
#define   ENG_PREGLOWING_TIME                   0L
      // timer for precrank time (ignition on before start)
#define   ENG_PRECRANK_TIME                  1000L
      	  	  // timeout for speed signal after turning on starter (timeout Start_Cranking)
				//#define   ENG_STARTER_SPEED_DETECTION_TIME   1500L
				// replaced by parameter 14071
      // timeout for cranking (max starter on time, timeout Cranking)
#define   ENG_CRANKING_TIME                 12000L
   // timeout for cranking in test mode (max starter on time, timeout Cranking)
#define   ENG_CRANKING_TIME_IN_TEST_MODE    20000L
      // Cranking pause time (timeout Ign_Pause)
#define   ENG_CRANKING_PAUSE                12000L
      // Timeout Cooldown_Not_Run (wait if engine will run after loosing running signal)
#define   ENG_TIMEOUT_COOLDOWN_NOT_RUN       3000L
      // Timeout Cooldown_Run (wait if engine is running stable)
#define   ENG_TIMEOUT_COOLDOWN_RUN           3000L
      // Flushing time, rmi091209: replaced by parameter: ENG_FLUSHING_TIME__PARREFIND
//#define   ENG_FLUSHING_TIME                  4000L
      // Number of start attempts
#define   ENG_NUMBER_OF_START_ATTEMPTS          3
      // Timeout Stopping:
#define   ENG_STOPPING_TIME                 25000L

      // scaling factor for P205 lube oil pressure
      // Raw data is -25000...25000 = -20mA ...+20mA
      // Sensor is 4...20mA = 0.00 ... 10.00 bar
      // factor = sensor range/raw value range = 1000 / 20000

      // changed for plant Kraft 10.10.2007
      // 4...20mA = 0.00 ... 16.00 bar
#define   ENG_LUBEOILPRESFACTOR               0.08

      // scaling factor for P207 charge air pressure
      // Raw data is -25000...25000 = -20mA ...+20mA
      // Sensor is 4...20mA = 0.00 ... 4.00 bar
      // factor = sensor range/raw value range = 400 / 20000
#define   ENG_CHARGEAIRPRESFACTOR               0.02

      // Lubeoil min pressure limit in grid parallel operation in bar/100
      // value 3.5bar for MAN2842 Dirk Richter 03.Aug.2007
#define   ENG_LUBE_OIL_MIN_GRID_PARALLEL     350

      // Lubeoil min pressure limit when running in bar/100
      // value 3.0bar for MAN2842 Dirk Richter 03.Aug.2007
#define   ENG_LUBE_OIL_MIN_RUNNING           300
      // activation of the min oil pressure after engine running, in ms,
      // running in the 1000ms task
#define   ENG_LUBE_OIL_MIN_ACTIVATION_DELAY  5000L

      // Lubeoil max pressure limit in bar/100
#define   ENG_LUBE_OIL_MAX                   900
      // activation temperature limit for lubeoil overpressure: above (in 0.1°C)
#define   ENG_LUBE_OIL_MAX_ACTIVATION_TEMPERATURE         750

      // power reduction due to oil overtemperature, unit is W per 0,1 degree deviation,
      // example: 2500 = 25kW per degree
#define   ENG_POWER_REDUCTION_PER_DEGREE_OIL_TEMP             2500L
      // start derating at (in 0.1°C)
#define   ENG_START_DERATING_POWER_AT_OIL_TEMP                1050
#define   ENG_START_DERATING_POWER_AT_OIL_TEMP_DELAY           500L
#define   ENG_START_DERATING_POWER_AT_OIL_TEMP_RECDELAY       5000L

      // stop limit for oil temperature too hot, in 0.1°C
#define   ENG_MAX_OIL_TEMPERATURE_TRIP                        1200
#define   ENG_MAX_OIL_TEMPERATURE_DELAY                       1000L
#define   ENG_MAX_OIL_TEMPERATURE_RECOVER                     1000
#define   ENG_MAX_OIL_TEMPERATURE_RECDELAY                  300000L


      // Charge air min pressure limit in grid parallel operation in bar/100
      // value 0.8bar, nominal is 1.15bar
      // only active above a certain power (in W)
//#define   ENG_CHARGE_AIR_MIN_GRID_PARALLEL    80
//#define   ENG_CHARGE_AIR_MIN_ACTIVATION_POWER    250000L
//#define   ENG_CHARGE_AIR_MIN_GRID_PARALLEL_DELAY    20000L

      // Charge Air max pressure limit in bar/100
//#define   ENG_CHARGE_AIR_MAX                 150
//#define   ENG_CHARGE_AIR_MAX_DELAY         10000L

// Charge air temperature limit in 0.1 °C
//#define   ENG_CHARGE_AIR_TEMP_MAX            650
//#define   ENG_CHARGE_AIR_TEMP_MAX_DELAY     1000L

      // power reduction due to cooling water temperature at engine exit, unit is W per 0,1 degree deviation,
      // example: 3000 = 30kW per degree
#define   ENG_POWER_REDUCTION_PER_DEGREE_WATER_TEMPERATURE_EXIT             3000L
      // start derating at (in 0.1°C)
#define   ENG_START_DERATING_POWER_AT_WATER_TEMPERATURE_EXIT                 870
#define   ENG_START_DERATING_POWER_AT_WATER_TEMPERATURE_EXIT_DELAY           500L
#define   ENG_START_DERATING_POWER_AT_WATER_TEMPERATURE_EXIT_RECDELAY       5000L

      // stop limit for water temperature engine exit too hot, in 0.1°C
#define   ENG_MAX_WATER_TEMPERATURE_EXIT_TRIP                                980
#define   ENG_MAX_WATER_TEMPERATURE_EXIT_DELAY                              1000L
//#define   ENG_MAX_OIL_TEMPERATURE_RECOVER                     1000
//#define   ENG_MAX_OIL_TEMPERATURE_RECDELAY                  300000L

#define   RPM_ANALOG		0
#define   RPM_PICKUP		1
#define   RPM_CAN			2

// min stab time for (temperature T201 > (setpoint T201-xK))
#define ENG_MIN_WARMING_TIME            60000L
#define ENG_WARMING_HYST                200

#define ENG_R_TYPE		1L
#define ENG_V_TYPE		2L

// load reduction on digital input
#define   ENG_SUPERVISION_LR_ON_DI_DELAY           1000L
#define   ENG_SUPERVISION_LR_ON_DI_RECOVER         1000L


#define CRANK_CASE_PRESSURE_SENSOR_VALUE_4MA		-300	//in unit [1/10mbar]
#define CRANK_CASE_PRESSURE_SENSOR_VALUE_20MA		 300	//in unit [1/10mbar]

//#define ENG_TYPE_V	0
//#define ENG_TYPE_R	1

// [ms]
#define POWER_START_NOT_READY_DELAY			1000L
#define POWER_START_NOT_READY_REC_DELAY		1000L

#endif /*ENG_H_*/
