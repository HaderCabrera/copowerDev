
/**
 * @file MIX.h
 * @ingroup Application
 * This is the handler for the mixture control
 * of the REC gas engine control system .
 *
 * @remarks
 * @ void MIX_control() is called from 10Hz control Task
 * @author mvo
 * 
 * @date 03-jun-2008
 * 
 * changes:
 * 29.10.2008  MVO  1240  timeout leaving lean from 1000 to 3000ms (for small mixer woodward necessary)
 * 18.11.2008  MVO  1300  constants MIX_ENRICHMENT_RATE_DUE_TO_TEMPERATURE and
 *                        MIX_REFERENCE_TEMPERATURE_ENGINE_ENTRANCE,
 *                        variable MIX.TemperatureOffset
 * 21.11.2008  MVO  1301  MIX_ENRICHMENT_RATE reduced from 0.8 to 0.3 %/K
 * 09.12.2008  MVO  1303  speed limitation for moving stepper motor MIX_MAX_SPEED_IN_CONTROL_MODE
 * 04.11.2009  GFH        deviation of receiver pressure in control mode
 *                        max deviation time changed from 30s to 3s for a faster reaction in case of a damaged cylinder inlet valve
 *                        stopcondition changed from level 7 to level 3
 * 11.03.2010  GFH  1331  change MIX_TIMEOUT_MOVING_TO_PARALLEL_POS from 90s to 1000s
 *                        change MIX_TIMEOUT_MOVING_TO_ISLAND_POS from 90s to 1000s
 * 1341  GFH  07.09.2010  release delay for monitoring of deviation
 * 1420  MVO  06.12.2012  expansion of mixer control for two mixers of a Guascor engine
 *                        The mixers can be calibrated independently. The common control loop
 *                        affects both mixers in parallel. This engine uses two mixers but
 *                        then only one common throttle.
 * 1422  GFH  16.09.2013  support of gas mixer with analogue control
 */


#ifndef MIX_H_
#define MIX_H_


#include "deif_types.h"
#include "appl_types.h"

extern void MIX_init(void);
extern void MIX_control_10ms(void);
extern void MIX_control_20ms(void);
extern void MIX_control_100ms(void);
extern void MIX_control_1000ms(void);

extern void Mix_Read_MixerSetpointsFromParameters(void);
extern void Mix_Sort_MixerSetpoints(void);
extern DS16 Mix_Calculate_Setpoint_For_Mixer_Position(DU16 Index_A, DU16 Index_B);
extern DS16 Mix_Calculate_Ramp_Position(DS16 TargetPosition, DS16 ActualPosition); // rmiMIXRAMP

extern void Mix_Calculate_Constant_pTDeviationControl(void);

extern void MIX_Set_AnalogOutZero(void);

extern DS32 Interpolate (DS32 x, 
                  DS32 x1,
                  DS32 x2,
                  DS32 y1,
                  DS32 y2);

// control Modes for MIX
enum t_MIX_mode
{
   MIX_BLOCK,                     // mixer frozen       
   MIX_MOVE_LEAN,                 // move to limit stop lean
   MIX_MOVE_RICH,                 // move to maximum rich position 100%
   MIX_MOVE_TO_START_POSITION,    // move to start position
   MIX_MOVE_TO_IDLE_POSITION,     // move to idle position
   MIX_MOVE_TO_PARALLEL_POSITION, // move to parallel position
   MIX_MOVE_TO_ISLAND_POSITION,   // move to island position
   MIX_CALIBRATE,                 // search limit stop lean, then move to rich, then to start position
   MIX_CTRL,                      // active control of mixture
   MIX_TEST_DEMANDED              // test mode demanded
};

// states of MIX
enum t_MIX_state
{
  MIX_BOOT,                       // during bootup
  MIX_SYSTEM_OFF,                 // System off
  MIX_SEARCHING_LEAN,
  MIX_LIMIT_LEAN_REACHED,
  MIX_MOVING_RICH,
  MIX_POS_RICH_REACHED,
  MIX_MOVING_TO_START_POS,
  MIX_START_POSITION_REACHED,
  MIX_MOVING_TO_IDLE_POS,
  MIX_IDLE_POSITION_REACHED,
  MIX_MOVING_TO_PARALLEL_POS,
  MIX_PARALLEL_POSITION_REACHED,
  MIX_MOVING_TO_ISLAND_POS,
  MIX_ISLAND_POSITION_REACHED,
  MIX_UNDER_CTRL,
  MIX_UNDER_TEST
};

// number of setpoints in the curve for the mixer
#define NUMBER_OF_MIXER_SETPOINTS                  8

struct t_MIX_Setpoint_Mixer 
{
   // A mixer setpoint consists of a receiver pressure and a receiver temperature at a given power
   DS32        Psum; 	// reference power in W
                        
   //rmiKW DS16        PsumRel; // reference power in % of nominal [0.01%]
   //rmiKW                      // -32000...+32000 = -320.00 ... +320.00 % of nominal
   DS16        p;       // Receiver pressure in mbar
                        // -1000...+32000 = -1.000 ... +32.000 bar relative to environment pressure
                        // absolute pressure may be calcuated from p by adding 1013mbar
   DS16        theta;   // Receiver temperature in 0,1 °C
                        // -500...+2500 = -50.0 ...+ 250.0 °C
                        // absolute temperature may be calculated by adding 2732 (=273,15K)
                        // used for cylinder combustion chamber temperature in [0.1°C] if configured
   	
};		

struct MIX_protection
{
	t_protection_state  State;
	t_protection_state  LastState;
	DU32                StateTimer;
	DS16                Limit;
	DBOOL               Exceeded;
};

struct MIX_protection_CTRLDEV
{
	t_protection_state  State;
	t_protection_state  LastState;
	DU32                StateTimer;
	DS16                Limit;
	DS16                p_dev;	// used for trending
	DS16                dp_lim;	// used for trending
	DBOOL               Exceeded;
};

struct stepperMotor {
    DBOOL forceLean;
    DBOOL reset;
    DBOOL leanReached;
    DBOOL disabled;

    DS16  setpointPosition;
    DS16  actualPosition;

    DBOOL DO_MixerClock;
    DBOOL DO_MixerDirectionLean;
};



// global Variables of MIX
typedef struct MIXstruct
{
   // input
   struct Temp_Input ReceiverTemperature;

   DBOOL     DI_LimitStopLean[2];
   DBOOL     DI_LimitStopRich;

   DS16      LambdaVoltage;
   DS16      LambdaVoltageFilteredValue;	// rmiIET

   DS16      AI_I_ReceiverPressure;        // Raw input value receiver pressure 5000 = 4mA, 25000 = 20mA
   DS16      AI_I_ReceiverPressureB;
   DS16      AI_I_LambdaVoltage;
   DS16      AI_I_GasMixerPos;

   DS16      AI_I_ReceiverTemp; // 4...20mA <=> -18...149°C

   // internal
   enum t_MIX_state  state[2];
   enum t_MIX_mode   mode[2];

   DBOOL     Manual;
   DBOOL     AdjustmentDuringStart_Possible;
   DBOOL     AdjustmentDuringStart_Activated;

   DBOOL     Fast;                                // indicates that the user wants the mixer to move fast
   DBOOL     CalibrationDone[2];
   
   DBOOL     GasMixerDirectionLeanTestDemand[2];
   DBOOL     GasMixerDirectionRichTestDemand[2];
   DBOOL     ResetStepCounter[2];
   
//   DS16      MixerPos1_IOHandler;	// mixer position 1 from iohandler
   DS32      MixerFullRange_In;
   DS32      MixerFullRange_Out;

   DS32      ActualPositionOfGasMixer[2];         // [steps]
   DS16      ActualPositionOfGasMixerPercent[2];  // [0.01%] for Modbus and displaying, calculated from steps
   DS16		 TargetMixerPosition[2];          // rmiMIXRAMP
   DS16      SetpointMixerPosition[2];
   DS16      SetpointMixerPositionPercent[2];	// [0.01%]
   DS16      TemperatureOffset;            // enrichment of gas micture due to low engine temperature [0,01%]
   //DS16		 AdditionalTemperatureOffsetTecJet; // additional enrichment due to low engine temperature in case of TecJet [0.01%]
   DS16		 AdditionalEnrichmentTecJet;	// additional enrichment in the first minutes in case of TecJet [0.01%]
   DS32		 DKFactor;
   DU32		 TecJetRampTimeStartToIdle;		// [ms] ramp time to adjust flow from start pos to idle pos

   DBOOL ReceiverTemperature_Available;

   struct t_MIX_Setpoint_Mixer Setpoint_Mixer[2][NUMBER_OF_MIXER_SETPOINTS];
   struct t_MIX_Setpoint_Mixer ActualAverage;
   
   struct MIX_protection		PositionDeviation;
   struct MIX_protection		MixerControlRelease;
   struct MIX_protection		MixerControlReleaseTimeout;

   struct MIX_protection_CTRLDEV	ControlDeviation;

   DBOOL     ValuesAreStable;              // indicates that p,T,P triples are stable enough to save a mixer setpoint
   DBOOL     Config;
   DBOOL	 EvaluateConditionsToStartMixerControl;  // marker from MAIN to tell MIX that
     	   	   	   	   	   	   	   	   	   	   	   	 // control is going to be activated

   DS16      Setpoint_Cylinder_Temperatue;

   DS16      LambdaSetp;
   DS16      PressSetp[2]; // Gas A and B
   DS16      TempSetp[2];  // Gas A and B

   DS16      Setpoint_Receiver_Pressure;
   DS16      ReceiverPressure;             // calculated from raw input value AI_I_ReceiverPressure in mbar
   DS16      ReceiverPressureFilteredValue;

   DS16      ReceiverPressureB;
   DS16      ReceiverPressureBFilteredValue;

   // Average value of receiver pressure A and B
   DS16      ReceiverPressureAvg;
   DS16      ReceiverPressureAvgFilteredValue;

   DS32  	 MaxPower;  // max allowed engine power due to internal conditions of mixer control
   DS32  	 MaxPower_Temp;  // max allowed engine power due to hot receiver temperature
   
   DU32      Flow; // [0.001m³/h]

   DU16      LambdaSetpointTecJet;   // [0.1]
   DU16      CalculatedLambda;       // [0.1]

   // TECJET diagnostic values (temporarily 24.07.2015)
   DS32		 TecJetDiagnosticsLoadDependentPart;

   // constant calculated by parameters for deviation control of p/T regulation
   DS32      Constant_pTDeviationControl;

   // added for 2 Tecjet control
   DS32      Tecjet_Max_Flow_Rate;
   DU16      Tecjet_Fraction_1; // [0.001]
   DU16      Tecjet_Fraction_2; // [0.001]

   DBOOL     Option_ENSMP;

   //output 

   DS16      AO_GasMixer[2]; // Second analog output for mixer bank B of V-Engine - Same signal as for mixer 1 even in TEST-Mode
   DS16      AO_AirMixer;
   DS16      AnalogOutZero;

   DS16      GasMixerPercent_Cummins;

   struct stepperMotor StepperMotor[2];

   DBOOL     DO_MoveDirLean;
   DBOOL     DO_MoveDirRich;
   DBOOL     DO_MoveFast;
   DBOOL     DO_LimitStopLean;
   DBOOL     DO_LimitStopRich;

} t_MIX;

extern t_MIX MIX;



extern DS32 StepperPositionSetpoint;
extern DS32 TheoreticalTemp[2];
extern DS32 TheoreticalPressure[2];
extern DS32 PressdivTemp;





// constants

// number of seconds for averaging P, p, and theta
#define MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING       60

// stability criteria (max deviation allowed)
// for power in 0.01% relative to nominal
// #define MIX_POWER_TOLERANCE                       300
// for power in mW (3% of GEN_NOMINAL_LOAD__MAX_VALUE)
#define MIX_POWER_TOLERANCE                       (GEN_NOMINAL_LOAD__MAX_VALUE*3L/100L)
// for pressure in mbar
#define MIX_PRESSURE_TOLERANCE                     100
// for temperature in 0.1 Kelvin
#define MIX_TEMPERATURE_TOLERANCE                  20

// timeout position not reached in time	
#define MIX_TIMEOUT_MOVING_TO_START_POS			630000L
#define MIX_TIMEOUT_MOVING_TO_IDLE_POS			630000L
// changed from 90s to 1000s because of ramp function
// at MIX_NUMBER_OF_STEPS_IN_RAMP = 3 the time for 10000 steps is 1000s
#define MIX_TIMEOUT_MOVING_TO_PARALLEL_POS		1000000L//90000L
// changed from 90s to 1000s because of ramp function
// at MIX_NUMBER_OF_STEPS_IN_RAMP = 3 the time for 10000 steps is 1000s
#define MIX_TIMEOUT_MOVING_TO_ISLAND_POS		1000000L//90000L
#define MIX_TIMEOUT_MOVING_TO_LEAN_POS			630000L
#define MIX_TIMEOUT_MOVING_TO_RICH_POS			630000L

// mixer offset for running a cold engine [0,1%/K below normal]
#define MIX_ENRICHMENT_RATE_DUE_TO_TEMPERATURE       3

// reference temperature engine entrance for no mixer offset (= warm enough)[0,1 °C]
#define MIX_REFERENCE_TEMPERATURE_ENGINE_ENTRANCE  700

#define MIX_TIMER_NEXT_CALIBRATION_NECESSARY	900000L

#define MIX_RECOVER_TEMP_HYST					20

#define MIX_TIMEOUT_LEAVING_LEAN_POS			21000L

// deviation of receiver pressure in control mode
// max deviation time changed from 30s to 3s for a faster reaction in case of a damaged cylinder inlet valve
// stopcondition changed from level 7 to level 3 
#define MIX_MAX_DEVIATION_TIME					3000L//30000L
#define MIX_DEVIATION_RELEASE_DELAY				30000L

#define MIX_MAX_LOSS_OF_STEPS					300L		
/*
#define MIX_NUMBER_OF_STEPS_IN_TEST_MODE		100
#define MIX_NUMBER_OF_STEPS_IN_CONFIGURATION     10
*/

// maximum speed of stepper motor during active control [%/s]
#define MIX_MAX_SPEED_IN_CONTROL_MODE            0.5

// step in 0.01% of max steps -> from lean to rich = 100%
// 1%
#define MIX_STEP_IN_TEST_MODE		            100L
//0.05%
#define MIX_STEP_IN_CONFIGURATION                 5L
#define MIX_BIG_STEP_IN_CONFIGURATION            50L

// number of staps in ramp (when moving to parallel position), rmiMIXRAMP
#define MIX_NUMBER_OF_STEPS_IN_RAMP               3

// how many times should the Receiverpressure be taken into the average for creating the filtered value?
#define MIX_NUMBER_OF_RECP_VALUES_FOR_FILTERING  50
// how many times should the LambdaVoltage be taken into the average for creating the filtered value?
#define MIX_NUMBER_OF_LV_VALUES_FOR_FILTERING    10

// calculate new setpoint for gas mixer position
#define MIX_TIMER_CALCULATE_NEW_SETPOINT_POS	0L

#define MixerInd1                                  0
#define MixerInd2                                  1

#define MIX_ANALOG_OUT_ZERO				5000
#define MIX_ANALOG_OUT_FULL				25000

// [0.01%]
#define MIX_POSITION_DEVIATION_LIMIT	500
#define MIX_POSITION_DEVIATION_DELAY	10000L

// release active mixer control, raw value
#define MIX_ACCEPTED_MAX_TO_START_CONTROL		23000L
// release active mixer control, delay [ms]
#define MIX_DELAY_FOR_STARTING_LANBDA_CONTROL   60000L

// timeout supervision for lambda control activation [ms]
#define MIX_TIMEOUT_FOR_STARTING_LAMBDA_CONTROL 600000L

// power limitation [ 0.01% of nominal above relased load] in case of bad lambda signal
#define MIX_POWER_OFFSET_PERCENT_IF_BAD_SIGNAL      500L

// [0.01]
#define MIX_LAMBDA_STEP_IN_CONFIG				1
// [0.1rpm]
#define MIX_RELEASE_SPEED_FOR_LAMBDA_CONTROL	5000

// TecJet handling
// additional enrichment for a duration of [ms] after engine running
//#define MIX_TEC_ADDITIONAL_START_ENRICHMENT_TIME		180000L
// temporarily disabled
#define MIX_TEC_ADDITIONAL_START_ENRICHMENT_TIME		100L
// how much [0.01%]
//#define MIX_TEC_ADDITIONAL_START_ENRICHMENT_FRRACTION		4000L
#define MIX_TEC_ADDITIONAL_START_ENRICHMENT_FRRACTION		1L

// [ms]
#define MIX_TEC_MIN_RAMP_TIME_FROM_STRT_TO_IDLE				1000L
#define MIX_TEC_MAX_RAMP_TIME_FROM_STRT_TO_IDLE				300000L

// Calibration of gas mixer before start or after engine stop
#define MIX_CALIBRATION_AFTER_ENGINE_STOP	TRUE

// Option TecJet
#define MIX_OPTION_TECJET	(tecjet[TECJET_1].Option || tecjet[TECJET_2].Option)

// RegMode DO
#define MIX_REG_DO_RESET			0
#define MIX_REG_DO_REG_MANUAL       1
#define MIX_REG_DO_REG_POS			2
#define MIX_REG_DO_REG_CTR			3
// [ms]
#define MIX_REG_DO_PERIOD			3000

#endif /*MIX_H_*/


