/**
 * @file MIX.c
 * @ingroup Application
 * This is the handler for the gas mixture control system
 * of the REC gas engine control system.
 *
 * @remarks
 * This module is called every 100ms.
 *
 * @author mvo
 * @date 03-jun-2008
 * 
 * 
 * SW   author   date     content
 * 1237  MVO  11.09.2008  STOPCONDITION_70244 introduced = limit stops rich and lean at the same time
 * 1239  MVO  24.10.2008  mixer position for start, idle, parallel, island shifted by CH4.MixerOffset
 * 1300  MVO  18.11.2008  MIX.TemperatureOffset = gas Enrichment on a cold engine
 *                        function Mix_Calculate_Enrichment
 * 1301  MVO  20.11.2008  correction of idle and parallel positions
 * 1302  MVO  27.11.2008  while-loop corrected in MIX_Control (NUMBER... -1)
 * 1303  MVO  09.12.2008  regulation direction of lambda control inverted to fit BOSCH LSM and LSU sensors
 * 1303  MVO  09.12.2008  speed limitation for moving stepper motor
 * 1304  MVO  15.12.2008  correction of idle and parallel positions
 * 1307  MVO  23.12.2008  speed limitation for moving stepper motor deactivated
 * 1309  GFH  24.02.2009  calculate new mixer setpoint SIG.ENTRY in state "MIX_MoveStart" and "MIX_MoveIdle" and "MIX_MoveParallel" and "MIX_MoveIsland"
 * 						  calculate new mixer setpoint SIG.ENTRY and SID.DO in state "MIX_StartPositionReached" and "MIX_IdlePositionReached" and "MIX_ParallelPositionReached" and "MIX_IslandPositionReached
 * 1309  GFH  26.02.2009  new calculation of mixer setpoint (DS32)
 * 1309  GFH  02.03.2009  limit MIX.SetpointMixerPosition to max number of steps
 * 1313  RMI  04.05.2009  MIX_CONTROL: max deviation check only if NO lambda control, lambda Kp,Ki,Kd/10000, rmiIET
 * 1314  RMI  06.07.2009  StepperPositionSetpoint limited (GFH)
 * 1320  MVO  10.09.2009  Allow negative step counter when moving lean in test mode
 *       GFH  04.11.2009  deviation of receiver pressure in control mode
 *                        max deviation time changed from 30s to 3s for a faster reaction in case of a damaged cylinder inlet valve
 *                        stopcondition changed from level 7 to level 3
 * 1326  RMI  15.12.2009  Mix_Calculate_Setpoint_For_Mixer_Position returns Setpoint, 
 * 						  Mix_Calculate_Ramp_Position in MoveParallel and MoveIsland, rmiMIXRAMP
 * 						  internal StepperPosition100, to take care, that low Ki-values take effect in lambda control
 * 1331  GFH  11.03.2010  wait until desired mixer position is reached before next ramp
 * 1341  GFH  07.09.2010  release delay for monitoring of deviation
 * 1341 22.09.2010 GFH	HGS: no sensor defect of lambda voltage if GCB is open
 * 1410 22.03.2012 GFH  weaken intensity of regulator by factor 10
 * 1420 06.12.2012 MVO  expansion of mixer control for two mixers of a Guascor engine
 *                      The mixers can be calibrated independently. The common control loop
 *                      affects both mixers in parallel. This engine uses two mixers but
 *                      then only one common throttle.
 * 1420 12.12.2012 GFH  two mixers for Guascor engine
 * 1421 22.03.2013 MVO  big or small steps possible in EMIS Config using MIX.Fast
 * 1422 16.09.2013 GFH  support of gas mixer with analogue control
 * 	    15.03.2016 MVO  due to problems with the position feedback of the pro act on site Ortdogu:
 *						using the throttle position setpoint instead of the feeback for the calculation of the
 *						TecJet flow setpoint
 * 436729 05.10.2016 MVO  test version 989 for IET: adjustable ramp time for moving
 *		  				  tec jet from start flow to idle flow
 *		  30.12.2016 MVO  limit stop supervision off if TecJet
 *		  30.12.2016 MVO  TecJet Maxflow depending on selection of Tecjet 1,2 or both
 */
 
#include <stdio.h>
#include <stdlib.h>

#include "options.h"
#include "deif_types.h"
#include "appl_types.h"
#include "iohandler.h"
#include "STOPCONDITIONS.h"
#include "statef.h"
#include "debug.h"
#include "CH4.h"
#include "O2.h"
#include "CYL.h"
#include "DK.h"
#include "ENG.h"
#include "MIX.h"
#include "PAR.h"
#include "IOA.h"
#include "GAS.h"
#include "GBV.h"
#include "HVS.h"
#include "TEC.h"
#include "TUR.h"

#define CALCULATION_FACTOR ((DF32)CH4.CH4Value * 9.356 / 100000)
/*to_do_zzh for O2*/
#define TEMP_COMPENSATION_1  (((DF32)((DS16)tecjet[TECJET_1].read.FuelTemperature - 40)*10 + 2730)/((DF32)MIX.ReceiverTemperature.Value + 2730))
#define TEMP_COMPENSATION_2  (((DF32)((DS16)tecjet[TECJET_2].read.FuelTemperature - 40)*10 + 2730)/((DF32)MIX.ReceiverTemperature.Value + 2730))

// Module Macros

// State function declaration
static void Transit( const STATE2 newState, DU8 mixer );

// State function declaration
static void MIX_SystemOff(const DU8 sig, DU8 mixer);
static void MIX_SearchLimitLean(const DU8 sig, DU8 mixer);
static void MIX_PositionLeanReached(const DU8 sig, DU8 mixer);
static void MIX_MoveRich(const DU8 sig, DU8 mixer);
static void MIX_PositionRichReached(const DU8 sig, DU8 mixer);
static void MIX_MoveStart(const DU8 sig, DU8 mixer);
static void MIX_StartPositionReached(const DU8 sig, DU8 mixer);
static void MIX_MoveIdle(const DU8 sig, DU8 mixer);
static void MIX_IdlePositionReached(const DU8 sig, DU8 mixer);
static void MIX_MoveParallel(const DU8 sig, DU8 mixer);
static void MIX_ParallelPositionReached(const DU8 sig, DU8 mixer);
static void MIX_MoveIsland(const DU8 sig, DU8 mixer);
static void MIX_IslandPositionReached(const DU8 sig, DU8 mixer);
static void MIX_Control(const DU8 sig, DU8 mixer);
static void MIX_UnderTest(const DU8 sig, DU8 mixer);

// MIX data structure for global use
t_MIX MIX;

DS32 StepperPositionSetpoint;
DS32 TheoreticalTemp[2];
DS32 TheoreticalPressure[2];
DS32 PressdivTemp;

// Local variables
static STATE2 myState[2];
//static STATE2 myLastState[2];
static DU32   myStateCnt[2];

DS32  reduction;      // power reduction in W from this unit

static DU8   MIX_RingBufferPointer;
static struct t_MIX_Setpoint_Mixer MIX_RingBuffer[MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING];
static DBOOL MixFirstCalibration[2];
static DU32 TimerCalculateNewMixerPosition[2];

static DBOOL MIX_SetpointInitDone[2];

static void MIX_StepperMotorControl(DU8 MixerIndex);

// Local function declaration

static void MIX_Regulation_DO(DU8 reg_mode)
{
	DS16 Change;
	DS16 Deviation;
	DS32 ConstP;

	static DS16  DeviationOld;
    static DS32  PulseTimeLeft;
    static DU32  TimeLeftTilNextCalculation;
    static DBOOL DirectionLean;
    static DBOOL DirectionRich;

    if (!MIX.Option_ENSMP OR (reg_mode == MIX_REG_DO_RESET))
	{
		DeviationOld               = 0;
	    PulseTimeLeft              = 0L;
	    TimeLeftTilNextCalculation = 0L;
		DirectionLean              = FALSE;
		DirectionRich              = FALSE;
		MIX.DO_MoveDirLean         = OFF;
		MIX.DO_MoveDirRich         = OFF;
	}
	else if (reg_mode == MIX_REG_DO_REG_MANUAL)
	{
		if (MIX.GasMixerDirectionLeanTestDemand[MixerInd1])
		{
			MIX.GasMixerDirectionLeanTestDemand[MixerInd1] = FALSE;

        	MIX.DO_MoveDirLean = ON;
        	MIX.DO_MoveDirRich = OFF;

			if (MIX.Fast) PulseTimeLeft = 1000;
			else          PulseTimeLeft = 100;
		}

		if (MIX.GasMixerDirectionRichTestDemand[MixerInd1])
		{
			MIX.GasMixerDirectionRichTestDemand[MixerInd1] = FALSE;

			if (MIX.Fast) PulseTimeLeft = 1000;
			else          PulseTimeLeft = 100;

        	MIX.DO_MoveDirRich = ON;
        	MIX.DO_MoveDirLean = OFF;
		}

		if (PulseTimeLeft > 0)
			PulseTimeLeft -= 100;
		else
		{
			PulseTimeLeft = 0;
        	MIX.DO_MoveDirLean = OFF;
        	MIX.DO_MoveDirRich = OFF;
		}

	}
	else // if ( (reg_mode == MIX_REG_DO_REG_POS) OR (reg_mode == MIX_REG_DO_REG_CTR) )
	{
        // set outputs for direction lean/rich here...
        if ( ( DirectionLean ) && ( PulseTimeLeft >= 100 ) )
        {
        	MIX.DO_MoveDirLean = ON;
        	PulseTimeLeft -= 100;
        }
        // run out or not activated
        else MIX.DO_MoveDirLean = OFF;

        if ( ( DirectionRich ) && ( PulseTimeLeft >= 100 ) )
        {
        	MIX.DO_MoveDirRich = ON;
        	PulseTimeLeft -= 100;
        }
        // run out or not activated
        else MIX.DO_MoveDirRich = OFF;

        if ( TimeLeftTilNextCalculation < 100)
        {
        	if (reg_mode == MIX_REG_DO_REG_POS)
        	{
                Deviation = MIX.ActualPositionOfGasMixerPercent[MixerInd1] - MIX.SetpointMixerPositionPercent[MixerInd1];
        	}
        	else // if (reg_mode == MIX_REG_DO_REG_CTR)
        	{
        		Deviation = (MIX.Setpoint_Receiver_Pressure - MIX.ReceiverPressureAvgFilteredValue)/5;
        	}

         	TimeLeftTilNextCalculation = MIX_REG_DO_PERIOD + 100L;

         	Change = Deviation - DeviationOld;
         	DeviationOld = Deviation;

         	if ((DO_FUNCT[IOA_DO_MIX_MOVE_FAST].Assigned == ASSIGNED) && MIX.DO_MoveFast)
         		ConstP = 10;
         	else
         		ConstP = 50;

         	PulseTimeLeft = (Deviation * ConstP + Change);
         	DirectionRich = FALSE;
         	DirectionLean   = FALSE;
         	if ( PulseTimeLeft < -100 )
         	{
         		DirectionRich = TRUE;

         		PulseTimeLeft = - PulseTimeLeft;
         	}
         	else if ( PulseTimeLeft > 100 )
         	{
         		DirectionLean = TRUE;
         	}
         	// else deviation within limits: do not move
        }

        TimeLeftTilNextCalculation -= 100;
	}
}

static void SetMaxFlowRateTecJet(void)
{
	if (tecjet[TECJET_1].Option && !tecjet[TECJET_2].Option)
	{
		// only tecjet 1
		MIX.Tecjet_Max_Flow_Rate = (DS32)PARA[ParRefInd[TEC_MAX_FLOW__PARREFIND]].Value;
	}
	else if (!tecjet[TECJET_1].Option && tecjet[TECJET_2].Option)
	{
		// only tecjet 2
		MIX.Tecjet_Max_Flow_Rate = (DS32)PARA[ParRefInd[TEC_2_MAX_FLOW__PARREFIND]].Value;
	}
	else
	{
		// either no tectjet or two of them
		MIX.Tecjet_Max_Flow_Rate =
				(DS32)PARA[ParRefInd[TEC_MAX_FLOW__PARREFIND]].Value + (DS32)PARA[ParRefInd[TEC_2_MAX_FLOW__PARREFIND]].Value;
	}
}


/*
#define tmin	500		// to be replaced by parameter
#define dpmin	250		// to be replaced by parameter
#define dpmax3s	2000	// to be replaced by parameter
#define k		((3000 - tmin) * (dpmax3s - dpmin))	// constant calculated by parameters
*/
#define tmin	PARA[ParRefInd[MIX_DEV_MIN_TIME__PARREFIND]].Value
#define dpmin	PARA[ParRefInd[MIX_MAX_DEVIATION__PARREFIND]].Value
#define dpmax3s	PARA[ParRefInd[MIX_DEV_MAX_DP_3SEC__PARREFIND]].Value
#define k		MIX.Constant_pTDeviationControl
// Check maximum control deviation

#ifdef a
#undef a
#endif
#define a MIX.ControlDeviation

static void MIX_ControlDeviation (void)
{
	DU32 DelayTime;
	DS16 Deviation;

	static DU32 WaitBeforeRelease = 0L;

	static DS32 dp_int = 0L;
	static DU32 t = 0L;

	// not under control
	if (MIX.state[MixerInd1] != MIX_UNDER_CTRL)
	{
		a.State = COLD;
		a.StateTimer  = 0L;

		WaitBeforeRelease = 0L;

		// reset dp_int and t for p/T-Control
		dp_int = 0;
		t = 0L;

		// set a.p_dev and a.dp_lim for trending
		a.p_dev = 0;
		a.dp_lim = MAX_DS16;

		STOP_Tripped[STOPCONDITION_30231] = FALSE;
	}
	else // under control
	// stay here for delay time until monitoring is released
	if ( WaitBeforeRelease < MIX_DEVIATION_RELEASE_DELAY)
	{
		WaitBeforeRelease += 100L;
	}
	else // under control and released
	{
		// if state has changed, clear state timer and remember new state
		if (a.State != a.LastState )
		{
			a.StateTimer = 0;
			a.LastState  = a.State;
		}
		else // count the state timecounter by adding 100ms
		{
			if (a.StateTimer < (MAX_DU32-1000)) a.StateTimer += 100L;
		}

	   // lambda
	   if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 1L)
	   {
		   // no monitoring
		   DelayTime = MIX_MAX_DEVIATION_TIME;
		   Deviation = 0;
		   a.Limit = MAX_DS16;
	   }
	   // p/T
	   else if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 2L)
	   {
		   DS32 LimitDS32;

		   /* monitoring absolute deviation of receiver pressure in control mode
		   DelayTime = MIX_MAX_DEVIATION_TIME;
		   Deviation = abs(MIX.Setpoint_Receiver_Pressure - MIX.ReceiverPressureFilteredValue);
		   a.Limit = (DS16)PARA[ParRefInd[MIX_MAX_DEVIATION__PARREFIND]].Value;
		   */

		   // monitoring of delta pressure * time (|dp|*t)
		   // parameters
		   // tmin			:= min time before protection
		   // dpmin			:= min pressure deviation before protection
		   // dpmax3s		:= parameter to define protection curve
		   // |dp|			:= actual absolute deviation between setpoint and actual pressure
		   // t				:= actual time when protection is activated
		   // limit curve is defined by
		   // a.Limit = dpmin + k / (t - tmin)
		   // k = (3000ms - tmin) * (dpmax3s - dpmin); constant
		   // dp_int	:= integrator of |dp|
		   // dp_int = dp_int + (|dp| - dpmin);
		   // dp_int is limited to zero
		   // t is set back to zero when (dp_int <= 0)

		   DelayTime = 0L;
		   Deviation = abs(MIX.Setpoint_Receiver_Pressure - MIX.ReceiverPressureAvgFilteredValue);

		   // dp_int
		   // increasing if (|dp| > dpmin)
		   // decreasing if (|dp| < dpmin)
		   dp_int += (Deviation - dpmin);

		   if (dp_int < 0) dp_int = 0;

		   // reset time
		   if (dp_int <= 0) t = 0L;
		   // count time
		   else             t += 100L;

		   // calculate limit
		   if (t > (DU32)tmin)
		   {
			   LimitDS32 = dpmin + k / (t - tmin);
			   if (LimitDS32 > MAX_DS16) LimitDS32 = MAX_DS16;

			   a.Limit = (DS16)LimitDS32;
		   }
		   else
			   a.Limit = MAX_DS16;
	   }
#if (OPTION_CYLINDER_MONITORING == TRUE)
		// combustion chamber temperature
		else if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 3L)
		{
			DelayTime = MIX_MAX_DEVIATION_TIME;
			Deviation = abs(MIX.Setpoint_Cylinder_Temperatue - CYL.CylinderAverageTemp);
			a.Limit = (DS16)PARA[ParRefInd[MIX_CYL_MAX_DEVIATION__PARREFIND]].Value;
		}
#endif
		else
		{
			// no monitoring
			DelayTime = MIX_MAX_DEVIATION_TIME;
			Deviation = 0;
			a.Limit = MAX_DS16;
		}

		a.Exceeded = ( dp_int >= a.Limit );

		// used for trending
		if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 2L)
		{
			a.p_dev = dp_int;
			a.dp_lim = a.Limit;
		}
		else
		{
			a.p_dev = 0;
			a.dp_lim = MAX_DS16;

			// reset dp_int and t for p/T-Control
			dp_int = 0;
			t = 0L;
		}

		switch(a.State)
		{
			case COLD:
				STOP_Tripped[STOPCONDITION_30231] = FALSE;
				if ( a.Exceeded )
				{
					if (DelayTime == 0L) a.State = TRIP;
					else                 a.State = HOT;
				}
			break;

			case HOT:
				if (!a.Exceeded ) a.State = COLD;
				if ( a.StateTimer >= DelayTime) a.State = TRIP;
				break;

			case TRIP:
				STOP_Set(STOPCONDITION_30231);
				STOP_Tripped[STOPCONDITION_30231] = TRUE;
				if (!a.Exceeded) a.State = COLD;
				break;

			default:
				a.State = COLD;
				break;
		}
	}
}

// Check maximum position deviation

#ifdef a
#undef a
#endif
#define a MIX.PositionDeviation

static void MIX_PositionDeviation (void)
{

   // if state has changed, clear state timer and remember new state
   if (a.State != a.LastState )
   {
     a.StateTimer = 0;
     a.LastState  = a.State;
   }
   else // count the state timecounter by adding 100ms
   {
     if (a.StateTimer < (MAX_DU32-1000)) a.StateTimer += 100L;
   }

   a.Exceeded = ( ( (MIX.state[MixerInd1] == MIX_START_POSITION_REACHED)
		         || (MIX.state[MixerInd1] == MIX_IDLE_POSITION_REACHED)
		         || (MIX.state[MixerInd1] == MIX_MOVING_TO_PARALLEL_POS)
		         || (MIX.state[MixerInd1] == MIX_PARALLEL_POSITION_REACHED)
		         || (MIX.state[MixerInd1] == MIX_MOVING_TO_ISLAND_POS)
		         || (MIX.state[MixerInd1] == MIX_ISLAND_POSITION_REACHED)
		         || (MIX.state[MixerInd1] == MIX_UNDER_CTRL) )
	       	   && (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
		       && (AI_I_FUNCT[IOA_AI_I_MIX_GASMIXER_POS].Assigned == ASSIGNED)
		       && (abs(MIX.SetpointMixerPositionPercent[MixerInd1] - MIX.ActualPositionOfGasMixerPercent[MixerInd1]) > MIX_POSITION_DEVIATION_LIMIT) );

   switch(a.State)
   {
      case COLD:
      	STOP_Tripped[STOPCONDITION_70231] = FALSE;
      	if ( a.Exceeded ) a.State = HOT;
      	break;

      case HOT:
      	if (!a.Exceeded ) a.State = COLD;
      	if ( a.StateTimer >= MIX_POSITION_DEVIATION_DELAY) a.State = TRIP;
      	break;

      case TRIP:
      	STOP_Set(STOPCONDITION_70231);
      	STOP_Tripped[STOPCONDITION_70231] = TRUE;
      	if (!a.Exceeded) a.State = COLD;
      	break;

      default:
      	a.State = COLD;
      	break;
    }
}



// **********************************************************
// ****************  mixer control  *************************
// **********************************************************
// *                                                        *
// *    evaluate conditions for active mixer control        *
// **********************************************************
// *** function name:     Mixer_Control_Release()         ***
// *** protection struct: MIX.MixerControlRelease         ***
// *** input value:       MIX.LambdaVoltageFilteredValue  ***
// *** trip limit:        MIX_ACCEPTED_MAX_TO_START_CONTROL *
// *** recover limit:     none                            ***
// *** trip delay:        MIX_DELAY_FOR_STARTING:LAMBDA_CONTROL*
// *** recover delay:     none							  ***
// *** Cycle time:        1000ms                          ***
// *** Stop condition:    none             ***
// **********************************************************
// *** 29-jul-2014 mvo                                    ***
// **********************************************************


#ifdef a
#undef a
#endif
#define a MIX.MixerControlRelease

static void Mixer_Control_Release(void)
{

	DBOOL  overrun;  // marker to overrun state machine

   // if state has changed, clear state timer and remember new state
   if (a.State != a.LastState )
   {
     a.StateTimer = 0;
     a.LastState  = a.State;
   }
   else // count the state timecounter by adding 1000ms
   {
     if (a.StateTimer < (MAX_DU32-1000)) a.StateTimer += 1000L;
   }

   // no lambda control or no assignment of lambda voltage -> just give green light for control
   overrun = ( // no lambda control -> mixer control is o.k. -> green light
		   	   !(PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 1L)
				   ||
				// lambda voltage not assigned? -> mixer control is o.k. -> green light
				!(AI_I_FUNCT[LAMBDA_VOLTAGE].Assigned == ASSIGNED) );

   a.Limit = MIX_ACCEPTED_MAX_TO_START_CONTROL;

   a.Exceeded = (
		   (MIX.LambdaVoltageFilteredValue < a.Limit) // sensor value is within limits
		   &&
		   MIX.EvaluateConditionsToStartMixerControl  // evaluation demanded by MAIN
   );

   switch(a.State)
   {
      case COLD:
    	if (overrun)
    	{
    		a.State = HOT;
    	}
    	else if ( a.Exceeded )
    	{
    		a.State = HOT;
    	}
      	break;

      case HOT:
    	if (overrun)
    	{
    		a.State = TRIP;
    	}
    	else if (!a.Exceeded )
    	{
    		a.State = COLD;
    	}
    	else
    	{
    		// a.Exceeded && not overrun:
    		if ( a.StateTimer >= MIX_DELAY_FOR_STARTING_LANBDA_CONTROL)
    			a.State = TRIP;
    	}
      	break;

      case TRIP:
      	if ( (!a.Exceeded) && (!overrun) )
      		a.State = COLD;
      	break;

      default:
      	a.State = COLD;
      	break;
    }
}



// **********************************************************
// ****************  mixer control  *************************
// **********************************************************
// *                                                        *
// *    timeout supervision for active mixer control        *
// *********************************************************
// *** function name:     Mixer_Control_Release_Timeout() ***
// *** protection struct: MIX.MixerControlReleaseTimeout  ***
// *** input value:       MIX.MixerControlRelase.State    ***
// *** trip delay:        MIX_TIMEOUT_FOR_STARTING_LAMBDA_CONTROL*
// *** recover delay:     none							  ***
// *** Cycle time:        1000ms                          ***
// *** Stop condition:    50125R			              ***
// **********************************************************
// *** 29-jul-2014 mvo                                    ***
// **********************************************************
#ifdef a
#undef a
#endif
#define a MIX.MixerControlReleaseTimeout

static void Mixer_Control_Release_Timeout(void)
{

   // if state has changed, clear state timer and remember new state
   if (a.State != a.LastState )
   {
     a.StateTimer = 0;
     a.LastState  = a.State;
   }
   else // count the state timecounter by adding 1000ms
   {
     if (a.StateTimer < (MAX_DU32-1000)) a.StateTimer += 1000L;
   }

   a.Limit = MIX_ACCEPTED_MAX_TO_START_CONTROL;

   a.Exceeded = (
		   MIX.EvaluateConditionsToStartMixerControl // check conditions, command from MAIN
		   &&
		   ELM.ReleaseLoadForMixerControl			 // load is high enough to activate control
		   &&
		   (MIX.MixerControlRelease.State != TRIP)	 // but no release so far
   );

   switch(a.State)
   {
      case COLD:
    	if ( a.Exceeded )	a.State = HOT;
      	break;

      case HOT: 
    	if (!a.Exceeded )
    	{
    		a.State = COLD;
    	}
    	else
    	{
    		// a.Exceeded
    		if ( a.StateTimer >= MIX_TIMEOUT_FOR_STARTING_LAMBDA_CONTROL)
    			a.State = TRIP;
    	}
      	break;

      case TRIP:
    	  STOP_Set(STOPCONDITION_50125);
      	if (!a.Exceeded)
      		a.State = COLD;
      	break;

      default:
      	a.State = COLD;
      	break;
    }
}




// read NUMBER_OF_MIXER_SETPOINTS setpoint triples (P,p,T) into array
// to be called during initialization and after each change of these parameters
void Mix_Read_MixerSetpointsFromParameters(void)
{
	// p/T
	if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 2L)
	{
		// gas type A
		{
			// copy first curvepoint from the parameters into Setpoint array
			MIX.Setpoint_Mixer[0][0].Psum  = (DS32)PARA[ParRefInd[MIX_POWER1A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][0].p     = (DS16)PARA[ParRefInd[MIX_PRESS1A__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[0][0].theta = (DS16)PARA[ParRefInd[MIX_TEMP1A__PARREFIND]].Value;

			// copy second curvepoint from the parameters into Setpoint array
			MIX.Setpoint_Mixer[0][1].Psum  = (DS32)PARA[ParRefInd[MIX_POWER2A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][1].p     = (DS16)PARA[ParRefInd[MIX_PRESS2A__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[0][1].theta = (DS16)PARA[ParRefInd[MIX_TEMP2A__PARREFIND]].Value;

			// copy third curvepoint from the parameters into Setpoint array
			MIX.Setpoint_Mixer[0][2].Psum  = (DS32)PARA[ParRefInd[MIX_POWER3A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][2].p     = (DS16)PARA[ParRefInd[MIX_PRESS3A__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[0][2].theta = (DS16)PARA[ParRefInd[MIX_TEMP3A__PARREFIND]].Value;

			// copy fourth curvepoint from the parameters into Setpoint array
			MIX.Setpoint_Mixer[0][3].Psum  = (DS32)PARA[ParRefInd[MIX_POWER4A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][3].p     = (DS16)PARA[ParRefInd[MIX_PRESS4A__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[0][3].theta = (DS16)PARA[ParRefInd[MIX_TEMP4A__PARREFIND]].Value;

			// new points 5...8

			MIX.Setpoint_Mixer[0][4].Psum  = (DS32)PARA[ParRefInd[MIX_POWER5A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][4].p     = (DS16)PARA[ParRefInd[MIX_PRESS5A__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[0][4].theta = (DS16)PARA[ParRefInd[MIX_TEMP5A__PARREFIND]].Value;

			MIX.Setpoint_Mixer[0][5].Psum  = (DS32)PARA[ParRefInd[MIX_POWER6A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][5].p     = (DS16)PARA[ParRefInd[MIX_PRESS6A__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[0][5].theta = (DS16)PARA[ParRefInd[MIX_TEMP6A__PARREFIND]].Value;

			MIX.Setpoint_Mixer[0][6].Psum  = (DS32)PARA[ParRefInd[MIX_POWER7A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][6].p     = (DS16)PARA[ParRefInd[MIX_PRESS7A__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[0][6].theta = (DS16)PARA[ParRefInd[MIX_TEMP7A__PARREFIND]].Value;

			MIX.Setpoint_Mixer[0][7].Psum  = (DS32)PARA[ParRefInd[MIX_POWER8A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][7].p     = (DS16)PARA[ParRefInd[MIX_PRESS8A__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[0][7].theta = (DS16)PARA[ParRefInd[MIX_TEMP8A__PARREFIND]].Value;
		}
		// gas type B
		{
			// copy first curvepoint from the parameters into Setpoint array
			MIX.Setpoint_Mixer[1][0].Psum  = (DS32)PARA[ParRefInd[MIX_POWER1B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][0].p     = (DS16)PARA[ParRefInd[MIX_PRESS1B__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[1][0].theta = (DS16)PARA[ParRefInd[MIX_TEMP1B__PARREFIND]].Value;

			// copy second curvepoint from the parameters into Setpoint array
			MIX.Setpoint_Mixer[1][1].Psum  = (DS32)PARA[ParRefInd[MIX_POWER2B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][1].p     = (DS16)PARA[ParRefInd[MIX_PRESS2B__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[1][1].theta = (DS16)PARA[ParRefInd[MIX_TEMP2B__PARREFIND]].Value;

			// copy third curvepoint from the parameters into Setpoint array
			MIX.Setpoint_Mixer[1][2].Psum  = (DS32)PARA[ParRefInd[MIX_POWER3B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][2].p     = (DS16)PARA[ParRefInd[MIX_PRESS3B__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[1][2].theta = (DS16)PARA[ParRefInd[MIX_TEMP3B__PARREFIND]].Value;

			// copy fourth curvepoint from the parameters into Setpoint array
			MIX.Setpoint_Mixer[1][3].Psum  = (DS32)PARA[ParRefInd[MIX_POWER4B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][3].p     = (DS16)PARA[ParRefInd[MIX_PRESS4B__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[1][3].theta = (DS16)PARA[ParRefInd[MIX_TEMP4B__PARREFIND]].Value;

			// new points 5...8

			MIX.Setpoint_Mixer[1][4].Psum  = (DS32)PARA[ParRefInd[MIX_POWER5B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][4].p     = (DS16)PARA[ParRefInd[MIX_PRESS5B__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[1][4].theta = (DS16)PARA[ParRefInd[MIX_TEMP5B__PARREFIND]].Value;

			MIX.Setpoint_Mixer[1][5].Psum  = (DS32)PARA[ParRefInd[MIX_POWER6B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][5].p     = (DS16)PARA[ParRefInd[MIX_PRESS6B__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[1][5].theta = (DS16)PARA[ParRefInd[MIX_TEMP6B__PARREFIND]].Value;

			MIX.Setpoint_Mixer[1][6].Psum  = (DS32)PARA[ParRefInd[MIX_POWER7B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][6].p     = (DS16)PARA[ParRefInd[MIX_PRESS7B__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[1][6].theta = (DS16)PARA[ParRefInd[MIX_TEMP7B__PARREFIND]].Value;

			MIX.Setpoint_Mixer[1][7].Psum  = (DS32)PARA[ParRefInd[MIX_POWER8B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][7].p     = (DS16)PARA[ParRefInd[MIX_PRESS8B__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_VALUE;
			MIX.Setpoint_Mixer[1][7].theta = (DS16)PARA[ParRefInd[MIX_TEMP8B__PARREFIND]].Value;
		}
	}
	// combustion chamber temperature
	else if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 3L)
	{
		// gas type A
		{
			MIX.Setpoint_Mixer[0][0].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER1A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][0].p     = 0;
			MIX.Setpoint_Mixer[0][0].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP1A__PARREFIND]].Value;

			MIX.Setpoint_Mixer[0][1].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER2A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][1].p     = 0;
			MIX.Setpoint_Mixer[0][1].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP2A__PARREFIND]].Value;

			MIX.Setpoint_Mixer[0][2].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER3A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][2].p     = 0;
			MIX.Setpoint_Mixer[0][2].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP3A__PARREFIND]].Value;

			MIX.Setpoint_Mixer[0][3].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER4A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][3].p     = 0;
			MIX.Setpoint_Mixer[0][3].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP4A__PARREFIND]].Value;

			MIX.Setpoint_Mixer[0][4].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER5A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][4].p     = 0;
			MIX.Setpoint_Mixer[0][4].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP5A__PARREFIND]].Value;

			MIX.Setpoint_Mixer[0][5].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER6A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][5].p     = 0;
			MIX.Setpoint_Mixer[0][5].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP6A__PARREFIND]].Value;

			MIX.Setpoint_Mixer[0][6].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER7A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][6].p     = 0;
			MIX.Setpoint_Mixer[0][6].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP7A__PARREFIND]].Value;

			MIX.Setpoint_Mixer[0][7].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER8A__PARREFIND]].Value;
			MIX.Setpoint_Mixer[0][7].p     = 0;
			MIX.Setpoint_Mixer[0][7].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP8A__PARREFIND]].Value;
		}
		
		{
			MIX.Setpoint_Mixer[1][0].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER1B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][0].p     = 0;
			MIX.Setpoint_Mixer[1][0].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP1B__PARREFIND]].Value;

			MIX.Setpoint_Mixer[1][1].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER2B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][1].p     = 0;
			MIX.Setpoint_Mixer[1][1].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP2B__PARREFIND]].Value;

			MIX.Setpoint_Mixer[1][2].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER3B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][2].p     = 0;
			MIX.Setpoint_Mixer[1][2].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP3B__PARREFIND]].Value;

			MIX.Setpoint_Mixer[1][3].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER4B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][3].p     = 0;
			MIX.Setpoint_Mixer[1][3].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP4B__PARREFIND]].Value;

			MIX.Setpoint_Mixer[1][4].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER5B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][4].p     = 0;
			MIX.Setpoint_Mixer[1][4].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP5B__PARREFIND]].Value;

			MIX.Setpoint_Mixer[1][5].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER6B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][5].p     = 0;
			MIX.Setpoint_Mixer[1][5].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP6B__PARREFIND]].Value;

			MIX.Setpoint_Mixer[1][6].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER7B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][6].p     = 0;
			MIX.Setpoint_Mixer[1][6].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP7B__PARREFIND]].Value;

			MIX.Setpoint_Mixer[1][7].Psum  = (DS32)PARA[ParRefInd[MIX_CYL_POWER8B__PARREFIND]].Value;
			MIX.Setpoint_Mixer[1][7].p     = 0;
			MIX.Setpoint_Mixer[1][7].theta = (DS16)PARA[ParRefInd[MIX_CYL_TEMP8B__PARREFIND]].Value;
		}
	}
}

// sort setpoints within the array by Power
void Mix_Sort_MixerSetpoints(void)
{
	DU8 gas;
	DBOOL Sorted = FALSE;
	int j;
	struct t_MIX_Setpoint_Mixer c;

	for (gas=0; gas<=1; gas++)
	do
	{
		Sorted = TRUE;
	    for (j = 0; j < NUMBER_OF_MIXER_SETPOINTS - 1; j++)
	    {
	    	if (MIX.Setpoint_Mixer[gas][j].Psum > MIX.Setpoint_Mixer[gas][j+1].Psum)
	    	{
	    	    // swap point j and j+1
	    	    c.Psum  = MIX.Setpoint_Mixer[gas][j].Psum;
	    	    c.p     = MIX.Setpoint_Mixer[gas][j].p;
	    	    c.theta = MIX.Setpoint_Mixer[gas][j].theta;

	    	    MIX.Setpoint_Mixer[gas][j].Psum    = MIX.Setpoint_Mixer[gas][j+1].Psum;
	    	    MIX.Setpoint_Mixer[gas][j].p       = MIX.Setpoint_Mixer[gas][j+1].p;
	    	    MIX.Setpoint_Mixer[gas][j].theta   = MIX.Setpoint_Mixer[gas][j+1].theta;

	    	    MIX.Setpoint_Mixer[gas][j+1].Psum  = c.Psum;
	    	    MIX.Setpoint_Mixer[gas][j+1].p     = c.p;
	    	    MIX.Setpoint_Mixer[gas][j+1].theta = c.theta;

	    	    // remember that the set was not sorted, yet
	    	    Sorted = FALSE;
	    	}
	    }
	}
	while (!Sorted);

}

// find function value y at position x (point x/y) on a line defined by 2 points (x1/y1) and (x2/y2)
DS32 Interpolate (DS32 x, 
                  DS32 x1,
                  DS32 x2,
                  DS32 y1,
                  DS32 y2)
{
	if ((x2-x1) != 0) // avoid division by zero
	    return (y1 + ( (y2- y1) *(x - x1) / (x2 - x1) ) );
	else 
	    return (y1+(y2 -y1)/2);

}



// calculate gas mixture enrichment on a cold engine
static DS16 Mix_Calculate_Enrichment(void)
{
	DS16  InputValue; // selected temperature measurement value
	DS16  DeltaT = 0; // how many degrees below normal engine temperature?
	DS16  ReturnValue;
	DBOOL Available = FALSE;
	
	// use Parameter MIX_COLD_START_ENRICHMENT__PARREFIND
	// and MIX_REFERENCE_TEMPERATURE_ENGINE_ENTRANCE to calculate
	// how much the gas mixture must be enriched to run stable.
	// input value: selected by Parameter 22113
	// 0 = engine inlet, 1 = engine exit

	switch ((DU8)PARA[ParRefInd[MIX_COLD_STRT_ENRI_MEAS_POINT__PARREFIND]].Value)
	{
		//selection of input value and MSR point depending on parameter setting
		case 0:
			InputValue = ENG.T201_Filtered;					// [0.01°C]
			if (ENG.CoolingWaterEngineEntrance_Available)
				Available = TRUE;
			break;

		case 1:
			InputValue = ENG.CoolingWaterEngineExit.Value * 10; // CoolingWaterEngineExit [0.1°C] -> InputValue [0.01°C]
			if (ENG.CoolingWaterEngineExit_Available)
				Available = TRUE;
			break;

			// case 2 oil temperature?

		default:
			// Available = FALSE;
			break;
	}
	
	if (Available)//(AI_R_U_FUNCT[MSR_point_RU_index].Assigned == ASSIGNED)
	{
		// temperature is assigned
	    if (InputValue /10 < (DS16)MIX_REFERENCE_TEMPERATURE_ENGINE_ENTRANCE)
	    {
		    // engine is cold, calculate DeltaT [0,1K]
	        DeltaT = (DS16)MIX_REFERENCE_TEMPERATURE_ENGINE_ENTRANCE - (InputValue /10);
	    }
	    else
	        DeltaT = 0; // normal temperature reached or exceeded
	    
	    // limit DeltaT, enrichment only down to 0°C
	    if (DeltaT > (DS16)MIX_REFERENCE_TEMPERATURE_ENGINE_ENTRANCE)
	        DeltaT = (DS16)MIX_REFERENCE_TEMPERATURE_ENGINE_ENTRANCE;
	    
	    // return enrichment in [0,01%] calc. by DeltaT [0,1K] and ENRICHMENT_RATE [0,1%/K]
	    //ReturnValue = (DeltaT * (DS16)MIX_ENRICHMENT_RATE_DUE_TO_TEMPERATURE);
	    // return enrichment in [0,01%] calc. by DeltaT [0,1K] and ENRICHMENT_RATE [0,01%/K]
	    ReturnValue = ((DS32)DeltaT * PARA[ParRefInd[MIX_COLD_START_ENRICHMENT__PARREFIND]].Value / 10);

	}
	else
	{
		// engine entrance temperature is not assigned
		ReturnValue = 0;
	}
	return (ReturnValue);
}


// calculate setpoint for mixer position
// depending on the active gas type GAS.GasTypeBActive
// (Note: Calculation is the same for both mixers and independant of the engine side.)
DS16 Mix_Calculate_Setpoint_For_Mixer_Position(DU16 Index_A, DU16 Index_B)
{

	DS16	SetpointMixerPos;
	DS16	AdditionalEnrichment = 0;  //[0.01%]

 	// Calculate enrichment in 0.01% (DS16) due to low engine temperature
	MIX.TemperatureOffset = Mix_Calculate_Enrichment();

	// Calculate additional enrichment in case of TecJet
	// - for cold engine
	// - for first minutes of running
	if (MIX_OPTION_TECJET)
	{
		//MIX.AdditionalTemperatureOffsetTecJet = (DS32)MIX.TemperatureOffset * MIX_TECJET_ADDITIONAL_COLD_START_ENRICHMENT_FACTOR / 100L;

		if (  (ENG.EngRunningCounter > 0L) && (ENG.EngRunningCounter <= MIX_TEC_ADDITIONAL_START_ENRICHMENT_TIME)  )
		{
			// enrich mixture for the first running minutes
			AdditionalEnrichment =
					(MIX_TEC_ADDITIONAL_START_ENRICHMENT_TIME - ENG.EngRunningCounter)
					/ (MIX_TEC_ADDITIONAL_START_ENRICHMENT_TIME/MIX_TEC_ADDITIONAL_START_ENRICHMENT_FRRACTION);
			if (AdditionalEnrichment < 0) AdditionalEnrichment = 0;
			MIX.AdditionalEnrichmentTecJet = AdditionalEnrichment;
		}
		else
		{
			MIX.AdditionalEnrichmentTecJet = 0;
		}
	}
	else
	{
		//MIX.AdditionalTemperatureOffsetTecJet = 0;
		MIX.AdditionalEnrichmentTecJet = 0;
	}

	if (!GAS.GasTypeBActive) // gas A
		SetpointMixerPos = 	(MIX.MixerFullRange_Out
			* ((DS32)PARA[ParRefInd[Index_A]].Value + (DS32)PARA[ParRefInd[Index_A]].Value
					* (MIX.TemperatureOffset
							//+ MIX.AdditionalTemperatureOffsetTecJet
							+ MIX.AdditionalEnrichmentTecJet) / 10000L
			+ CH4.MixerOffset
			/*+ O2.MixerOffset*/) / 1000L); // to_do_zzh for O2 offset
	else // gas B
		SetpointMixerPos = 	(MIX.MixerFullRange_Out
			* ((DS32)PARA[ParRefInd[Index_B]].Value + (DS32)PARA[ParRefInd[Index_B]].Value
					* (MIX.TemperatureOffset
							//+ MIX.AdditionalTemperatureOffsetTecJet
							+ MIX.AdditionalEnrichmentTecJet) / 10000L
			/*+ CH4.MixerOffset*/) / 1000L);

	//limit SetpointMixerPosition to max number of steps
	if (SetpointMixerPos > (DS16)MIX.MixerFullRange_Out)
		SetpointMixerPos = (DS16)MIX.MixerFullRange_Out;

	return (SetpointMixerPos);
} // end: Mix_Calculate_Setpoint_For_Mixer_Position


// calculate ramp position of mixer, rmiMIXRAMP
// if the mixer moves to parallel position, this shall be done via ramp
// therefore this function changes the SetpointMixerPosition only by a difference of one step
// to the actual position, depending on the difference between target and actual position +/-
DS16 Mix_Calculate_Ramp_Position(DS16 TargetPosition, DS16 ActualPosition)
{
 DS16	SetpointMixerPosi;
 DS16    delta = TargetPosition - ActualPosition;
 
    if (delta > MIX_NUMBER_OF_STEPS_IN_RAMP) 
        delta = MIX_NUMBER_OF_STEPS_IN_RAMP;
    else if (-delta > MIX_NUMBER_OF_STEPS_IN_RAMP)
        delta = -MIX_NUMBER_OF_STEPS_IN_RAMP;
    
	if (delta != 0)
	    SetpointMixerPosi = ActualPosition + delta;
	else											// target == actual
	    SetpointMixerPosi = ActualPosition;
	// endif: target > actual
					
	// limit SetpointMixerPosi to max number of steps
	if (SetpointMixerPosi > (DS16)MIX.MixerFullRange_Out)
		SetpointMixerPosi = (DS16)MIX.MixerFullRange_Out;
		
    return (SetpointMixerPosi);
} // end: Mix_Calculate_Setpoint_For_Mixer_Position


static DS32 MIX_RunningTimeRamp (DS32 Actual, DS16 Target, DS32 Range)
{
	DS16 step;

	if (Actual != (DS32)Target)
	{
		// step each 100ms
		step = (Range * 100 + (DS32)PARA[ParRefInd[MIX_RUNNING_TIME_0_TO_100_PERCENT__PARREFIND]].Value / 2) / (DS32)PARA[ParRefInd[MIX_RUNNING_TIME_0_TO_100_PERCENT__PARREFIND]].Value;

		if (abs(Actual - (DS32)Target) < step)
		{
			return ((DS32)Target);
		}
		else if (Actual > (DS32)Target)
		{
			return (Actual - step);
		}
		else
		{
			return (Actual + step);
		}
	}
	else return (Actual);
}

/**
 * 
 * Transit function which is called at any state change
 *  
 * @author  JAH/AES
 * @date    4-nov-2006
 */

static void Transit( const STATE2 newState, DU8 mixer )
{
	if ( newState != myState[mixer] )
	{
		if ( myState[mixer] != 0 )
		{
			myState[mixer](SIG_EXIT, mixer);
		}
		
		myState[mixer] = newState;

	    if ( myState[mixer] != 0 )
	    {
	    	 myState[mixer](SIG_ENTRY, mixer);
	    }
	    
		myStateCnt[mixer] = 0;
	}
}

/* modes
 * MIX_BLOCK,          // mixer frozen       
   MIX_MOVE_LEAN,      // move to limit stop lean
   MIX_MOVE_FAT,       // move to maximum fat position 100%
   MIX_CTRL,           // active control of mixture
   MIX_TEST_DEMANDED   // test mode demanded
   */

/* states
 * MIX_BOOT,                    // during bootup
   MIX_SYSTEM_OFF,              // System off
   MIX_SEARCHING_LEAN,
   MIX_LIMIT_LEAN_REACHED,
   MIX_MOVING_FAT,
   MIX_POS_FAT_REACHED,
   MIX_MOVING_TO_START_POS,
   MIX_START_POSITION_REACHED,
   MIX_UNDER_CTRL,
   MIX_UNDER_TEST
   */

/**
 * In this state the mixture control is shut off
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_SystemOff(const DU8 sig, DU8 mixer)
{
	switch(sig)
	{
		case SIG_ENTRY:
            MIX.state[mixer]                          = MIX_SYSTEM_OFF;
            
            MIX.StepperMotor[mixer].forceLean = FALSE;

            if (MIX_OPTION_TECJET)
            {
            	MIX.SetpointMixerPosition[mixer] = 0;
            }
            else if (AI_I_FUNCT[IOA_AI_I_MIX_GASMIXER_POS].Assigned == NOT_ASSIGNED)
            {
            	MIX.SetpointMixerPosition[mixer] = MIX.ActualPositionOfGasMixer[mixer];
            }

		break;

		case SIG_EXIT:
		break;

		default:

			// if mode == MIX_BLOCK stay here
			if (MIX.mode[mixer] == MIX_BLOCK)
			{
			  //Transit(MIX_Control);// only for testing gas mixer!!! - to be removed !!!
				break;
			}

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
		if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_SearchLimitLean, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveRich, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIdle, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveParallel, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIsland, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX_OPTION_TECJET))
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_SearchLimitLean, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
            
		break;
	}

}

/**
 * In this state the gas mixer moves to its lean position.
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_SearchLimitLean(const DU8 sig, DU8 mixer)
{
	DU16 FuncInd;
	DU16 StopCondInd;
	DU32 TimeoutDelay;

	if (mixer == MixerInd1)
	{
		FuncInd = MIXER_LIMIT_LEAN;
		StopCondInd = STOPCONDITION_70221;
	}
	else//MixerInd2
	{
		FuncInd = MIXER_B_LIMIT_LEAN;
		StopCondInd = STOPCONDITION_70706;
	}

	switch(sig)
	{
		case SIG_ENTRY:
	        MIX.state[mixer]                          = MIX_SEARCHING_LEAN;
	        
	        // activate movement to limit stop lean on the IOM
	        MIX.StepperMotor[mixer].forceLean = TRUE;
	        
            MIX.TargetMixerPosition[mixer] = 0;

		break;

		case SIG_EXIT:
		break;

		default:

			if (MIX.mode[mixer] == MIX_BLOCK)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_SystemOff, mixer);
				break;
			}

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2) // Se podría eleminar,ya que se cuenta con un solo mixer
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

        	TimeoutDelay = MIX_TIMEOUT_MOVING_TO_LEAN_POS;

            if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
            {
            	TimeoutDelay = MAX_DU32; // No Timeout

				// increase/decrease setpoint by ramp (running time)
            	MIX.SetpointMixerPosition[mixer] = MIX_RunningTimeRamp((DS32)MIX.SetpointMixerPosition[mixer], MIX.TargetMixerPosition[mixer], MIX.MixerFullRange_Out);

            	if (MIX.SetpointMixerPosition[mixer] == MIX.TargetMixerPosition[mixer])
				{
					Transit(MIX_PositionLeanReached, mixer);
					break;
				}
            }
            else
            {
				// all movement ends as soon as the limit stop has been reached
				if ( (!MIX.DI_LimitStopLean[mixer])
					&& (DI_FUNCT[FuncInd].Assigned == ASSIGNED) )
				{
					Transit(MIX_PositionLeanReached, mixer);
					break;
				}
			}

            // !!! state timeout with stop condition here
            // lean position not reached within...
            if (myStateCnt[mixer] >= TimeoutDelay)
            {
            	STOP_Set( StopCondInd );
            }

			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			  // stay here
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveRich, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			  // stay here, we are already calibrating
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX_OPTION_TECJET) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}            
		break;
	}
}



/**
 * In this state the gas mixer has reached to limit stop lean.
 * It stays there until otherwise demanded.
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_PositionLeanReached(const DU8 sig, DU8 mixer)
{

	DU16 StopCondInd;

	if (mixer == MixerInd1)
	{
		StopCondInd = STOPCONDITION_70224;
	}
	else//MixerInd2
	{
		StopCondInd = STOPCONDITION_70715;
	}

	switch(sig)
	{
		case SIG_ENTRY:
	        MIX.state[mixer]                          = MIX_LIMIT_LEAN_REACHED;
	        
	        MIX.StepperMotor[mixer].forceLean = FALSE;

	        if (!(MIX_OPTION_TECJET))
	        {
		        // compare step counter
	        
		        if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == NOT_ASSIGNED)
		        // create stop condition if loss of too many steps
		        if ( (abs(MIX.ActualPositionOfGasMixer[mixer]) > MIX_MAX_LOSS_OF_STEPS) && !MixFirstCalibration[mixer] )
		        	STOP_Set(StopCondInd);
	        }

	        // reset step counter on IOM
	        MIX.StepperMotor[mixer].reset = TRUE;

		break;

		case SIG_EXIT:
		    MixFirstCalibration[mixer] = FALSE;
		break;

		default:

			if (MIX.mode[mixer] == MIX_BLOCK)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_SystemOff, mixer);
				break;
			}

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

			if (STOP_is_Set(StopCondInd))
			{
				// steps lost
				Transit(MIX_SystemOff, mixer);
				break;
			}
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			  // stay here
			  // we are already there
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveRich, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIdle, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveParallel, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIsland, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX_OPTION_TECJET) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			  // Calibrate means: continue moving to 100% position
			  // 
			  Transit(MIX_MoveRich, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
			
            
		break;
	}
}




/**
 * In this state the mixer moves to the stored 100% position (which means maximum position Rich). 
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_MoveRich(const DU8 sig, DU8 mixer)
{
	DU16 StopCondInd;
	DU32 TimeoutDelay;

	if (mixer == MixerInd1)
	{
		StopCondInd = STOPCONDITION_70236;
	}
	else//MixerInd2
	{
		StopCondInd = STOPCONDITION_70713;
	}

	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                          = MIX_MOVING_RICH;

			MIX.StepperMotor[mixer].forceLean = FALSE;

			MIX.TargetMixerPosition[mixer]          = MIX.MixerFullRange_Out;
			
		break;

		case SIG_EXIT:
		break;

		default:
			
            if (MIX.mode[mixer] == MIX_BLOCK)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_SystemOff, mixer);
				break;
			}

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

        	TimeoutDelay = MIX_TIMEOUT_MOVING_TO_RICH_POS;

            if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
            {
            	TimeoutDelay = MAX_DU32; // No Timeout

				// increase/decrease setpoint by ramp (running time)
            	MIX.SetpointMixerPosition[mixer] = MIX_RunningTimeRamp((DS32)MIX.SetpointMixerPosition[mixer], MIX.TargetMixerPosition[mixer], MIX.MixerFullRange_Out);

            	if (MIX.SetpointMixerPosition[mixer] == MIX.TargetMixerPosition[mixer])
				{
					Transit(MIX_PositionRichReached, mixer);
					break;
				}
            }
            else
            {
				MIX.SetpointMixerPosition[mixer] = MIX.TargetMixerPosition[mixer];
            	if ( ( (MIX.DI_LimitStopRich) && (DI_FUNCT[MIXER_LIMIT_RICH].Assigned == ASSIGNED) )
					|| (MIX.ActualPositionOfGasMixer[mixer] >= (DS16)PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value) ) // !!! check position
				{
					// position 100% reached?
					// then continue with positionRichReached
					Transit(MIX_PositionRichReached, mixer);
					break;
				}
			}
            // for IET max position used (replace #define by parameter !!!)

            // !!! state timeout with stop condition
            // rich position not reached within...
            if (myStateCnt[mixer] >= TimeoutDelay)
            {
            	STOP_Set( StopCondInd );
            }
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_SearchLimitLean, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
			  // we are already here
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX_OPTION_TECJET) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			  // Calibrate means: continue moving to 100% position
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}

            		
							
		break;
	}
}


/**
 * In this state the mixer has reached its 100% position (which means maximum position rich). 
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_PositionRichReached(const DU8 sig, DU8 mixer)
{
	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                           = MIX_POS_RICH_REACHED;
		break;

		case SIG_EXIT:
		break;

		default:

            if (MIX.mode[mixer] == MIX_BLOCK)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_SystemOff, mixer);
				break;
			}

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_SearchLimitLean, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
			  // we are already here
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIdle, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveParallel, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIsland, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CALIBRATE)
			{
			  // Seen from the 100% position calibrate means: move to start position
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
				
		break;
	}
}


/**
 * In this state the mixer moves to its stored start position.  
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_MoveStart(const DU8 sig, DU8 mixer)
{
	DU16 StopCondInd;
	DU32 TimeoutDelay;

	if (mixer == MixerInd1)
	{
		StopCondInd = STOPCONDITION_70232;
	}
	else//MixerInd2
	{
		StopCondInd = STOPCONDITION_70709;
	}

	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                           = MIX_MOVING_TO_START_POS;
			
			MIX.StepperMotor[mixer].forceLean = FALSE;
			// calculate setpoint for mixer position
			MIX.TargetMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position		// rmiMIXRAMP
			    						(MIX_START_POS_A__PARREFIND, MIX_START_POS_B__PARREFIND);
		break;

		case SIG_EXIT:
		break;

		default:

            if (MIX.mode[mixer] == MIX_BLOCK)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_SystemOff, mixer);
				break;
			}

			// go to "start position reached" at any position
            if (MAIN.Simulation)
            {
            	Transit(MIX_StartPositionReached, mixer);
            	break;
            }

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

        	TimeoutDelay = MIX_TIMEOUT_MOVING_TO_START_POS;

            if (MIX_OPTION_TECJET)
            {
            	Transit(MIX_StartPositionReached, mixer);
            	break;
            }
            else if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
            {
            	TimeoutDelay = MAX_DU32; // No Timeout

				// increase/decrease setpoint by ramp (running time)
            	MIX.SetpointMixerPosition[mixer] = MIX_RunningTimeRamp((DS32)MIX.SetpointMixerPosition[mixer], MIX.TargetMixerPosition[mixer], MIX.MixerFullRange_Out);

            	if (MIX.SetpointMixerPosition[mixer] == MIX.TargetMixerPosition[mixer])
				{
					Transit(MIX_StartPositionReached, mixer);
					break;
				}
            }
            else
            {
            	MIX.SetpointMixerPosition[mixer] = MIX.TargetMixerPosition[mixer];

            	// start position reached?
				// !!!
				if (MIX.ActualPositionOfGasMixer[mixer] == MIX.SetpointMixerPosition[mixer])
				{
					Transit(MIX_StartPositionReached, mixer);
					break;
				}
            }
            
            // timeout
            // start position not reached within...
            if (myStateCnt[mixer] >= TimeoutDelay)
            {
            	STOP_Set( StopCondInd );
            }
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			    break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
            	// State change if MAIN_CONTROL has changed mode
			    Transit(MIX_MoveRich, mixer);
			    break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			    // We are already moving there
			    break;
			}
            
            if (MIX.mode[mixer] == MIX_CALIBRATE)
			{
			  // Calibration means: continue moving to start position
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // First we have to move into start position, so stay here
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
				
		break;
	}
}





/**
 * In this state the mixer has reached the start position.
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_StartPositionReached(const DU8 sig, DU8 mixer)
{
	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                    = MIX_START_POSITION_REACHED;
			MIX.CalibrationDone[mixer]          = TRUE;
			// calculate setpoint for mixer position
			TimerCalculateNewMixerPosition[mixer] = 0L;
			MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position		// rmiMIXRAMP
										(MIX_START_POS_A__PARREFIND, MIX_START_POS_B__PARREFIND);
		break;

		case SIG_EXIT:
		break;

		default:

            if (MIX.mode[mixer] == MIX_BLOCK)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_SystemOff, mixer);
				break;
			}

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control - stay here
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  // stay here...
			  break;
			}

            if (TimerCalculateNewMixerPosition[mixer] >= MIX_TIMER_CALCULATE_NEW_SETPOINT_POS)
            {
				// calculate setpoint for mixer position
				TimerCalculateNewMixerPosition[mixer] = 0L;
				MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
											(MIX_START_POS_A__PARREFIND, MIX_START_POS_B__PARREFIND);
            }
            else
            	TimerCalculateNewMixerPosition[mixer] += 100L;

            // new calibration required
            if (!MIX.CalibrationDone[mixer])
            {
			    Transit(MIX_SearchLimitLean, mixer);
			    break;
			}
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			    break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
            	// State change if MAIN_CONTROL has changed mode
			    Transit(MIX_MoveRich, mixer);
			    break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			    // We have already come there.
			    break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIdle, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveParallel, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIsland, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CALIBRATE)
			{
			  // Calibration means: Stay in start position.
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_Control, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
				
		break;
	}
}




/**
 * In this state the mixer moves to its stored idle position.  
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_MoveIdle(const DU8 sig, DU8 mixer)
{
	DU16 StopCondInd;
	DU32 TimeoutDelay;

    if (mixer == MixerInd1)
    {
    	StopCondInd = STOPCONDITION_70233;
    }
    else//MixerInd2
    {
    	StopCondInd = STOPCONDITION_70710;
    }

	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                           = MIX_MOVING_TO_IDLE_POS;
 			// calculate setpoint for mixer position
			MIX.TargetMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
										(MIX_IDLE_POS_A__PARREFIND, MIX_IDLE_POS_B__PARREFIND);
		break;

		case SIG_EXIT:
		break;

		default:

            if (MIX.mode[mixer] == MIX_BLOCK)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_SystemOff, mixer);
				break;
			}

			// go to "idle position reached" at any position
            if (MAIN.Simulation)
            {
            	Transit(MIX_IdlePositionReached, mixer);
            	break;
            }

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

			TimeoutDelay = MIX_TIMEOUT_MOVING_TO_IDLE_POS;

            if (MIX_OPTION_TECJET)
            {
             	//if (MIX.SetpointMixerPosition[mixer] == MIX.TargetMixerPosition[mixer])
				if (myStateCnt[mixer] >= MIX.TecJetRampTimeStartToIdle)
				{
					Transit(MIX_IdlePositionReached, mixer);
					break;
				}
				//else (MIX.TecJetRampTimeStartToIdle - myStateCnt) cannot be zero
            	// run ramp
            	MIX.SetpointMixerPosition[mixer] =
            		(DS32)MIX.SetpointMixerPosition[mixer]
						+ ((DS32)100L*((DS32)MIX.TargetMixerPosition[mixer] - (DS32)MIX.SetpointMixerPosition[mixer])
								/((DS32)MIX.TecJetRampTimeStartToIdle - (DS32)myStateCnt[mixer]));
            }
            else if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
            {
            	TimeoutDelay = MAX_DU32; // No Timeout

				// increase/decrease setpoint by ramp (running time)
            	MIX.SetpointMixerPosition[mixer] = MIX_RunningTimeRamp((DS32)MIX.SetpointMixerPosition[mixer], MIX.TargetMixerPosition[mixer], MIX.MixerFullRange_Out);

            	if (MIX.SetpointMixerPosition[mixer] == MIX.TargetMixerPosition[mixer])
				{
					Transit(MIX_IdlePositionReached, mixer);
					break;
				}
            }
            else
            {
            	MIX.SetpointMixerPosition[mixer] = MIX.TargetMixerPosition[mixer];

            	// idle position reached?
				// !!!
				if (MIX.ActualPositionOfGasMixer[mixer] == MIX.SetpointMixerPosition[mixer])
				{
					Transit(MIX_IdlePositionReached, mixer);
					break;
				}
            }
            
            // timeout
            // idle position not reached within...
            if (myStateCnt[mixer] >= TimeoutDelay)
            {
            	STOP_Set( StopCondInd );
            }
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			    break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
            	// State change if MAIN_CONTROL has changed mode
			    Transit(MIX_MoveRich, mixer);
			    break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
            	// State change if MAIN_CONTROL has changed mode
			    Transit(MIX_MoveStart, mixer);
			    break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{
			    // We are already moving there
			    break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveParallel, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIsland, mixer);
			  break;
			}
            
            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
				
		break;
	}
}





/**
 * In this state the mixer has reached the idle position.
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_IdlePositionReached(const DU8 sig, DU8 mixer)
{
	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                           = MIX_IDLE_POSITION_REACHED;
			// calculate setpoint for mixer position
			TimerCalculateNewMixerPosition[mixer] = 0L;
			MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
										(MIX_IDLE_POS_A__PARREFIND, MIX_IDLE_POS_B__PARREFIND);
		break;

		case SIG_EXIT:
		break;

		default:

            if (MIX.mode[mixer] == MIX_BLOCK)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_SystemOff, mixer);
			  break;
			}

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

            if (TimerCalculateNewMixerPosition[mixer] >= MIX_TIMER_CALCULATE_NEW_SETPOINT_POS)
            {
				// calculate setpoint for mixer position
				TimerCalculateNewMixerPosition[mixer] = 0L;
				MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
											(MIX_IDLE_POS_A__PARREFIND, MIX_IDLE_POS_B__PARREFIND);
            }
            else
            	TimerCalculateNewMixerPosition[mixer] += 100L;
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_SearchLimitLean, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
              // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveRich, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{

			  // We have already come there.
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveParallel, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIsland, mixer);
			  break;
			}
            
            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_Control, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
				
		break;
	}
}



/**
 * In this state the mixer moves to its stored parallel position.  
 *   
 * @author  MVO
 * @date    2008-06-03
 * 
 * 
 */
static void MIX_MoveParallel(const DU8 sig, DU8 mixer)
{
	DU16 StopCondInd;
	DU32 TimeoutDelay;

    if (mixer == MixerInd1)
    {
    	StopCondInd = STOPCONDITION_70234;
    }
    else//MixerInd2
    {
    	StopCondInd = STOPCONDITION_70711;
    }

	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                           = MIX_MOVING_TO_PARALLEL_POS;
 			// calculate target for mixer position (not the setpoint), 
 			// because we want to reach the target slowly via ramp
			MIX.TargetMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
									  (MIX_PARALLEL_POS_A__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
            
		break;

		case SIG_EXIT:
		break;

		default:

            if (MIX.mode[mixer] == MIX_BLOCK)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_SystemOff, mixer);
				break;
			}

			// go to "parallel position reached" at any position
            if (MAIN.Simulation)
            {
            	Transit(MIX_ParallelPositionReached, mixer);
            	break;
            }

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

        	TimeoutDelay = MIX_TIMEOUT_MOVING_TO_PARALLEL_POS;

            if (MIX_OPTION_TECJET)
            {
				Transit(MIX_ParallelPositionReached, mixer);
				break;
            }
            else if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
            {
            	TimeoutDelay = MAX_DU32; // No Timeout

				// increase/decrease setpoint by ramp every 2nd loop
            	if (myStateCnt[mixer] % 200 == 0L)
            	{
            		MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Ramp_Position(MIX.TargetMixerPosition[mixer], MIX.SetpointMixerPosition[mixer]);
            	}

            	// setpoint has reached target
            	if (MIX.SetpointMixerPosition[mixer] == MIX.TargetMixerPosition[mixer])
				{
					Transit(MIX_ParallelPositionReached, mixer);
					break;
				}
            }
            else
            {
				// parallel position reached?
				if (MIX.ActualPositionOfGasMixer[mixer] == MIX.TargetMixerPosition[mixer])
				{
					Transit(MIX_ParallelPositionReached, mixer);
					break;
				}
            }

            // calculate a new setpoint to reach the target via a ramp
			if (MIX.ActualPositionOfGasMixer[mixer] == MIX.SetpointMixerPosition[mixer]) // position reached
				MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Ramp_Position				// rmiMIXRAMP
											(MIX.TargetMixerPosition[mixer], MIX.ActualPositionOfGasMixer[mixer]);
            
            // timeout
            // parallel position not reached within...
            if (myStateCnt[mixer] >= TimeoutDelay)
            {
            	STOP_Set( StopCondInd );
            }
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			    break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
            	// State change if MAIN_CONTROL has changed mode
			    Transit(MIX_MoveRich, mixer);
			    break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
            	// State change if MAIN_CONTROL has changed mode
			    Transit(MIX_MoveStart, mixer);
			    break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIdle, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			    // We are already moving there
			    break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIsland, mixer);
			  break;
			}
            
            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_Control, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
				
		break;
	}
}





/**
 * In this state the mixer has reached the parallel position.
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_ParallelPositionReached(const DU8 sig, DU8 mixer)
{
	DS16	LowerValue, HigherValue; // auxiliary variables for interpolation of mixer position
									 // in case of TecJet
	DS16	PowerSetpointRelative;	 // [0.01% of nominal power]

	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                           = MIX_PARALLEL_POSITION_REACHED;
 			// calculate setpoint for mixer position
			TimerCalculateNewMixerPosition[mixer] = 0L;
			MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
										(MIX_PARALLEL_POS_A__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
		break;

		case SIG_EXIT:
		break;

		default:

            if (MIX.mode[mixer] == MIX_BLOCK)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_SystemOff, mixer);
			  break;
			}

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

            if (TimerCalculateNewMixerPosition[mixer] >= MIX_TIMER_CALCULATE_NEW_SETPOINT_POS)
            {
	 			// calculate setpoint for mixer position
				TimerCalculateNewMixerPosition[mixer] = 0L;
				if (MIX_OPTION_TECJET)
				{
					PowerSetpointRelative = (TUR.Reg.PowerSetPoint * 10L / (DS32)(PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value/1000L));

					// interpolate flow setpoints (=mixer positions) depending on power
					// @ 0%: MIX_PARALLEL_POS_A__PARREFIND
					// @ 10%: TEC_FLOW_SETP_AT_10PERCENT_LOAD__PARREFIND
					// @ 20%: TEC_FLOW_SETP_AT_20PERCENT_LOAD__PARREFIND
					// @ 30%: TEC_FLOW_SETP_AT_30PERCENT_LOAD__PARREFIND
					// @ 40%: TEC_FLOW_SETP_AT_40PERCENT_LOAD__PARREFIND

					if (PowerSetpointRelative < 1000L) // power setpoint below 10%
					{
						// interpolation between 0% and 10%
						LowerValue = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
								(MIX_PARALLEL_POS_A__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
						HigherValue = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
								(TEC_FLOW_SETP_AT_10PERCENT_LOAD__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
						MIX.SetpointMixerPosition[mixer] = Interpolate ((DS32)PowerSetpointRelative,
                                (DS32)0, (DS32)1000, (DS32)LowerValue, (DS32)HigherValue );
					}
					else if (PowerSetpointRelative < 2000L) // power setpoint below 20%
					{
						// interpolation between 10% and 20%
						LowerValue = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
								(TEC_FLOW_SETP_AT_10PERCENT_LOAD__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
						HigherValue = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
								(TEC_FLOW_SETP_AT_20PERCENT_LOAD__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
						MIX.SetpointMixerPosition[mixer] = Interpolate ((DS32)PowerSetpointRelative,
                                (DS32)1000, (DS32)2000, (DS32)LowerValue, (DS32)HigherValue );
					}
					else if (PowerSetpointRelative < 3000L) // power setpoint below 30%
					{
						// interpolation between 20% and 30%
						LowerValue = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
								(TEC_FLOW_SETP_AT_20PERCENT_LOAD__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
						HigherValue = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
								(TEC_FLOW_SETP_AT_30PERCENT_LOAD__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
						MIX.SetpointMixerPosition[mixer] = Interpolate ((DS32)PowerSetpointRelative,
                                (DS32)2000, (DS32)3000, (DS32)LowerValue, (DS32)HigherValue );
					}
					else if (PowerSetpointRelative < 4000L) // power setpoint below 40%
					{
						// interpolation between 30% and 40%
						LowerValue = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
								(TEC_FLOW_SETP_AT_30PERCENT_LOAD__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
						HigherValue = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
								(TEC_FLOW_SETP_AT_40PERCENT_LOAD__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
						MIX.SetpointMixerPosition[mixer] = Interpolate ((DS32)PowerSetpointRelative,
                                (DS32)3000, (DS32)4000, (DS32)LowerValue, (DS32)HigherValue );
					}
					else // use setpoint for 40% power
					{
						MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
												(TEC_FLOW_SETP_AT_40PERCENT_LOAD__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
						LowerValue = HigherValue = MIX.SetpointMixerPosition[mixer];
					}

				}
				else
				{
					MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
																(MIX_PARALLEL_POS_A__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
				}

				MIX_SetpointInitDone[mixer] = TRUE;
            }
            else
            	TimerCalculateNewMixerPosition[mixer] +=100L;
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_SearchLimitLean, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
              // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveRich, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIdle, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			  // We have already come there.
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIsland, mixer);
			  break;
			}
            
             if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_Control, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
				
		break;
	}
}



/**
 * In this state the mixer moves to its stored island position.  
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_MoveIsland(const DU8 sig, DU8 mixer)
{
	DU16 StopCondInd;
	DU32 TimeoutDelay;

    if (mixer == MixerInd1)
    {
    	StopCondInd = STOPCONDITION_70235;
    }
    else//MixerInd2
    {
    	StopCondInd = STOPCONDITION_70712;
    }

	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                           = MIX_MOVING_TO_ISLAND_POS;
 			// calculate setpoint for mixer position
			MIX.TargetMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
									  (MIX_ISLAND_POS_A__PARREFIND, MIX_ISLAND_POS_B__PARREFIND);
		break;

		case SIG_EXIT:
		break;

		default:

            if (MIX.mode[mixer] == MIX_BLOCK)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_SystemOff, mixer);
				break;
			}

			// go to "island position reached" at any position
            if (MAIN.Simulation)
            {
            	Transit(MIX_IslandPositionReached, mixer);
            	break;
            }

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

        	TimeoutDelay = MIX_TIMEOUT_MOVING_TO_ISLAND_POS;

            if (MIX_OPTION_TECJET)
            {
				Transit(MIX_IslandPositionReached, mixer);
				break;
            }
            else if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
            {
            	TimeoutDelay = MAX_DU32; // No Timeout

				// increase/decrease setpoint by ramp every 2nd loop
            	if (myStateCnt[mixer] % 200 == 0L)
            	{
            		MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Ramp_Position(MIX.TargetMixerPosition[mixer], MIX.SetpointMixerPosition[mixer]);
            	}

            	// setpoint has reached target
            	if (MIX.SetpointMixerPosition[mixer] == MIX.TargetMixerPosition[mixer])
				{
					Transit(MIX_IslandPositionReached, mixer);
					break;
				}
            }
            else
            {
				// island position reached?
				// !!!
				if (MIX.ActualPositionOfGasMixer[mixer] == MIX.TargetMixerPosition[mixer])	// rmiMIXRAMP
				{
					Transit(MIX_IslandPositionReached, mixer);
					break;
				}

				// calculate a new setpoint to reach the target via a ramp
				if (MIX.ActualPositionOfGasMixer[mixer] == MIX.SetpointMixerPosition[mixer]) // position reached
					MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Ramp_Position				// rmiMIXRAMP
												(MIX.TargetMixerPosition[mixer], MIX.ActualPositionOfGasMixer[mixer]);
            }
            
            // timeout
            // idle position not reached within...
            if (myStateCnt[mixer] >= TimeoutDelay)
            {
            	STOP_Set( StopCondInd );
            }
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			    break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
            	// State change if MAIN_CONTROL has changed mode
			    Transit(MIX_MoveRich, mixer);
			    break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
            	// State change if MAIN_CONTROL has changed mode
			    Transit(MIX_MoveStart, mixer);
			    break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIdle, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveParallel, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			    // We are already moving there
			    break;

			}
            
            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_Control, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
				
		break;
	}
}





/**
 * In this state the mixer has reached the island position.
 *   
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_IslandPositionReached(const DU8 sig, DU8 mixer)
{
	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                           = MIX_ISLAND_POSITION_REACHED;
 			// calculate setpoint for mixer position
			TimerCalculateNewMixerPosition[mixer] = 0L;
			MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
										(MIX_ISLAND_POS_A__PARREFIND, MIX_ISLAND_POS_B__PARREFIND);
			MIX_SetpointInitDone[mixer] = TRUE;
		break;

		case SIG_EXIT:
		break;

		default:

            if (MIX.mode[mixer] == MIX_BLOCK)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_SystemOff, mixer);
			  break;
			}

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

            if (TimerCalculateNewMixerPosition[mixer] >= MIX_TIMER_CALCULATE_NEW_SETPOINT_POS)
            {
	 			// calculate setpoint for mixer position
				TimerCalculateNewMixerPosition[mixer] = 0L;
				MIX.SetpointMixerPosition[mixer] = Mix_Calculate_Setpoint_For_Mixer_Position	// rmiMIXRAMP
											(MIX_ISLAND_POS_A__PARREFIND, MIX_ISLAND_POS_B__PARREFIND);
            }
            else
            	TimerCalculateNewMixerPosition[mixer] +=100L;
			
			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_SearchLimitLean, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
              // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveRich, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIdle, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveParallel, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			  // We have already come there.
			  break;
			}
            
            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_Control, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}

		break;
	}
}


/**
 * In this state the mixer is controlled actively.
 *
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_Control(const DU8 sig, DU8 mixer)
{
	DU16 FuncInd;
	DU16 StopCondInd1;
	DU16 StopCondInd2;
	DS32 MIX_reg_divide;

	DS32 temp;
	DS32 change;
	static DS32 leftover;

	DS16 DeltaTemp[2];
	DS16 PressureCorrection;

	// regulation values
	DS32 dev; // deviation between actual pressure and setpoint in mbar
	DS32 de;  // change of deviation
	static DS32 devold;
	static DS32 deold;

	//DS32 StepperPositionSetpoint;
	static DS32 StepperPositionSetpointOld;
	DU8 i;

	if (mixer == MixerInd1)
	{
		FuncInd = MIXER_LIMIT_LEAN;

		StopCondInd1 = STOPCONDITION_70222;
		StopCondInd2 = STOPCONDITION_70223;
	}
	else//MixerInd2
	{
		FuncInd = MIXER_B_LIMIT_LEAN;

		StopCondInd1 = STOPCONDITION_70707;
		StopCondInd2 = STOPCONDITION_70708;
	}

	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                         = MIX_UNDER_CTRL;

			// only for mixer 1
			if (mixer == MixerInd1)
			{
				// reset old deviation and setpoint
				devold = 0;
				deold= 0;
				leftover = 0L;
				
				if (MIX_OPTION_TECJET)
	    		{
					/*
	    			MIX.LambdaSetpointTecJet = MIX.CalculatedLambda;
	    			MIX.SetpointMixerPosition[mixer] = Interpolate(MIX.LambdaSetpointTecJet,
	                		                                PARA[ParRefInd[TEC_LAMBDA_SETPOINT_FULL_LOAD__PARREFIND]].MaxValue,
	                		                                PARA[ParRefInd[TEC_LAMBDA_SETPOINT_FULL_LOAD__PARREFIND]].MinValue,
	                		                                0,
	                		                                PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value);
	                */

	    		}
				else if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
				{
					MIX.SetpointMixerPosition[mixer] = MIX.AO_GasMixer[mixer] - MIX.AnalogOutZero;
				}
				else
				{
					MIX.SetpointMixerPosition[mixer] = MIX.ActualPositionOfGasMixer[mixer];
				}
				
				StepperPositionSetpointOld = MIX.SetpointMixerPosition[mixer];
			}
			else // MixerInd2
			{
				MIX.SetpointMixerPosition[mixer] = MIX.SetpointMixerPosition[MixerInd1];
			}
			

    		if (MIX.SetpointMixerPosition[mixer] < 0)
    			MIX.SetpointMixerPosition[mixer] = 0;

    		if (MIX.SetpointMixerPosition[mixer] > (DS16)MIX.MixerFullRange_Out)
    			MIX.SetpointMixerPosition[mixer] = (DS16)MIX.MixerFullRange_Out;

        break;
            
		case SIG_EXIT:
			MIX_SetpointInitDone[mixer] = FALSE;
		break;

		default:

			PressureCorrection = (MIX.ReceiverPressureAvgFilteredValue + 1013); // why? empiric result...

    		if (MIX.mode[mixer] == MIX_BLOCK)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_SystemOff, mixer);
				break;
			}

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

			// not if analog gas mixer for mixer 1
			if ( (mixer == MixerInd2) || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == NOT_ASSIGNED) )
			{

				if (!(MIX_OPTION_TECJET))
				{
					// limit stop lean reached in control mode
					if ( (DI_FUNCT[FuncInd].Assigned == ASSIGNED) && (!MIX.DI_LimitStopLean[mixer]) )
					{
						STOP_Set(StopCondInd1);
					}

					// limit stop rich reached in control mode
					if ( ( (DI_FUNCT[MIXER_LIMIT_RICH].Assigned == ASSIGNED) && (MIX.DI_LimitStopRich) ) // for mixer B this is wrong
						|| (MIX.ActualPositionOfGasMixer[mixer] >= (DS16)MIX.MixerFullRange_In) ) // !!! check position
					{
						STOP_Set(StopCondInd2);
					}
				}
			}

            // only for mixer 1
			if (mixer == MixerInd1)
			{
            	DU8  gas;

				// lambda control
	            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 1L)
	            {
	            	MIX_reg_divide = 10000L;
	            	
	            	// Now we have a setpoint. PID controller
	                if (GBV.Active)
	                {
	                	if (ELM.T1E.sec.PsumRelative == 0)
	                		MIX.LambdaSetp = PARA[ParRefInd[MIX_SETPOINT_LAMBDA__PARREFIND]].Value;
	                	else
	                		MIX.LambdaSetp = (PARA[ParRefInd[MIX_SETPOINT_LAMBDA__PARREFIND]].Value*GBV.Pa + PARA[ParRefInd[MIX_SETPOINT_LAMBDA_GASB__PARREFIND]].Value*GBV.Pb) / ELM.T1E.sec.PsumRelative;
	                }
	                else if (!GAS.GasTypeBActive) // gas type A
	                	MIX.LambdaSetp = PARA[ParRefInd[MIX_SETPOINT_LAMBDA__PARREFIND]].Value;
	            	else // gas type B
	            		MIX.LambdaSetp = PARA[ParRefInd[MIX_SETPOINT_LAMBDA_GASB__PARREFIND]].Value;

	                // Deviation
	                dev = MIX.LambdaSetp - MIX.LambdaVoltageFilteredValue;

	            	// (higher lambda voltage = less oxygen partial pressure = too rich
	            	// lower lambda voltage = more oxygen partial pressure = too lean)

	            	// in case of inverted Lambda-Signal
	            	if (PARA[ParRefInd[MIX_INVERT_LAMBDA_REGULATION__PARREFIND]].Value)
	            		dev = -dev;
	            }
	            // p/T control
	            else if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 2L)
	            {
	            	DU16 ParIndex;

	            	MIX_reg_divide = 10000L;
	            	
	            	// LeaNox control
	            	
	            	for (gas=0; gas<=1; gas++)
	            	{
	            		i = 1;
		                while ( (ELM.T1E.sec.Psum > MIX.Setpoint_Mixer[gas][i].Psum) && (i < NUMBER_OF_MIXER_SETPOINTS - 1) ) i++;

		                TheoreticalTemp[gas]     = Interpolate ((DS32)ELM.T1E.sec.Psum,
		                                                        (DS32)MIX.Setpoint_Mixer[gas][i-1].Psum,  (DS32)MIX.Setpoint_Mixer[gas][i].Psum,
		                                                        (DS32)MIX.Setpoint_Mixer[gas][i-1].theta, (DS32)MIX.Setpoint_Mixer[gas][i].theta);
		                DeltaTemp[gas]           = MIX.ReceiverTemperature.Value - (DS16)TheoreticalTemp[gas]; // in 0,1 °C
		                TheoreticalPressure[gas] = Interpolate ((DS32)ELM.T1E.sec.Psum,
		                                                        (DS32)MIX.Setpoint_Mixer[gas][i-1].Psum, (DS32)MIX.Setpoint_Mixer[gas][i].Psum,
		                                                        (DS32)MIX.Setpoint_Mixer[gas][i-1].p,    (DS32)MIX.Setpoint_Mixer[gas][i].p);

		                if (gas == 0) ParIndex = MIX_P_T_FACTOR_A__PARREFIND;
		                else          ParIndex = MIX_P_T_FACTOR_B__PARREFIND;

		                MIX.PressSetp[gas] = (TheoreticalPressure[gas] *1000L + DeltaTemp[gas] * (DS32)PARA[ParRefInd[ParIndex]].Value) /1000L;
	            	}

	                if (GBV.Active)
	                {
	                	if (ELM.T1E.sec.PsumRelative == 0)
	                		MIX.Setpoint_Receiver_Pressure = MIX.PressSetp[0];
	                	else
	                		MIX.Setpoint_Receiver_Pressure = ((DS32)MIX.PressSetp[0]*GBV.Pa + (DS32)MIX.PressSetp[1]*GBV.Pb) / ELM.T1E.sec.PsumRelative;
	                }
	                else if (!GAS.GasTypeBActive) // gas type A
	                {
		                MIX.Setpoint_Receiver_Pressure = MIX.PressSetp[0];
	                }
	                else // gas type B
	                {
		                MIX.Setpoint_Receiver_Pressure = MIX.PressSetp[1];
	                }
#ifdef TODO_OLD
	                // ******************* Control algorithm for mixer ************************************
	                // get setpoint for receiver pressure from temperature, power and curve
	            
	                // Determine the valid section of the setpoint curve
	                // This is needed for both methods
	                i = 1; // start with points 0 and 1
	                //as long a actual power is bigger than power of the curve point, increase counter i 
	                //rmiKW while (    (ELM.T1E.sec.PsumRelative > MIX.Setpoint_Mixer[i].PsumRel)
	                while (    (ELM.T1E.sec.Psum > MIX.Setpoint_Mixer[i].Psum)
	                    && (i < NUMBER_OF_MIXER_SETPOINTS - 1) )
	                    i++; 
	                // valid points are now ...[i-1] and ...[i]
	
	                // Regulation algorithm DEIF (using p/T values) not used by IET software
	                /* 
	                if (algorithm == DEIF) // use p/T values
	                {
	            
	            	    // Calculation of pressure setpoint by using p/T directly
	                    // 2. calculate a p/T setpoint at the actual power by linear interpolation between 
	                    // 2 curve points in 0.001 mbar (abs) /0.1K (abs)
	                    // y = Interpolate (x, x1, x2, y1, y2);
	                    PressdivTemp = Interpolate ((DS32)ELM.T1E.sec.PsumRelative,
	                                        (DS32)MIX.Setpoint_Mixer[i-1].PsumRel, (DS32)MIX.Setpoint_Mixer[i].PsumRel,
	                                        (DS32)(1013 + MIX.Setpoint_Mixer[i-1].p)*1000/(2732 + MIX.Setpoint_Mixer[i-1].theta),
	                                        (DS32)(1013 + MIX.Setpoint_Mixer[i].p)*1000/(2732 + MIX.Setpoint_Mixer[i].theta) );
	            
	            	    // P setpoint = p/T from curve x actual receiver temperature (absolut, K)	
	            	    MIX.Setpoint_Receiver_Pressure = PressdivTemp * (MIX.ReceiverTemperature.Value + 2732)/1000 - 1013;
	            	    // setpoint in mbar overpressure (compared to environment) 
	            	
	                }
	                else // use p and T values and a correction factor */
	            
	                // use p and T values and a correction factor
	                //P setpoint = theoretical pressure + delta T x factor
	            	
	                // construct a temperature of a theoretical calibration point at actual power
	                // by linear interpolation between 2 curve points in 0,1 degrees Celsius
	                // y = Interpolate (x, x1, x2, y1, y2); 
	                //rmiKW TheoreticalTemp = Interpolate ((DS32)ELM.T1E.sec.PsumRelative, 
	                //rmiKW                               (DS32)MIX.Setpoint_Mixer[i-1].PsumRel, (DS32)MIX.Setpoint_Mixer[i].PsumRel,
	                TheoreticalTemp = Interpolate ((DS32)ELM.T1E.sec.Psum, 
	                                           (DS32)MIX.Setpoint_Mixer[i-1].Psum, (DS32)MIX.Setpoint_Mixer[i].Psum,
	                                           (DS32)MIX.Setpoint_Mixer[i-1].theta, (DS32)MIX.Setpoint_Mixer[i].theta);
	                DeltaTemp       = MIX.ReceiverTemperature.Value - (DS16)TheoreticalTemp; // in 0,1 °C
	                TheoreticalPressure = Interpolate ((DS32)ELM.T1E.sec.Psum,
	                                               (DS32)MIX.Setpoint_Mixer[i-1].Psum, (DS32)MIX.Setpoint_Mixer[i].Psum,
	                                               (DS32)MIX.Setpoint_Mixer[i-1].p, (DS32)MIX.Setpoint_Mixer[i].p);
	                // calculate setpoint for receiver pressure in mbar overpressure
	                // factor is 10...1000 = 0,10...10,00, has to be divided by 100
	                if (!GAS.GasTypeBActive) // gas type A
	                {
		                MIX.Setpoint_Receiver_Pressure = (TheoreticalPressure *1000L 
		                                       + DeltaTemp * (DS32)PARA[ParRefInd[MIX_P_T_FACTOR_A__PARREFIND]].Value) /1000L;
	                }
	                else // gas type B
	                {
		                MIX.Setpoint_Receiver_Pressure = (TheoreticalPressure *1000L 
		                                       + DeltaTemp * (DS32)PARA[ParRefInd[MIX_P_T_FACTOR_B__PARREFIND]].Value) /1000L;
	                }
#endif
	                // offset (island)
	                if (HVS.stateL1E != HVS_L1E_IS_ON)
	                	MIX.Setpoint_Receiver_Pressure += (DS16)PARA[ParRefInd[MIX_OFFSET_ISLAND__PARREFIND]].Value;
	                
	                // Now we have a setpoint. PID controller (common for both regulation algorithms)
	                dev = MIX.ReceiverPressureAvgFilteredValue - MIX.Setpoint_Receiver_Pressure;
	            }
	            // combustion chamber temperature control
	            else if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 3L)
	            {
	            	MIX_reg_divide = 100000L;

#if (OPTION_CYLINDER_MONITORING == TRUE)
	            	// Temperature/Power

	            	for (gas=0; gas<=1; gas++)
	            	{
		            	i = 1;
		                while ( (ELM.T1E.sec.Psum > MIX.Setpoint_Mixer[gas][i].Psum) && (i < NUMBER_OF_MIXER_SETPOINTS - 1) ) i++;

		                MIX.TempSetp[gas] = Interpolate ((DS32)ELM.T1E.sec.Psum,
	                                                     (DS32)MIX.Setpoint_Mixer[gas][i-1].Psum,  (DS32)MIX.Setpoint_Mixer[gas][i].Psum,
	                                                     (DS32)MIX.Setpoint_Mixer[gas][i-1].theta, (DS32)MIX.Setpoint_Mixer[gas][i].theta);
	            	}

	                if (GBV.Active)
	                {
	                	if (ELM.T1E.sec.PsumRelative == 0)
			                MIX.Setpoint_Cylinder_Temperatue = MIX.TempSetp[0];
	                	else
	                		MIX.Setpoint_Cylinder_Temperatue = ((DS32)MIX.TempSetp[0]*GBV.Pa + (DS32)MIX.TempSetp[1]*GBV.Pb) / ELM.T1E.sec.PsumRelative;
	                }
	                else if (!GAS.GasTypeBActive) // gas type A
	                {
		                MIX.Setpoint_Cylinder_Temperatue = MIX.TempSetp[0];
	                }
	                else // gas type B
	                {
		                MIX.Setpoint_Cylinder_Temperatue = MIX.TempSetp[1];
	                }
#ifdef TODO_OLD
	                // ******************* Control algorithm for mixer ************************************
	                // get setpoint for cylinder temperature

	                // Determine the valid section of the setpoint curve
	                i = 1; // start with points 0 and 1
	                //as long a actual power is bigger than power of the curve point, increase counter i
	                while ( (ELM.T1E.sec.Psum > MIX.Setpoint_Mixer[i].Psum) && (i < NUMBER_OF_MIXER_SETPOINTS - 1) ) i++;
	                // valid points are now ...[i-1] and ...[i]

	                // construct a temperature of a theoretical calibration point at actual power
	                // by linear interpolation between 2 curve points in 0,1 degrees Celsius
	                // y = Interpolate (x, x1, x2, y1, y2);
	                MIX.Setpoint_Cylinder_Temperatue = Interpolate ((DS32)ELM.T1E.sec.Psum,
                            (DS32)MIX.Setpoint_Mixer[i-1].Psum, (DS32)MIX.Setpoint_Mixer[i].Psum,
                            (DS32)MIX.Setpoint_Mixer[i-1].theta, (DS32)MIX.Setpoint_Mixer[i].theta) /*- CYL.CylinderAverageTemp*/;
#endif
	                // offset (island)
	                if (HVS.stateL1E != HVS_L1E_IS_ON)
	                	MIX.Setpoint_Cylinder_Temperatue += (DS16)PARA[ParRefInd[MIX_CYL_OFFSET_ISLAND__PARREFIND]].Value;

	                dev = MIX.Setpoint_Cylinder_Temperatue - CYL.CylinderAverageTemp;
#else
	                dev = 0;
#endif
	            }
	            else // no emission control
	            {
	            	MIX_reg_divide = 1L;
	            	dev = 0;
	            }
	            
	            // common for lambda control and p,T control
	            
	            if (MIX.Option_ENSMP)
	            {
	            	StepperPositionSetpoint = (DS32)MIX.ActualPositionOfGasMixer[MixerInd1] * (DS32)MIX.MixerFullRange_Out / (DS32)MIX.MixerFullRange_In;
	            }
	            else
	            {
		            // calculate change of deviation
		            de  = dev - devold;

		            // remember leftover from deviation
		            temp =  leftover // from last cycle
				    		   + (de * (DS32)PARA[ParRefInd[MIX_REG_CONST_KP__PARREFIND]].Value // Kp
				               + dev * (DS32)PARA[ParRefInd[MIX_REG_CONST_KI__PARREFIND]].Value // Ki
				               + (de - deold) * (DS32)PARA[ParRefInd[MIX_REG_CONST_KD__PARREFIND]].Value); // Kd

		            change = (temp + MIX_reg_divide/2L)/MIX_reg_divide;

			        leftover = temp - change * MIX_reg_divide;

		            deold = de;
		            devold = dev;

			        StepperPositionSetpoint = StepperPositionSetpointOld + change;
	            }

		        if (tecjet[TECJET_1].Option && tecjet[TECJET_2].Option)
		        {
		        	// limitation of StepperPositionSetpoi
		        	if (StepperPositionSetpoint < 0)
		        		StepperPositionSetpoint = 0;

		        	if (StepperPositionSetpoint > MIX.Tecjet_Max_Flow_Rate)
		        		StepperPositionSetpoint = MIX.Tecjet_Max_Flow_Rate;

		        }
		        else if(tecjet[TECJET_1].Option && !tecjet[TECJET_2].Option)
		        {
		        	// limitation of StepperPositionSetpoi
		        	if (StepperPositionSetpoint < 0)
		        		StepperPositionSetpoint = 0;

		        	if (StepperPositionSetpoint > (DS32)PARA[ParRefInd[TEC_MAX_FLOW__PARREFIND]].Value)
		        		StepperPositionSetpoint = (DS32)PARA[ParRefInd[TEC_MAX_FLOW__PARREFIND]].Value;
		        }
		        else if(!tecjet[TECJET_1].Option && tecjet[TECJET_2].Option)
		        {
		        	// limitation of StepperPositionSetpoi
		        	if (StepperPositionSetpoint < 0)
		        		StepperPositionSetpoint = 0;

		        	if (StepperPositionSetpoint > (DS32)PARA[ParRefInd[TEC_2_MAX_FLOW__PARREFIND]].Value)
		        		StepperPositionSetpoint = (DS32)PARA[ParRefInd[TEC_2_MAX_FLOW__PARREFIND]].Value;
		        }
		        else
		        {
		        	// limitation of StepperPositionSetpoint,			gfh, 06.07.2009
		        	if (StepperPositionSetpoint < 0)
		        		StepperPositionSetpoint = 0;
	
		        	if (StepperPositionSetpoint > (DS16)MIX.MixerFullRange_Out)
		        		StepperPositionSetpoint = (DS16)MIX.MixerFullRange_Out;
		        }
	            StepperPositionSetpointOld = StepperPositionSetpoint;
	            
	    		// end of PID controller
	
	            // overwrite setpoint for first cycle in MIX_Control
	            if (myStateCnt[mixer] <= 100L)
	            {
	            	// gas mixer, not TecJet
		            if (!(MIX_OPTION_TECJET))
		            {
		            	if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
		            		StepperPositionSetpoint = MIX.AO_GasMixer[mixer] - MIX.AnalogOutZero;
		            	else
			            	StepperPositionSetpoint = MIX.ActualPositionOfGasMixer[mixer];
		            }
		            else if(tecjet[TECJET_1].Option && !tecjet[TECJET_2].Option)
		            {
		            	// TecJet
		            	if ((MIX.DKFactor != 0) && (PressureCorrection != 0))
		            	StepperPositionSetpoint = tecjet[TECJET_1].write.FuelFlowRate
													* 1013
													/ PressureCorrection // increases stability
													*100 / MIX.DKFactor; // throttle dependency, important for stability
		            }
		            else if(!tecjet[TECJET_1].Option && tecjet[TECJET_2].Option)
		            {
		            	// TecJet2
		            	if ((MIX.DKFactor != 0) && (PressureCorrection != 0))
		            	StepperPositionSetpoint = tecjet[TECJET_2].write.FuelFlowRate
													* 1013
													/ PressureCorrection // increases stability
													*100 / MIX.DKFactor; // throttle dependency, important for stability
		            }
		            else //tecjet[TECJET_1].Option && tecjet[TECJET_2].Option
		            {
		            	// both two TecJet
		            	if ((MIX.DKFactor != 0) && (PressureCorrection != 0))
		            	StepperPositionSetpoint = (tecjet[TECJET_1].write.FuelFlowRate + tecjet[TECJET_2].write.FuelFlowRate)
													* 1013
													/ PressureCorrection // increases stability
													*100 / MIX.DKFactor; // throttle dependency, important for stability
		            }
		            // in any case:
		            StepperPositionSetpointOld = StepperPositionSetpoint;
	            }
/*            
	            if (MIX.SetpointMixerPosition == MIX.ActualPositionOfGasMixer)
	            {
	    		// Write new setpoint, slowly, only x steps per turn (regulation)
	    		if (StepperPositionSetpoint >= (MIX.ActualPositionOfGasMixer + x))
	    		{
		    		  MIX.SetpointMixerPosition = MIX.ActualPositionOfGasMixer + 1;
	
	    		}
	    		else if (StepperPositionSetpoint <= (MIX.ActualPositionOfGasMixer - x))
	    		{
		    		  MIX.SetpointMixerPosition = MIX.ActualPositionOfGasMixer - 1;
	
	    		}
	    		else
	    			  MIX.SetpointMixerPosition = MIX.ActualPositionOfGasMixer; // stop moving
	            }	  
*/
	    		// old code reactivated 23.12.2008 MVO 1.30.7
	            MIX.SetpointMixerPosition[mixer] = StepperPositionSetpoint;

				tecjet[TECJET_1].write.FuelFlowRate = StepperPositionSetpoint
														* MIX.Tecjet_Fraction_1 / 1000
														* PressureCorrection/1013 // to increase stability
														* MIX.DKFactor/100; // throttle dependency (important for stability)
				tecjet[TECJET_2].write.FuelFlowRate = StepperPositionSetpoint
														* MIX.Tecjet_Fraction_2 / 1000
														* PressureCorrection/1013 // to increase stability
														* MIX.DKFactor/100; // throttle dependency (important for stability)

	            if (MIX_OPTION_TECJET)
	            {
					// update tecjet
	            	/* removed for test 24.07.2015 MVO
					MIX.LambdaSetpointTecJet = Interpolate(MIX.SetpointMixerPosition[mixer],
														   0,
														   PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value,
														   PARA[ParRefInd[TEC_LAMBDA_SETPOINT_FULL_LOAD__PARREFIND]].MaxValue,
														   PARA[ParRefInd[TEC_LAMBDA_SETPOINT_FULL_LOAD__PARREFIND]].MinValue);
					*/
	            }
			}
			else // MixerInd2
			{
				MIX.SetpointMixerPosition[mixer] = MIX.SetpointMixerPosition[MixerInd1];
			}

			if (MIX.mode[mixer] == MIX_MOVE_LEAN)
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			    break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_RICH)
            {
            	// State change if MAIN_CONTROL has changed mode
			    Transit(MIX_MoveRich, mixer);
			    break;
			}
            
            if (MIX.mode[mixer] == MIX_MOVE_TO_START_POSITION)
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_MoveStart, mixer);
			    break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_IDLE_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIdle, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_PARALLEL_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveParallel, mixer);
			  break;
			}

            if (MIX.mode[mixer] == MIX_MOVE_TO_ISLAND_POSITION)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveIsland, mixer);
			  break;
			}
            
            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (!MIX.CalibrationDone[mixer]) )
			{
			    // State change if MAIN_CONTROL has changed mode
			    Transit(MIX_SearchLimitLean, mixer);
			  break;
			}

            if ( (MIX.mode[mixer] == MIX_CALIBRATE)
            	&& (MIX.CalibrationDone[mixer]) )
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_MoveStart, mixer);
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_CTRL)
			{
			  // We are already controlling the mixer actively. Stay here.
			  break;
			}
            
            if (MIX.mode[mixer] == MIX_TEST_DEMANDED)
			{
			  // State change if MAIN_CONTROL has changed mode
			  Transit(MIX_UnderTest, mixer);
			  break;
			}
				
		break;
	}
		
}


/**
 * In this state the mixer is under test. 
 * 
 * @author  MVO
 * @date    2008-06-03
 */
static void MIX_UnderTest(const DU8 sig, DU8 mixer)
{	
	static DS16 NumberOfSteps;
	static DS16 BigSteps;
	
	DU16 FuncInd;

    if (mixer == MixerInd1)
    {
    	FuncInd = MIXER_LIMIT_LEAN;
    }
    else//MixerInd2
    {
    	FuncInd = MIXER_B_LIMIT_LEAN;
    }
	
	switch(sig)
	{
		case SIG_ENTRY:
			MIX.state[mixer]                           = MIX_UNDER_TEST;

			MIX.StepperMotor[mixer].forceLean = FALSE;
			
			MIX.GasMixerDirectionLeanTestDemand[mixer] = OFF;
			MIX.GasMixerDirectionRichTestDemand[mixer] = OFF;
		    // BigSteps used for MIX.Manual, initialization here
			BigSteps                               	   = MIX.MixerFullRange_Out * MIX_BIG_STEP_IN_CONFIGURATION / 10000L;
			if 	( MIX.Manual )
	        {
	            NumberOfSteps                          = MIX.MixerFullRange_Out * MIX_STEP_IN_CONFIGURATION / 10000L;
	        }
	        else
	        {
	            NumberOfSteps                          = MIX.MixerFullRange_Out * MIX_STEP_IN_TEST_MODE / 10000L;
	        }
	        if (NumberOfSteps < 1) NumberOfSteps = 1;		// be sure, that we have at least one step

	        if (tecjet[TECJET_1].Option && (tecjet[TECJET_1].write.FuelFlowRate != 0))
	        	MIX.LambdaSetpointTecJet = (((DF32)MIX.Flow * MIX.Tecjet_Fraction_1 / 1000 / tecjet[TECJET_1].write.FuelFlowRate * TEMP_COMPENSATION_1) - 1) / CALCULATION_FACTOR;
	        else if(tecjet[TECJET_2].Option && (tecjet[TECJET_2].write.FuelFlowRate != 0))
	        	MIX.LambdaSetpointTecJet = (((DF32)MIX.Flow * MIX.Tecjet_Fraction_2 / 1000 / tecjet[TECJET_2].write.FuelFlowRate * TEMP_COMPENSATION_2) - 1) / CALCULATION_FACTOR;
		break;

		case SIG_EXIT:

			MIX.GasMixerDirectionLeanTestDemand[mixer] = OFF;
			MIX.GasMixerDirectionRichTestDemand[mixer] = OFF;
			if (AI_I_FUNCT[IOA_AI_I_MIX_GASMIXER_POS].Assigned == NOT_ASSIGNED)
			{
				MIX.SetpointMixerPosition[mixer] = MIX.ActualPositionOfGasMixer[mixer];
			}
			MIX.Config							       = FALSE;

		break;

		default:

            if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 0L)
			{
			  // no emission control
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}
			
			if (mixer == MixerInd2)
			if (!PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value || (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED))
			{
			  Transit(MIX_StartPositionReached, mixer);
			  break;
			}

			if (MIX.mode[mixer] == MIX_CTRL)
			{
				// State change if MAIN_CONTROL has changed mode
				Transit(MIX_Control, mixer);
				break;
			}

			if (MIX.mode[mixer]   !=  MIX_TEST_DEMANDED)
			{
				// in all other cases except test demand go to system off
				Transit(MIX_SystemOff, mixer);
				break;
			}
			
         
		    // Put test demands through to the outputs

		    if (MIX_OPTION_TECJET)
		    {
		    	if (MIX.GasMixerDirectionLeanTestDemand[mixer])
		    	{
		    		if (MIX.Fast)
		    			MIX.LambdaSetpointTecJet += 5*MIX_LAMBDA_STEP_IN_CONFIG;
		    		else
		    			MIX.LambdaSetpointTecJet += MIX_LAMBDA_STEP_IN_CONFIG;

		    		if (MIX.LambdaSetpointTecJet > (DU16)PARA[ParRefInd[TEC_LAMBDA_SETPOINT_0kW__PARREFIND]].MaxValue)
		    			MIX.LambdaSetpointTecJet = (DU16)PARA[ParRefInd[TEC_LAMBDA_SETPOINT_0kW__PARREFIND]].MaxValue;

					MIX.GasMixerDirectionLeanTestDemand[mixer] = OFF;
		    	}
		    	else if (MIX.GasMixerDirectionRichTestDemand[mixer])
		    	{
		    		if (MIX.Fast)
		    			MIX.LambdaSetpointTecJet -= 5*MIX_LAMBDA_STEP_IN_CONFIG;
		    		else
		    			MIX.LambdaSetpointTecJet -= MIX_LAMBDA_STEP_IN_CONFIG;

		    		if (MIX.LambdaSetpointTecJet < (DU16)PARA[ParRefInd[TEC_LAMBDA_SETPOINT_0kW__PARREFIND]].MinValue)
		    			MIX.LambdaSetpointTecJet = (DU16)PARA[ParRefInd[TEC_LAMBDA_SETPOINT_0kW__PARREFIND]].MinValue;

					MIX.GasMixerDirectionRichTestDemand[mixer] = OFF;
		    	}
		    }
		    // analog gas mixer
		    else if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
		    {
		    	if (MIX.Option_ENSMP)
		    	{
		    		MIX.SetpointMixerPosition[mixer] = (DS32)MIX.ActualPositionOfGasMixer[MixerInd1] * (DS32)MIX.MixerFullRange_Out / (DS32)MIX.MixerFullRange_In;
		    	}
		    	else
		    	{
			    	if (MIX.GasMixerDirectionLeanTestDemand[mixer])
			    	{
						if (MIX.Manual)
						{
				    		if (MIX.Fast)
				    			MIX.SetpointMixerPosition[mixer] = MIX.SetpointMixerPosition[mixer] - BigSteps;
				    		else
				    		    MIX.SetpointMixerPosition[mixer] = MIX.SetpointMixerPosition[mixer] - NumberOfSteps;

				    		// limitation
				    		if (MIX.SetpointMixerPosition[mixer] < 0) MIX.SetpointMixerPosition[mixer] = 0;
						}
						else  // decrease setpoint by ramp (running time)
							MIX.SetpointMixerPosition[mixer] = MIX_RunningTimeRamp((DS32)MIX.SetpointMixerPosition[mixer], 0, MIX.MixerFullRange_Out);

						if (MIX.SetpointMixerPosition[mixer] == 0)
							MIX.GasMixerDirectionLeanTestDemand[mixer] = OFF;
			    	}
			    	else
			    	if (MIX.GasMixerDirectionRichTestDemand[mixer])
			    	{
						if (MIX.Manual)
						{
				    		if (MIX.Fast)
				    			MIX.SetpointMixerPosition[mixer] = MIX.SetpointMixerPosition[mixer] + BigSteps;
				    		else
				    		    MIX.SetpointMixerPosition[mixer] = MIX.SetpointMixerPosition[mixer] + NumberOfSteps;

				    		// limitation
				    		if (MIX.SetpointMixerPosition[mixer] > MIX.MixerFullRange_Out) MIX.SetpointMixerPosition[mixer] = MIX.MixerFullRange_Out;
						}
						else  // increase setpoint by ramp (running time)
							MIX.SetpointMixerPosition[mixer] = MIX_RunningTimeRamp((DS32)MIX.SetpointMixerPosition[mixer], MIX.MixerFullRange_Out, MIX.MixerFullRange_Out);

						if (MIX.SetpointMixerPosition[mixer] == MIX.MixerFullRange_Out)
							MIX.GasMixerDirectionRichTestDemand[mixer] = OFF;
			    	}
		    	}
		    }
		    else
		    {
			    if (MIX.ResetStepCounter[mixer])
			    {
			        MIX.StepperMotor[mixer].reset = TRUE;
			        MIX.SetpointMixerPosition[mixer] = 0;
			        MIX.GasMixerDirectionLeanTestDemand[mixer] = OFF;
			        MIX.GasMixerDirectionRichTestDemand[mixer] = OFF;

			        if (MIX.ActualPositionOfGasMixer[mixer] == 0)
			        	MIX.ResetStepCounter[mixer] = FALSE;
			    }
			    else if ( (MIX.GasMixerDirectionLeanTestDemand[mixer])
			         // lean is requested
			    	&& (MIX.DI_LimitStopLean[mixer] || DI_FUNCT[FuncInd].Assigned == NOT_ASSIGNED)
			    	&& (MIX.ActualPositionOfGasMixer[mixer] > -25000)
			    	)
			    {
			    	if ((MIX.ActualPositionOfGasMixer[mixer] ==  MIX.SetpointMixerPosition[mixer]) || MAIN.Simulation)
			    	{
			    		// setpoint reached? -> decrease setpoint
			    		if (MIX.Fast)
			    			MIX.SetpointMixerPosition[mixer] = MIX.SetpointMixerPosition[mixer] - BigSteps;
			    		else
			    		    MIX.SetpointMixerPosition[mixer] = MIX.SetpointMixerPosition[mixer] - NumberOfSteps;

			    		if (MIX.SetpointMixerPosition[mixer] < -25000)
			    		{
			    			MIX.SetpointMixerPosition[mixer] = -25000; // do not go below -25000
			    		}
			    	}
			    }
			    else if ( (MIX.GasMixerDirectionRichTestDemand[mixer] )
			         // rich is requested
			    	&& (!MIX.DI_LimitStopRich || DI_FUNCT[MIXER_LIMIT_RICH].Assigned == NOT_ASSIGNED)
			    	&& (MIX.ActualPositionOfGasMixer[mixer] < (DS32)PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value) )
			    {
			    	if ((MIX.ActualPositionOfGasMixer[mixer] ==  MIX.SetpointMixerPosition[mixer]) || MAIN.Simulation)
			    	{
			    		// setpoint reached? -> increase setpoint
			    		if (MIX.Fast)
			    			MIX.SetpointMixerPosition[mixer] = MIX.SetpointMixerPosition[mixer] + BigSteps;
			    		else
			    		    MIX.SetpointMixerPosition[mixer] = MIX.SetpointMixerPosition[mixer] + NumberOfSteps;
			    		if (MIX.SetpointMixerPosition[mixer] > (DS16)PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value)
			    			MIX.SetpointMixerPosition[mixer] = (DS16)PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value;
			    	}
			    }
			    else if (STOP_is_Set(STOPCONDITION_20099) && //test operation
			    		( ( MIX.ActualPositionOfGasMixer[mixer] ==  (DS32)MIX.SetpointMixerPosition[mixer])
			    			|| (!MIX.DI_LimitStopLean[mixer])
			    			|| (MIX.ActualPositionOfGasMixer[mixer] <= -25000)
			    			) )
		    	{
		    		MIX.SetpointMixerPosition[mixer] = (DS16)MIX.ActualPositionOfGasMixer[mixer];

		    		MIX.GasMixerDirectionLeanTestDemand[mixer] = OFF;
		    		MIX.GasMixerDirectionRichTestDemand[mixer] = OFF;
		    	}
			}

	    	//  mixer is in configuration mode
	    	if (MIX.Manual && !MIX.Option_ENSMP)
	    	{
	    		MIX.GasMixerDirectionLeanTestDemand[mixer] = OFF;
	    		MIX.GasMixerDirectionRichTestDemand[mixer] = OFF;
	    	}
		    
		break;
	}

}

void MIX_control_20ms(void)
{
	DU8 i;

	static DS16 RP[MIX_NUMBER_OF_RECP_VALUES_FOR_FILTERING];  // used for filtering the Receiver pressure
	static DS16 RPB[MIX_NUMBER_OF_RECP_VALUES_FOR_FILTERING]; // Receiver pressure B
	DS32 RPsum;                                               // used for filtering the Receiver pressure

	// intermediate values for TecJet Flowsetpoint calculation
	DS16 SetpointMixerPositionRelative;	// 0...10000 = 0...100%
	DS16 SpeedRelative;					// 0...10000 = 0...100%
#if (CLIENT_VERSION == IET)
	DS16 SpeedSetRelative;				// test
	//DS16 FuelTemperatureAbsolute;		// 0...5000  = 0...500K
	//DS16 ReceiverTemperatureAbsolute;	// 0...5000  = 0...500K
	//DS16 DensityFactorRelative;			// 0...10000 = 1...100%
	DS16 ThrottlePositionRelative;		// 0...10000 = 1...100%
	DS16 ThrottlePositionSetpointRelative; // 0...10000 = 0...100%
#endif
	DS16 PressureCorrection;			// 0...3000 = 0...3000mbar abs
	DS32 DKFactor;						//
	DS32 LoadDependentPart;				// 0...3600000 = 0...3600 Nm³/h additional flow

	if (PARA[ParRefInd[MIX_OPTION_RECEIV_PRESS_SENSOR__PARREFIND]].Value)
	{
		DS32 value_4mA;
		DS32 value_20mA;

		// wire break for receiver pressure
		if (AI_I_FUNCT[RECEIVER_PRESSURE].Assigned == ASSIGNED)
		{
			if( MIX.AI_I_ReceiverPressure > (DS16)PARA[ParRefInd[MIX_RECEIV_PRESS_WIREBREAK__PARREFIND]].Value )
			{
			  // no wire break
			  STOP_Tripped[STOPCONDITION_70237] = FALSE; // can be acknowledged

			  value_4mA  = PARA[ParRefInd[MIX_RECEIV_PRESS_VALUE_FOR_4mA__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_SENSOR;
			  value_20mA = PARA[ParRefInd[MIX_RECEIV_PRESS_VALUE_FOR_20mA__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_SENSOR;

			  MIX.ReceiverPressure = value_4mA
						+ ( (DS32) MIX.AI_I_ReceiverPressure - 5000L )
						* ( value_20mA - value_4mA )
						/ 20000L;
			}
			else
			{
			  // wire break
			  MIX.ReceiverPressure = -32768; // is this a good idea?
			  STOP_Set(STOPCONDITION_70237);
			  STOP_Tripped[STOPCONDITION_70237] = TRUE; // tripped = cannot be acknowledged
			}
		}
		else STOP_Tripped[STOPCONDITION_70237] = FALSE; // can be acknowledged

		// Receiver pressure B
		if (AI_I_FUNCT[IOA_AI_I_MIX_RECEIVER_PRESSURE_B].Assigned == ASSIGNED)
		{
			if( MIX.AI_I_ReceiverPressureB > (DS16)PARA[ParRefInd[MIX_RECEIV_PRESS_WIREBREAK__PARREFIND]].Value )
			{
			  // no wire break
			  STOP_Tripped[STOPCONDITION_70246] = FALSE;

			  value_4mA  = PARA[ParRefInd[MIX_RECEIV_PRESS_VALUE_FOR_4mA__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_SENSOR;
			  value_20mA = PARA[ParRefInd[MIX_RECEIV_PRESS_VALUE_FOR_20mA__PARREFIND]].Value + PAR_OFFSET_REC_PRESS_SENSOR;

			  MIX.ReceiverPressureB = value_4mA
						+ ( (DS32) MIX.AI_I_ReceiverPressureB - 5000L )
						* ( value_20mA - value_4mA )
						/ 20000L;
			}
			else
			{
			  // wire break
			  MIX.ReceiverPressureB = -32768;
			  STOP_Set(STOPCONDITION_70246);
			  STOP_Tripped[STOPCONDITION_70246] = TRUE;
			}
		}
		else STOP_Tripped[STOPCONDITION_70246] = FALSE;
	}
	else
	{
		STOP_Tripped[STOPCONDITION_70237] = FALSE;
		STOP_Tripped[STOPCONDITION_70246] = FALSE;
	}

	// calculate filtered value MIX.ReceiverPressureFilteredValue from MIX.ReceiverPressure
	if (MIX.ReceiverPressure != -32768)
	{
		RPsum = 0L;
		for (i = (MIX_NUMBER_OF_RECP_VALUES_FOR_FILTERING - 1); i>0; i--)
		{
			RPsum += RP[i];
			RP[i] = RP[i-1];

		}
		RP[0] = MIX.ReceiverPressure; // new value
		RPsum += MIX.ReceiverPressure;

		MIX.ReceiverPressureFilteredValue = RPsum / MIX_NUMBER_OF_RECP_VALUES_FOR_FILTERING;
	}

	// Receiver pressure B
	if (MIX.ReceiverPressureB != -32768)
	{
		RPsum = 0L;
		for (i = (MIX_NUMBER_OF_RECP_VALUES_FOR_FILTERING - 1); i>0; i--)
		{
			RPsum += RPB[i];
			RPB[i] = RPB[i-1];

		}
		RPB[0] = MIX.ReceiverPressureB; // new value
		RPsum += MIX.ReceiverPressureB;

		MIX.ReceiverPressureBFilteredValue = RPsum / MIX_NUMBER_OF_RECP_VALUES_FOR_FILTERING;
	}

	// Average receiver pressure
	{
		DS32 Pressure = 0L;
		DS32 PressureFilteredValue = 0L;
		DU8 ValidSensors = 0;

		if (AI_I_FUNCT[RECEIVER_PRESSURE].Assigned == ASSIGNED)
		{
			if (MIX.ReceiverPressure != -32768)
			{
				Pressure += MIX.ReceiverPressure;
				PressureFilteredValue += MIX.ReceiverPressureFilteredValue;

				ValidSensors++;
			}
		}

		if (AI_I_FUNCT[IOA_AI_I_MIX_RECEIVER_PRESSURE_B].Assigned == ASSIGNED)
		{
			if (MIX.ReceiverPressureB != -32768)
			{
				Pressure += MIX.ReceiverPressureB;
				PressureFilteredValue += MIX.ReceiverPressureBFilteredValue;

				ValidSensors++;
			}
		}

		// Update Average
		if (ValidSensors != 0)
		{
			MIX.ReceiverPressureAvg = Pressure / ValidSensors;
			MIX.ReceiverPressureAvgFilteredValue = PressureFilteredValue / ValidSensors;
		}
		else // No valid sensor
		{
			MIX.ReceiverPressureAvg = -32768;

			// keep last valid filtered value
		}
	}

	  // TECJET
	  // calculate mixture flow [0.001m³/h]
	  // 3 Liter times 20 cylinders

	if(tecjet[TECJET_1].Option && !tecjet[TECJET_2].Option)
	{
		MIX.Tecjet_Fraction_1 = 1000;
		MIX.Tecjet_Fraction_2 = 0;
	}
	if(!tecjet[TECJET_1].Option && tecjet[TECJET_2].Option)
	{
		MIX.Tecjet_Fraction_1 = 0;
		MIX.Tecjet_Fraction_2 = 1000;
	}
	if(tecjet[TECJET_1].Option && tecjet[TECJET_2].Option)
	{
		MIX.Tecjet_Fraction_1 = (DS32)PARA[ParRefInd[TEC_MAX_FLOW__PARREFIND]].Value * 100 / ((MIX.Tecjet_Max_Flow_Rate + 5)/ 10);
		MIX.Tecjet_Fraction_2 = (DS32)PARA[ParRefInd[TEC_2_MAX_FLOW__PARREFIND]].Value * 100 / ((MIX.Tecjet_Max_Flow_Rate + 5)/ 10);
	}

	  if (MIX_OPTION_TECJET)
	  {
			/*
			air @273K:         1,29 kg/m³
			CH4 @273K:      16g/22,4l = 0,71 kg/m³
			lambda =            (air/17kg) / (CH4/1kg)
			lambda =             (air/(17kg/1,29kg*m³)) / (CH4/(1kg/0,71kg*m³))
			lambda =             (air/9,356m³) / (CH4/m³)

			CH4 =                   landfillgas*CH4_content
			lambda =             (air/9,356m³) / (landfillgas*CH4_content/m³)
			air/m³ =               9,356*lambda*CH4_content*landfillgas/m³

			gemisch/m³ =   landfillgas/m³ + air/m³
			gemisch/m³ =   landfillgas/m³ * (1 + 9,356*lambda*CH4_content)
			landfillgas/m³ =   gemisch/m³ / (1 + 9,356*lambda*CH4_content)

			temperature compensation:  V(T1)/V(T2) = (T1+273K)/(T2+273K)
			landfillgas/m³ =   gemisch/m³ * (Tgas+273K)/(Tgemisch+273K) / (1 + 9,356*lambda*CH4_content)

			Tgas = Tgemisch, Lambda = 1, CH4_content = 50%:
			landfillgas/m³ = 0,176*gemisch/m³

			*/

		  //       = speed[0.1rpm] / 10 / 60 * ( 3 * 10) * 3600 * pressure
		  MIX.Flow = (DF32)ENG.S200EngineSpeed * 180 * MIX.ReceiverPressureAvg / PAR_OFFSET_REC_PRESS_VALUE;

		  // 0...TEC_MAX_FLOW <=> 0...PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].MaxValue
		  // TEC_MAX_FLOW = 3600 m³/h = 1000 l/s = 3.600.000 raw

		  SetpointMixerPositionRelative =										// 0...10000 = 0...100%
				(DS32)MIX.SetpointMixerPosition[MixerInd1]	* 10000L		// Setpoint [steps]
				/ PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value;// parameter in steps

		  SpeedRelative = 													// 0...15000 = 0...150%
				(DS32)ENG.S200EngineSpeed * 10000							// S200 [0.1rpm]
				/ (TUR.NominalSpeed / 100);	// parameter in mrpm
#if (CLIENT_VERSION == IET)
		  SpeedSetRelative = TUR.GovernorAnalogOutputInternal
				/ (DS16)PARA[ParRefInd[NUMBER_OF_TEETH_AT_FLYWHEEL__PARREFIND]].Value * 150;

//		  if(tecjet[TECJET_1].Option)
//		  {
//			  FuelTemperatureAbsolute =											// [0.1K]
//					((DS16)tecjet[TECJET_1].read.FuelTemperature - 40)*10 + 2730; // FuelTemp 0...250 = -40...+210°C
//		  }
//		  else // must be tecjet[TECJET_2].Option
//		  {
//			  FuelTemperatureAbsolute =											// [0.1K]
//					((DS16)tecjet[TECJET_2].read.FuelTemperature - 40)*10 + 2730; // FuelTemp 0...250 = -40...+210°C
//		  }
//
//		  ReceiverTemperatureAbsolute =										// [0.1K]
//				MIX.ReceiverTemperature.Value + 2730;						// Rec Temp [0.1°C]
//
//		  DensityFactorRelative =											// 5000...15000 = 50...150%
//				(DS32)FuelTemperatureAbsolute * 10000
//				/ ReceiverTemperatureAbsolute;

		  ThrottlePositionRelative = 										// 0...10000 = 0...100%
				DK.ThrottlePositionPercent * 10;							//DK.Thr [0.1%]

		  ThrottlePositionSetpointRelative =										// 0...10000 = 0...100%
				(TUR.GovernorAnalogOutputInternal - 5000L) / 2L;			// DKsetpoint
#endif
		  //PressureCorrection = (MIX.ReceiverPressureFilteredValue + 1013) * 10 / (MIX.ReceiverTemperature.Value + 2732);
		  PressureCorrection = (MIX.ReceiverPressureAvgFilteredValue + 1013); // why? empiric result...
		  //PressureCorrection = MIX.ReceiverPressureFilteredValue; 	// rec press absolute

		  /*
			LoadDependentPart =													// 0...3600000 = 0...3600Nm³/h
				ELM.T1E.sec.Psum * 10 										// Psum[W]
				/(DS32)PARA[ParRefInd[TEC_HEAT_VALUE__PARREFIND]].Value; 	//Parameter [0.1kWh/Nm³]
		   */
		  LoadDependentPart =
				TUR.Reg.PowerSetPoint * 10
				/ (DS32)PARA[ParRefInd[TEC_HEAT_VALUE__PARREFIND]].Value; 	//Parameter [0.1kWh/Nm³]

		  if (LoadDependentPart < 0) LoadDependentPart = 0;					// limitation with reverse power
		  MIX.TecJetDiagnosticsLoadDependentPart = LoadDependentPart;			// save temp value for diagnostics


		  if (MIX.state[MixerInd1] == MIX_START_POSITION_REACHED)
		  {

			  ////////////////
			  ////////////////test MVO 22072015
			  // temp. removed:

			  /*
			  if (ENG.S200EngineSpeed < MIX_RELEASE_SPEED_FOR_LAMBDA_CONTROL)
			  {
				  tecjet[TECJET_1].write.FuelFlowRate = (DF32)MIX.SetpointMixerPosition[MixerInd1]
															* TEC_MAX_FLOW
															/ PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].MaxValue
															* ENG.S200EngineSpeed / (TUR.NominalSpeed / 100) * TEMP_COMPENSATION_1;
			  }
			  else
			  {
				  tecjet[TECJET_1].write.FuelFlowRate = MIX.Flow / ((DF32)PARA[ParRefInd[TEC_LAMBDA_SETPOINT_0kW__PARREFIND]].Value * CALCULATION_FACTOR + 1) * TEMP_COMPENSATION_1;
			  }
			  */

			  // MVO 22072015

			  // use actual throttle position for DKFactor
			  DKFactor = 10000L
#if (CLIENT_VERSION == IET)
					 + (DS32)(ThrottlePositionRelative - 5000)
#else
					 + (DS32)((TUR.IOM_GOV_Out-5000)/2 - 5000) // Always use throttle position setpoint
#endif
					 * (DS32)PARA[ParRefInd[MIX_DK_DEPENDENCY_TECJET__PARREFIND]].Value / 5000;

			  if (DKFactor < 0)
				  DKFactor = 0;

			  MIX.DKFactor = DKFactor;

			  tecjet[TECJET_1].write.FuelFlowRate = 								// [l/h] 0...3,600,000 = 0...3,600 m³/h
					(DS32)MIX.Tecjet_Max_Flow_Rate / 100	// [l/h], resolution not below 100l/h
					* MIX.Tecjet_Fraction_1 / 1000
					* SetpointMixerPositionRelative / 10000
					* SpeedRelative / 10000
					//* DensityFactorRelative / 10000
					* DKFactor/100;
					//* PressureCorrection / 1013;

			  tecjet[TECJET_2].write.FuelFlowRate = 								// [l/h] 0...3,600,000 = 0...3,600 m³/h
					(DS32)MIX.Tecjet_Max_Flow_Rate / 100	// [l/h], resolution not below 100l/h
					* MIX.Tecjet_Fraction_2 / 1000
					* SetpointMixerPositionRelative / 10000
					* SpeedRelative / 10000
					//* DensityFactorRelative / 10000
					* DKFactor/100;
					//* PressureCorrection / 1013;

		  }
		  else if (  (MIX.state[MixerInd1] == MIX_MOVING_TO_IDLE_POS)
				  || (MIX.state[MixerInd1] == MIX_IDLE_POSITION_REACHED))
		  {

			  // removed:
			  // tecjet[TECJET_1].write.FuelFlowRate = MIX.Flow / ((DF32)PARA[ParRefInd[TEC_LAMBDA_SETPOINT_0kW__PARREFIND]].Value * CALCULATION_FACTOR + 1) * TEMP_COMPENSATION_1;

			  // use actual throttle position for DKFactor
			  DKFactor = 10000L
#if (CLIENT_VERSION == IET)
					 + (DS32)(ThrottlePositionRelative - 5000)
#else
					 + (DS32)((TUR.IOM_GOV_Out-5000)/2 - 5000) // Always use throttle position setpoint
#endif
					 * (DS32)PARA[ParRefInd[MIX_DK_DEPENDENCY_TECJET__PARREFIND]].Value / 5000;

			  if (DKFactor < 0)
				  DKFactor = 0;

			  MIX.DKFactor = DKFactor;

			  tecjet[TECJET_1].write.FuelFlowRate = 								// [l/h] 0...3,600,000 = 0...3,600 m³/h
						(DS32)MIX.Tecjet_Max_Flow_Rate / 100	// [l/h], resolution not below 100l/h
						* MIX.Tecjet_Fraction_1 / 1000
					* SetpointMixerPositionRelative / 10000
#if (CLIENT_VERSION == IET)
					* SpeedSetRelative / 10000
#else
					* SpeedRelative / 10000
#endif
					//* DensityFactorRelative / 10000
					* DKFactor/100;
					//* PressureCorrection / 1013;

			  tecjet[TECJET_2].write.FuelFlowRate = 								// [l/h] 0...3,600,000 = 0...3,600 m³/h
						(DS32)MIX.Tecjet_Max_Flow_Rate / 100	// [l/h], resolution not below 100l/h
						* MIX.Tecjet_Fraction_2 / 1000
					* SetpointMixerPositionRelative / 10000
#if (CLIENT_VERSION == IET)
					* SpeedSetRelative / 10000
#else
					* SpeedRelative / 10000
#endif
					//* DensityFactorRelative / 10000
					* DKFactor/100;
					//* PressureCorrection / 1013;
		  }
		  else if (  (MIX.state[MixerInd1] == MIX_MOVING_TO_PARALLEL_POS)
				  || (MIX.state[MixerInd1] == MIX_PARALLEL_POSITION_REACHED)
				  || (MIX.state[MixerInd1] == MIX_MOVING_TO_ISLAND_POS)
				  || (MIX.state[MixerInd1] == MIX_ISLAND_POSITION_REACHED))
		  {
			 // temporarily removed 23.07.2015
			  /*
			  tecjet[TECJET_1].write.FuelFlowRate = (DF32)MIX.Flow / (Interpolate(ELM.T1E.sec.PsumRelative,
																				  0,
																				  10000,
																				  PARA[ParRefInd[TEC_LAMBDA_SETPOINT_0kW__PARREFIND]].Value,
																				  PARA[ParRefInd[TEC_LAMBDA_SETPOINT_FULL_LOAD__PARREFIND]].Value)
																				  * CALCULATION_FACTOR + 1) * TEMP_COMPENSATION_1;
				*/
			  // test MVO 22072015

			  // use throttle position setpoint for DKFactor
			  DKFactor = 10000L
#if (CLIENT_VERSION == IET)
					 + (DS32)(ThrottlePositionSetpointRelative - 5000)
#else
					 + (DS32)((TUR.IOM_GOV_Out-5000)/2 - 5000) // Always use throttle position setpoint
#endif
					 * (DS32)PARA[ParRefInd[MIX_DK_DEPENDENCY_TECJET__PARREFIND]].Value / 5000;

			  if (DKFactor < 0)
				  DKFactor = 0;

			  MIX.DKFactor = DKFactor;

			  if (MIX_SetpointInitDone[MixerInd1])
			  {
				  tecjet[TECJET_1].write.FuelFlowRate = 								// [l/h] 0...3,600,000 = 0...3,600 m³/h
									((DS32)MIX.Tecjet_Max_Flow_Rate / 100	// [l/h], resolution not below 100l/h
											* MIX.Tecjet_Fraction_1 / 1000
									* SetpointMixerPositionRelative / 10000
									//* SpeedSetRelative / 10000
									* DKFactor/100
									+
									LoadDependentPart)
									//* DensityFactorRelative / 10000
									//* EtaCorrection / 100
									* PressureCorrection / 1013;
			  }

			  if (MIX_SetpointInitDone[MixerInd2])
			  {
				  tecjet[TECJET_2].write.FuelFlowRate = 								// [l/h] 0...3,600,000 = 0...3,600 m³/h
									((DS32)MIX.Tecjet_Max_Flow_Rate / 100	// [l/h], resolution not below 100l/h
											* MIX.Tecjet_Fraction_2 / 1000
									* SetpointMixerPositionRelative / 10000
									//* SpeedSetRelative / 10000
									* DKFactor/100
									+
									LoadDependentPart)
									//* DensityFactorRelative / 10000
									//* EtaCorrection / 100
									* PressureCorrection / 1013;
			  }
		  }
		  else if (MIX.state[MixerInd1] == MIX_UNDER_CTRL)
		  {
			  // new approach 24.07.2015 MVO
			  // adjusted in MIX_control
			  //tecjet[TECJET_1].write.FuelFlowRate = 0;

			  // use throttle position setpoint for DKFactor
			  DKFactor = 10000L
#if (CLIENT_VERSION == IET)
					 + (DS32)(ThrottlePositionSetpointRelative - 5000)
#else
					 + (DS32)((TUR.IOM_GOV_Out-5000)/2 - 5000) // Always use throttle position setpoint
#endif
					 * (DS32)PARA[ParRefInd[MIX_DK_DEPENDENCY_TECJET__PARREFIND]].Value / 5000;

			  if (DKFactor < 0)
				  DKFactor = 0;

			  MIX.DKFactor = DKFactor;
		  }
		  else if (MIX.state[MixerInd1] == MIX_UNDER_TEST)
		  {
			  if (ENG.Running)
			  {
				  tecjet[TECJET_1].write.FuelFlowRate = MIX.Flow * MIX.Tecjet_Fraction_1 / 1000 / ((DF32)MIX.LambdaSetpointTecJet * CALCULATION_FACTOR + 1) * TEMP_COMPENSATION_1;
				  tecjet[TECJET_2].write.FuelFlowRate = MIX.Flow * MIX.Tecjet_Fraction_2 / 1000 / ((DF32)MIX.LambdaSetpointTecJet * CALCULATION_FACTOR + 1) * TEMP_COMPENSATION_2;
			  }
			  else
			  {
				  tecjet[TECJET_1].write.FuelFlowRate = 0;
				  tecjet[TECJET_2].write.FuelFlowRate = 0;
			  }
		  }
		  else
		  {
			  tecjet[TECJET_1].write.FuelFlowRate = 0;
			  tecjet[TECJET_2].write.FuelFlowRate = 0;
		  }

		  if (tecjet[TECJET_1].write.FuelFlowRate != 0)
			  MIX.CalculatedLambda = ((DF32)MIX.Flow * MIX.Tecjet_Fraction_1 / 1000  / tecjet[TECJET_1].write.FuelFlowRate * TEMP_COMPENSATION_1 - 1) / CALCULATION_FACTOR;
		  else if (tecjet[TECJET_2].write.FuelFlowRate != 0)
			  MIX.CalculatedLambda = ((DF32)MIX.Flow * MIX.Tecjet_Fraction_2 / 1000  / tecjet[TECJET_2].write.FuelFlowRate * TEMP_COMPENSATION_2 - 1) / CALCULATION_FACTOR;
	  }
}


//////////////////// public control loop MIX_control_100ms
/**
 * @void MIX_control_100ms(void)
 * 
 * Control of gas mixer system. Is called every 100ms.
 * 
 * 
 */


void MIX_control_100ms(void)
{	  
	DS16 TDiff;
	static DU32 CalibrationDoneCounter[2];
	static DU32 LimitStopLeanTimer[2];
	// rmiIET: filtering of the lambda voltage, because in measured values there were peeks detected
	static DS16 LV[MIX_NUMBER_OF_RECP_VALUES_FOR_FILTERING];  // used for filtering the lambda voltage
	DS32 LVsum;                                               // used for filtering the lambda voltage

	DU8 i;                                                   // loop counter
	DU8 MixerInd;

	DU16 FuncInd;
	DU16 StopCondInd1;
	DU16 StopCondInd2;

	// CUMMINS
	if (PARA[ParRefInd[CUMMINS_OPTION__PARREFIND]].Value AND (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT_2].Assigned == ASSIGNED))
	{
		MIX.GasMixerPercent_Cummins = (DS32)(MIX.AO_GasMixer[MixerInd1] - MIX.AnalogOutZero) * 10000 / MIX.MixerFullRange_Out;
	}

	MIX.Option_ENSMP = ((DO_FUNCT[IOA_DO_MIX_MOVE_DIR_LEAN].Assigned == ASSIGNED)
					 && (DO_FUNCT[IOA_DO_MIX_MOVE_DIR_RICH].Assigned == ASSIGNED)
					 && (AI_I_FUNCT[IOA_AI_I_MIX_GASMIXER_POS].Assigned == ASSIGNED));

	if (AI_R_U_FUNCT[RECEIVER_TEMP].Assigned == ASSIGNED)
	{
		MIX.ReceiverTemperature_Available = TRUE;
		MIX.ReceiverTemperature.Value = io_calculate_AI_R_U_value( RECEIVER_TEMP,
																	MIX.ReceiverTemperature.Raw,
																	PARA[ParRefInd[MIX_OPTION_RECEIV_TEMP_SENSOR__PARREFIND]].Value,
																	STOPCONDITION_70226,
																	STOPCONDITION_70227 );
	}
	else if ((AI_I_FUNCT[IOA_AI_I_MIX_RECEIVER_TEMP].Assigned == ASSIGNED) && PARA[ParRefInd[MIX_OPTION_RECEIV_TEMP_SENSOR__PARREFIND]].Value)
	{
		MIX.ReceiverTemperature_Available = TRUE;
		MIX.ReceiverTemperature.Value = io_calculate_AI_I_value( IOA_AI_I_MIX_RECEIVER_TEMP,
																	 MIX.AI_I_ReceiverTemp,				// value_raw
																	 -180,								// value_4mA
																	 1490,				 				// value_20mA
																	 STOPCONDITION_70226,				// SC_wirebreak
																	 STOPCONDITION_70227				// SC_overload
																	 );
	}
	else
	{
		STOP_Tripped[STOPCONDITION_70226] = FALSE;
		STOP_Tripped[STOPCONDITION_70227] = FALSE;
		MIX.ReceiverTemperature_Available = FALSE;
		MIX.ReceiverTemperature.Value = 0;
	}

	  // *******************************************
	  // Power reduction due to receiver temperature
	  // Calculate Difference to maxvalue (unit 0,1K)
	  if (HVS.stateT1E != HVS_T1E_IS_ON)
	  		TDiff = 0;
	  else if (MIX.ReceiverTemperature_Available)
	  		TDiff = MIX.ReceiverTemperature.Value - (DS16)PARA[ParRefInd[MIX_LOAD_RED_RECEIVER_TEMP_START_VALUE__PARREFIND]].Value;
	  else
	  		TDiff = 0;

	  if (TDiff < 0) TDiff = 0;
	  // if the temperature is above the maximum: calculate load reduction
	  if (TDiff > 0)
	  {
	  	 reduction  = (DS32)(TDiff) * (PARA[ParRefInd[MIX_LOAD_RED_RECEIVER_TEMP_RAMP__PARREFIND]].Value);

	  	 // limit MaxPower to zero
	  	 if ( reduction > PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value )
	  	   MIX.MaxPower_Temp = 0;
	  	 else
	  	   MIX.MaxPower_Temp =   PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value - reduction;
	  }
	  else
	  {
	  	 reduction = 0L;
	    MIX.MaxPower_Temp = PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value; // set to nominal power, no load reduction
	  }

  // not if TecJet
  MIX.Manual = MIX.Config || (!(MIX_OPTION_TECJET) && MIX.AdjustmentDuringStart_Activated);
  
  // only for mixer one if analog feedback is assigned
  MIX_PositionDeviation();

  // maximum control deviation
  MIX_ControlDeviation();

  if (MIX_OPTION_TECJET)
  {
	  MIX.MixerFullRange_In = (DS32)PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value;

	  MIX.MixerFullRange_Out = MIX.MixerFullRange_In;

      if (tecjet[TECJET_1].Option)
      {
          MIX.ActualPositionOfGasMixer[MixerInd1] = (DF32)tecjet[TECJET_1].read.ActualFuelValvePosition/250*PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value;
      }
      else if (tecjet[TECJET_2].Option)
      {
          MIX.ActualPositionOfGasMixer[MixerInd1] = (DF32)tecjet[TECJET_2].read.ActualFuelValvePosition/250*PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value;
      }
  }
  else
  // gas mixer with analogue setpoint control
  if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == ASSIGNED)
  {
	  DS32 Value;

	  if (PARA[ParRefInd[CUMMINS_OPTION__PARREFIND]].Value) // For CUMMINS
	  {
		  if (!ENG.Running)
		  {
			  if (MIX.state[MixerInd1] == MIX_UNDER_TEST)
				  Value = MIX.SetpointMixerPosition[MixerInd1];
			  else
				  Value = Mix_Calculate_Setpoint_For_Mixer_Position(MIX_START_POS_A__PARREFIND, MIX_START_POS_B__PARREFIND);
		  }
		  else
		  {
			  DS32 x1, x2, y1, y2;
			  DS32 Offset;
	   static DS32 OffsetBegin;

			  switch (MIX.state[MixerInd1])
			  {
			  // Starting
			  case MIX_MOVING_TO_START_POS:
			  case MIX_START_POSITION_REACHED:
			  case MIX_MOVING_TO_IDLE_POS:
			  case MIX_IDLE_POSITION_REACHED:
				  Offset = 0;
				  OffsetBegin = MIX.AO_GasMixer[MixerInd1] - MIX.AnalogOutZero;
				  x1 = PARA[ParRefInd[CUMMINS_THROTTLE_LOW_IDLE__PARREFIND]].Value;
				  x2 = PARA[ParRefInd[CUMMINS_THROTTLE_NOMINAL__PARREFIND]].Value;
				  y1 = Mix_Calculate_Setpoint_For_Mixer_Position(MIX_START_POS_A__PARREFIND, MIX_START_POS_B__PARREFIND);
				  y2 = Mix_Calculate_Setpoint_For_Mixer_Position(MIX_IDLE_POS_A__PARREFIND,  MIX_IDLE_POS_B__PARREFIND);
				  break;
			  // On load
			  case MIX_MOVING_TO_PARALLEL_POS:
			  case MIX_PARALLEL_POSITION_REACHED:
			  case MIX_MOVING_TO_ISLAND_POS:
			  case MIX_ISLAND_POSITION_REACHED:
				  Offset = 0;
				  OffsetBegin = MIX.AO_GasMixer[MixerInd1] - MIX.AnalogOutZero;
				  x1 = PARA[ParRefInd[CUMMINS_THROTTLE_NOMINAL__PARREFIND]].Value;
				  x2 = PARA[ParRefInd[CUMMINS_THROTTLE_LOAD__PARREFIND]].Value;
				  y1 = Mix_Calculate_Setpoint_For_Mixer_Position(MIX_IDLE_POS_A__PARREFIND,  MIX_IDLE_POS_B__PARREFIND);
				  y2 = Mix_Calculate_Setpoint_For_Mixer_Position(MIX_PARALLEL_POS_A__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
				  break;
			  // Under control
			  case MIX_UNDER_CTRL:
			  // Under configuration
			  case MIX_UNDER_TEST:
				  Offset = MIX.SetpointMixerPosition[MixerInd1] - OffsetBegin;
				  x1 = PARA[ParRefInd[CUMMINS_THROTTLE_NOMINAL__PARREFIND]].Value;
				  x2 = PARA[ParRefInd[CUMMINS_THROTTLE_LOAD__PARREFIND]].Value;
				  y1 = Mix_Calculate_Setpoint_For_Mixer_Position(MIX_IDLE_POS_A__PARREFIND,  MIX_IDLE_POS_B__PARREFIND);
				  y2 = Mix_Calculate_Setpoint_For_Mixer_Position(MIX_PARALLEL_POS_A__PARREFIND, MIX_PARALLEL_POS_B__PARREFIND);
				  break;
			  default:
				  Offset = 0;
				  OffsetBegin = MIX.AO_GasMixer[MixerInd1] - MIX.AnalogOutZero;
				  x1 = 0;
				  x2 = 0;
				  y1 = MIX.SetpointMixerPosition[MixerInd1];
				  y2 = MIX.SetpointMixerPosition[MixerInd1];
				  break;
			  }

			  Value = Interpolate((TUR.IOM_GOV_Out-5000)/20, x1, x2, y1, y2) + Offset;
		  }

		  // Limitation
		  if (Value < 0)
			  Value = 0;
		  else if (Value > MIX.MixerFullRange_Out)
			  Value = MIX.MixerFullRange_Out;
	  }
	  else
		  Value = MIX.SetpointMixerPosition[MixerInd1];

	  MIX.AO_GasMixer[MixerInd1] = Value + MIX.AnalogOutZero;

	  // Second analog output for mixer bank B of V-Engine - Same signal as for mixer 1 even in TEST-Mode
	  if (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT_2].Assigned == ASSIGNED)
		  MIX.AO_GasMixer[MixerInd2] = MIX.AO_GasMixer[MixerInd1];

	  MIX.MixerFullRange_In = (DS32)PARA[ParRefInd[MIX_FEEDBACK_AT_100_PERCENT__PARREFIND]].Value - (DS32)PARA[ParRefInd[MIX_FEEDBACK_AT_0_PERCENT__PARREFIND]].Value;

	  MIX.MixerFullRange_Out = MIX_ANALOG_OUT_FULL - MIX.AnalogOutZero;

	  // feedback from AI
	  if (AI_I_FUNCT[IOA_AI_I_MIX_GASMIXER_POS].Assigned == ASSIGNED)
	  {
		  DS16 UnfilteredValue;
		  static DS16 Pos[10];
		  DS32 PosSum;

		  UnfilteredValue = (DS32)MIX.AI_I_GasMixerPos - (DS32)PARA[ParRefInd[MIX_FEEDBACK_AT_0_PERCENT__PARREFIND]].Value;

		  // calculate filtered value
		  PosSum = 0L;
		  for (i = (10 - 1); i>0; i--)
		  {
			  PosSum += Pos[i];
			  Pos[i] = Pos[i-1];
		  }
		  Pos[0] = UnfilteredValue;
		  PosSum += UnfilteredValue;
		  MIX.ActualPositionOfGasMixer[MixerInd1] = PosSum / 10;
	  }
	  // feedback from setpoint
	  else
	  {
		  MIX.ActualPositionOfGasMixer[MixerInd1] = MIX_RunningTimeRamp(MIX.ActualPositionOfGasMixer[MixerInd1], MIX.SetpointMixerPosition[MixerInd1], MIX.MixerFullRange_In);
	  }
  }
  else
  // gas mixer with stepper-motor-control
  // -> feedback from IOM
  {
	  MIX.MixerFullRange_In = (DS32)PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value;

	  MIX.MixerFullRange_Out = MIX.MixerFullRange_In;

	  MIX.ActualPositionOfGasMixer[MixerInd1] = MIX.StepperMotor[MixerInd1].actualPosition;
  }

  if (PARA[ParRefInd[MIX_SECOND_MIXER__PARREFIND]].Value)
      MIX.ActualPositionOfGasMixer[MixerInd2] = MIX.StepperMotor[MixerInd2].actualPosition;

  // gas mixer with analogue setpoint control for air
  if (AO_FUNCT[IOA_AO_MIX_AIRMIXER_SETPOINT].Assigned == ASSIGNED)
  {
	  DS32 Value;

	  Value = (DS16)Interpolate(
			  ELM.T1E.sec.PsumRelative,
			  0L,
			  PARA[ParRefInd[MIX_AIR_LOAD_AT_MAX_POS__PARREFIND]].Value,
			  PARA[ParRefInd[MIX_AIR_MIN_POS__PARREFIND]].Value + MIX.AnalogOutZero,
			  PARA[ParRefInd[MIX_AIR_MAX_POS__PARREFIND]].Value + MIX.AnalogOutZero);

	  // Limitation
	  if (Value < (PARA[ParRefInd[MIX_AIR_MIN_POS__PARREFIND]].Value + MIX.AnalogOutZero))
		  MIX.AO_AirMixer = (PARA[ParRefInd[MIX_AIR_MIN_POS__PARREFIND]].Value + MIX.AnalogOutZero);
	  else
	  if (Value > (PARA[ParRefInd[MIX_AIR_MAX_POS__PARREFIND]].Value + MIX.AnalogOutZero))
		  MIX.AO_AirMixer = (PARA[ParRefInd[MIX_AIR_MAX_POS__PARREFIND]].Value + MIX.AnalogOutZero);
	  else
		  MIX.AO_AirMixer = Value;

  }

	// if calibration had been done in the last 15 min, do not again
	if ( (MIX.CalibrationDone[MixerInd1])
		&& (!STOP_is_Set(STOPCONDITION_70221)) //timeout lean position reached
		&& (!STOP_is_Set(STOPCONDITION_70232)) //timeout start position reached
		&& (!STOP_is_Set(STOPCONDITION_70233)) //timeout idle position reached
		&& (!STOP_is_Set(STOPCONDITION_70234)) //timeout parallel position reached
		&& (!STOP_is_Set(STOPCONDITION_70235)) //timeout island position reached
		&& (!STOP_is_Set(STOPCONDITION_70236)) //timeout rich position reached
		&& (!STOP_is_Set(STOPCONDITION_70224)) //steps missing
		&& ( CalibrationDoneCounter[MixerInd1] < (DU32)PARA[ParRefInd[MIX_TIMER_NEXT_CALIBRATION_NECESSARY__PARREFIND]].Value ) )
	{
		CalibrationDoneCounter[MixerInd1] += 100L;
	}
	else
	{
		CalibrationDoneCounter[MixerInd1] = 0L;
		MIX.CalibrationDone[MixerInd1] = FALSE;
	}
    

    // if calibration had been done in the last 15 min, do not again
    if ( (MIX.CalibrationDone[MixerInd2])
    	&& (!STOP_is_Set(STOPCONDITION_70706)) //timeout lean position reached
    	&& (!STOP_is_Set(STOPCONDITION_70709)) //timeout start position reached
    	&& (!STOP_is_Set(STOPCONDITION_70710)) //timeout idle position reached
    	&& (!STOP_is_Set(STOPCONDITION_70711)) //timeout parallel position reached
    	&& (!STOP_is_Set(STOPCONDITION_70712)) //timeout island position reached
    	&& (!STOP_is_Set(STOPCONDITION_70713)) //timeout rich position reached
    	&& (!STOP_is_Set(STOPCONDITION_70715)) //steps missing
    	&& ( CalibrationDoneCounter[MixerInd2] < (DU32)PARA[ParRefInd[MIX_TIMER_NEXT_CALIBRATION_NECESSARY__PARREFIND]].Value ) )
    {
    	CalibrationDoneCounter[MixerInd2] += 100L;
    }
    else
    {
    	CalibrationDoneCounter[MixerInd2] = 0L;
    	MIX.CalibrationDone[MixerInd2] = FALSE;
    }
    
    // wire break for lambda voltage
    if (AI_I_FUNCT[LAMBDA_VOLTAGE].Assigned == ASSIGNED)                 // input assigned
    {
        if ( MIX.AI_I_LambdaVoltage > 3500 )
        {
    	    // calculate lambdavoltage
    	    MIX.LambdaVoltage = MIX.AI_I_LambdaVoltage;
    	    // make stop condition wire break acknowledgeable
    	    STOP_Tripped[STOPCONDITION_70225] = FALSE;
        }
        else
        {
    	    // wire break    
    	    STOP_Set(STOPCONDITION_70225);              // set stop condition for wire break
    	    STOP_Tripped[STOPCONDITION_70225] = TRUE;   // set stop condition to tripped
        }
    }
    else
    {
        // not assigned or option not selected
        STOP_Tripped[STOPCONDITION_70225] = FALSE;
        MIX.LambdaVoltage = 0;
    }
      
      
    // calculate filtered value MIX.LambdaVoltageFilteredValue from MIX.LambdaVoltage, rmiIET
    if (MIX.LambdaVoltage != -32768)
    {
  	   LVsum = 0L;
  	   for (i = (MIX_NUMBER_OF_LV_VALUES_FOR_FILTERING - 1); i>0; i--)
  	   {
  		  LVsum += LV[i];
  		  LV[i] = LV[i-1];
  		
  	   }
  	   LV[0] = MIX.LambdaVoltage; // new value
  	   LVsum += MIX.LambdaVoltage;
  	
  	   MIX.LambdaVoltageFilteredValue = LVsum / MIX_NUMBER_OF_LV_VALUES_FOR_FILTERING;
    }
    
    // stop condition for mixture temperature
    if (HVS.stateT1E != HVS_T1E_IS_ON)
    {
    	STOP_Tripped[STOPCONDITION_70229] = FALSE;
    	STOP_Tripped[STOPCONDITION_70230] = FALSE;
    }
    else if ( (MIX.ReceiverTemperature_Available)
          && (PARA[ParRefInd[MIX_OPTION_RECEIV_TEMP_SENSOR__PARREFIND]].Value) )
    {
	    //over temperature mixture
	    if (MIX.ReceiverTemperature.Value >= (DS16)PARA[ParRefInd[MIX_MAX_TEMP_MIXTURE__PARREFIND]].Value)
	    {
	    	STOP_Set(STOPCONDITION_70229);
	    	STOP_Tripped[STOPCONDITION_70229] = TRUE;
	    }
	    else if (MIX.ReceiverTemperature.Value < ((DS16)PARA[ParRefInd[MIX_MAX_TEMP_MIXTURE__PARREFIND]].Value - MIX_RECOVER_TEMP_HYST))
	    {
	    	STOP_Tripped[STOPCONDITION_70229] = FALSE;
	    }
	    //under temperature mixture
		if (ELM.T1E.sec.PsumRelative >= (DS16)PARA[ParRefInd[MIX_RELEASE_POWER_CONTROL__PARREFIND]].Value)
		{    
		    if (MIX.ReceiverTemperature.Value <= (DS16)PARA[ParRefInd[MIX_MIN_TEMP_MIXTURE__PARREFIND]].Value)
		    {
		    	STOP_Set(STOPCONDITION_70230);
		    	STOP_Tripped[STOPCONDITION_70230] = TRUE;
		    }
		    else if (MIX.ReceiverTemperature.Value > ((DS16)PARA[ParRefInd[MIX_MIN_TEMP_MIXTURE__PARREFIND]].Value + MIX_RECOVER_TEMP_HYST))
		    {
		    	STOP_Tripped[STOPCONDITION_70230] = FALSE;
		    }
		}
	    else
	    {
	    	STOP_Tripped[STOPCONDITION_70230] = FALSE;
	    }
    }
    else
    {
    	STOP_Tripped[STOPCONDITION_70229] = FALSE;
    	STOP_Tripped[STOPCONDITION_70230] = FALSE;
    }


	for (MixerInd = MixerInd1; MixerInd <= MixerInd2; MixerInd++)
	{

		if (MixerInd == MixerInd1)
		{
			FuncInd = MIXER_LIMIT_LEAN;
			StopCondInd1 = STOPCONDITION_70220;
			StopCondInd2 = STOPCONDITION_70244;
		}
		else//MixerInd2
		{
		    FuncInd = MIXER_B_LIMIT_LEAN;
			StopCondInd1 = STOPCONDITION_70705;
			StopCondInd2 = STOPCONDITION_70714;
		}

		if ( (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == NOT_ASSIGNED)
				&& (!tecjet[TECJET_1].Option) && (!tecjet[TECJET_2].Option) )
		{
			// supervision of Limit stop lean
			if (DI_FUNCT[FuncInd].Assigned == ASSIGNED)
			{
				if (!MIX.DI_LimitStopLean[MixerInd]) // limit stop lean is still present
				{
					if (   (MIX.state[MixerInd] == MIX_MOVING_RICH)
						|| (MIX.state[MixerInd] == MIX_MOVING_TO_START_POS)
						|| (MIX.state[MixerInd] == MIX_MOVING_TO_IDLE_POS)
						|| (MIX.state[MixerInd] == MIX_MOVING_TO_PARALLEL_POS)
						|| (MIX.state[MixerInd] == MIX_MOVING_TO_ISLAND_POS) )
					{
						LimitStopLeanTimer[MixerInd] += 100L;
						//limitstop lean does not disappear
						if (LimitStopLeanTimer[MixerInd] > MIX_TIMEOUT_LEAVING_LEAN_POS)
						{
							STOP_Set(StopCondInd1);
							STOP_Tripped[StopCondInd1] = TRUE;
						}
					}
					else
					{
						STOP_Tripped[StopCondInd1] = FALSE;
						LimitStopLeanTimer[MixerInd] = 0L;
					}

				}
				else
				{
					STOP_Tripped[StopCondInd1] = FALSE;
					LimitStopLeanTimer[MixerInd] = 0L;
				}
			}
			else
			{
				STOP_Tripped[StopCondInd1] = FALSE;
				LimitStopLeanTimer[MixerInd] = 0L;
			}
		}
		else
		{
			STOP_Tripped[StopCondInd1] = FALSE;
			LimitStopLeanTimer[MixerInd] = 0L;
		}

		if ( (AO_FUNCT[IOA_AO_MIX_GASMIXER_SETPOINT].Assigned == NOT_ASSIGNED)
				&& (!tecjet[TECJET_1].Option) && (!tecjet[TECJET_2].Option) )
		{
			// stop condition both limit stop at the same time STOPCONDITION_70244
			// 11.09.2008 MVO
			if (DI_FUNCT[FuncInd].Assigned == ASSIGNED)
			// function "limit stop lean" is assigned to an input
			{
				if (!MIX.DI_LimitStopLean[MixerInd])
				// limit stop lean is reached
				{
					if ( ( (DI_FUNCT[MIXER_LIMIT_RICH].Assigned == ASSIGNED) && (MIX.DI_LimitStopRich) )
					// limit stop rich assigned and reached
						|| (MIX.ActualPositionOfGasMixer[MixerInd] >= (DS16)MIX.MixerFullRange_In) )
						// or max position reached (this is also interpreted as "limit stop rich reached")
					{
						STOP_Set(StopCondInd2);
						STOP_Tripped[StopCondInd2] = TRUE; // cannot be acknowledged
					}
					else
						// not in position rich
						STOP_Tripped[StopCondInd2] = FALSE;
				}
				else
				// "lean" not reached -> stopcondition can be acknowledged
				STOP_Tripped[StopCondInd2] = FALSE;
			}
			else
				// "lean" not assigned -> stopcondition can be acknowledged
				STOP_Tripped[StopCondInd2] = FALSE;
		}
		else
			// "lean" not assigned -> stopcondition can be acknowledged
			STOP_Tripped[StopCondInd2] = FALSE;

		// activate step counter on IOM
        if (MIX.ActualPositionOfGasMixer[MixerInd] == 0)
		  	MIX.StepperMotor[MixerInd].reset = FALSE;

        MIX.StepperMotor[MixerInd].setpointPosition = MIX.SetpointMixerPosition[MixerInd];

        // calculate relative mixer position [0.01%]
        if (MIX.MixerFullRange_In != 0)
        {
              MIX.ActualPositionOfGasMixerPercent[MixerInd] = (DS32) MIX.ActualPositionOfGasMixer[MixerInd] * 10000
                                        / MIX.MixerFullRange_In;
        }

        // calculate relative mixer position setpoint [0.01%]
        if (MIX.MixerFullRange_Out != 0)
        {
              MIX.SetpointMixerPositionPercent[MixerInd] = (DS32) MIX.SetpointMixerPosition[MixerInd] * 10000
                                        / MIX.MixerFullRange_Out;
        }

		// stop condition for receiver pressure

		// stop condition for lambda voltage


		if (myStateCnt[MixerInd] < ( MAX_DU32 - 1000 )) myStateCnt[MixerInd] = myStateCnt[MixerInd] + 100;
		if (myState[MixerInd] != 0) myState[MixerInd](SIG_DO, MixerInd);
	}

	// Set DO's for EN-SM-P
	if (MIX.state[MixerInd1] == MIX_UNDER_CTRL)
		MIX.DO_MoveFast = (abs(MIX.Setpoint_Receiver_Pressure - MIX.ReceiverPressureAvgFilteredValue) > 300); // > 300mbar
	else
		MIX.DO_MoveFast = (abs(MIX.SetpointMixerPositionPercent[MixerInd1] - MIX.ActualPositionOfGasMixerPercent[MixerInd1]) > 300) // > 3%
				       || ( MIX.state[MixerInd1] == MIX_SEARCHING_LEAN)
				       || ((MIX.state[MixerInd1] == MIX_LIMIT_LEAN_REACHED) && (MIX.mode[MixerInd1] != MIX_MOVE_LEAN))
				       || ( MIX.state[MixerInd1] == MIX_MOVING_RICH)
				       || ((MIX.state[MixerInd1] == MIX_POS_RICH_REACHED) && (MIX.mode[MixerInd1] != MIX_MOVE_RICH))
				       || ( MIX.state[MixerInd1] == MIX_MOVING_TO_START_POS)
				       || ( MIX.state[MixerInd1] == MIX_MOVING_TO_IDLE_POS)
				       || ( MIX.state[MixerInd1] == MIX_MOVING_TO_PARALLEL_POS)
				       || ( MIX.state[MixerInd1] == MIX_MOVING_TO_ISLAND_POS)
				       || ((MIX.state[MixerInd1] == MIX_UNDER_TEST) && !MIX.Config);

	if      ( MIX.state[MixerInd1] <= MIX_SYSTEM_OFF)                MIX_Regulation_DO(MIX_REG_DO_RESET);
	else if ((MIX.state[MixerInd1] == MIX_UNDER_TEST) && MIX.Config) MIX_Regulation_DO(MIX_REG_DO_REG_MANUAL);
	else if ( MIX.state[MixerInd1] == MIX_UNDER_CTRL)                MIX_Regulation_DO(MIX_REG_DO_REG_CTR);
	else                                                             MIX_Regulation_DO(MIX_REG_DO_REG_POS);

	if (!MIX.DO_LimitStopLean) MIX.DO_LimitStopLean = (MIX.ActualPositionOfGasMixerPercent[MixerInd1] <= 0);
	else                       MIX.DO_LimitStopLean = (MIX.ActualPositionOfGasMixerPercent[MixerInd1] <= 100);

	if (!MIX.DO_LimitStopRich) MIX.DO_LimitStopRich = (MIX.ActualPositionOfGasMixerPercent[MixerInd1] >= 10000);
	else                       MIX.DO_LimitStopRich = (MIX.ActualPositionOfGasMixerPercent[MixerInd1] >= 9900);
}



//////////////////// public controlloop MIX_control_1000ms
/**
 * @void MIX_control_1000ms(void)
 * 
 * Control of gas mixer system. Is called every 1000ms.
 * 
 * 
 */

void MIX_control_1000ms(void)
{
	DS16 TemperatureValue;

    // 1000ms loop for calculating average values of p, T, P 
    // and determining if these values are within the stability criteria.
    
    DS32   Subtotal;             // used for summing up the contents of the ring buffers
    DS32   Difference;           // temporary value difference
    DU8    i;                    // temporary loop counter
    DBOOL  PowerIsStable, PressureIsStable, TemperatureIsStable; // temporary stability markers

    // ******************************************************************************************
    // Move common ring buffer pointer to next position
    if (MIX_RingBufferPointer < MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING -1)
        MIX_RingBufferPointer++;
    else
        MIX_RingBufferPointer = 0;

    
    // ******************************************************************************************
    // feed power buffer with new value
    //rmiKW MIX_RingBuffer[MIX_RingBufferPointer].PsumRel = ELM.T1E.sec.PsumRelative;
    MIX_RingBuffer[MIX_RingBufferPointer].Psum = ELM.T1E.sec.Psum;
    
    // sum up the power values
    Subtotal = 0;
    for ( i = 0; i < MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING; i++ )
        //rmiKW Subtotal += MIX_RingBuffer[i].PsumRel;
        Subtotal += MIX_RingBuffer[i].Psum;
        
    // calculate average of power for the last MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING seconds
    //rmiKW MIX.ActualAverage.PsumRel = Subtotal / MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING;
    MIX.ActualAverage.Psum = Subtotal / MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING;


    // ******************************************************************************************
    // feed pressure buffer with new (filtered) values
    MIX_RingBuffer[MIX_RingBufferPointer].p = MIX.ReceiverPressureAvgFilteredValue;
    
    // sum up the pressure values
    Subtotal = 0;
    for ( i = 0; i < MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING; i++ )
        Subtotal += MIX_RingBuffer[i].p;   
   
    // calculate average of pressure for the last MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING seconds
    MIX.ActualAverage.p       = Subtotal / MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING;


    // ******************************************************************************************
    // feed temperature buffer with new values
    // p/T
    if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 2L)
    {
    	TemperatureValue = MIX.ReceiverTemperature.Value;
    }
    // combustion chamber temperature
    else if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 3L)
    {
#if (OPTION_CYLINDER_MONITORING == TRUE)
    	TemperatureValue = CYL.TxxxAverageAFilteredValue;
#else
    	TemperatureValue = 0;
#endif
    }
    else TemperatureValue = 0;

    MIX_RingBuffer[MIX_RingBufferPointer].theta = TemperatureValue;
    
    // sum up the temperature values
    Subtotal = 0;
    for ( i = 0; i < MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING; i++ )
        Subtotal += MIX_RingBuffer[i].theta;   
   
    // calculate average of temperature for the last MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING seconds
    MIX.ActualAverage.theta   = Subtotal / MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING;


    // ******************************************************************************************
    // Determine if values for p, T, and P are stable enough to save a mixer setpoint
    PowerIsStable       = TRUE;
    PressureIsStable    = TRUE;
    TemperatureIsStable = TRUE;
    for ( i = 0; i < MIX_SIZE_OF_RINGBUFFER_FOR_AVERAGING; i++ )
    {
    	// compare new average values to all points in the buffer
    	//rmiKW Difference = MIX_RingBuffer[i].PsumRel - MIX.ActualAverage.PsumRel;
    	Difference = MIX_RingBuffer[i].Psum - MIX.ActualAverage.Psum;
    	// abs(Difference)
    	if ( Difference < 0 )
    	    Difference = -Difference;
    	// check if stability is still valid
    	PowerIsStable = PowerIsStable && (Difference <= MIX_POWER_TOLERANCE);
    	
    	// compare new average values to all points in the buffer
    	Difference = MIX_RingBuffer[i].p - MIX.ActualAverage.p;
    	// abs(Difference)
    	if ( Difference < 0 )
    	    Difference = -Difference;
    	// check if stability is still valid
    	PressureIsStable = PressureIsStable && (Difference <= MIX_PRESSURE_TOLERANCE);
    	
    	// compare new average values to all points in the buffer
    	Difference = MIX_RingBuffer[i].theta - MIX.ActualAverage.theta;
    	// abs(Difference)
    	if ( Difference < 0 )
    	    Difference = -Difference;
    	// check if stability is still valid
    	TemperatureIsStable = TemperatureIsStable && (Difference <= MIX_TEMPERATURE_TOLERANCE);
    }
    
	// indicate stability globally

    // p/T
    if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 2L)
    {
		MIX.ValuesAreStable = (PowerIsStable && PressureIsStable && TemperatureIsStable);
    }
    // combustion chamber temperature
    else if (PARA[ParRefInd[MIX_OPTION_LAMBDA_CONTROL__PARREFIND]].Value == 3L)
    {
		MIX.ValuesAreStable = (PowerIsStable && TemperatureIsStable);
    }
    else MIX.ValuesAreStable = TRUE;

    // evaluate conditions to run active mixer control
    Mixer_Control_Release();
    //limit power due to lambda sensor signal
    if (MIX.MixerControlRelease.State == TRIP)
    {
    	// control active -> no power limitation
    	MIX.MaxPower = PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value; // full power
    }
    else
    {
    	MIX.MaxPower = (DS32)PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value / 100L *
    			((DS32)PARA[ParRefInd[MIX_RELEASE_POWER_CONTROL__PARREFIND]].Value // [0.01%]
			+ MIX_POWER_OFFSET_PERCENT_IF_BAD_SIGNAL) / 100L;
    }
    // timeout supervision for release of active mixer control
    Mixer_Control_Release_Timeout();

    // check max flow rate for tecjet(s)
    SetMaxFlowRateTecJet(); // would only be necessary if one of the tecjet options are changed or if one of the max flow parameters have been touched
}

void Mix_Calculate_Constant_pTDeviationControl(void)
{
	// ((3000 - tmin) * (dpmax3s - dpmin))
	MIX.Constant_pTDeviationControl =
			( (3000 - PARA[ParRefInd[MIX_DEV_MIN_TIME__PARREFIND]].Value)
			* (PARA[ParRefInd[MIX_DEV_MAX_DP_3SEC__PARREFIND]].Value - PARA[ParRefInd[MIX_MAX_DEVIATION__PARREFIND]].Value));
}

void MIX_Set_AnalogOutZero(void)
{
	if (PARA[ParRefInd[MIX_ANALOG_CONTROL_SIGNAL__PARREFIND]].Value) // 4...20mA
		MIX.AnalogOutZero = MIX_ANALOG_OUT_ZERO;
	else // 0...20mA
		MIX.AnalogOutZero = 0;
}


void MIX_init(void)
{
	// Initialization of the MIX struct
	MIX_Set_AnalogOutZero();

	//(PAR.init happens before MIX_init, so MIX_init can refer to parameters)7
	MIX.MixerFullRange_In = (DS32)PARA[ParRefInd[MIX_MAX_NUMBER_OF_STEPS__PARREFIND]].Value;
	MIX.MixerFullRange_Out = MIX.MixerFullRange_In;

	MIX.TemperatureOffset               = 0;
	//MIX.AdditionalTemperatureOffsetTecJet = 0;
	MIX.MaxPower						= PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value;

	SetMaxFlowRateTecJet();

	/*
	 * old
	calculation of Ramp time for mixer (moving from start to idle)
	identical to speed ramp time
	if (PARA[ParRefInd[SPEED_RAMP_UP__PARREFIND]].Value/1000L > 0L)
		MIX.TecJetRampTimeStartToIdle = (TUR.NominalSpeed - PARA[ParRefInd[STRT_VALUE_SPEED_RAMP__PARREFIND]].Value*1000L) / (PARA[ParRefInd[SPEED_RAMP_UP__PARREFIND]].Value/1000L);
	else
		// speed ramp less than 1 rpm/s
		MIX.TecJetRampTimeStartToIdle = MIX_TEC_MAX_RAMP_TIME_FROM_STRT_TO_IDLE;
	*/

	MIX.TecJetRampTimeStartToIdle = PARA[ParRefInd[TEC_RAMP_TIME_START_TO_IDLE__PARREFIND]].Value;

	// minimum time for flow ramp to TecJet
	if (MIX.TecJetRampTimeStartToIdle < MIX_TEC_MIN_RAMP_TIME_FROM_STRT_TO_IDLE)
		MIX.TecJetRampTimeStartToIdle = MIX_TEC_MIN_RAMP_TIME_FROM_STRT_TO_IDLE;

	MIX.CalibrationDone[MixerInd1]      = FALSE;
	MIX.CalibrationDone[MixerInd2]      = FALSE;
	
	MIX.ResetStepCounter[MixerInd1]                = FALSE;
	MIX.ResetStepCounter[MixerInd2]                = FALSE;

	// read NUMBER_OF_MIXER_SETPOINTS setpoint triples (P,p,T) into array
	Mix_Read_MixerSetpointsFromParameters();
	
	// sort setpoints within the array by Power
	Mix_Sort_MixerSetpoints();

	// initialize DU8 Ring buffer for value triples of p,t, and P
	MIX_RingBufferPointer = 0;
	
	// initialize first calibration bit
	MixFirstCalibration[MixerInd1] = TRUE;
	MixFirstCalibration[MixerInd2] = TRUE;
	
	// set mix to auto
	MIX.Config = FALSE;
	MIX.Fast   = FALSE;

	MIX.Manual = FALSE;
	MIX.AdjustmentDuringStart_Possible = FALSE;
	MIX.AdjustmentDuringStart_Activated = FALSE;
	 
	// Timer to calculate new mixer position
	TimerCalculateNewMixerPosition[MixerInd1] = 0L;
	TimerCalculateNewMixerPosition[MixerInd2] = 0L;

	// control deviation
	MIX.ControlDeviation.State = COLD;
	Mix_Calculate_Constant_pTDeviationControl();

	MIX.state[MixerInd1]                = MIX_BOOT;
	MIX.state[MixerInd2]                = MIX_BOOT;

	myState[MixerInd1] = 0;
	myState[MixerInd2] = 0;

	MIX.mode[MixerInd1]                 = MIX_BLOCK;
	MIX.mode[MixerInd2]                 = MIX_BLOCK;

	Transit(MIX_SystemOff, MixerInd1);
	Transit(MIX_SystemOff, MixerInd2);

}

void MIX_control_10ms()
{
    MIX_StepperMotorControl(0);
    MIX_StepperMotorControl(1);
}

#define MIX_STEPPER_MOTOR_POSITION_MIN -32768L
#define MIX_STEPPER_MOTOR_POSITION_MAX  32767L

static void MIX_StepperMotorControl(DU8 MixerIndex)
{
    static DBOOL oldDirectionLean[2];
    static DS32 stepDeviation[2];

    struct stepperMotor *StepperMotor;
    StepperMotor = &MIX.StepperMotor[MixerIndex];

    if (StepperMotor->DO_MixerClock)
    {
        if (!StepperMotor->disabled)
        {
            if (StepperMotor->DO_MixerDirectionLean && StepperMotor->actualPosition > MIX_STEPPER_MOTOR_POSITION_MIN)
            {
                StepperMotor->actualPosition--;
            }
            else if (!StepperMotor->DO_MixerDirectionLean && StepperMotor->actualPosition < MIX_STEPPER_MOTOR_POSITION_MAX)
            {
                StepperMotor->actualPosition++;
            }
        }

        StepperMotor->DO_MixerClock = 0;
    }
    else // if (!(MIX.DO_MixerClock[MixerIndex]))
    {
        oldDirectionLean[MixerIndex]    = StepperMotor->DO_MixerDirectionLean;
        StepperMotor->disabled      = FALSE;

        stepDeviation[MixerIndex] = (DS32)StepperMotor->setpointPosition - StepperMotor->actualPosition;

        if (stepDeviation[MixerIndex] < 0L)
        {
            StepperMotor->DO_MixerDirectionLean = TRUE;
        }
        else if (stepDeviation[MixerIndex] > 0L)
        {
            StepperMotor->DO_MixerDirectionLean = FALSE;
        }
        else
        {
            StepperMotor->disabled = TRUE;
        }

        if (StepperMotor->forceLean)
        {
            stepDeviation[MixerIndex] = -2L;
            StepperMotor->DO_MixerDirectionLean = TRUE;
            StepperMotor->disabled = FALSE;
        }

        if (!MIX.DI_LimitStopLean[MixerIndex] && stepDeviation[MixerIndex] < 0L)
        {
            StepperMotor->disabled = TRUE;
        }

        if (StepperMotor->reset)
        {
            StepperMotor->actualPosition = 0;
            StepperMotor->disabled = TRUE;
        }

        if (!StepperMotor->disabled && (oldDirectionLean[MixerIndex] == StepperMotor->DO_MixerDirectionLean) )
        {
            StepperMotor->DO_MixerClock = TRUE;
        }
    }
}


