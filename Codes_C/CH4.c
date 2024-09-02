/**
 * @file CH4.c
 * @ingroup Application
 * This is the handler for the CH4 option
 * of the REC gas engine control system.
 *
 * @remarks
 * This module is called every 100ms.
 *
 * @author mvo
 * @date 03-jun-2008
 * 
 * changes:
 * 1309 24.02.2009 GFH  new function "static void CH4_LoadRed(void)"
 * 1310 11.03.2009 GFH  use modbus value MBA.bCH4Calibrating and MBA.sCH4Value if assigned
 * 1313 04.05.2009 rmi  CH4_PERCENT_VALUE_FOR_20mA__PARREFIND supported, rmiIET
 * 1332 22.04.2010 GFH  deactivate CH4 value regulation if natural gas operation
 * 1332 23.04.2010 GFH  deactivate CH4 value load reduction if natural gas operation
 * 1343 23.12.2010 GFH  set CH4.MixerOffset to "0" if CH4 has a wire break
 * 
 */

#include "deif_types.h"
#include "appl_types.h"
#include "STOPCONDITIONS.h"

#include "statef.h"
#include "debug.h"
#include "ARC.h"
#include "CH4.h"
#include "GAS.h"
#include "GBV.h"
#include "PAR.h"
#include "PMS.h"
#include "modbusappl.h"


// CH4 data structure for global use
t_CH4 CH4;

// Local variables
static STATE myState 	 = 0;
static DU32  myStateCnt  = 0;


// Local function declaration


// ************************************************************* 
// ***********   CH4 measurement under calibration   ***********
// *************************************************************
// *                                                           *
// *              Freeze CH4 value, but stop if timeout        *
// *                                                           *
// *************************************************************
// *** function name:     CH4_Calibrating()                  ***
// *** protection struct: CH4.Calibrating                    ***
// *** input values:      CH4.DI_CalibrateCH4                ***
// *** Stop condition:    STOPCONDITION_50166                ***
// *************************************************************
#ifdef a
#undef a
#endif
#define a CH4.Calibrating

static void CH4_Calibrating(void)
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

   if (  ( CH4.CH4ValueAvailable )
      && ( CH4.OptionAndActive )									// CH4 option selected
      && ( !CH4.WireBreakAI )
      && ( !GAS.GasTypeBActive ) )                                  // poor gas is active
   {
       // check input and set a.Exceeded accordingly
       if ( ( (DI_FUNCT[CH4_CALIBRATING].Assigned == ASSIGNED) && CH4.DI_CalibrateCH4 ) // calibrating digital input assigned
       		|| ( (MBA.WriteConfigurationDigital & MBA_CONFIG_CH4_CALIBATING) && MBA.bCH4Calibrating) // calibrating modbus assigned
       	  )
           a.Exceeded = TRUE;
       else 
           a.Exceeded = FALSE;
   }
   else
       a.Exceeded = FALSE;
       
   switch(a.State)
   {
      case COLD: // no condition is pending
           if ( a.Exceeded ) 
               a.State = HOT;
           break;
           	
      case HOT: // condition present, the delay time is running
           // in this state freeze the CH4 value
           if (!a.Exceeded ) a.State = COLD; // condition is gone, return to COLD	
           if (a.StateTimer >= CH4_CALIBRATING_TIMEOUT)  // after delay time, trip the protection function
           { 
           	   a.State = TRIP; 
           } 	
           break;
           	
      case TRIP: // the delay time has tripped, condition is still pending
           // calibration timeout -> engine stopping
           STOP_Set(STOPCONDITION_50166); // set SC
           STOP_Tripped[STOPCONDITION_50166] = TRUE; // tripped = cannot be acknowledged
           if (!a.Exceeded)
               a.State = RECOVER;	
           break;
           
      case RECOVER:
           STOP_Tripped[STOPCONDITION_50166] = FALSE; // can be acknowledged
           if ( a.Exceeded )
               a.State = TRIP;
           if (a.StateTimer >= CH4_CALIBRATING_REC_DELAY) 
           { a.State = COLD;   
           }	
           break;    
    }

} // end of void CH4_Calibrating()

// ************************************************************* 
// **************    CH4 value low -> load reduction    ********
// *************************************************************
// *                                                           *
// *              load reduction due to low CH4 value          *
// *                                                           *
// *************************************************************
// *** function name:     CH4_LoadRed()                      ***
// *** protection struct: CH4.CH4ValueLow                    ***
// *** input values:      CH4.CH4Value                       ***
// *** Stop condition:    STOPCONDITION_50168                ***
// *************************************************************
#ifdef a
#undef a
#endif
#define a CH4.CH4ValueLow

static void CH4_LoadRed(void)
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

   // define Limit
   a.Limit = (DS16)PARA[ParRefInd[CH4_LIMIT_FOR_MAXLOAD__PARREFIND]].Value; // in 0,1% CH4 
   
   if (  ( CH4.CH4ValueAvailable )
      && ( CH4.OptionAndActive )									// CH4 option selected
      && ( !CH4.WireBreakAI )
      && ( !GAS.GasTypeBActive ) )                                  // poor gas is active
   {
       // check input and set a.Exceeded accordingly
       if ( CH4.CH4Value < a.Limit ) 
           a.Exceeded = TRUE;
       else 
           a.Exceeded = FALSE;
   }
   else
       a.Exceeded = FALSE;
       
   switch(a.State)
   {
      case COLD: // no condition is pending
           if ( a.Exceeded ) 
               a.State = HOT;
           else
               STOP_Clear(STOPCONDITION_50168); // auto acknowledge
           break;
           	
      case HOT: // condition present, the delay time is running
           if (!a.Exceeded ) a.State = COLD; // condition is gone, return to COLD	
           if (a.StateTimer >= CH4_LOW_DELAY)  // after delay time, trip the protection function
           { 
           	   a.State = TRIP; 
           } 	
           if (GAS.GasTypeBActive)
           {
           		a.State = COLD;
           		STOP_Tripped[STOPCONDITION_50168] = FALSE; // can be acknowledged
           }
           break;
           	
      case TRIP: // the delay time has tripped, condition is still pending
           // calibration timeout -> engine stopping
           STOP_Set(STOPCONDITION_50168); // set SC
           STOP_Tripped[STOPCONDITION_50168] = TRUE; // tripped = cannot be acknowledged
           if (!a.Exceeded)
              a.State = RECOVER;	

           if (GAS.GasTypeBActive)
           {
           		a.State = COLD;
           		STOP_Tripped[STOPCONDITION_50168] = FALSE; // can be acknowledged
           }	
           break;
           
      case RECOVER:
           STOP_Tripped[STOPCONDITION_50168] = FALSE; // can be acknowledged
           if ( a.Exceeded )
               a.State = TRIP;
           if (a.StateTimer >= CH4_LOW_REC_DELAY) 
           { a.State = COLD;   
           }	
           if (!STOP_is_Set(STOPCONDITION_50168) ) a.State = COLD; // has been acknowledged while in RECOVER
           
           if (GAS.GasTypeBActive)
           {
           		a.State = COLD;
           }
           break;  
    }

} // end of void CH4_LoadRed()


// ************************************************************* 
// **************    CH4 value too low -> stop    **************
// *************************************************************
// *                                                           *
// *              Stop engine due to too low CH4 value         *
// *                                                           *
// *************************************************************
// *** function name:     CH4_Supervision()                  ***
// *** protection struct: CH4.CH4ValueTooLow                 ***
// *** input values:      CH4.CH4Value                       ***
// *** Stop condition:    STOPCONDITION_50167                ***
// *************************************************************
#ifdef a
#undef a
#endif
#define a CH4.CH4ValueTooLow

static void CH4_Supervision(void)
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

   // define Limit
   if (a.State < TRIP)
       a.Limit = (DS16)PARA[ParRefInd[CH4_STOP_LIMIT__PARREFIND]].Value; // in 0,1% CH4
   else
       a.Limit = (DS16)PARA[ParRefInd[CH4_STOP_LIMIT__PARREFIND]].Value + CH4_TOO_LOW_HYST; // in 0,1% CH4

   if (  ( CH4.CH4ValueAvailable )
      && ( CH4.OptionAndActive )									// CH4 option selected
      && ( !CH4.WireBreakAI )
      && ( !GAS.GasTypeBActive ) )                                  // poor gas is active
   {
       // check input and set a.Exceeded accordingly
       if ( CH4.CH4Value < a.Limit ) 
           a.Exceeded = TRUE;
       else 
           a.Exceeded = FALSE;
   }
   else
       a.Exceeded = FALSE;
       
   switch(a.State)
   {
      case COLD: // no condition is pending
           if ( a.Exceeded ) 
               a.State = HOT;
           else
               STOP_Clear(STOPCONDITION_50167); // auto acknowledge
           break;
           	
      case HOT: // condition present, the delay time is running
           if (!a.Exceeded ) a.State = COLD; // condition is gone, return to COLD	
           if (a.StateTimer >= CH4_TOO_LOW_DELAY)  // after delay time, trip the protection function
           { 
           	   a.State = TRIP; 
           } 	
           if (GAS.GasTypeBActive)
           {
           		a.State = COLD;
           		STOP_Tripped[STOPCONDITION_50167] = FALSE; // can be acknowledged
           }
           break;
           	
      case TRIP: // the delay time has tripped, condition is still pending
           // calibration timeout -> engine stopping
           STOP_Set(STOPCONDITION_50167); // set SC
           STOP_Tripped[STOPCONDITION_50167] = TRUE; // tripped = cannot be acknowledged
           if (!a.Exceeded)
               a.State = RECOVER;	

           if (GAS.GasTypeBActive)
           {
           		a.State = COLD;
           		STOP_Tripped[STOPCONDITION_50167] = FALSE; // can be acknowledged
           }	
           break;
           
      case RECOVER:
           STOP_Tripped[STOPCONDITION_50167] = FALSE; // can be acknowledged
           if ( a.Exceeded )
               a.State = TRIP;
           if (a.StateTimer >= CH4_TOO_LOW_REC_DELAY) 
           { a.State = COLD;   
           }	
           if (!STOP_is_Set(STOPCONDITION_50167) ) a.State = COLD; // has been acknowledged while in RECOVER
           
           if (GAS.GasTypeBActive)
           {
           		a.State = COLD;
           }
           break;   
    }

} // end of void CH4_CH4_Supervision()



//////////////////// public controlloop CH4_control_100ms
/**
 * @void CH4_control_100ms(void)
 * 
 * CH4 option. Is called every 100ms.
 * 
 * 
 */


void CH4_control_100ms(void)
{	  
    DS16 CH4_Internal;
    DS16 CH4Diff;
    DS32 DeltaP;
    DS16 DeltaCH4;
    
    DS32 NominalPower;

    CH4.Option = PARA[ParRefInd[CH4_OPTION_CONTROL__PARREFIND]].Value;
    CH4.OptionAndActive = CH4.Option
    		&& (!GBV.Active || PARA[ParRefInd[GBV_PAR_CH4_OPTION_ACTIVE__PARREFIND]].Value);
/*
    CH4.CH4ValueAvailable = ( (AI_I_FUNCT[CH4_VALUE].Assigned == ASSIGNED)
	        					|| (PARA[ParRefInd[MBA_CONFIGURATION_ANALOG__PARREFIND]].Value & MBA_CONFIG_CH4_VALUE)
	        					|| (PMS.EngineIDConfigured[ARC.nEngineId-1] && (PARA[ParRefInd[PMS_REG_CH4__PARREFIND]].Value != 0L) ) );
*/
    // always true
    // at least take parameter value
    CH4.CH4ValueAvailable = TRUE;
    
    if (PMS.EngineIDConfigured[ARC.nEngineId-1] // PMS function
	    && (PARA[ParRefInd[PMS_REG_CH4__PARREFIND]].Value != 0L))
	{
		NominalPower = PMS.NominalPowerTotal*1000L;
	}
	else
	{
		NominalPower = (DS32)PARA[ParRefInd[GEN_NOMINAL_LOAD__PARREFIND]].Value;
	}

    if (CH4.Option)	// CH4 option selected
    {
	    if ( PMS.EngineIDConfigured[ARC.nEngineId-1]
			&& (     PARA[ParRefInd[PMS_REG_CH4__PARREFIND]].Value != 0L)
			&& ((DU8)PARA[ParRefInd[PMS_REG_CH4__PARREFIND]].Value != ARC.nEngineId) )
	    {
	    	CH4.WireBreakAI = FALSE;

	    	// take CH4 value from ARCnet in 0.1%
	    	CH4_Internal = PMS.CH4Value;
	    	STOP_Tripped[STOPCONDITION_50165] = FALSE;
	    }
	    else if (MBA.WriteConfigurationAnalog & MBA_CONFIG_CH4_VALUE)
	    {
	    	CH4.WireBreakAI = FALSE;

	    	// take CH4 value from modbus in 0.1%
	    	CH4_Internal = MBA.sCH4Value;
	    	STOP_Tripped[STOPCONDITION_50165] = FALSE;
	    }
	    else if (AI_I_FUNCT[CH4_VALUE].Assigned == ASSIGNED)
	    {
	    	CH4.WireBreakAI = STOP_is_Set(STOPCONDITION_50165);

	    	// calculate CH4 value (in 0.1%) from raw data (5000...25000)

	    	// CH4_Internal = (AI-Value - Zero-Value) * 20mA-Value (+10000 for rounding)/ 20000
			// 0mA (= 5000raw) = ( 5000 - 5000)*ParaValueAt20mA[0.1%]/20000 % = 0 %
			// 20mA(=25000raw) = (25000 - 5000)*ParaValueAt20mA[0.1%]/20000 % = (default=)1000 [100 %]

			// no wire break
			if( CH4.AI_I_CH4Value > 3500 )
			{
			  // no wire break
			  STOP_Tripped[STOPCONDITION_50165] = FALSE; // can be acknowledged

			  CH4_Internal = (((DS32)(CH4.AI_I_CH4Value - CH4_VALUE_ZERO)
				* (DS32)PARA[ParRefInd[CH4_PERCENT_VALUE_FOR_20mA__PARREFIND]].Value + 10000)/20000);	// rmiIET
			}
			else
			{
			  // wire break
			  CH4_Internal = -32768; // is this a good idea?
			  if (!GAS.GasTypeBActive) // poor gas is active
			  {
				  STOP_Set(STOPCONDITION_50165);
				  STOP_Tripped[STOPCONDITION_50165] = TRUE; // tripped = cannot be acknowledged
			  }
			  else
			  {
				  STOP_Tripped[STOPCONDITION_50165] = FALSE;
			  }
			}
	    }
		else
		{
	    	CH4.WireBreakAI = FALSE;

			CH4_Internal = PARA[ParRefInd[CH4_MEASURED_VALUE__PARREFIND]].Value;
			STOP_Tripped[STOPCONDITION_50165] = FALSE;
		}
    }
	else
	{
    	CH4.WireBreakAI = FALSE;

		CH4_Internal = PARA[ParRefInd[CH4_MEASURED_VALUE__PARREFIND]].Value;
		STOP_Tripped[STOPCONDITION_50165] = FALSE;
	}


	// CH4 regulation from ARCnet
	if (PMS.EngineIDConfigured[ARC.nEngineId-1] // PMS function
	    && (      PARA[ParRefInd[PMS_REG_CH4__PARREFIND]].Value != 0L)				// CH4 regulation from ARCnet
	    && ( (DU8)PARA[ParRefInd[PMS_REG_CH4__PARREFIND]].Value != ARC.nEngineId) )	// not my Id
    {
    	CH4.MaxPower_CH4 = (DS32)ARC_MP_CH4((DU8)PARA[ParRefInd[PMS_REG_CH4__PARREFIND]].Value) * 1000L;
    }
	// CH4 regulation not from ARCnet
	else
	{
		if (CH4.OptionAndActive	// CH4 option selected
			&& CH4.CH4ValueAvailable
			&& (!GAS.GasTypeBActive) )
		{
			// wire-break -> set to min load
			if (CH4.WireBreakAI)
			{
				CH4.MaxPower_CH4 = NominalPower
					* PARA[ParRefInd[CH4_MINLOAD__PARREFIND]].Value / 1000;
			}
			else
			{
				// calculate max power due to CH4 value

				// calculate difference to full load limit
				CH4Diff =   (DS16)PARA[ParRefInd[CH4_LIMIT_FOR_MAXLOAD__PARREFIND]].Value
						  - CH4.CH4Value; // in 0,1% CH4
				if (CH4Diff < 0)
				{
					// full load allowed
					CH4.MaxPower_CH4 = NominalPower; // set to nominal power, no load reduction
				}
				else if (CH4_Internal < (DS16)PARA[ParRefInd[CH4_LIMIT_FOR_MINLOAD__PARREFIND]].Value)
				{
					// CH4 is lower than min
					CH4.MaxPower_CH4 = NominalPower
						* PARA[ParRefInd[CH4_MINLOAD__PARREFIND]].Value / 1000; // set to min load
					// set marker for power reduced by CH4 here
				}
				else if (PARA[ParRefInd[CH4_LIMIT_FOR_MAXLOAD__PARREFIND]].Value != PARA[ParRefInd[CH4_LIMIT_FOR_MINLOAD__PARREFIND]].Value)
				{
					// derate power according to settings CH4_LIMIT_FOR_MAXLOAD__PARREFIND, CH4_MINLOAD__PARREFIND,
					// and CH4_LIMIT_FOR_MINLOAD__PARREFIND
					DeltaP  =   NominalPower
							  *( (DS32)1000 - (DS32)PARA[ParRefInd[CH4_MINLOAD__PARREFIND]].Value) / 1000; // size of proportional derating range
					DeltaCH4 =  (DS16)PARA[ParRefInd[CH4_LIMIT_FOR_MAXLOAD__PARREFIND]].Value
							  - (DS16)PARA[ParRefInd[CH4_LIMIT_FOR_MINLOAD__PARREFIND]].Value; // in 0,1% CH4
					CH4.MaxPower_CH4 =  ( NominalPower
										  * PARA[ParRefInd[CH4_MINLOAD__PARREFIND]].Value / 1000 )
										+ ( DeltaP / DeltaCH4 ) * ( CH4.CH4Value - (DS16)PARA[ParRefInd[CH4_LIMIT_FOR_MINLOAD__PARREFIND]].Value );

					if (CH4.MaxPower_CH4 < 0) CH4.MaxPower_CH4 = 0;   // never set MaxPower below zero
					// set marker for power reduced by CH4 here
				}
				else // wrong setting -> set to min load
				{
					CH4.MaxPower_CH4 = NominalPower
						* PARA[ParRefInd[CH4_MINLOAD__PARREFIND]].Value / 1000;
				}
			}
		}
	    else
	    {
	    	// full load allowed
	   	    CH4.MaxPower_CH4 = NominalPower; // set to nominal power, no load reduction
	    }
	}

    // result of all this is a CH4 value
    if (CH4.Calibrating.State != HOT)
    // not calibrating: copy the internal value to the global variable CH4.CH4Value
        CH4.CH4Value = CH4_Internal;
    // else freeze the CH4.CH4Value where it is (= do nothing)
    
    
    // adjust mixer starting position
    if (  ( CH4.CH4ValueAvailable )
       && ( CH4.OptionAndActive )									// CH4 option selected
       && ( !CH4.WireBreakAI )
       && ( !GAS.GasTypeBActive ) )                                  // poor gas is active
    {
    	// CH4 option selected, CH4 input assigned
        CH4.MixerOffset  = ( ( (DS16)PARA[ParRefInd[CH4_STANDARD_MIX__PARREFIND]].Value
                               - CH4.CH4Value) * (DS16)PARA[ParRefInd[CH4_MIX_ADJUSTMENT__PARREFIND]].Value / 10 ); 
    }
    else
    {
    	// option deselected or input not used anymore -> set adjustment back to 0
    	CH4.MixerOffset     = 0;
    }
    
    
    
    // call supervision state functions
    CH4_Calibrating();        // check if calibrating, set stopcondition if timed out
    CH4_Supervision();        // stop because of too low CH4 value
    
    if (!PMS.EngineIDConfigured[ARC.nEngineId-1]   // no PMS function
       || ((DU8)PARA[ParRefInd[PMS_REG_CH4__PARREFIND]].Value == ARC.nEngineId) // we have to regulate
	   || (PARA[ParRefInd[PMS_REG_CH4__PARREFIND]].Value == 0L)) // no ID for regulation assigned
	{
	    CH4_LoadRed();            // load reduction because of too low CH4 value
	    
	}
	else // no monitoring of load reduction
	{
		CH4.CH4ValueLow.State    = COLD;
		STOP_Tripped[STOPCONDITION_50168] = FALSE;
	}
    
    if (myStateCnt < ( MAX_DU32 - 1000 )) myStateCnt = myStateCnt + 100;
    if (myState != 0) myState(SIG_DO);
}



void CH4_init(void)
{
	
	// Initialization of the CH4 struct
	CH4.CH4Value                        = 0;  // 0,0% CH4 when starting up
	CH4.CH4ValueAvailable               = FALSE;
	CH4.MaxPower_CH4                    = 0L; //
	CH4.MixerOffset                     = 0;  // initially no change of the starting position
	
	CH4.Calibrating.State               = COLD;
	CH4.CH4ValueLow.State               = COLD;
	CH4.CH4ValueTooLow.State            = COLD;
	
	
	// power demand set to nominal power
}





