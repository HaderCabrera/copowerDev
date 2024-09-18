/**
 * @file GBV.h
 * @ingroup Application
 * This is the handler for the gas blending valve
 * of the gas engine CHP project.
 *
 * @remarks
 * @ void GBV_control() is called from 10Hz control Task
 * @author GFH
 * @date 06.10.2016
 *
 * Changes:
 */

#ifndef GBV_H_
#define GBV_H_


#include "deif_types.h"
#include "appl_types.h"

extern void GBV_init(void);
extern void GBV_control_100ms(void);

extern DBOOL GBV_GetStopPowerRampDown(void);

// internal values of each protection
struct GBV_protection
{
	t_protection_state State;
    t_protection_state LastState;
    DU32               StateTimer;
    DS16               Limit;
    DBOOL              Exceeded;
};

// control Modes for GBV
enum t_GBV_mode
{
	GBV_MODE_BLOCK,
	GBV_MODE_ENABLE,
	GBV_MODE_TEST
};

// states of GBV
enum t_GBV_state
{
	GBV_STATE_BOOT,
	GBV_STATE_OFF,
	GBV_STATE_VALVE_IS_CLOSING,
	GBV_STATE_VALVE_IS_CLOSED,
	GBV_STATE_VALVE_IS_OPENING,
	GBV_STATE_VALVE_IS_OPEN,
	GBV_STATE_REGULATE,
	GBV_STATE_TEST
};

// global Variables of EXH
typedef struct GBVstruct
{
	// inputs
	DBOOL DI_Release;
	DBOOL DI_ValveClosed;
	DBOOL DI_ValveOpen;

	DS16 AI_I_FlowDemand;
	DS16 AI_I_FlowValue;

	// internal values

	DBOOL Option;
	DBOOL Active;
	DBOOL ReleasedBySignal;

	DBOOL DefinedPositionReached;

	DU8 Mode_Signal;
	DU8 Mode_Power;
	DU8 Mode_FlowDemand;

	DS16 RegValue;							// [0.1m³/h]
	DS16 RegSetpoint;						// [0.1m³/h]
	DS16 RegSetpoint_Manual;				// [0.1m³/h]
	DS16 RegSetpoint_Auto;					// [0.1m³/h]

	DS16 FlowDemand, FlowDemandFiltered;	// [0.1m³/h]
	DS16 FlowValue,  FlowValueFiltered;		// [0.1m³/h]

	DS16 SetpointValvePositionPercent;		// [0.1%] only A

	DS32 MaxPower;

	DS16 Pa; // [0.01%] Partial load of electrical power - Gas A
	DS16 Pb; // [0.01%] Partial load of electrical power - Gas B

	struct GBV_protection  ReleasedByPower;
	struct GBV_protection  ControlDeviation;

	enum t_GBV_state state;
	enum t_GBV_mode  mode;

	// outputs

	DBOOL DO_OpenValve;

	DS16 AO_SetpointValve;
	DS16 AO_SetpointValveInverted;

} t_GBV;

extern t_GBV GBV;

// macros / constants

#define GBV_RESET		0
#define GBV_REGULATE	1

// delay to avoid error messages during startup
#define GBV_WAIT_AFTER_START	               4000L

#define GBV_OPTION_OFF				0
#define GBV_OPTION_PARAMETER		1
#define GBV_OPTION_ANALOG_IN		2
#define GBV_OPTION_MODBUS			3

// [0.1m³/h]
#define GBV_PAR_FLOW_GASB_100		(PARA[ParRefInd[GBV_PAR_FLOW_GASB_100__PARREFIND]].Value)
// [0.1m³/h]
#define GBV_PAR_FLOW_GASB_50		(PARA[ParRefInd[GBV_PAR_FLOW_GASB_50__PARREFIND]].Value)

// [0.1m³/h]
#define GBV_FLOW_DEMAND_4mA			0
// [0.1m³/h]
#define GBV_FLOW_DEMAND_20mA		((DS16)PARA[ParRefInd[GBV_PAR_FLOW_DEMAND_20mA__PARREFIND]].Value)
// [0.1m³/h]
#define GBV_FLOW_VALUE_4mA			0
// [0.1m³/h]
#define GBV_FLOW_VALUE_20mA			((DS16)PARA[ParRefInd[GBV_PAR_FLOW_VALUE_20mA__PARREFIND]].Value)
// [raw]
#define GBV_VALVE_POS_OUT_ZERO		5000
// [raw]
#define GBV_VALVE_POS_OUT_FULL		25000
// [0/1]
#define GBV_PAR_REG_OPTION			((DU8)PARA[ParRefInd[GBV_PAR_REG_OPTION__PARREFIND]].Value)
// [0.1m³/h]
#define GBV_PAR_REG_SETPOINT		((DS16)PARA[ParRefInd[GBV_PAR_REG_SETPOINT__PARREFIND]].Value)
// []
#define GBV_PAR_REG_KP				(PARA[ParRefInd[GBV_PAR_REG_KP__PARREFIND]].Value)
// []
#define GBV_PAR_REG_KI				(PARA[ParRefInd[GBV_PAR_REG_KI__PARREFIND]].Value)
// []
#define GBV_PAR_REG_KD				0
// [ms]
#define GBV_VALVE_LEAVE_CLOSE_DELAY	20000L
#define GBV_TIMEOUT_CLOSING_DELAY	10000L
// [ms]
#define GBV_VALVE_LEAVE_OPEN_DELAY	20000L
#define GBV_TIMEOUT_OPENING_DELAY	10000L
// [ms]
#define GBV_RAMP_TIME				960000L

// Manual mode
#define GBV_MODE_AUTO				0
#define GBV_MODE_MANUAL				1
#define GBV_MODE_ON					2
#define GBV_MODE_OFF				3

// ON / OFF by electrical power
#define GBV_RELEASED_BY_POWER		( (GBV.Mode_Power == GBV_MODE_ON) || ( (GBV.Mode_Power == GBV_MODE_AUTO) && (GBV.ReleasedByPower.State >= TRIP) ) ) // condition

// ON / OFF by Signal and by electrical power
#define GBV_RELEASE					(GBV.ReleasedBySignal && GBV_RELEASED_BY_POWER)

// [0.01%]
#define GBV_PAR_RELEASE_POWER		((DS16)PARA[ParRefInd[GBV_PAR_RELEASE_POWER__PARREFIND]].Value)
// [0.01%]
#define GBV_RELEASE_POWER_HYST		200L
// [ms]
#define GBV_PAR_ON_BY_POWER_DELAY	((DU32)PARA[ParRefInd[GBV_PAR_ON_BY_POWER_DELAY__PARREFIND]].Value)
// [ms]
#define GBV_OFF_BY_POWER_DELAY		1000L

// [0.01%]
#define GBV_MAX_DEVIATION_LIMIT		500
// [ms]
#define GBV_MAX_DEVIATION_ACT_DELAY	120000L
// [ms]
#define GBV_MAX_DEVIATION_DELAY		30000L

// Stop power ramp down until valve has fully closed or opened
#define GBV_STOP_POWER_RAMP_DOWN	(((GBV.state == GBV_STATE_VALVE_IS_CLOSING) || (GBV.state == GBV_STATE_VALVE_IS_OPENING)) && GBV.DO_OpenValve)

// Max power if
// [0.1%] of Pnom
#define GBV_MAX_POWER				500

// [ms]
#define GBV_VALVE_MOVING_TIME		60000L

#endif /*GBV_H_*/
