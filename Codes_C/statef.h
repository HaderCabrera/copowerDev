/**
 * No cabe todo dentro de la area visible.
* STATE2 es un Function Pointer definido en otro programa, pero mando aqui, el programa es corto, lo que ves es lo que tiene.
* El hace parte de un state machine, una subrutina para controlar del estado que esta el misturador. la subrutina Transit es llamada en toda vez que el Mixer cambia de estado, los estados son los siguientes:

  * MIX_BOOT,                    * during bootup
  * MIX_SYSTEM_OFF,              * System off
  * MIX_SEARCHING_LEAN,
  * MIX_LIMIT_LEAN_REACHED,
   * MIX_MOVING_FAT,
   * MIX_POS_FAT_REACHED,
   *MIX_MOVING_TO_START_POS,
  * MIX_START_POSITION_REACHED,
  * MIX_UNDER_CTRL,
   * MIX_UNDER_TEST

* toda vez que cambia de estado hay un chequeo para ver si no esta en Boot, y resetar el contador de tiempo. Abajo la subrutina que define STATE2
*/

/**
* @file statef.h
 * @ingroup Basis
 *
 * State functions. Interface to the state functions.
 *
 * @author JAH
 * @date 2006-09-19
 * 
 * changes:
 * 1341 25.08.2010 GFH	3x par. PID regulators
 * 
 */

#ifndef STATEF_H_
#define STATEF_H_

#include "deif_types.h"
#include "appl_types.h"

/** 
 * State transit macro. 
 * Handles the transit from one state(mystate) to the next state(newState)
 * First a test is made if state shift is done. Then exit conditions are run in current
 * state, and then entry conditions are run in the new state. The module state variable
 * is updated to the new state.
 */ 

#define TRANSIT( NEW_STATE, InModule )  {

  if(NEW_STATE != InModule.state)
    { 
      myState(SIG_EXIT); 
      myState = ModuleStates[NEW_STATE]; 
      InModule.state = NEW_STATE; 
      myState(SIG_ENTRY); 
    }
  }
//OLD_TRANSIT myState(SIG_EXIT); (myState = newState); myState(SIG_ENTRY);										
//#define TRANSIT(x) {if (x!=SLOG_STATE_VAR) {SLOG_STATE_VAR = x; StateLog(SLOG_MODULE_DEF, SLOG_STATE_VAR, SLOG_MODE_VAR); myState = ModuleStates[SLOG_STATE_VAR];}}

/** Function pointer for statemachine */
typedef void (*STATE)(const DU8 sig);
typedef void (*STATE2)(const DU8 sig, DU8 par);

/** Enum for state entry, and exit */ 
enum SIGNAL
{
	SIG_DO = 0,
	SIG_ENTRY,
	SIG_EXIT,
	SIG_FIRST_USER
};

#endif /*STATEF_H_*/