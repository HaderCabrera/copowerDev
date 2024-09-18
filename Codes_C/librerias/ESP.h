
/**
 * @file ESP.h
 * @ingroup Application
 * This is the handler for the Extended Start Procedure
 * of the REC gas engine control system .
 *
 * @remarks
 * @ void ESP_control() is called from 10Hz control Task
 * @author gfh
 * @date 21-jul-2014
 *
 */


#ifndef ESP_H_
#define ESP_H_


#include "deif_types.h"
#include "appl_types.h"
#include "modcd200.h"
#include "modcpu95.h"

extern void ESP_init(void);
extern void ESP_control_100ms(void);

// control Modes for ESP
enum t_ESP_mode
{
   ESP_MODE_OFF,
   ESP_MODE_ON,
   ESP_MODE_TEST
};

// states of ESP
enum t_ESP_state
{
  ESP_STATE_BOOT,
  ESP_STATE_OFF,
  ESP_STATE_FLUSH_PIPES_WITH_AIR,  // flushing mixture pipes with air
  ESP_STATE_FLUSH_ENGINE_WITH_AIR, // flushing engine with air
  ESP_STATE_WAIT_FOR_NEXT_CRANK,   // wait for next cranking
  ESP_STATE_FLUSH_PIPES_WITH_GAS,  // flushing mixture pipes with gas-mixture
  ESP_STATE_IGNITION,
  ESP_STATE_STARTING,
  ESP_STATE_READY, // ready for normal start
  ESP_STATE_TEST
};


// global Variables of ESP
typedef struct ESPstruct
{
   // input

   // internal

   DBOOL InhibitStartFailure;

   enum t_ESP_state  state;
   enum t_ESP_mode   mode;

   DBOOL     FanTestDemand;
   DBOOL     ValveTestDemand;

   //output
   DBOOL     DO_Fan;
   DBOOL     DO_Valve;

} t_ESP;

extern t_ESP ESP;

// constants

// [ms]
#define ESP_DELAY_OPEN_VALVE_AFTER_FAN_ON		300L
// [ms]
#define ESP_DELAY_CLOSE_VALVE_AFTER_STARTER_ON	300L
// [ms]
#define ESP_DELAY_FAN_OFF_AFTER_VALVE_CLOSED	300L
// [ms]
#define ESP_DELAY_OPEN_GAS_AFTER_VALVE_OPEN		300L
// [ms]
#define ESP_EXTRA_FLUSHING_TIME_AIR				3000L
// [ms]
#define ESP_TIMEOUT_STARTING					5000L


#endif /*ESP_H_*/
