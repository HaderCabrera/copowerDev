/**
 * @file CYL.h
 * @ingroup Application
 * This is the handler for the cylinder temperature data
 * of the Gas-CHP project.
 *
 * @remarks
 * @ void CYL_control() is called from 10Hz control Task
 * @author GFH
 * @date 04-11-2010
 *
 * changes:
 * xx.xx.xxxx  xxx  xxxx  xxx
 * 1421 22.05.2013 GFH  cleaned up
 *
 */

#ifndef CYL_H_
#define CYL_H_

#include "options.h"
#include "deif_types.h"
#include "appl_types.h"
#include "MAIN_CONTROL.h"

#if (OPTION_CYLINDER_MONITORING == TRUE)

extern void CYL_init(void);
extern void CYL_control_100ms(void);

// internal values of each protection
struct CYL_protection_variables
{
	t_protection_state State;
    t_protection_state LastState;
    DU32               StateTimer;
    DS16               Limit;
    DBOOL              Exceeded;
};

// protection functions for each cylinder
struct CYL_protection_variables_cylinder_temperature
{
	struct CYL_protection_variables     OvertempStop;
	struct CYL_protection_variables     Overtemp;
	struct CYL_protection_variables     Undertemp;
	struct CYL_protection_variables     AvrDeviation;
};

#define CYL_NBR_OF_CYLINDERS		20

#define CYL_NBR_OF_CYLINDERS_A		(CYL_NBR_OF_CYLINDERS/2)
#define CYL_NBR_OF_CYLINDERS_B		(CYL_NBR_OF_CYLINDERS/2)

// global Variables of CYL
typedef struct CYLstruct
{
   // input
   struct Temp_Input TempA[CYL_NBR_OF_CYLINDERS_A];
   struct Temp_Input TempB[CYL_NBR_OF_CYLINDERS_B];

   // internal

   // Cylinder temperatures side A
   DS16  TxxxAverageA;   // Average of T16x side A
   DS16  TxxxAverageAFilteredValue;		// Average of T16x side A filtered value
   DU8   NumberOfAssignedCylindersA;

   // Cylinder temperatures side B
   DS16  TxxxAverageB;   // Average of T17x side B
   DS16  TxxxAverageBFilteredValue;		// Average of T17x side B filtered value
   DU8   NumberOfAssignedCylindersB;

   DS16  CylinderAverageTemp;

   DS16  TMax;           // max temperature of all cylinders

   DS32  MaxPower;       // reduced max power due to too hot cylinders

   // cylinder temperature supervisions
   struct CYL_protection_variables_cylinder_temperature  A[CYL_NBR_OF_CYLINDERS_A];

   struct CYL_protection_variables_cylinder_temperature  B[CYL_NBR_OF_CYLINDERS_B];

   //enum t_CYL_state  state;
   //enum t_CYL_mode   mode;

   // output

} t_CYL;

extern t_CYL CYL;

#define		CYL_DELAY_TIME_T152_TEMP_TOO_HIGH	1000L
#define		CYL_DELAY_TIME_T153_TEMP_TOO_HIGH	1000L
#define		CYL_DELAY_TIME_T154_TEMP_TOO_HIGH	1000L
#define		CYL_DELAY_TIME_T155_TEMP_TOO_HIGH	1000L
#define		CYL_DELAY_TIME_T156_TEMP_TOO_HIGH	1000L

// Recover_Levels
#define   CYL_RECOVER_HYST_1                                 100
#define   CYL_RECOVER_HYST_2                                 200
// Delay_Levels
#define   CYL_MAX_TEMP_DELAY                                 1000L
#define   CYL_MIN_TEMP_DELAY                                 300000L
#define   CYL_MAX_TEMP_DEVIATION_DELAY                       5000L

// delay and recover delay for cylinder temperature load reduction
#define   CYL_MAX_CYLINDER_TEMPERATURE_DELAY                 1000L
#define   CYL_MAX_CYLINDER_TEMPERATURE_RECDELAY              10000L

// activation power for cylinder temp min monitoring
#define   CYL_UNDERTEMP_ACTIVATION_POWER			         4000

// filtering of average cylinder temperature values
#define   CYL_NUMBER_OF_AVERAGE_VALUES_FOR_FILTERING         10


#endif // OPTION_CYLINDER_MONITORING

#endif /*CYL_H_*/

