/*
 * options.h
 *
 *  Created on: 04.02.2020
 *      Author: gfh
 */

#ifndef OPTIONS_H_
#define OPTIONS_H_

// if develop version, set DEVELOP here to TRUE
#define DEVELOP FALSE

// define client-version here
#define DEIF       		1
#define IET        		2
#define CLIENT_VERSION	DEIF

#if (CLIENT_VERSION == IET)

// Options
#define OPTION_SMS                  FALSE
#define OPTION_CONF_LOGO            FALSE
#define OPTION_CONF_DIG_IO_INV      FALSE
#define OPTION_SUPPORT              FALSE
#define OPTION_CANOPEN              TRUE
#define OPTION_PMS_CAN_LOGGER       FALSE
#define OPTION_CYLINDER_MONITORING  TRUE
#define OPTION_THERMOREACTOR        TRUE

// Languages
#define OPTION_ENGLISH              TRUE
#define OPTION_DEUTSCH              TRUE
#define OPTION_NORSK                TRUE
#define OPTION_DANSK                FALSE
#define OPTION_ITALIANO             TRUE
#define OPTION_ROMANESC             TRUE
#define OPTION_SLOVENSKI            TRUE
#define OPTION_SVENSK               TRUE
#define OPTION_RUSSKI               TRUE
#define OPTION_CHINESE              TRUE
#define OPTION_TURKISH              TRUE
#define OPTION_FRANCAIS             FALSE
#define OPTION_BULGARIAN            TRUE
#define OPTION_CONF_LANG            FALSE

#endif

#if (CLIENT_VERSION == DEIF)

// Options
#define OPTION_SMS                  FALSE
#define OPTION_CONF_LOGO            TRUE
#define OPTION_CONF_DIG_IO_INV      TRUE
#define OPTION_SUPPORT              TRUE
#define OPTION_CANOPEN              TRUE
#define OPTION_PMS_CAN_LOGGER       FALSE
#define OPTION_CYLINDER_MONITORING  TRUE
#define OPTION_THERMOREACTOR        FALSE // DO NOT ENABLE
#define OPTION_SCR                  FALSE

// Languages
#define OPTION_ENGLISH              TRUE
#define OPTION_DEUTSCH              TRUE
#define OPTION_NORSK                FALSE
#define OPTION_DANSK                FALSE
#define OPTION_ITALIANO             FALSE
#define OPTION_ROMANESC             FALSE
#define OPTION_SLOVENSKI            FALSE
#define OPTION_SVENSK               FALSE
#define OPTION_RUSSKI               FALSE
#define OPTION_CHINESE              FALSE
#define OPTION_TURKISH              FALSE
#define OPTION_FRANCAIS             FALSE
#define OPTION_BULGARIAN            FALSE
#define OPTION_CONF_LANG            TRUE

// Colors
#define DEIF_DM4_COLORS             0
#define DEIF_DM400_COLORS           1
#define DEIF_AWC500_COLORS          2
#define DEIF_COLORS                 DEIF_AWC500_COLORS

#endif

#endif /* OPTIONS_H_ */
