// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pfc_calc_params.h
 *
 * @brief This file has definitions used in the application for PFC,
 *        calculated based on associated user parameter header file
 *        pfc_userparams.h.
 *
 * Component: BOARD
 *
 */
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="Disclaimer ">

/*******************************************************************************
* SOFTWARE LICENSE AGREEMENT
* 
* © [2024] Microchip Technology Inc. and its subsidiaries
* 
* Subject to your compliance with these terms, you may use this Microchip 
* software and any derivatives exclusively with Microchip products. 
* You are responsible for complying with third party license terms applicable to
* your use of third party software (including open source software) that may 
* accompany this Microchip software.
* 
* Redistribution of this Microchip software in source or binary form is allowed 
* and must include the above terms of use and the following disclaimer with the
* distribution and accompanying materials.
* 
* SOFTWARE IS "AS IS." NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY,
* APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED WARRANTIES OF NON-INFRINGEMENT,
* MERCHANTABILITY, OR FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT WILL 
* MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, INCIDENTAL OR 
* CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND WHATSOEVER RELATED TO
* THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS BEEN ADVISED OF THE 
* POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE FULLEST EXTENT ALLOWED BY
* LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS RELATED TO THE SOFTWARE WILL
* NOT EXCEED AMOUNT OF FEES, IF ANY, YOU PAID DIRECTLY TO MICROCHIP FOR THIS
* SOFTWARE
*
* You agree that you are solely responsible for testing the code and
* determining its suitability.  Microchip has no obligation to modify, test,
* certify, or support the code.
*
*******************************************************************************/
// </editor-fold>

#ifndef PFC_CALC_PARAMS_H
#define	PFC_CALC_PARAMS_H

#ifdef	__cplusplus
extern "C" {
#endif
    
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <stdint.h>

#include "board_service.h"
#include "pfc_general.h"
#include "pfc_userparams.h"

// </editor-fold>   
    
// <editor-fold defaultstate="collapsed" desc="DEFINITIONS/MACROS ">    
/** Calculate nominal value of PFC output voltage operating range*/  
#define PFC_OUPUT_VOLTAGE_REFERENCE             PFC_OUPUT_VOLTAGE_NOMINAL
 
/** Normalized RMS square: Input under voltage lower limit in Vrms^2*/
#define PFC_INPUT_UNDER_VOLTAGE_LO_RMS_SQUARE   PFC_INPUT_UNDER_VOLTAGE_LO*PFC_INPUT_UNDER_VOLTAGE_LO      
/** Normalized RMS square: Input under voltage upper limit in Vrms^2*/
#define PFC_INPUT_UNDER_VOLTAGE_HI_RMS_SQUARE   PFC_INPUT_UNDER_VOLTAGE_HI*PFC_INPUT_UNDER_VOLTAGE_HI        
/** Normalized RMS square: Input over voltage lower limit in Vrms^2*/
#define PFC_INPUT_OVER_VOLTAGE_RMS_SQUARE_LO    PFC_INPUT_OVER_VOLTAGE_LO*PFC_INPUT_OVER_VOLTAGE_LO
/** Normalized RMS square: Input over voltage upper limit in Vrms^2*/
#define PFC_INPUT_OVER_VOLTAGE_RMS_SQUARE_HI    PFC_INPUT_OVER_VOLTAGE_HI*PFC_INPUT_OVER_VOLTAGE_HI           
/** Normalize PFC output over voltage with PFC full scale DC output voltage */
#define PFC_OUTPUT_OVER_VOLTAGE_NORMALIZED      PFC_OUTPUT_OVER_VOLTAGE
/**  Normalize PFC output under voltage with PFC full scale DC output voltage */
#define PFC_OUTPUT_UNDER_VOLTAGE_NORMALIZED     PFC_OUTPUT_UNDER_VOLTAGE
    
/** Fault limits are converted to Q15 Representation and used by the firmware */
#define PFC_OUTPUT_OVER_VOLTAGE_LIMIT           PFC_OUTPUT_OVER_VOLTAGE_NORMALIZED
#define PFC_INPUT_UNDER_VOLTAGE_LIMIT_LO        PFC_INPUT_UNDER_VOLTAGE_LO_RMS_SQUARE  
#define PFC_INPUT_UNDER_VOLTAGE_LIMIT_HI        PFC_INPUT_UNDER_VOLTAGE_HI_RMS_SQUARE
#define PFC_INPUT_OVER_VOLTAGE_LIMIT_LO         PFC_INPUT_OVER_VOLTAGE_RMS_SQUARE_LO
#define PFC_INPUT_OVER_VOLTAGE_LIMIT_HI         PFC_INPUT_OVER_VOLTAGE_RMS_SQUARE_HI 
        
// </editor-fold>   

#ifdef	__cplusplus
}
#endif

#endif	/* PFC_CALC_PARAMS_H */

