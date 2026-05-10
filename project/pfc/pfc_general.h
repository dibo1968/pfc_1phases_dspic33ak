// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pfc_general.h
 *
 * @brief This module holds the definitions of general constants
 *
 * Component: PFC
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

#ifndef __GENERAL_H
#define __GENERAL_H

#ifdef __cplusplus
extern "C" {
#endif
    
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#include <stdint.h>
// </editor-fold> 
    
// <editor-fold defaultstate="collapsed" desc="DEFINITIONS/MACROS ">
#define PI                          3.141593
#define SQR_ROOT_OF_2               1.4142
    
#define Q15(Float_Value)	\
((Float_Value < 0.0) ? (int16_t)(32768 * (Float_Value) - 0.5) \
: (int16_t)(32767 * (Float_Value) + 0.5))

/** RMS Square of AC signals */
#define NORM_RMS_SQUARE(input,max)  (float)((input*input)/(max*max))
/** Function to calculate Normalize value of any input */
#define NORM_VALUE(input,max)       (float)(input/max)
 // </editor-fold>  
    
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif

#endif      // end of general_H
