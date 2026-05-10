// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pfc_measure.h
 *
 * @brief This module has variable type definitions of data structure and 
 * functions for signal conditioning of measured analog feedback signals.
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

#ifndef __PFC_MEASURE_H
#define __PFC_MEASURE_H

#ifdef __cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#include <stdint.h>
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DEFINITIONS/MACROS ">    
#define PFC_OFFSET_COUNT_BITS   10
#define PFC_OFFSET_COUNT_MAX    (int16_t)(1 << PFC_OFFSET_COUNT_BITS)
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPES ">       
typedef struct
{  
    int16_t inductorCurrent;
    float iL;             /* Inductor current */
    float offset;         /* Inductor current offset */
    
    uint16_t
        counter,            /* counter */
        status;             /* flag to indicate offset measurement completion */

    float
        sum;               /* Sum of inductor current for offset measurement */
}PFC_MEASURE_CURRENT_T;

typedef struct
{
    int16_t acVoltage,
            outputVoltage;
    float vac,            /* Input AC voltage */
        offsetVac,          /* Input AC voltage offset */
        vdc;                /* Output DC voltage */
} PFC_MEASURE_VOLTAGE_T;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">
void PFC_MeasureCurrentOffset (PFC_MEASURE_CURRENT_T *);
void PFC_MeasureCurrentInit (PFC_MEASURE_CURRENT_T *);

// </editor-fold>

#ifdef __cplusplus
}
#endif

#endif /* end of __MEASURE_H */
