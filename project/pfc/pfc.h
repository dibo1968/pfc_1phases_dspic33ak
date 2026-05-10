// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pfc.h
 *
 * @brief This module holds variable type definitions of data structure holding
 * PFC control parameters.
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

#ifndef __PFC_H
#define __PFC_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#include <xc.h>
#include <stdint.h>
#include <stdbool.h>

#include "pfc_general.h"
#include "pfc_pi.h"

#include "pfc_calc_params.h"
#include "pfc_measure.h"

// </editor-fold> 
 
#ifdef	__cplusplus
extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPES ">    
typedef struct
{
    float sum;
    float output;
    uint16_t scaler;
    uint16_t samples;
    uint16_t sampleLimit;
    uint16_t status;
}PFC_AVG_T;

typedef struct
{
    float sqrOutput;
    float sum;
    uint16_t samples;
    uint16_t sampleLimit;
    float peak;
    float peakcheck;
    uint16_t status;
}PFC_RMS_SQUARE_T;

typedef enum
{
    PFC_INIT = 0,
    PFC_OFFSET_MEAS = 1,
    PFC_WAIT_1CYCLE = 2,
    PFC_CTRL_RUN = 3,
    PFC_FAULT = 4,       
}PFC_CTRL_STATE_T;

typedef enum
{
    PFC_FAULT_NONE = 0,
    PFC_FAULT_IP_UV = 1,
    PFC_FAULT_IP_OV = 2,
    PFC_FAULT_OP_OV = 3,           
}PFC_FAULT_TYPE_T;

typedef struct
{
    uint32_t duty;
    uint16_t samplePoint;
    float  averageCurrent;
    uint16_t  rampRate;
    uint16_t  voltLoopExeRate;
    volatile float boostDutyRatio;
    volatile float currentReference;
    uint16_t faultStatus;
    uint16_t sampleCorrectionEnable;
    PFC_AVG_T vdcAVG;
    PFC_AVG_T vacAVG;
    PFC_RMS_SQUARE_T vacRMS;
    PFC_PI_T piVoltage;
    PFC_PI_T piCurrent;
    PFC_CTRL_STATE_T state;
    PFC_MEASURE_CURRENT_T pfcCurrent;
    PFC_MEASURE_VOLTAGE_T pfcVoltage;
    float iL;
    float rectifiedVac;
    float outputVdc;
}PFC_T;

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">
void PFC_ServiceInit(void);

// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* PFC_H */
