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
* � [2024] Microchip Technology Inc. and its subsidiaries
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
    PFC_PRECHARGE = 5,
}PFC_CTRL_STATE_T;

typedef enum
{
    PFC_FAULT_NONE = 0,
    PFC_FAULT_IP_UV = 1,
    PFC_FAULT_IP_OV = 2,
    PFC_FAULT_OP_OV = 3,
}PFC_FAULT_TYPE_T;

/** Selects how the mid-ON inductor current sample is turned into a cycle
 *  average for the current loop. Held in PFC_T.sampleCorrectionEnable so the
 *  three methods can be compared back to back on the same build. */
typedef enum
{
    /** No reconstruction. The raw mid-ON sample goes straight to the loop.
        Exact in CCM; in DCM the sample is Ipk/2 while the true average is
        (Ipk/2)*(d1+d2), so the loop over-reads by 1/(d1+d2) and the current
        settles that same factor BELOW reference. This is the baseline. */
    PFC_DCM_COMP_OFF        = 0,
    /** Legacy correction: factor = min(d1/D_ideal, 1). The clamp doubles as
        the mode detector, which means DCM is inferred by comparing a lagged
        controller output against a computed ratio. */
    PFC_DCM_COMP_RATIO      = 1,
    /** Valley estimation. The conduction mode is decided by predicting where
        the inductor current would land at the end of the OFF time and testing
        it against a FIXED zero threshold; the same factor is then applied,
        but only when DCM was actually detected.
        Ref: H. S. Nair and N. L. Narasamma, "An Improved Digital Algorithm for
        Boost PFC Converter Operating in Mixed Conduction Mode", IEEE JESTPE
        vol. 8 no. 4, Dec 2020, eq. (4) and (6). */
    PFC_DCM_COMP_VALLEY_EST = 2,
}PFC_DCM_COMP_T;

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
    /** Active reconstruction method, a PFC_DCM_COMP_T value. */
    uint16_t sampleCorrectionEnable;
    /** 1 when the ACTIVE method called this cycle DCM. Log this to compare
        how much each method chatters at the CCM/DCM boundary. */
    uint16_t dcmDetected;
    /** Predicted end-of-OFF-time inductor current, in A. Negative means DCM.
        Computed every cycle regardless of the selected method, so a single
        run lets you check what valley estimation WOULD have said while a
        different method is driving the loop. */
    float iValleyEst;
    /** Reconstruction factor actually applied this cycle, (d1+d2). 1.0 in CCM
        or when the method is OFF. */
    float sampleCorrFactor;
    PFC_AVG_T vdcAVG;
    PFC_AVG_T vacAVG;
    PFC_RMS_SQUARE_T vacRMS;
    PFC_PI_T piVoltage;
    PFC_PI_T piCurrent;
    PFC_CTRL_STATE_T state;
    PFC_MEASURE_CURRENT_T pfcCurrent;
    PFC_MEASURE_CURRENT_T pfcCurrent2;
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
