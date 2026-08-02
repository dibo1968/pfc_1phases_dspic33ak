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
    /* scaler removed 2026-08-02: the window is set directly from
       PFC_VDC_AVG_SAMPLES and is no longer a power-of-two shift, so there was
       nothing for it to hold. Never assigned, never read. */
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
    /* peak/peakcheck removed 2026-08-02: peakcheck was never touched at all and
       peak was only ever assigned 0 in PFC_ResetParams, so a field named "peak"
       reported a constant zero - worse than absent, since it reads like a real
       measurement on the scope. The input peak is computed where it is needed,
       as sqrtf(2*sqrOutput) in PFC_StatePrecharge. If a persistent peak is
       wanted later, add it back populated (appending is mirror-safe). */
    uint16_t status;
}PFC_RMS_SQUARE_T;

typedef struct
{
    int16_t  rawADC;       /* Raw load-current ADC reading (IL2). */
    float    current;      /* Scaled load current [A]. VERIFY scale/sign vs HW. */
    float    currentFilt;  /* IIR-filtered load current [A]. */
    float    powerFF;      /* Feed-forward power = gain * Vdc * currentFilt [W]. */
    float    scale;        /* Load-sensor ADC->A scale. */
    float    filtCoeff;    /* IIR coefficient (0..1). */
    float    gain;         /* Feed-forward gain (0..~1), bench-tuned. */
    uint16_t enable;       /* 0 = FF off (default), 1 = on. */
}PFC_LOAD_FF_T;

/** Vdc conditioning for the voltage-PI feedback path: a single real pole
 *  (broadband ADC noise) cascaded with a 100 Hz notch (double-line ripple).
 *  Buys back the phase the 10 ms block average costs, without touching
 *  KP_V/KI_V - see PFC_VDC_NOTCH_ENABLE_DEFAULT for the rationale and for the
 *  50 Hz-only caveat. The notch is direct form I, so x1/x2 and y1/y2 are its
 *  input and output histories. */
typedef struct
{
    float lpfOut;      /* Output of the anti-noise pole = the notch input. */
    float x1;          /* Notch input  history, z^-1. */
    float x2;          /* Notch input  history, z^-2. */
    float y1;          /* Notch output history, z^-1. */
    float y2;          /* Notch output history, z^-2. */
    float output;      /* Filtered Vdc [V]. Updated even when enable is 0. */
    float b0;          /* Notch numerator coefficients, a0-normalised. */
    float b1;
    float b2;
    float a1;          /* Notch denominator coefficients, a0-normalised. */
    float a2;
    float lpfCoeff;    /* Anti-noise pole coefficient (0..1). */
    uint16_t exeRate;  /* ISR counter, decimates the filter to the PI rate. */
    uint16_t enable;   /* 0 = PI uses vdcAVG.output, 1 = PI uses output. */
}PFC_VDC_NOTCH_T;

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
    PFC_FAULT_IP_UV = (1u << 0),    /* Input under-voltage  */
    PFC_FAULT_IP_OV = (1u << 1),    /* Input over-voltage   */
    PFC_FAULT_OP_OV = (1u << 2),    /* Output over-voltage  */
    PFC_FAULT_OP_UV = (1u << 3),    /* Output under-voltage (auto-recover) */
    PFC_FAULT_IP_OC = (1u << 4),    /* Input over-current   (latched) */
    PFC_FAULT_PRECHG = (1u << 5),   /* Precharge timeout    (latched) */
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
    /* samplePoint removed 2026-08-02: declared but never assigned or read. The
       ADC trigger position is fixed at PFC_ADC_SAMPLING_POINT in pwm.h. */
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
    /* Raw IL2 measurement, retained so the existing SiL/X2C bus mirror keeps
       compiling. The control path uses loadFF below. */
    PFC_MEASURE_CURRENT_T pfcCurrent2;
    PFC_LOAD_FF_T loadFF;
    PFC_MEASURE_VOLTAGE_T pfcVoltage;
    float iL;
    float rectifiedVac;
    float outputVdc;
    /** Total applied duty ratio for this cycle = dutyFF + piCurrent.output,
        clamped to [PFC_MIN_DUTY, PFC_MAX_DUTY]. This is the d1 that produced
        the NEXT current sample, so PFC_ConductionModeDetect and
        PFC_DcmAverageFactor must read this and not piCurrent.output - with the
        feed-forward enabled the PI output is only the trim. */
    float dutyRatio;
    /** Feed-forward part of dutyRatio. CCM: (Vo-Vg)/Vo. DCM: the duty that
        open-loop delivers currentReference. Zero when dutyFFEnable is 0. */
    float dutyFF;
    /** Total power command driving the current reference: the voltage-loop
        output plus the load feed-forward when enabled. Burst control MUST test
        this and not piVoltage.output - with the feed-forward carrying the
        load, the voltage-PI output legitimately sits near zero at full load. */
    float powerCommand;
    /** 1 = duty feed-forward active (default), 0 = the PI supplies the whole
        duty (legacy behaviour). Writable at run time so both can be compared
        on a single build, like sampleCorrectionEnable. */
    uint16_t dutyFFEnable;
    /** Notch + anti-noise pole on the bus measurement. Appended at the end of
        PFC_T on purpose: the SiL bus mirror copies field by field and indexes
        its own layout positionally, so appending leaves it valid. */
    PFC_VDC_NOTCH_T vdcNotch;
    /** The bus measurement the voltage PI actually closes on: vdcNotch.output
        when vdcNotch.enable is set, otherwise vdcAVG.output. Precharge, the
        OV/UV trips and loadFF keep reading vdcAVG.output directly. */
    float vdcFeedback;
    /** dutyFFEnable as last acted on by the current loop. Lets the loop
        detect a run-time toggle and refresh the PI limits / hand the
        operating point between integrator and feed-forward - the limits used
        to be set only in PFC_ParamsInit, so toggling 1->0 at run time left
        the PI clamped at +/-PFC_DUTY_TRIM_MAX and unable to supply the full
        duty. Appended at the end of PFC_T: the SiL bus mirror copies field by
        field, so appending leaves it valid. */
    uint16_t dutyFFEnablePrev;
    /** PFC_PRECHARGE dwell counter, in ISR ticks. Trips PFC_FAULT_PRECHG at
        PFC_PRECHARGE_TIMEOUT_COUNT. Appended - see dutyFFEnablePrev. */
    uint32_t prechargeCount;
}PFC_T;

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">
void PFC_ServiceInit(void);

// </editor-fold>

#ifdef	__cplusplus
}
#endif

#endif	/* PFC_H */
