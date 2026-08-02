// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file  pfc_userparams.h
 *
 * @brief This file has definitions to be configured by the user for PFC 
 * application by average current mode control(ACMC)
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

#ifndef __PFC_USERPARAMS_H
#define __PFC_USERPARAMS_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif
#include <stdint.h>
#include <stdbool.h>
#include "pfc_general.h"
#include "pwm.h"
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="DEFINITIONS/MACROS ">
        
 /* Define DEBUG_BOOST to debug the boost operation. 
  * In this mode apply fixed DC input voltage.*/              
#undef DEBUG_BOOST
        
/* Define the DEBUG_PFC_DUTY in terms of PWM clock period 
 * 
 * Duty_Cycle = 1 - (Vin/Vout)
 * DEBUG_PFC_DUTY = Duty_Cycle * PFC_LOOPTIME_TCY
 */        
#ifdef DEBUG_BOOST
    #define DEBUG_PFC_DUTY    0
#endif

/*PFC inductor current offset measurement*/
//#define ENABLE_PFC_CURRENT_OFFSET_CORRECTION

/* Define PFC input AC voltage frequency in Hz */
#define PFC_INPUT_FREQUENCY             50.0f 
        
/* Define the input AC voltage frequency in terms of PWM clock period */       
#define PFC_INPUT_FREQUENCY_COUNTER     (PFC_PWMFREQUENCY_HZ/PFC_INPUT_FREQUENCY )
         
/* Number of samples to average the DC bus voltage over.
 * Set to one 100 Hz double-line-frequency ripple period (= half the line
 * period, since the bus ripple is at twice the line frequency). Averaging
 * over an integer number of ripple periods structurally nulls the ripple and
 * its harmonics, keeping them out of the voltage-loop error and hence out of
 * the current reference - this is what limits input-current (3rd-harmonic) THD.
 * 64 kHz / (2 x 50 Hz) = 640 samples = 10 ms.
 * NOTE: 5x the old 128-sample (2 ms) window. That cost phase margin: the
 * window both averages (Tw/2) and holds (Th/2), so it contributes ~10 ms of
 * delay, ~25 deg at the voltage-loop crossover. Measured against the physical
 * plant 1/(C*Vo*s) the loop crosses at 6.9 Hz with PM ~54 deg on the old 2 ms
 * window and only ~34 deg on this one. KP_V / KI_V are unchanged; the margin
 * is recovered by the notch below instead. */
#define PFC_VDC_AVG_SAMPLES             (PFC_PWMFREQUENCY_HZ/(2*PFC_INPUT_FREQUENCY))

/* ---- Vdc feedback notch + anti-noise pole (voltage-PI path only) ----------
 * The block average above nulls the 100 Hz bus ripple structurally, but it
 * does so by low-passing everything, which is where the ~25 deg of lag comes
 * from. A 2nd-order notch reaches the same 100 Hz null for only ~4 deg,
 * because it only has to be selective at 100 Hz rather than low-pass the whole
 * band. A single real pole cascaded ahead of it restores the broadband noise
 * averaging the boxcar was also providing - a bare notch gives none - for a
 * further ~0.8 deg. Net: PM ~34 deg -> ~54 deg with KP_V / KI_V untouched.
 *
 * Only the voltage PI consumes this (PFC_T.vdcFeedback). Precharge, the OV/UV
 * trips and the load feed-forward deliberately keep reading vdcAVG.output,
 * which is slower but far more noise-immune.
 *
 * NOTE: tuned for a 50 Hz line. Off-tune the notch degrades faster than the
 * boxcar: -34 dB at 101 Hz (1 % line error) against -80 dB, and only -10 dB at
 * 120 Hz. At 50 Hz +/-1 % the resulting 100 Hz feedthrough into the power
 * command is ~0.2 %, which is immaterial. On 60 Hz mains it is NOT adequate -
 * retune f0 to 120 Hz along with PFC_INPUT_FREQUENCY, or leave this disabled. */

/* 1 = notch + pole active (default), 0 = the voltage PI closes on
 * vdcAVG.output exactly as before. Held in PFC_T.vdcNotch.enable so it is also
 * writable at run time from X2C-Scope, like dutyFFEnable and loadFF.enable. */
#define PFC_VDC_NOTCH_ENABLE_DEFAULT    1

/* RBJ notch biquad, f0 = 100 Hz, Q = 1.0, fs = PFC_PWMFREQUENCY_HZ /
 * VOLTAGE_LOOP_EXE_RATE = 5333.333 Hz:
 *      w0 = 2*pi*f0/fs;  alpha = sin(w0)/(2*Q);  a0 = 1 + alpha
 *      b = [1, -2cos(w0), 1] / a0;  a = [a0, -2cos(w0), 1-alpha] / a0
 * Literals rather than a runtime sinf/cosf so the coefficients are exact and
 * deterministic. DC gain is exactly 1.0 and the null at 100 Hz is exact.
 * RE-DERIVE THESE if f0, Q or VOLTAGE_LOOP_EXE_RATE changes. */
#define PFC_VDC_NOTCH_B0                0.944493355f
#define PFC_VDC_NOTCH_B1               -1.875893116f
#define PFC_VDC_NOTCH_B2                0.944493355f
#define PFC_VDC_NOTCH_A1               -1.875893116f
#define PFC_VDC_NOTCH_A2                0.888986709f

/* Single real pole ahead of the notch, 500 Hz at the same 5333.333 Hz rate:
 *      coeff = 1 - exp(-2*pi*fp/fs)
 * Cuts broadband ADC noise to ~0.14x (the 640-sample boxcar gave 0.03x, a bare
 * notch 1.0x) for ~0.8 deg of phase at 7 Hz. */
#define PFC_VDC_NOTCH_LPF_COEFF         0.445145090f


/* Counter for RMS calculation of rectified input AC voltage in terms of 
 * PWM clock period*/       
#define PFC_RMS_SQUARE_COUNTMAX         (PFC_PWMFREQUENCY_HZ/(2*PFC_INPUT_FREQUENCY))      

/* Define the base value of voltage 
    * Base value of the voltage is calculated as follows:
		Resistor divider gain (R_gain)              = 2.2kOhm/(300kOhm+2.2kOhm) 
                                                    = 0.00727995
		Maximum ADC input voltage (Vinadc)          = 3.3V
                                 Since   Vinadc     = Vbase * R_gain
                                         Vbase      = Vinadc / R_gain
                                         Vbase      = 3.3 / 0.00727995
                                                    = 453.3 V
		Therefore, maximum voltage or base voltage  = 453V */    
#define PFC_VOLTAGE_BASE                453.0f

#define ADC_VOLTAGE_SCALE               (float)(PFC_VOLTAGE_BASE/32768.0f) 
/*  Define the base value of inductor current
		Shunt Resistor (R_shunt)              		= 0.015Ohm
		Operational amplifier gain (OpAmp_Gain)		= 3.1kOhm/(560Ohm+39Ohm) 
                                                    = 5.1753
		Maximum ADC input voltage (Vinadc)          = 1.65V
                                 Since   Vinadc     = Ibase*R_shunt*OpAmp_Gain
                                         Ibase      = Vinadc/(R_shunt*OpAmp_Gain)
                                         Vbase      = 1.65 / (0.015 * 5.1753)
                                                    = 21.3 A
		Therefore, maximum current or base current  = 22A */ 
#define PFC_INPUT_MAX_CURRENT           22.0f
        
#define ADC_CURRENT_SCALE               (float)(PFC_INPUT_MAX_CURRENT/32768.0f)

/* ===== Load-current feed-forward ==================================
 * IL2 = load (output) current, measured after the DC-link capacitor on the
 * load path. Adding the measured load power (Vdc * I_load) to the voltage-loop
 * power command lets the converter answer load steps immediately instead of
 * waiting for the slow voltage loop, and cancels the load disturbance in the
 * DC-bus power balance. DISABLED by default - verify the measurement and tune
 * the gain on the bench / SiL before enabling. */

/* Load-sensor ADC->Amp scale. VERIFY against the actual load-current front-end
 * (shunt/amplifier) - it is NOT necessarily the inductor-current sensor. The
 * placeholder equals the inductor scale, which is also what the SiL model uses
 * to inject Iout, so this default is correct for SiL and must be checked for HW. */
#define PFC_LOAD_CURRENT_SCALE          ADC_CURRENT_SCALE

/* First-order IIR coefficient (0..1) for the load current. Higher = faster and
 * noisier; 0.05 at the 64 kHz ISR gives a corner near ~500 Hz. */
#define PFC_LOAD_FF_FILT_COEFF          0.05f

/* Feed-forward gain. Deliberately BELOW 1.0 - do not set it to 1.0.
 *
 * For any load whose current rises with bus voltage (a resistive load being the
 * worst case), powerFF = gain*Vdc*Iload has the same Vdc dependence as the load
 * itself, so
 *      d(powerFF - Pout)/dVdc = (2*gain - 2) * Vdc / R
 * and at gain = 1.0 that is exactly ZERO: the feed-forward cancels the load's
 * own self-regulation and the bus loses its restoring term. At the same time
 * the voltage PI has nothing left to do and sits against its zero clamp, so it
 * cannot trim downwards either.
 *
 * SiL 2026-07-27 at gain 1.0: 11.4 Hz limit cycle, 12.5 V p-p on vdcAVG.
 * (That run also exposed a genuine bug - burst control was testing
 * piVoltage.output instead of the total power command - now fixed in
 * PFC_BurstModeUpdate. Both were needed.)
 *
 * 0.9 leaves -0.197 W/V of damping and parks the voltage PI around 40 W,
 * clear of the PFC_MIN_POWER burst threshold, while still removing 90% of the
 * load step from the slow voltage loop. 0.8 (dip 4.81 V) is the proven-safe
 * value if margin matters more than the last volt. */
#define PFC_LOAD_FF_GAIN                0.9f

/* Runtime enable default: 1 = on. Verified in SiL 2026-07-26 - the injected
 * IL2 reads back as pfcCurrent2.iL to within 0.1% of the plant's Iout and
 * PFC_LOAD_CURRENT_SCALE matches Get_ADCBUF_PFC_IL2 in regs_stub.h. Set back
 * to 0 on hardware until the load-current front-end scale and sign are
 * measured on the bench. */
#define PFC_LOAD_FF_ENABLE_DEFAULT      1

/* Boost inductance in Henry. Must match the value used in the plant model
 * (L in mchp_pfc_foc_dsPIC33A_data.m). Used by the conduction mode detector
 * to predict the inductor current slopes.
 *
 * NOTE: if the boost choke is a swinging/powder core, L falls with current
 * and this single constant is only correct at one operating point. The mode
 * detector degrades gracefully (the boundary shifts slightly), but any
 * predictive duty computation added later would inherit the error directly. */
#define PFC_INDUCTANCE                  680e-6f

/* Ts/L, in A per Volt. Folded at compile time. */
#define PFC_TS_OVER_L                   (float)(PFC_LOOPTIME_SEC/PFC_INDUCTANCE)

/* 2L/Ts, in Volt per Amp. Used by the DCM branch of the duty feed-forward. */
#define PFC_TWO_L_OVER_TS               (float)(2.0f*PFC_INDUCTANCE/PFC_LOOPTIME_SEC)

/* ===== Duty feed-forward =========================================
 * Without it, the current PI's integrator has to synthesise the whole
 * (Vo-Vg)/Vo duty profile, which swings ~0.15..1.0 every half line cycle. An
 * integrator only moves at Ki*error, so tracking that ramp costs a SUSTAINED
 * current error of (dD/dt)/(KI_I/Ts) - measured at 0.8..1.3 A on a 2.2 A peak
 * reference in SiL at 375 W (31% THD, PF 0.933). Feeding the duty forward
 * leaves the PI to trim losses only.
 *
 * The feed-forward is conduction-mode dependent because the two plants differ:
 *   CCM  d = (Vo-Vg)/Vo                      (boostDutyRatio)
 *   DCM  d = sqrt(2L/Ts * Iref * D_ideal/Vg)
 * The two expressions are equal at the CCM/DCM boundary, so the composite is
 * continuous - which also removes the ~1 A current step seen at the boundary
 * crossing when the carried-over duty sat above D_ideal.
 */

/* Runtime enable default: 1 = feed-forward on. Held in PFC_T.dutyFFEnable so
 * it can be toggled from the debugger / X2C Scope without a rebuild. */
#define PFC_DUTY_FF_ENABLE_DEFAULT      1

/* Symmetric limit on the PI trim (piCurrent min/max output) once the
 * feed-forward carries the operating point. It only has to cover losses
 * (measured 0.003..0.013 in CCM) plus inductance/model error, so keep it tight
 * - this is now the current loop's anti-windup bound. Ignored when
 * dutyFFEnable is 0, where the PI reverts to a 0..PFC_MAX_DUTY output. */
#define PFC_DUTY_TRIM_MAX               0.25f

/* Floor applied to the rectified input voltage in the DCM feed-forward's
 * divisor, so it stays finite through the zero crossing. Because
 * currentReference is itself proportional to Vg, flooring the denominator
 * makes the feed-forward taper smoothly to zero rather than blow up. Keep it
 * small - above this the expression is exact. */
#define PFC_DUTY_FF_VG_MIN              1.0f

/* Selects how the mid-ON current sample is converted to a cycle average
 * before it reaches the current loop. Values are PFC_DCM_COMP_T (pfc.h):
 *
 *   0 = PFC_DCM_COMP_OFF        raw sample, no reconstruction (baseline)
 *   1 = PFC_DCM_COMP_RATIO      legacy factor = min(d1/D_ideal, 1)
 *   2 = PFC_DCM_COMP_VALLEY_EST valley-estimation mode detect, then factor
 *
 * This only sets the power-on default. The field is writable at run time
 * (pfcParam.sampleCorrectionEnable), so on hardware the three methods can be
 * switched from the debugger / X2C Scope without a rebuild. In SiL, change
 * this and rebuild the S-function. */
#define PFC_DCM_COMPENSATION_METHOD     2

/* PFC Fault Limits - Input voltage,Input current and Output voltages */
        
/* PFC Input over current limit in A (rms)*/
#define PFC_INPUT_OVER_CURRENT          12.0f

/* PFC maximum current-reference peak in A = sqrt(2) x ~10 Arms design input
   current. Kept below the 12 Arms / ~16.97 Apk software OCP trip
   (PFC_INPUT_OVER_CURRENT_PEAK) so the controller clamps before OCP fires. */
#define PFC_IREF_PEAK_MAX               14.14f

/* Small positive floor applied to the measured inductor current so it never
   reads negative (noise/offset) ahead of the current loop. */
#define PFC_IL_MIN                      0.0001f
        
/* Specify PFC Input Voltage Ranges in which PFC Control will start executing */ 
        
/* Specify PFC Input over voltage lower limit  in V  (rms) */
#define PFC_INPUT_OVER_VOLTAGE_LO       240.0f
        
/* Specify PFC Input over voltage upper limit  in V  (rms) */
#define PFC_INPUT_OVER_VOLTAGE_HI       255.0f
        
/* Specify PFC Input under voltage lower limit  in V (rms) */
#define PFC_INPUT_UNDER_VOLTAGE_LO      110.0f
        
/* Specify PFC Input under voltage upper limit in V  (rms) */
#define PFC_INPUT_UNDER_VOLTAGE_HI      130.0f	
        
/* Specify PFC DC over voltage limit in V */
#define PFC_OUTPUT_OVER_VOLTAGE         410.0f

/* Specify PFC DC over-voltage recovery (auto-clear) limit in V.
   Provides hysteresis below PFC_OUTPUT_OVER_VOLTAGE so an output OV fault only
   clears once the bus has bled down, not on the input-voltage recovery tests. */
#define PFC_OUTPUT_OVER_VOLTAGE_RECOVERY 395.0f
        
/* Specify PFC DC under voltage limit in V */
#define PFC_OUTPUT_UNDER_VOLTAGE        310.0f

/* Specify PFC DC under-voltage recovery (auto-clear) limit in V.
   Hysteresis above the PFC_OUTPUT_UNDER_VOLTAGE trip so a bus-collapse fault
   clears only once the bus has climbed back up. NOTE: with PWM disabled the bus
   is passively capped at the rectified input peak (~325 V at 230 Vrms), so this
   MUST stay below that peak for auto-recovery to occur - hence 320 V (a 10 V
   band) rather than a full 15 V mirror of the OV band. At lower line the bus
   cannot passively reach this level, so the fault holds off until the line/bus
   recovers. Tune to your nominal input. */
#define PFC_OUTPUT_UNDER_VOLTAGE_RECOVERY 320.0f
     
/* Specify PFC output voltage reference in V */
#define PFC_OUPUT_VOLTAGE_NOMINAL       380.0f

/* Precharge complete threshold, as a FRACTION of the rectified input peak
 * (sqrt(2)*Vrms, taken from vacRMS.sqrOutput).
 *
 * The bus charges passively through the diode bridge + inrush resistor and
 * asymptotes at the rectified peak minus the bridge drops (~321 V measured at
 * 230 Vrms, i.e. 98.7% of the ideal 325.3 V peak). Closing the relay short-
 * circuits the inrush resistor across whatever gap is left, so the surge is
 * proportional to that gap: SiL 2026-07-26 showed the old fixed 280 V
 * threshold closing 33 V short of the asymptote and drawing 23 A.
 *
 * A fraction rather than a fixed voltage because a fixed one cannot serve the
 * declared input range: at PFC_INPUT_UNDER_VOLTAGE_LO (110 Vrms) the peak is
 * only 156 V, so the old 280 V could never be reached and precharge would hang
 * below ~200 Vrms.
 *
 * 0.97 leaves ~7 V of margin to the asymptote at 230 Vrms and ~2.5 V at
 * 110 Vrms - the bridge drop is a fixed offset, so the margin shrinks with
 * line but stays positive. Raising this toward 1.0 shrinks the surge further
 * but risks precharge never completing. */
#define PFC_PRECHARGE_PEAK_FRACTION     0.97f
        
/* Specify Soft start ramp rate and ramp count .This is specified at the rate 
of PFC control loop execution rate  */
#define RAMP_COUNT                      (float)(PFC_VOLTAGE_BASE/32768.0f)
#define RAMP_RATE                       5

/* Define minimum PFC voltage control output at which PWM duty is applied to 
    the boost power converter. 
 * This implements burst control at very low load.*/   
        
#define PFC_MIN_POWER                  1.0f        

/* Run DiagnosticsStepIsr() (X2C scope) once every N ADC ISRs. At the 64 kHz
   ISR this decimates the scope update to 64 kHz / 4 = 16 kHz. */
#define PFC_DIAGNOSTICS_DECIMATION      4
/* Voltage-loop execution rate divisor: the voltage PI runs once every
 * VOLTAGE_LOOP_EXE_RATE current-loop (ISR) ticks. At the 64 kHz ISR:
 *     64 kHz / 12 = 5.33 kHz  (current tuning point - KP_V/KI_V are set for it).
 * The gate is (++counter >= VOLTAGE_LOOP_EXE_RATE), so the divisor is exactly
 * this value. Set to 16 for the originally-documented 64 kHz / 16 = 4 kHz, but
 * that slows the loop and invalidates the existing KP_V/KI_V - re-tune first. */
#define VOLTAGE_LOOP_EXE_RATE           12
           
/* KMUL is used as a scaling constant    
 */ 
#define KMUL                            1.0f
    
/* Define PFC PI parameters */      
/** PFC Current loop Coefficients */
//#define KP_I                            0.071959f
//#define KI_I                            0.0045213f
#define KP_I   0.036f      // was 0.071959f
#define KI_I   0.0022607f  // was 0.0045213f
#define PI_I_OUT_MAX                    PFC_MAX_DUTY
        
/** Voltage  loop Coefficients */
#define KP_V                            20.199f
#define KI_V                            0.0992f 

/* Voltage-error hysteresis band (V) for gain-scheduling the voltage-PI integral
   term. |error| above HI halves Ki (fast large-transient response); |error|
   below LO restores full Ki; inside [LO, HI] the last setting is held so the
   gain cannot dither tick-to-tick at the switch point (limit-cycle risk).
   Centred on the original 10 V step with a +/-2 V band. */
#define PFC_VOLTAGE_ERR_GAIN_HI         12.0f
#define PFC_VOLTAGE_ERR_GAIN_LO         8.0f
#define PI_V_OUT_MAX                    1500

// </editor-fold>
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __PFC_PARAMS_H
    