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
#define ENABLE_PFC_CURRENT_OFFSET_CORRECTION

/** When defined, operates in power reference control. 
   That is the voltage PI output correspond to the input power. */
#define PFC_POWER_CONTROL   
        
/* Define PFC input AC voltage frequency in Hz */
#define PFC_INPUT_FREQUENCY             50.0f 
        
/* Define the input AC voltage frequency in terms of PWM clock period */       
#define PFC_INPUT_FREQUENCY_COUNTER     (PFC_PWMFREQUENCY_HZ/PFC_INPUT_FREQUENCY )
         
/* Define sample numbers for DC bus voltage average   
    * In this case, 128 samples or 2^7 samples are taken and averaged.
      The scaler is chosen as power of 2 ,such that shifting is done 
      to calculate average from the sum of samples.  */        
#define PFC_AVG_SCALER                  7   //2^7 = 128 samples average 
        
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
/* PFC Fault Limits - Input voltage,Input current and Output voltages */
        
/* PFC Input over current limit in A (rms)*/
#define PFC_INPUT_OVER_CURRENT          12.0f
        
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
        
/* Specify PFC DC over voltage limit in V */
#define PFC_OUTPUT_UNDER_VOLTAGE        310.0f
     
/* Specify PFC output voltage reference in V */
#define PFC_OUPUT_VOLTAGE_NOMINAL       380.0f
        
/* Specify Soft start ramp rate and ramp count .This is specified at the rate 
of PFC control loop execution rate  */
#define RAMP_COUNT                      (float)(PFC_VOLTAGE_BASE/32768.0f)
#define RAMP_RATE                       20

/* Define minimum PFC voltage control output at which PWM duty is applied to 
    the boost power converter. 
 * This implements burst control at very low load.*/   
        
#define PFC_MIN_POWER                  1.0f        
/* Define Voltage loop execution rate . 
    * This is specified in integral number of PFC current control loop execution 
      rate. 
        In this case , PFC control loop frequency is 64 kHz .
        Then voltage loop is executed at 64kHz/ VOLTAGE_LOOP_EXE_RATE = 64kHz/16 
                                                                      = 4kHz  */   
#define VOLTAGE_LOOP_EXE_RATE           10
           
/* KMUL is used as a scaling constant    
 */ 
#define KMUL                            1.0f
    
/* Define PFC PI parameters */      
/** PFC Current loop Coefficients */
#define KP_I                            0.071959f
#define KI_I                            0.0045213f
#define PI_I_OUT_MAX                    PFC_MAX_DUTY
        
/** Voltage  loop Coefficients */
#define KP_V                            20.199f
#define KI_V                            0.0992f 
#define PI_V_OUT_MAX                    1500

// </editor-fold>
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __PFC_PARAMS_H
    