// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pwm.h
 *
 * @brief This header file lists the functions and definitions to configure the 
 * PWM Module
 * 
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: PWM
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

#ifndef _PWM_H
#define _PWM_H

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include <stdint.h>
        
#include "pfc_userparams.h"

// </editor-fold>

// <editor-fold defaultstate="expanded" desc="DEFINITIONS/CONSTANTS ">
        
// PFC PWM MODULE Related Definitions 
#define PFC_PWM_PDC                 PG4DC 

#undef ENABLE_PWM_FAULT        
#define PWM_FAULT_STATUS            PG1STATbits.FLTACT
        
#define _PWMInterrupt               _PWM4Interrupt
#define ClearPWMIF()                _PWM4IF = 0 
#define EnablePWMIF()               _PWM4IE = 1
#define DisablePWMIF()              _PWM4IE = 0
/*Specify PWM Module Clock in Mega Hertz*/
#define PWM_CLOCK_MHZ                       400 
//  PFC Period ,Duty related definitions 
        
/* Specify PFC PWM Frequency in Hertz */
#define PFC_PWMFREQUENCY_HZ         64000        
/* Specify PFC PWM Period in seconds, (1/ PWMFREQUENCY_HZ) */
#define PFC_LOOPTIME_SEC            0.000015625f
/* Specify PWM Period in micro seconds */
#define PFC_LOOPTIME_MICROSEC       15.625f          
/* Specify maximum duty cycle,Make Sure Maximum duty ratio is less than or equal
 * to 0.9 */
#define PFC_MAX_DUTY             0.95f   
/* Specify minimum duty cycle */
#define PFC_MIN_DUTY             0   
        
/* Loop time in terms of PWM clock period */
#define PFC_LOOPTIME_TCY            (uint32_t)((PWM_CLOCK_MHZ*8*PFC_LOOPTIME_MICROSEC)-16)
/* Specify ADC Triggering Point w.r.t PWM Output for sensing PFC parameters */
#define PFC_ADC_SAMPLING_POINT      (uint32_t)(PFC_LOOPTIME_TCY-50)

/* Minimum duty cycle in terms of PWM period */       
#define PFC_MIN_DUTY_COUNTS         (uint32_t)(PFC_MIN_DUTY*PFC_LOOPTIME_TCY)        
/* Maximum duty cycle in terms of PWM period */
#define PFC_MAX_DUTY_COUNTS         (uint32_t)(PFC_MAX_DUTY*PFC_LOOPTIME_TCY)

// </editor-fold>      

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
        
void InitPWMGenerators(void);   
void InitPWMGenerator4 (void);

// </editor-fold>
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
    
#endif      // end of __PWM_H


