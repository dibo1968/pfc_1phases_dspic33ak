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
        
#define MC1_PWM_PDC1      PG1DC
#define MC1_PWM_PDC2      PG2DC
#define MC1_PWM_PDC3      PG3DC
        
#define MC1_PWM_PHASE1    PG1PHASE
#define MC1_PWM_PHASE2    PG2PHASE
#define MC1_PWM_PHASE3    PG3PHASE

#define INVERTERA_PWM_TRIGA     PG1TRIGA   
#define INVERTERA_PWM_TRIGB     PG1TRIGB 
#define INVERTERA_PWM_TRIGC     PG1TRIGC 
/* Define to enable PWM Fault PCI*/        
#define ENABLE_PWM_FAULT_PCI
        
#define PCI_FAULT_ACTIVE_STATUS      PG1STATbits.FLTACT        
#define _PWMInterrupt               _PWM1Interrupt
#define ClearPWMIF()                _PWM1IF = 0 
#define EnablePWMIF()               _PWM1IE = 1
#define DisablePWMIF()              _PWM1IE = 0
 
// PFC PWM MODULE Related Definitions 
#define PFC_PWM_PDC                 PG4DC 
        
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
        
//#define SYNC_COUNT                  (uint32_t)(0.2*PFC_LOOPTIME_TCY) 
#define SYNC_COUNT                  (uint32_t)(0)        
 /*Mention value of PWM Counter value at which MCPWM will be synchronized with PFC PWM*/       
#define PFC_Sync_MC_Value           SYNC_COUNT          

        
/*Specify PWM Switching Frequency in Hertz and period LOOPTIME_SEC in Seconds*/
#define PWMFREQUENCY_HZ                     16000
#define LOOPTIME_SEC                        0.0000625f
        
/*Specify PWM Module Clock in Mega Hertz*/
//#define PWM_CLOCK_MHZ                       400       
/* Specify PWM module dead time in micro seconds*/
#define DEADTIME_MICROSEC                   1.5f
/*Dead time in terms of PWM clock period*/  
#define DEADTIME                            (uint32_t)(DEADTIME_MICROSEC*16*PWM_CLOCK_MHZ)
/* Loop Time in micro seconds*/
#define LOOPTIME_MICROSEC                   (LOOPTIME_SEC * 1000000.0f)
/*Loop time in terms of PWM clock period*/
#define LOOPTIME_TCY                        (uint32_t)((LOOPTIME_MICROSEC*8*PWM_CLOCK_MHZ)-16)
/*Specify ADC Triggering Point w.r.t PWM Output for sensing Analog Inputs*/ 
#define ADC_SAMPLING_POINT                  0

        
/****Bootstrap Capacitor Charging Parameters*/
/*Specify bootstrap charging time in Seconds (mention at least 0.01Secs)*/
#define BOOTSTRAP_CHARGING_TIME_SECS        0.015
/*Specify bootstrap Capacitor Tickle Charge Time in Micro Seconds
 * Minimum Time = 1uSec and Maximum Time = 5uSec */
#define TICKLE_CHARGE_TIME_MICROSEC         1.0
/*Calculate Bootstrap charging time in number of PWM Half Cycles*/
#define BOOTSTRAP_CHARGING_COUNTS           (uint32_t)(BOOTSTRAP_CHARGING_TIME_SECS/LOOPTIME_SEC)
/*Calculate Bootstrap Capacitor Tickle Charge duty*/
#define TICKLE_CHARGE_DUTY                  (LOOPTIME_TCY - (uint32_t)(TICKLE_CHARGE_TIME_MICROSEC*16*PWM_CLOCK_MHZ))

#define MIN_DUTY            (uint32_t)(2.0*DEADTIME)
#define MAX_DUTY            LOOPTIME_TCY - (uint32_t)(2.0*DEADTIME)
// </editor-fold>      

// <editor-fold defaultstate="expanded" desc="INTERFACE FUNCTIONS ">
        
void InitPWMGenerators(void);  
void InitPWMGenerator1 (void);
void InitPWMGenerator2 (void);
void InitPWMGenerator3 (void);
void InitPWMGenerator4 (void);

void InitDutyPWM123Generators(void);
void InitPWMGenerators(void);   
void ChargeBootstrapCapacitors(void);
// </editor-fold>
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
    
#endif      // end of __PWM_H


