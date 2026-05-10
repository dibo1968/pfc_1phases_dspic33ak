// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file adc.h
 *
 * @brief This header file lists the functions and definitions - to configure 
 * and enable ADC Module and its features 
 * 
 * Definitions in the file are for dsPIC33AK128MC106 MC DIM plugged onto 
 * Motor Control Development board from Microchip
 *
 * Component: ADC
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

#ifndef _ADC_H
#define _ADC_H
        
// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
        
#include <xc.h>
#include <stdint.h>
#include <math.h>        
        
// </editor-fold>

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
// <editor-fold defaultstate="collapsed" desc="DEFINITIONS/CONSTANTS ">
        
#define ADCBUF_PFC_IL       (int16_t)(AD1CH1DATA - 2048)<<4 // Range Changed : -32767 to 32767
#define ADCBUF_PFC_VAC      (int16_t)(AD2CH0DATA - 2048)<<4 // Range Changed : -32767 to 32767      
#define ADCBUF_VDC          (uint16_t)(AD1CH0DATA)<<4    // Range Changed : 0 to 65535 

/* IL (AD1CH2) is the ADC Interrupt source */
#define EnablePFCADCInterrupt()     _AD1CH1IE = 1
#define DisablePFCADCInterrupt()    _AD1CH1IE = 0        
#define ClearPFCADCIF()             _AD1CH1IF = 0   
#define ClearPFCADCIF_ReadADCBUF()  AD1CH1DATA        
#define PFC_ADCInterrupt            _AD1CH1Interrupt          

// </editor-fold>       

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">        
void InitializeADCs(void);

// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of ADC_H

