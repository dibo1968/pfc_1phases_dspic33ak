// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pfc_pi.h
 *
 * @brief This module includes variables and function declaration required 
 * for implementation of PI controller
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

#ifndef __PFC_PI_H
#define __PFC_PI_H

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">
    
#ifdef __XC16__  // See comments at the top of this header file
    #include <xc.h>
#endif // __XC16__


// </editor-fold> 

#ifdef __cplusplus  // Provide C++ Compatability
    extern "C" {
#endif
        
// <editor-fold defaultstate="collapsed" desc="CONSTANTS/DEFINTIONS ">
                       
// </editor-fold>         
                
// <editor-fold defaultstate="collapsed" desc="VARIABLE TYPES">
/**
 * State variables related to PI controller 
 */    
typedef struct
{
    float output;         /* PI controller output */        
    float integralOut;    /* Integral output of PI controller*/ 
    float propOut;        /* Proportional output of PI controller*/
    
    float input;          /* Measured signal to the PI controller */
    float reference;      /* Reference signal to the PI controller */
    float error;          /* Error signal to the PI controller 
                             * (Reference - measured)*/

    float kp;             /* Proportional gain coefficient term */
    float ki;             /* Integral gain coefficient term */
    float kpScale;        /* Normalizing term for proportional coefficient */
    float kiScale;        /* Normalizing term for integral coefficient */
    float minOutput;      /* Minimum output limit */      
    float maxOutput;      /* Maximum output limit */
}PFC_PI_T;  

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">

extern void PFC_ControllerPIUpdate(PFC_PI_T *);   
    
// </editor-fold> 
        
#ifdef __cplusplus  // Provide C++ Compatibility
    }
#endif
#endif      // end of __PFC_PI_H
    