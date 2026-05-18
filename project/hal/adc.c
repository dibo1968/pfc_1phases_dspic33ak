// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file adc.c
 *
 * @brief This module configures and enables the ADC Module 
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include <stdint.h>

#include "adc.h"

// </editor-fold> 

// <editor-fold defaultstate="collapsed" desc="Function Declarations">
void InitializeADCs(void);

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">
/**
 * <B> Function: InitializeADCs()  </B>
 * @brief Function to configure ADC Module
 * @param None.
 * @return None.
 * @example
 * <code>
 * status = InitializeADCs();
 * </code>
 */
void InitializeADCs (void)
{
    
    /* Channel Configuration for IL*/
    /*ADC1 Channel 2 for IL : AD1ANN2/AD1AN8/RP23/RB6 */ 
    AD1CH1CONbits.PINSEL = 8;      
    AD1CH1CONbits.SAMC   = 3;      
    AD1CH1CONbits.LEFT   = 0;
    AD1CH1CONbits.DIFF   = 0;

    /* Channel Configuration for VAC*/
    /*ADC2 Channel 3 for VAC : AD2ANN2/AD2AN8/RP24/IOMF0/RB7 */ 
    AD2CH0CONbits.PINSEL = 8;        
    AD2CH0CONbits.SAMC   = 3;         
    AD2CH0CONbits.LEFT   = 0;
    AD2CH0CONbits.DIFF   = 0; 
    
    /* Channel Configuration for VDC*/
    /*ADC1 Channel 4 for VDC : AD1AN6/RP8/IOMF1/RA7 */ 
    AD1CH0CONbits.PINSEL = 6;        
    AD1CH0CONbits.SAMC   = 3;         
    AD1CH0CONbits.LEFT   = 0;
    AD1CH0CONbits.DIFF   = 0;
    
    /* Channel Configuration for IL2 */
    /* AD1AN3 :OA3OUT/AD1AN3/CMP3A/RP6/RA5*/
    /*AD1CH3CONbits.PINSEL = 3;
    AD1CH3CONbits.SAMC   = 3;
    AD1CH3CONbits.LEFT   = 0;
    AD1CH3CONbits.DIFF   = 0;*/
    
    /* Channel Configuration for IL2 */
    /* AD1AN7 : PGC2/DACOUT1/AD1AN7/AD2AN3/CMP1D/CMP2D/CMP3D/RP2/SCL2/RA1 */
    AD1CH3CONbits.PINSEL = 7;
    AD1CH3CONbits.SAMC   = 3;
    AD1CH3CONbits.LEFT   = 0;
    AD1CH3CONbits.DIFF   = 0;
	
    /* Turn on the ADC Core 1 */   
    AD1CONbits.ON = 1;     
    /* Waiting till the ADC Core 1 is ready*/
    while(AD1CONbits.ADRDY == 0);  
    
    /* Turn on the ADC Core 2 */
    AD2CONbits.ON = 1;             
    /* Waiting till the ADC Core 2 is ready*/
    while(AD2CONbits.ADRDY == 0);   
    

    /*AD1CH2 - IL used for ADC Interrupt in PFC*/
    /* Set ADC interrupt priority IPL 7  */ 
    _AD1CH1IP = 7;
    /* Clear ADC interrupt flag */
    _AD1CH1IF = 0;
    /* Disable the AD1CH2 interrupt  */
    _AD1CH1IE = 0;
    
    /* Trigger Source Selection for Corresponding Analog Inputs bits
    01011   =  PWM4 Trigger 2
    01010   =  PWM4 Trigger 1
    00000    = No trigger is enabled  */
    
    /*PWM4 ADC Trigger 2 for IL - AD1CH2*/
    AD1CH1CONbits.TRG1SRC = 0b01011;      
    /*PWM4 ADC Trigger 2 for VAC - AD2CH3*/
    AD2CH0CONbits.TRG1SRC = 0b01011;    
    /*PWM4 ADC Trigger 2 for VDC - AD1CH4*/
    AD1CH0CONbits.TRG1SRC = 0b01011;
    /* PWM4 ADC Trigger 2 for IL2 */
    AD1CH2CONbits.TRG1SRC = 0b01011;
    /* PWM4 ADC Trigger 2 for IL2 */
    AD1CH3CONbits.TRG1SRC = 0b01011;	
}
// </editor-fold> 