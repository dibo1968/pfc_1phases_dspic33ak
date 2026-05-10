// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file traps.c
 *
 * @brief This file has interrupt vectors for handling traps and 
 * _Defualt_Interrupt.
 *
 * Component: APPLICATION
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

#include <stdint.h>

#include "xc.h"
#include "board_service.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">
/*
Primary Exception Vector handlers:
These routines are used if INTCON2bits.ALTIVT = 0.
All trap service routines in this file simply ensure that device
continuously executes code within the trap service routine. Users
may modify the basic framework provided here to suit to the needs
of their application.
*/
void __attribute__((__interrupt__,no_auto_psv)) _OscillatorFail(void)
{
    HAL_TrapHandler();
    while (1);
}

void __attribute__((__interrupt__,no_auto_psv)) _AddressError(void)
{
    HAL_TrapHandler();
    while (1);
}
void __attribute__((__interrupt__,no_auto_psv)) _StackError(void)
{
    HAL_TrapHandler();
    while (1);
}

void __attribute__((__interrupt__,no_auto_psv)) _MathError(void)
{
    HAL_TrapHandler();
    while (1);
}
void __attribute__((__interrupt__,no_auto_psv)) _HardTrapError(void)
{
    HAL_TrapHandler();
    while (1);
}

void __attribute__((__interrupt__,no_auto_psv)) _SoftTrapError(void)
{
    HAL_TrapHandler();
    while (1);
}

void __attribute__((__interrupt__,no_auto_psv)) _DefaultInterrupt(void)
{
    HAL_TrapHandler();
    while (1);
}
// </editor-fold>



