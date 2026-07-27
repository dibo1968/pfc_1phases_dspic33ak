// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file main.c
 *
 * @brief This is the main entry to the application.
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
#include <stdbool.h>

#include <xc.h>

#include "board_service.h"
#include "diagnostics.h"
#include "pfc.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc=" Global Variables ">

/* TEMPORARY bring-up diagnostic - comment out to remove. Blinks LED2 from the
   main loop to prove the loop is still being scheduled. See the block in main(). */
#define MAIN_LOOP_ALIVE_BLINK

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: int main (void)  </B>
*
* @brief main() function,entry point of the application.
*
*/
int main (void)
{
    InitOscillator();
    SetupGPIOPorts();
    
#ifdef ENABLE_DIAGNOSTICS
    DiagnosticsInit();
#endif
    HAL_InitPeripherals();

    LED1 = 1;
#ifdef MAIN_LOOP_ALIVE_BLINK
    /* RD11 carries LD2 on the Digital Power PIM. ANSELD was already cleared in
       SetupGPIOPorts(), so only the direction needs setting. */
    TRISDbits.TRISD11 = 0;
#endif
    PFC_ServiceInit();
    
    while(1)
    {

#ifdef MAIN_LOOP_ALIVE_BLINK
        /* TEMPORARY bring-up diagnostic. LED1 is set once before this loop, so a
           lit LED1 does not prove the loop is still being scheduled - a toggling
           LED does. If nothing blinks, the 64 kHz ADC ISR is starving main() and
           X2CScope_Communicate() never runs.

           The LED pin differs by module, so drive all three candidates:
             RD5  - firmware LED1, "LD2" on the Motor Control DIM
             RC9  - firmware LED2, "LD3" on the Motor Control DIM
             RD11 - "LD2" (red) on the Digital Power PIM, which has no LED on
                    RD5 or RC9 at all. Its LD1 (green) is a power indicator and
                    is not driven by the MCU.
           Delete this block once the UART is up. */
        {
            static uint32_t aliveCount = 0;
            if (++aliveCount >= 200000)
            {
                LED1 = !LED1;
                LED2 = !LED2;
                LATDbits.LATD11 = !LATDbits.LATD11;
                aliveCount = 0;
            }
        }
#endif

#ifdef ENABLE_DIAGNOSTICS
        DiagnosticsStepMain();
#endif
        BoardService();
        if(IsPressed_Button1())
        {
            
        }
    }
}
/**
* <B> Function: _T1Interrupt  </B>
*
* @brief T1 Interrupt Vector
 * Executes BoardServiceStepIsr()
*
*/
void __attribute__((__interrupt__,__auto_psv__)) _T1Interrupt (void)
{

    BoardServiceStepIsr();
    TIMER1_InterruptFlagClear();
}
// </editor-fold>