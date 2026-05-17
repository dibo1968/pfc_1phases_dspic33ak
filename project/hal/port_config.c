// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file port_config.c
 *
 * @brief This module initializes the GPIO pins as analog/digital,input or 
 * output etc. Also to PPS functionality to Re-mappable input or output pins.
 * 
 * Definitions in this file are for dsPIC33AK128MC106
 *
 * Component: PORTS
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

#include "port_config.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: SetupGPIOPorts() </B>
*
* @brief Function initialize all ports as input and digital pins
*        
* @param none.
* @return none.
* 
* @example
* <CODE> SetupGPIOPorts(); </CODE>
*
*/
void SetupGPIOPorts(void)
{
    /* Reset all PORTx register (all inputs) */
    
    #ifdef TRISA
        TRISA = 0xFFFF;
        LATA  = 0x0000;
    #endif
    #ifdef ANSELA
        ANSELA = 0x0000;
    #endif

    #ifdef TRISB
        TRISB = 0xFFFF;
        LATB  = 0x0000;
    #endif
    #ifdef ANSELB
        ANSELB = 0x0000;
    #endif

    #ifdef TRISC
        TRISC = 0xFFFF;
        LATC  = 0x0000;
    #endif
    #ifdef ANSELC
        ANSELC = 0x0000;
    #endif

    #ifdef TRISD
        TRISD = 0xFFFF;
        LATD  = 0x0000;
    #endif
    #ifdef ANSELD
        ANSELD = 0x0000;
    #endif

    MapGPIOHWFunction();

}
/**
* <B> Function: MapGPIOHWFunction() </B>
*
* @brief Function maps port pins as input or output, analog or digital
*        
* @param none.
* @return none.
* 
* @example
* <CODE> MapGPIOHWFunction(); </CODE>
*
*/
/* DC Bus Voltage (Vdc)   : PIN #02: AD1AN6/RP8/IOMF1/RA7*/
/* Input AC Voltage (Vac) : PIN #30: AD2ANN2/AD2AN8/RP24/IOMF0/RB7*/
/* Inductor Current (IL)  : PIN #29: AD1ANN2/AD1AN8/RP23/RB6 */
/* Inductor Current (IL2) : PIN #03: PGC2/DACOUT1/AD1AN7/AD2AN3/CMP1D/CMP2D/CMP3D/RP2/SCL2/RA1 */
/* PFC Enable pin         : PIN #42: RP38/PWM4L/RC5 */
/* PWM4H                  : PIN #43: PGEC3/RP35/PWM4H/RC2         */
/* LED1                   : PIN #55: RP54/ASCL1/RD5   */
/* Push button SW1        : PIN #49: RP58/IOMF7/RD9   */
/* SW2 : DIM:036 -        : PIN #50 : RP59/RD10  */

/*dibo a adaugat in plus*/
 /*UART_RX :              : PIN #46 : RP44/IOMD8/IOMF8/RC11 (Input)
 /*UART_TX :              : PIN #45 : RP43/IOMD9/IOMF9/RC10(Output)   */
/*Inrush relay            : PIN #36: RP40/RC7*/
void MapGPIOHWFunction(void)
{  
      /* ANALOG SIGNALS */
    /* DC Bus Voltage (Vdc) : PIN 02: AD1AN6/RP8/IOMF1/RA7*/
    ANSELAbits.ANSELA7  = 1;
    TRISAbits.TRISA7    = 1;
    
    /* Input AC Voltage (Vac) : PIN 30: AD2ANN2/AD2AN8/RP24/IOMF0/RB7*/
    ANSELBbits.ANSELB7  = 1;
    TRISBbits.TRISB7    = 1;
    
    /* Inductor Current (IL) : PIN 29: AD1ANN2/AD1AN8/RP23/RB6 */
    ANSELBbits.ANSELB6  = 1;
    TRISBbits.TRISB6    = 1;
    /* Inductor Current (IL2) : PIN 15 OA3OUT/AD1AN3/CMP3A/RP6/RA5 */
    ANSELAbits.ANSELA5  = 1;
    TRISAbits.TRISA5    = 1;

    //PFC Enable pin : PIN 42: RP38/PWM4L/RC5 */
    TRISCbits.TRISC5   = 0; 
    
    /* Digital SIGNALS */   

    /* PFC Control - PWM Outputs   
     * PWM4H : PIN #43  PGEC3/RP35/PWM4H/RC2         */  
    TRISCbits.TRISC2 = 0 ;
     
    /* Debug LEDs */
    /* LED1 : DIM:030 - PIN #55 : RP54/ASCL1/RD5   */
    TRISDbits.TRISD5 = 0;
    /* LED2 : DIM:032 - PIN #34 : RP42/IOMD10/SDO2/IOMF10/PCI19/RC9  */
    TRISCbits.TRISC9 = 0;

    /* Push button Switches */
    /* SW1 : DIM:034 - PIN #49 : RP58/IOMF7/RD9   */
    TRISDbits.TRISD9 = 1;            
    /* SW2 : DIM:036 - PIN #50 : RP59/RD10  */
    TRISDbits.TRISD10 = 1;
    /* Inrush relay : PIN #36: RP40/RC7*/
    TRISCbits.TRISC7 = 1;
	
    /* Configuring FLTLAT_OC_OV (DIM:040) - Pin #32 : RP28/SDI2/RB11 as PCI9 */
	_PCI9R = 28;
    
	/** Diagnostic Interface
        Re-map UART Channels to the device pins connected to the following 
        pins on the Motor Control Development Board.
        UART_RX : DIM:054 - PIN #46 : RP44/IOMD8/IOMF8/RC11 (Input)
        UART_TX : DIM:052 - PIN #45 : RP43/IOMD9/IOMF9/RC10(Output)   */
    
    _U1RXR = 44;
    _RP43R = 9;
    
}
// </editor-fold>