// <editor-fold defaultstate="collapsed" desc="Description/Instruction ">
/**
 * @file pwm.c
 *
 * @brief This module configures and enables the PWM Module 
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

// <editor-fold defaultstate="collapsed" desc="HEADER FILES ">

#include <xc.h>
#include <stdint.h>

#include "pwm.h"
#include "pfc_userparams.h"

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="INTERFACE FUNCTIONS ">

/**
* <B> Function: InitPWMGenerators()    </B>
*
* @brief Function initializes the registers common for all PWM modules and 
* configures individual PWM module. The function enables the PWM module 
* and completes the boot strap charging sequence
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerators();     </CODE>
*
*/
void InitPWMGenerators(void)
{

    PCLKCON      = 0x0000;
    /* PWM Clock Divider Selection bits
       0b11 = 1:16 ; 0b10 = 1:8 ;0b01 = 1:4 ; 0b00 = 1:2*/
    PCLKCONbits.DIVSEL = 0;
    /* PWM Master Clock Selection bits
       1 = CLKGEN5 ; 0 = UPB clock */
    PCLKCONbits.MCLKSEL = 1;
    /* Lock bit: 0 = Write-protected registers and bits are unlocked   */
    PCLKCONbits.LOCK = 0;

    /* Initialize Master Phase Register */
    MPHASE       = 0x0000;
    /* Initialize Master Duty Cycle */
    MDC          = 0x0000;
    /* Initialize Master Period Register */
    MPER         = 0x0000;
    
    /* Initialize FREQUENCY SCALE REGISTER*/
    FSCL        = 0x0000;
    /* Initialize FREQUENCY SCALING MINIMUM PERIOD REGISTER */
    FSMINPER    = 0x0000;
    /* Initialize Linear Feedback Shift Register */
    LFSR        = 0x0000;
    /* Initialize Combinational Trigger Register Low */
    CMBTRIG     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 1 LOW */
    LOGCONA     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 1 HIGH */
    LOGCONB     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 2 LOW */
    LOGCONC     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 2 HIGH */
    LOGCOND     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 3 LOW */
    LOGCONE     = 0x0000;
    /* Initialize LOGIC CONTROL REGISTER 3 High */
    LOGCONF     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER A */
    PWMEVTA     = 0x0000;     
    /* PWM EVENT OUTPUT CONTROL REGISTER B */
    PWMEVTB     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER C */
    PWMEVTC     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER D */
    PWMEVTD     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER E */
    PWMEVTE     = 0x0000;
    /* PWM EVENT OUTPUT CONTROL REGISTER F */
    PWMEVTF     = 0x0000;
    
    /* Function call to Initialize individual PWM modules*/
    InitPWMGenerator4 ();

    /* Clearing and disabling PWM Interrupt */
    _PWM4IF = 0;
    _PWM4IE = 0;
    _PWM4IP = 7;
    
    /* Enable the PWM module after initializing generators*/
	PG4CONbits.ON = 1;         
}



/**
* <B> Function: InitPWMGenerator4()    </B>
*
* @brief Function to configure PWM Module # 3
*        
* @param none.
* @return none.
* 
* @example
* <CODE> InitPWMGenerator4();     </CODE>
*
*/
void InitPWMGenerator4 (void)
{

    /* Initialize PWM GENERATOR 3 CONTROL REGISTER LOW */
    PG4CON      = 0x0000;
    /* PWM Generator 3 Enable bit : 1 = Is enabled, 0 = Is not enabled */
    /* PWM Generator is disabled prior to configuring module */
    PG4CONbits.ON = 0;
    /* Clock Selection bits
       0b01 = Macro uses Master clock selected by the PCLKCON.MCLKSEL bits*/
    PG4CONbits.CLKSEL = 1;
    /* PWM Mode Selection bits
     * 110 = Dual Edge Center-Aligned PWM mode (interrupt/register update once per cycle)
       100 = Center-Aligned PWM mode(interrupt/register update once per cycle)*/
    PG4CONbits.MODSEL = 4;   
    /* Trigger Count Select bits
       000 = PWM Generator produces 1 PWM cycle after triggered */
    PG4CONbits.TRGCNT = 0;
    
    /* Initialize PWM GENERATOR 4 CONTROL REGISTER HIGH */
    /* Master Duty Cycle Register Select bit
       1 = Macro uses the MDC register instead of PG4DC
       0 = Macro uses the PG4DC register*/
    PG4CONbits.MDCSEL = 0;
    /* Master Period Register Select bit
       1 = Macro uses the MPER register instead of PG4PER
       0 = Macro uses the PG4PER register */
    PG4CONbits.MPERSEL = 0;
    /* MPHSEL: Master Phase Register Select bit
       1 = Macro uses the MPHASE register instead of PG4PHASE
       0 = Macro uses the PG4PHASE register */
    PG4CONbits.MPHSEL = 0;
    /* Master Update Enable bit
       1 = PWM Generator broadcasts software set/clear of UPDATE status bit and 
           EOC signal to other PWM Generators
       0 = PWM Generator does not broadcast UPDATE status bit or EOC signal */
    PG4CONbits.MSTEN = 0;
    /* PWM Buffer Update Mode Selection bits 
       0b010 = Slaved SOC Update Data registers at start of next cycle if a 
       master update request is received. A master update request will be 
       transmitted if MSTEN = 1 and UPDATE = 1 for the requesting PWM
       Generator.. */
	PG4CONbits.UPDMOD = 0b000;
    /* PWM Generator Trigger Mode Selection bits
       0b00 = PWM Generator operates in Single Trigger mode */
    PG4CONbits.TRGMOD = 0;
    /* Start of Cycle Selection bits
       0000 = Local EOC*/
    PG4CONbits.SOCS = 0;
    
    /* Clear PWM GENERATOR 3 STATUS REGISTER*/
    PG4STAT      = 0x0000;
    /* Initialize PWM GENERATOR 3 I/O CONTROL REGISTER LOW */
    PG4IOCON    = 0x0000;

    /* Current Limit Mode Select bit
       0 = If PCI current limit is active, then the CLDAT<1:0> bits define 
       the PWM output levels */
    PG4IOCONbits.CLMOD = 0;
    /* Swap PWM Signals to PWM3H and PWM3L Device Pins bit 
       0 = PWM3H/L signals are mapped to their respective pins */
    PG4IOCONbits.SWAP = 0;
    /* User Override Enable for PWM3H Pin bit
       0 = PWM Generator provides data for the PWM3H pin*/
    PG4IOCONbits.OVRENH = 0;
    /* User Override Enable for PWM3L Pin bit
       0 = PWM Generator provides data for the PWM3L pin*/
    PG4IOCONbits.OVRENL = 0;
    /* Data for PWM3H/PWM3L Pins if Override is Enabled bits
       If OVERENH = 1, then OVRDAT<1> provides data for PWM3H.
       If OVERENL = 1, then OVRDAT<0> provides data for PWM3L */
    PG4IOCONbits.OVRDAT = 0;
    /* User Output Override Synchronization Control bits
       00 = User output overrides via the OVRENL/H and OVRDAT<1:0> bits are 
       synchronized to the local PWM time base (next start of cycle)*/
    PG4IOCONbits.OSYNC = 0;
    /* Data for PWM3H/PWM3L Pins if FLT Event is Active bits
       If Fault is active, then FLTDAT<1> provides data for PWM3H.
       If Fault is active, then FLTDAT<0> provides data for PWM3L.*/
    PG4IOCONbits.FLTDAT = 0;
    /* Data for PWM3H/PWM3L Pins if CLMT Event is Active bits
       If current limit is active, then CLDAT<1> provides data for PWM3H.
       If current limit is active, then CLDAT<0> provides data for PWM3L.*/
    PG4IOCONbits.CLDAT = 0;
    /* Data for PWM3H/PWM3L Pins if Feed-Forward Event is Active bits
       If feed-forward is active, then FFDAT<1> provides data for PWM3H.
       If feed-forward is active, then FFDAT<0> provides data for PWM3L.*/
    PG4IOCONbits.FFDAT = 0;
    /* Data for PWM3H/PWM3L Pins if Debug Mode is Active and PTFRZ = 1 bits
       If Debug mode is active and PTFRZ=1,then DBDAT<1> provides PWM3H data.
       If Debug mode is active and PTFRZ=1,then DBDAT<0> provides PWM3L data. */
    PG4IOCONbits.DBDAT = 0;
    
    /* Initialize PWM GENERATOR 3 I/O CONTROL REGISTER HIGH */    
    /* Time Base Capture Source Selection bits
       000 = No hardware source selected for time base capture ? software only*/
    PG4IOCONbits.CAPSRC = 0;
    /* Dead-Time Compensation Select bit 
       0 = Dead-time compensation is controlled by PCI Sync logic */
    PG4IOCONbits.DTCMPSEL = 0;
    /* PWM Generator Output Mode Selection bits
       00 = PWM Generator outputs operate in Complementary mode*/
    PG4IOCONbits.PMOD = 1;
    /* PWM3H Output Port Enable bit
       1 = PWM Generator controls the PWM3H output pin
       0 = PWM Generator does not control the PWM3H output pin */
    PG4IOCONbits.PENH = 1;
    /* PWM3L Output Port Enable bit
       1 = PWM Generator controls the PWM3L output pin
       0 = PWM Generator does not control the PWM3L output pin */
    PG4IOCONbits.PENL = 0;
    /* PWM3H Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG4IOCONbits.POLH = 0;
    /* PWM3L Output Polarity bit
       1 = Output pin is active-low
       0 = Output pin is active-high*/
    PG4IOCONbits.POLL = 0;
    
    /* Initialize PWM GENERATOR 3 EVENT REGISTER LOW*/
    PG4EVT      = 0x0000;
    /* ADC Trigger 1 Post-scaler Selection bits
       00000 = 1:1 */
    PG4EVTbits.ADTR1PS = 0;
    /* ADC Trigger 1 Source is PG4TRIGC Compare Event Enable bit
       0 = PG4TRIGC register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG4EVTbits.ADTR1EN3  = 0;
    /* ADC Trigger 1 Source is PG4TRIGB Compare Event Enable bit
       0 = PG4TRIGB register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG4EVTbits.ADTR1EN2 = 0;
    /* ADC Trigger 1 Source is PG4TRIGA Compare Event Enable bit
       0 = PG4TRIGA register compare event is disabled as trigger source for 
           ADC Trigger 1 */
    PG4EVTbits.ADTR1EN1 = 0;
    /* Update Trigger Select bits
       01 = A write of the PG4DC register automatically sets the UPDATE bit
       00 = User must set the UPDATE bit manually*/
    PG4EVTbits.UPDTRG = 1;
    /* PWM Generator Trigger Output Selection bits
       000 = EOC event is the PWM Generator trigger*/
    PG4EVTbits.PGTRGSEL = 0;
    
    /* Initialize PWM GENERATOR 3 EVENT REGISTER HIGH */
    /* FLTIEN: PCI Fault Interrupt Enable bit
       0 = Fault interrupt is disabled */
    PG4EVTbits.FLTIEN = 0;
    /* PCI Current Limit Interrupt Enable bit
       0 = Current limit interrupt is disabled */
    PG4EVTbits.CLIEN = 0;
    /* PCI Feed-Forward Interrupt Enable bit
       0 = Feed-forward interrupt is disabled */
    PG4EVTbits.FFIEN = 0;
    /* PCI Sync Interrupt Enable bit
       0 = Sync interrupt is disabled */
    PG4EVTbits.SIEN = 0;
    /* Interrupt Event Selection bits
       00 = Interrupts CPU at EOC
       01 = Interrupts CPU at TRIGA compare event
       10 = Interrupts CPU at ADC Trigger 1 event
       11 = Time base interrupts are disabled */
    PG4EVTbits.IEVTSEL = 3;
    /* ADC Trigger 3 Source is PG4TRIGC Compare Event Enable bit
       0 = PG4TRIGC register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG4EVTbits.ADTR2EN3 = 0;
    /* ADC Trigger 2 Source is PG4TRIGB Compare Event Enable bit
       0 = PG4TRIGB register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG4EVTbits.ADTR2EN2 = 0;
    /* ADC Trigger 2 Source is PG4TRIGA Compare Event Enable bit
       0 = PG4TRIGA register compare event is disabled as 
           trigger source for ADC Trigger 2 */
    PG4EVTbits.ADTR2EN1 = 1;
    /* ADC Trigger 1 Offset Selection bits
       00000 = No offset */
    PG4EVTbits.ADTR1OFS = 0;
    
    /* PWM GENERATOR 3 Fault PCI REGISTER LOW */
#ifndef ENABLE_PWM_FAULT
    /* PWM GENERATOR 3 Fault PCI REGISTER LOW */
    PG4FPCI     = 0x0000;
#else
       /* PWM GENERATOR 3 Fault PCI REGISTER LOW */
    PG4FPCI     = 0x0000;
    /* Termination Synchronization Disable bit
       1 = Termination of latched PCI occurs immediately
       0 = Termination of latched PCI occurs at PWM EOC */
    PG4FPCIbits.TSYNCDIS = 0;
    /* Termination Event Selection bits
       001 = Auto-Terminate: Terminate when PCI source transitions from 
             active to inactive */
    PG4FPCIbits.TERM = 1;
    /* Acceptance Qualifier Polarity Select bit: 0 = Not inverted 1 = Inverted*/
    PG4FPCIbits.AQPS = 0;
    /* Acceptance Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to 0)
       110 = Selects PCI Source #9
       101 = Selects PCI Source #8
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)        
       000 = No acceptance qualifier is used (qualifier forced to 1) */
    PG4FPCIbits.AQSS = 0;
    /* PCI Synchronization Control bit
       1 = PCI source is synchronized to PWM EOC
       0 = PCI source is not synchronized to PWM EOC*/
    PG4FPCIbits.PSYNC = 0;
    /* PCI Polarity Select bit 0 = Not inverted 1 = Inverted */
    PG4FPCIbits.PPS = 1;
    /* PCI Source Selection bits
       11111 = PCI Source #31
       ? ?
       00001 = PCI Source #1
       00000 = Software PCI control bit (SWPCI) only*/
    PG4FPCIbits.PSS = 9;
    /* PCI Bypass Enable bit
       0 = PCI function is not bypassed */
    PG4FPCIbits.BPEN   = 0;
    /* PCI Bypass Source Selection bits(1)
       000 = PCI control is sourced from PG1 PCI logic when BPEN = 1 */
    PG4FPCIbits.BPSEL   = 0;
    /* PCI Termination Polarity Select bits 
       0 = Inverter, 1 = Non Inverted */
    PG4FPCIbits.TERMPS = 0;
    /* PCI Acceptance Criteria Selection bits
       101 = Latched any edge(2)
       100 = Latched rising edge
       011 = Latched
       010 = Any edge
       001 = Rising edge
       000 = Level-sensitive*/
    PG4FPCIbits.ACP   = 3;
    /* PCI SR Latch Mode bit
       1 = SR latch is Reset-dominant in Latched Acceptance modes
       0 = SR latch is Set-dominant in Latched Acceptance modes*/
    PG4FPCIbits.LATMOD  = 0;
    /* Termination Qualifier Polarity Select bit 1 = Inverted 0 = Not inverted*/
    PG4FPCIbits.TQPS   = 0;
    /* Termination Qualifier Source Selection bits
       111 = SWPCI control bit only (qualifier forced to '0')(3)
       110 = Selects PCI Source #9 (pwm_pci[9] input port)
       101 = Selects PCI Source #8 (pwm_pci[8] input port)
       100 = Selects PCI Source #1 (PWM Generator output selected by the PWMPCI<2:0> bits)
       011 = PWM Generator is triggered
       010 = LEB is active
       001 = Duty cycle is active (base PWM Generator signal)
       000 = No termination qualifier used (qualifier forced to '1')(3)*/
    PG4FPCIbits.TQSS  = 0;
#endif  
    
    /* PWM GENERATOR 3 Current Limit PCI REGISTER LOW */
    PG4CLPCI    = 0x0000;
    /* PWM GENERATOR 3 Feed Forward PCI REGISTER LOW */
    PG4FFPCI    = 0x0000;
    /* PWM GENERATOR 3 Sync PCI REGISTER LOW */
    PG4SPCI     = 0x0000;
    
    /* Initialize PWM GENERATOR 3 LEADING-EDGE BLANKING REGISTER LOW */
    PG4LEB      = 0x0000;
    
    /* Initialize PWM GENERATOR 3 PHASE REGISTER */
    PG4PHASE     = 0x0000;
    /* Initialize PWM GENERATOR 3 DUTY CYCLE REGISTER */
    PG4DC        = PFC_MIN_DUTY_COUNTS;
    /* Initialize PWM GENERATOR 3 DUTY CYCLE ADJUSTMENT REGISTER */
    PG4DCA       = 0x0000;
    /* Initialize PWM GENERATOR 3 PERIOD REGISTER */
    PG4PER       = PFC_LOOPTIME_TCY ;
    /* Initialize PWM GENERATOR 3 DEAD-TIME REGISTER LOW */
    PG4DTbits.DTH = 0x0000;
    /* Initialize PWM GENERATOR 3 DEAD-TIME REGISTER HIGH */
    PG4DTbits.DTL = 0x0000;

    /* Initialize PWM GENERATOR 3 TRIGGER A REGISTER */
    PG4TRIGAbits.CAHALF = 1;
    PG4TRIGAbits.TRIGA = PFC_ADC_SAMPLING_POINT;  
    
    /* Initialize PWM GENERATOR 3 TRIGGER B REGISTER */
    PG4TRIGB     = 0x0000;
    /* Initialize PWM GENERATOR 3 TRIGGER C REGISTER */
    PG4TRIGC     = 0x0000;
    
}
// </editor-fold>