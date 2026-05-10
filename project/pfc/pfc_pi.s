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

.ifdecl __ASM30__
    .include "xc.inc"
.endif

; Definitions	
.equ _output,0
.equ _integralOut,2
.equ _propOut,4

.equ _input,6
.equ _reference,8
.equ _error,10

.equ _kp,12
.equ _ki,14
.equ _kpScale,16
.equ _kiScale,18
.equ _outMin,20
.equ _outMax,22
        
.global   _PFC_PIController
.global   PFC_PIController

_PFC_PIController:
PFC_PIController:

    ; Save CORCON, then configure as needed
    push    CORCON
    mov     #0x00E0,w3
    mov     w3,CORCON
    
    ; Load error (Reference - measued) to w5 register
    mov     w1,[w0+_error]
    mov     w1,w5
    
    ; Load previous integral Output to Accumulator B 
    mov	    [w0+_integralOut],w3
    lac	    w3,b
    
    ; Load ki into W4 register
    mov	    [w0+_ki],w4
    ; Multiply ki * error and load into accumulator A
    ; A = ki * error 
    mpy	    w4*w5,a
    
    ; Scale accumualtor by same factor by which coeffiecients are scaled
    ; A = A * kiScale;
    mov    [w0+_kiScale],w3
    neg	    w3,w3
    sftac   a,w3
    
    ; Add previous intergral Output and Ki * error and store it in accumulator A
    ; A = integralOut + ki * error ; 
    add	    a
    
    ; Round accumulator A and move it to intergral output (integralOut)
    sac.r   a,w3
    mov	    w3,[w0+_integralOut]
    
   ; Load w4 with kp 
    mov	    [w0+_kp],w4
    
   ; Caluclate kp*error and load it in Accumulator B
   ; B = kp * error
    mpy	    w4*w5,b
    
    ; Scale accumualtor by same factor by which coeffiecients are scaled
    ; B = B * kpScale
    mov     [w0+_kpScale],w3
    neg	    w3,w3
    sftac   b,w3
    
    ; Round the accumulator B and load into propotional Output (propOut)
    sac.r   b,w3
    mov	    w3,[w0+_propOut]
 
    ; Add accumulator A and B
    ; B = kp * error + ki * error + integralOut(previous) and 
    add	    b
    sac.r   b,w3
    
    ; Bound check the PI output and limit it ouput between minimum and 
    ; maximum value specified.
    mov     [w0+_outMin], w6
    cpsgt   w3, w6
    mov     w6, w3
    mov     [w0+_outMax], w6
    cpslt   w3, w6
    mov     w6, w3
    mov	    w3,[w0+_output]
         
    pop CORCON
    return    
.end
