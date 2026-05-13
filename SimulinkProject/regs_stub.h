#ifndef REGS_STUB_H
#define REGS_STUB_H

/* Host-side stubs for dsPIC33AK registers used in PFC SIL build.
 * These are defined ONCE here (storage), because regs_stub.h is only
 * included in the single translation unit pfc_sf_wrapper.c.
 * Do NOT include this header from multiple .c files without converting
 * to extern declarations + a single definition site. */
 
 #include "pfc_userparams.h"

volatile uint16_t ADCBUF_VDC     = 0;   /* DC link ADC, unsigned */
volatile int16_t ADCBUF_PFC_VAC = 0;   /* AC line ADC, bipolar */
volatile int16_t ADCBUF_PFC_IL  = 0;   /* Inductor current, bipolar */
volatile int16_t ADCBUF_PFC_IL2  = 0;   /* Inductor current, bipolar */
volatile unsigned long  PG4DC          = 0;   /* PWM duty register (32-bit on AK) */

uint16_t Get_ADCBUF_VDC(float Vdc) {
	return (uint16_t)(Vdc/PFC_VOLTAGE_BASE*32768*2);
}

int16_t Get_ADCBUF_VAC(float Vac) {
	//return (int16_t)((float)(Vac/(ADCBUF_PFC_VAC)));
	return (int16_t)(Vac/PFC_VOLTAGE_BASE*32768);
}

int16_t Get_ADCBUF_PFC_IL2(float Il) {
	return (int16_t)(Il/PFC_INPUT_MAX_CURRENT*32768);
}

int16_t Get_ADCBUF_PFC_IL(float Il) {
	return (int16_t)(Il/PFC_INPUT_MAX_CURRENT*32768);
}

#endif /* REGS_STUB_H */