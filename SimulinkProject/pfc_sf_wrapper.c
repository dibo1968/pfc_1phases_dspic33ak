
/*
 * Include Files
 *
 */
#if defined(MATLAB_MEX_FILE)
#include "tmwtypes.h"
#include "simstruc_types.h"
#else
#define SIMPLIFIED_RTWTYPES_COMPATIBILITY
#include "rtwtypes.h"
#undef SIMPLIFIED_RTWTYPES_COMPATIBILITY
#endif



/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef __XC16__
  #define __attribute__(x)
#endif

#include "regs_stub.h"
#include <math.h>
#include "..\project\pfc\pfc_measure.c"
#include "..\project\pfc\pfc_pi.c"
#include "..\project\pfc\pfc.c"
#include "board_service.c"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define u_1_width 1
#define u_2_width 1
#define u_3_width 1
#define y_width 1
#define y_1_width 1
#define y_2_width 1
#define y_3_width 1
#define y_4_width 1
#define y_5_width 3
#define y_6_width 3

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
 
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
extern void pfc_sf_Start_wrapper(void);

void pfc_sf_Start_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
PFC_ServiceInit();
/* %%%-SFUNWIZ_wrapper_Start_Changes_END --- EDIT HERE TO _BEGIN */
}
/*
 * Output function
 *
 */
extern void pfc_sf_Outputs_wrapper(const real32_T *Vdc_in,
			const real32_T *Vac_in,
			const real32_T *Il_in,
			const real32_T *Iout_in,
			uint8_T *state_out,
			uint16_T *fault_status_out,
			real32_T *test1,
			real32_T *test2,
			real32_T *pwm_fac,
			real32_T *pfcParam_piVoltage_out,
			real32_T *pfcParam_piCurrent_out);

void pfc_sf_Outputs_wrapper(const real32_T *Vdc_in,
			const real32_T *Vac_in,
			const real32_T *Il_in,
			const real32_T *Iout_in,
			uint8_T *state_out,
			uint16_T *fault_status_out,
			real32_T *test1,
			real32_T *test2,
			real32_T *pwm_fac,
			real32_T *pfcParam_piVoltage_out,
			real32_T *pfcParam_piCurrent_out)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
ADCBUF_VDC = Get_ADCBUF_VDC(Vdc_in[0]);
ADCBUF_PFC_VAC = Get_ADCBUF_VAC(Vac_in[0]);
ADCBUF_PFC_IL = Get_ADCBUF_PFC_IL(Il_in[0]);
ADCBUF_PFC_IL2 = Get_ADCBUF_PFC_IL2(Iout_in[0]);
PFC_ADCInterrupt();

state_out[0] = (unsigned char)pfcParam.state;
fault_status_out[0] = (unsigned short)pfcParam.faultStatus;
pwm_fac[0] = (float)((PFC_PWM_PDC*1.0f)/(PFC_LOOPTIME_TCY*1.0f));
    
test1[0] = (float)pfcParam.piVoltage.output;
test2[0] = (float)pfcParam.voltLoopExeRate;

pfcParam_piVoltage_out[0] = pfcParam.piVoltage.reference;
pfcParam_piVoltage_out[1] = pfcParam.vdcAVG.output;
pfcParam_piVoltage_out[2] = pfcParam.piVoltage.output;

pfcParam_piCurrent_out[0] = pfcParam.currentReference;
pfcParam_piCurrent_out[1] = pfcParam.averageCurrent;
pfcParam_piCurrent_out[2] = pfcParam.piCurrent.output;
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


