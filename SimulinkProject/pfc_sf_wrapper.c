
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

#include "pfc_sf_bus.h"

/* %%%-SFUNWIZ_wrapper_includes_Changes_BEGIN --- EDIT HERE TO _END */
#ifndef __XC16__
  #define __attribute__(x)
#endif
#include <string.h>   /* for memset */
#include "regs_stub.h"
#include <math.h>
#include "..\project\pfc\pfc_measure.c"
#include "..\project\pfc\pfc_pi.c"
#include "..\project\pfc\pfc.c"
#include "board_service.c"
#include "pfc_bus_copy.h"
/* %%%-SFUNWIZ_wrapper_includes_Changes_END --- EDIT HERE TO _BEGIN */
#define u_width 1
#define u_1_width 1
#define u_2_width 1
#define u_3_width 1
#define u_4_width 4
#define y_width 1
#define y_1_width 1
#define y_2_width 1

/*
 * Create external references here.  
 *
 */
/* %%%-SFUNWIZ_wrapper_externs_Changes_BEGIN --- EDIT HERE TO _END */
/* extern double func(double a); */
/* %%%-SFUNWIZ_wrapper_externs_Changes_END --- EDIT HERE TO _BEGIN */

/*
 * Start function
 *
 */
extern void pfc_sf_Start_wrapper(void);

void pfc_sf_Start_wrapper(void)
{
/* %%%-SFUNWIZ_wrapper_Start_Changes_BEGIN --- EDIT HERE TO _END */
memset(&pfcParam, 0, sizeof(pfcParam));   /* cold-start every run, MEX cache or not */
    ADCBUF_VDC = 0; ADCBUF_PFC_VAC = 0; ADCBUF_PFC_IL = 0; ADCBUF_PFC_IL2 = 0;
    PG4DC = 0; PFC_INRUSH_RELAY = 0;
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
			const real32_T *cfg,
			real32_T *pwm_fac,
			boolean_T *SiL_Out_PFC_INRUSH_RELAY,
			PFC_T_Bus *pfcParam_out);

void pfc_sf_Outputs_wrapper(const real32_T *Vdc_in,
			const real32_T *Vac_in,
			const real32_T *Il_in,
			const real32_T *Iout_in,
			const real32_T *cfg,
			real32_T *pwm_fac,
			boolean_T *SiL_Out_PFC_INRUSH_RELAY,
			PFC_T_Bus *pfcParam_out)
{
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_BEGIN --- EDIT HERE TO _END */
ADCBUF_VDC = Get_ADCBUF_VDC(Vdc_in[0]);
    ADCBUF_PFC_VAC = Get_ADCBUF_VAC(Vac_in[0]);
    ADCBUF_PFC_IL = Get_ADCBUF_PFC_IL(Il_in[0]);
    ADCBUF_PFC_IL2 = Get_ADCBUF_PFC_IL2(Iout_in[0]);

    if (cfg[0] >= 0) pfcParam.dutyFFEnable           = (uint16_t)cfg[0];
    if (cfg[1] >= 0) pfcParam.sampleCorrectionEnable = (uint16_t)cfg[1];
    if (cfg[2] >= 0) pfcParam.vdcNotch.enable        = (uint16_t)cfg[2];
    if (cfg[3] >= 0) pfcParam.loadFF.enable          = (uint16_t)cfg[3];

    PFC_ADCInterrupt();
    
    pwm_fac[0] = (float)((PFC_PWM_PDC*1.0f)/(PFC_LOOPTIME_TCY*1.0f));
    SiL_Out_PFC_INRUSH_RELAY[0] = PFC_INRUSH_RELAY;

    PFC_COPY_TO_BUS(pfcParam_out, pfcParam);
/* %%%-SFUNWIZ_wrapper_Outputs_Changes_END --- EDIT HERE TO _BEGIN */
}


