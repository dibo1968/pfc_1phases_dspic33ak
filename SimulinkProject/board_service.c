#include "board_service.h"

volatile int pfc_pwm_enabled = 0;   /* observable from wrapper */

void HAL_PFCPWMDisableOutputs(void)   { pfc_pwm_enabled = 0; }
void HAL_PFCPWMEnableOutputs(void)    { pfc_pwm_enabled = 1; }
void DisablePFCADCInterrupt(void)     {}
void EnablePFCADCInterrupt(void)      {}
void ClearPFCADCIF_ReadADCBUF(void)   {}
void GetDCLinkVoltage(float *dclink)  { (void)dclink; }  /* no-op on purpose */
void ClearPFCADCIF(void)              {}