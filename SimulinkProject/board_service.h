#ifndef __BOARD_SERVICE_H
#define __BOARD_SERVICE_H

void HAL_PFCPWMDisableOutputs(void);
void HAL_PFCPWMEnableOutputs(void);
void DisablePFCADCInterrupt(void);
void EnablePFCADCInterrupt(void);
void ClearPFCADCIF_ReadADCBUF(void);
void GetDCLinkVoltage(float *);
void ClearPFCADCIF(void);

extern volatile int pfc_pwm_enabled;
// </editor-fold

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_SERVICE_H */
