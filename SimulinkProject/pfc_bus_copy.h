#ifndef PFC_BUS_COPY_H
#define PFC_BUS_COPY_H

/* Copy a firmware PFC_T instance into the Simulink bus struct that the
 * S-Function Builder generates for the PFC_T_Bus bus object.
 *
 *     PFC_COPY_TO_BUS(pfcParam_out, pfcParam);
 *
 * dst : pointer to the bus struct (the S-function output port argument)
 * src : the PFC_T instance
 *
 * This is a macro on purpose. The S-Function Builder picks the C typedef
 * name from the bus object name, so never spelling the type here keeps the
 * header valid if the bus is renamed or regenerated.
 *
 * Keep in sync with PFC_T in project/pfc/pfc.h and with pfc_bus_defs.m.
 *
 * DELIBERATE DIVERGENCE (2026-08-02). Six dead fields were deleted from the
 * firmware structs - PFC_PI_T.kpScale/.kiScale, PFC_AVG_T.scaler,
 * PFC_RMS_SQUARE_T.peak/.peakcheck and PFC_T.samplePoint - and the copies below
 * went with them. They are intentionally still present in pfc_sf_bus.h,
 * pfc_bus_defs.m and the generated readback in pfc_sf.c, because removing bus
 * ELEMENTS shifts the positional busInfo[] indices in pfc_sf.c and needs a MEX
 * regeneration, whereas leaving them costs nothing: they were only ever zero, so
 * the unwritten bus elements read exactly as before. Drop them from the bus on
 * the next occasion the S-Function is rebuilt in MATLAB anyway.
 *
 * Note the asymmetry when editing this file: APPENDING a firmware field is safe
 * and requires nothing here; DELETING one requires deleting its copy here too,
 * or this header stops compiling.
 */

#define PFC_BUS_COPY_AVG(d, s)                                              \
    do {                                                                    \
        (d).sum         = (s).sum;                                          \
        (d).output      = (s).output;                                       \
        (d).samples     = (s).samples;                                      \
        (d).sampleLimit = (s).sampleLimit;                                  \
        (d).status      = (s).status;                                       \
    } while (0)

#define PFC_BUS_COPY_RMS(d, s)                                              \
    do {                                                                    \
        (d).sqrOutput   = (s).sqrOutput;                                    \
        (d).sum         = (s).sum;                                          \
        (d).samples     = (s).samples;                                      \
        (d).sampleLimit = (s).sampleLimit;                                  \
        (d).status      = (s).status;                                       \
    } while (0)

#define PFC_BUS_COPY_PI(d, s)                                               \
    do {                                                                    \
        (d).output      = (s).output;                                       \
        (d).integralOut = (s).integralOut;                                   \
        (d).propOut     = (s).propOut;                                      \
        (d).input       = (s).input;                                        \
        (d).reference   = (s).reference;                                    \
        (d).error       = (s).error;                                        \
        (d).kp          = (s).kp;                                           \
        (d).ki          = (s).ki;                                           \
        (d).minOutput   = (s).minOutput;                                    \
        (d).maxOutput   = (s).maxOutput;                                    \
    } while (0)

#define PFC_BUS_COPY_CURRENT(d, s)                                          \
    do {                                                                    \
        (d).inductorCurrent = (s).inductorCurrent;                          \
        (d).iL              = (s).iL;                                       \
        (d).offset          = (s).offset;                                   \
        (d).counter         = (s).counter;                                  \
        (d).status          = (s).status;                                   \
        (d).sum             = (s).sum;                                      \
    } while (0)

#define PFC_BUS_COPY_VOLTAGE(d, s)                                          \
    do {                                                                    \
        (d).acVoltage     = (s).acVoltage;                                  \
        (d).outputVoltage = (s).outputVoltage;                              \
        (d).vac           = (s).vac;                                        \
        (d).offsetVac     = (s).offsetVac;                                  \
        (d).vdc           = (s).vdc;                                        \
    } while (0)

#define PFC_BUS_COPY_LOADFF(d, s)                                           \
    do {                                                                    \
        (d).rawADC      = (s).rawADC;                                       \
        (d).current     = (s).current;                                      \
        (d).currentFilt = (s).currentFilt;                                  \
        (d).powerFF     = (s).powerFF;                                      \
        (d).scale       = (s).scale;                                        \
        (d).filtCoeff   = (s).filtCoeff;                                    \
        (d).gain        = (s).gain;                                         \
        (d).enable      = (s).enable;                                       \
    } while (0)

#define PFC_COPY_TO_BUS(dst, src)                                           \
    do {                                                                    \
        (dst)->duty                   = (src).duty;                         \
        (dst)->averageCurrent         = (src).averageCurrent;               \
        (dst)->rampRate               = (src).rampRate;                     \
        (dst)->voltLoopExeRate        = (src).voltLoopExeRate;              \
        (dst)->boostDutyRatio         = (src).boostDutyRatio;               \
        (dst)->currentReference       = (src).currentReference;             \
        (dst)->faultStatus            = (src).faultStatus;                  \
        (dst)->sampleCorrectionEnable = (src).sampleCorrectionEnable;       \
        (dst)->dcmDetected            = (src).dcmDetected;                  \
        (dst)->iValleyEst             = (src).iValleyEst;                   \
        (dst)->sampleCorrFactor       = (src).sampleCorrFactor;             \
        PFC_BUS_COPY_AVG((dst)->vdcAVG,          (src).vdcAVG);             \
        PFC_BUS_COPY_AVG((dst)->vacAVG,          (src).vacAVG);             \
        PFC_BUS_COPY_RMS((dst)->vacRMS,          (src).vacRMS);             \
        PFC_BUS_COPY_PI((dst)->piVoltage,        (src).piVoltage);          \
        PFC_BUS_COPY_PI((dst)->piCurrent,        (src).piCurrent);          \
        (dst)->state                  = (int32_T)(src).state;               \
        PFC_BUS_COPY_CURRENT((dst)->pfcCurrent,  (src).pfcCurrent);         \
        PFC_BUS_COPY_CURRENT((dst)->pfcCurrent2, (src).pfcCurrent2);        \
        PFC_BUS_COPY_VOLTAGE((dst)->pfcVoltage,  (src).pfcVoltage);         \
        (dst)->iL                     = (src).iL;                           \
        (dst)->rectifiedVac           = (src).rectifiedVac;                 \
        (dst)->outputVdc              = (src).outputVdc;                    \
        PFC_BUS_COPY_LOADFF((dst)->loadFF,       (src).loadFF);             \
        (dst)->dutyRatio              = (src).dutyRatio;                    \
        (dst)->dutyFF                 = (src).dutyFF;                       \
        (dst)->dutyFFEnable           = (src).dutyFFEnable;                 \
    } while (0)

#endif /* PFC_BUS_COPY_H */
