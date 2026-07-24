function pfc_bus_defs()
%PFC_BUS_DEFS Create the Simulink.Bus objects that mirror the PFC C structs.
%
%   PFC_BUS_DEFS() creates these bus objects in the base workspace:
%
%       PFC_AVG_T_Bus              <- PFC_AVG_T             (pfc.h)
%       PFC_RMS_SQUARE_T_Bus       <- PFC_RMS_SQUARE_T      (pfc.h)
%       PFC_PI_T_Bus               <- PFC_PI_T              (pfc_pi.h)
%       PFC_MEASURE_CURRENT_T_Bus  <- PFC_MEASURE_CURRENT_T (pfc_measure.h)
%       PFC_MEASURE_VOLTAGE_T_Bus  <- PFC_MEASURE_VOLTAGE_T (pfc_measure.h)
%       PFC_T_Bus                  <- PFC_T                 (pfc.h)  <-- port type
%
%   Set the S-Function Builder output port data type to  Bus: PFC_T_Bus
%   and the whole pfcParam struct comes out as one signal.
%
%   The "_Bus" suffix is deliberate. The S-Function Builder emits a C
%   typedef named after the bus object, and pfc.h is already included in
%   the same translation unit (via pfc_sf_wrapper.c), so a bus literally
%   named PFC_T would collide with the firmware typedef.
%
%   Element names match the C field names one-for-one, so logged signals
%   read exactly like the firmware source.
%
%   Call this before the model compiles - from the model PreLoadFcn, or at
%   the end of mchp_pfc_foc_dsPIC33A_data.m.
%
%   Keep in sync with PFC_T in project/pfc/pfc.h and the copy macro in
%   pfc_bus_copy.h.

% Leaf types first: PFC_T_Bus references them by name.

makeBus('PFC_AVG_T_Bus', 'Running-average accumulator (PFC_AVG_T).', { ...
    'sum'                   'single'
    'output'                'single'
    'scaler'                'uint16'
    'samples'               'uint16'
    'sampleLimit'           'uint16'
    'status'                'uint16'
    });

makeBus('PFC_RMS_SQUARE_T_Bus', 'RMS/peak accumulator (PFC_RMS_SQUARE_T).', { ...
    'sqrOutput'             'single'
    'sum'                   'single'
    'samples'               'uint16'
    'sampleLimit'           'uint16'
    'peak'                  'single'
    'peakcheck'             'single'
    'status'                'uint16'
    });

makeBus('PFC_PI_T_Bus', 'PI controller state (PFC_PI_T).', { ...
    'output'                'single'
    'integralOut'           'single'
    'propOut'               'single'
    'input'                 'single'
    'reference'             'single'
    'error'                 'single'
    'kp'                    'single'
    'ki'                    'single'
    'kpScale'               'single'
    'kiScale'               'single'
    'minOutput'             'single'
    'maxOutput'             'single'
    });

makeBus('PFC_MEASURE_CURRENT_T_Bus', 'Current feedback (PFC_MEASURE_CURRENT_T).', { ...
    'inductorCurrent'       'int16'
    'iL'                    'single'
    'offset'                'single'
    'counter'               'uint16'
    'status'                'uint16'
    'sum'                   'single'
    });

makeBus('PFC_MEASURE_VOLTAGE_T_Bus', 'Voltage feedback (PFC_MEASURE_VOLTAGE_T).', { ...
    'acVoltage'             'int16'
    'outputVoltage'         'int16'
    'vac'                   'single'
    'offsetVac'             'single'
    'vdc'                   'single'
    });

% Top level. 'state' is int32 because a C enum is an int on the MEX host
% compiler, which keeps the bus byte layout identical to PFC_T. To see the
% state names instead of 0..5 in the Data Inspector, swap int32 for
% 'Enum: pfc_ctrl_state' (that classdef already exists in this folder).
makeBus('PFC_T_Bus', 'Complete PFC control parameter set (PFC_T).', { ...
    'duty'                  'uint32'
    'samplePoint'           'uint16'
    'averageCurrent'        'single'
    'rampRate'              'uint16'
    'voltLoopExeRate'       'uint16'
    'boostDutyRatio'        'single'
    'currentReference'      'single'
    'faultStatus'           'uint16'
    'sampleCorrectionEnable' 'uint16'
    'dcmDetected'           'uint16'
    'iValleyEst'            'single'
    'sampleCorrFactor'      'single'
    'vdcAVG'                'Bus: PFC_AVG_T_Bus'
    'vacAVG'                'Bus: PFC_AVG_T_Bus'
    'vacRMS'                'Bus: PFC_RMS_SQUARE_T_Bus'
    'piVoltage'             'Bus: PFC_PI_T_Bus'
    'piCurrent'             'Bus: PFC_PI_T_Bus'
    'state'                 'int32'
    'pfcCurrent'            'Bus: PFC_MEASURE_CURRENT_T_Bus'
    'pfcCurrent2'           'Bus: PFC_MEASURE_CURRENT_T_Bus'
    'pfcVoltage'            'Bus: PFC_MEASURE_VOLTAGE_T_Bus'
    'iL'                    'single'
    'rectifiedVac'          'single'
    'outputVdc'             'single'
    });

end

% -------------------------------------------------------------------------
function makeBus(busName, description, spec)
%MAKEBUS Build one Simulink.Bus from an {name, datatype} table.

elems = Simulink.BusElement.empty(0, size(spec, 1));
for k = 1:size(spec, 1)
    elems(k) = Simulink.BusElement;
    elems(k).Name = spec{k, 1};
    elems(k).DataType = spec{k, 2};
end

bus = Simulink.Bus;
bus.Description = description;
bus.Elements = elems;

assignin('base', busName, bus);
end
