classdef pfc_ctrl_state < Simulink.IntEnumType
  enumeration
    PFC_INIT(0)
    PFC_OFFSET_MEAS(1)
    PFC_WAIT_1CYCLE(2)
    PFC_CTRL_RUN(3)
    PFC_FAULT(4)
    PFC_PRECHARGE(5)
  end
end