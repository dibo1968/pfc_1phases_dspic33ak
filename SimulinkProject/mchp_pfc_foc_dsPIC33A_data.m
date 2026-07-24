%% ************************************************************************
% Model         :   Sensorless FOC of PMSM using PLL estimator integrated 
%                   with Power Factor Correction                   
% Description   :   Set Parameters for Single-Stage Boost Power Factor
%                   Correction,Motor and Inverter parameters

% File name     :   mchp_pfc_foc_dsPIC33A_data.m
% Copyright 2026 Microchip Technology Inc.

%% Simulation Parameters
 clc
 clear all
 close all
 s = tf('s');
%% Set PWM Switching frequency
PWM_frequency 	= 64e3;             %Hz // converter s/w freq
T_pwm           = 1/PWM_frequency;  %s  // PWM switching time period

PWM_frequency_mc 	= 16e3;                %Hz // converter s/w freq
T_pwm_mc            = 1/PWM_frequency_mc;  %s  // PWM switching time period

%% Set Sample Times
Ts          	= T_pwm;        %sec        // simulation time step for controller

Ts_mc          	   = T_pwm_mc;        %sec        // simulation time step for controller
Ts_simulink_mc     = T_pwm_mc/2;      %sec        // simulation time step for model simulation
Ts_motor_mc        = T_pwm_mc/2;      %Sec        // Simulation sample time
Ts_inverter_mc     = T_pwm_mc/2;      %sec        // simulation time step for average value inverter
Ts_speed_mc        = Ts_mc;           %Sec        // Sample time for speed controller
%% Set data type for controller & code-gen
dataType        = 'single';    

%% System Parameters
%% Simulation Parameters
%% Set circuit parameters
Po          = 0.25*1500;                             % Output Power of the converter
Vout        = 380;                              % Output Voltage
Vin_rms     = 230;                              % Inout Voltage rms value [ 90V - 250V range]
Vin_pk      = Vin_rms*1.414;                    % Input Voltage peak value 
Vrms_min    = 200;                              % Minimum Input Voltage rms value for Kmul
Vpk_min     = Vrms_min*1.414;                   % Minimum Input Voltage peak value for Kmul
Iin_max_rms = 10;                               % Maximum Input Current rms value
Iin_max_pk  = Iin_max_rms*1.414;                % Maximum Input Current peak value
Pout_max    = 1500;                             % Maximum Power 
Max_Duty    = 0.95;                             % Maximum Duty Cycle

Vac_Frequency   = 50;                           % AC Input Voltage Frequency in Hz
Vac_TimePeriod  = 1/Vac_Frequency;              % AC Input Voltage Time Period in Sec
Vac_Samples     = Vac_TimePeriod/T_pwm;         % No. of ADC Samples of AC Input Voltage 


Input_UV_TH_LOW  = 110*110;         % Minimum Input Voltage rms Square value-Threshold Low
Input_UV_TH_HIGH = 130*130;         % Minimum Input Voltage rms Square value-Threshold High

Input_OV_TH_HIGH  = 250*250;        % Maximum Input Voltage rms Square value-Threshold High
Input_OV_TH_LOW   = 240*240;        % Maximum Input Voltage rms Square value-Threshold Low

Output_OV_TH      = 400;             % Maximum Output Voltage

L = 680e-6;                         % Inductance Value(Choose the proper value if it is swinging inductor based on load)
C = 3*470e-6;                       % Output Capacitance
R = (Vout^2)/Po;                    % Equivalent load Resistance

%% Set PWM Switching frequency
fsw = 64e3;                         % PWM frequency
T_pwm = 1/fsw;                      
%% Set Sample Ratio/Sample Times
Isr = 1;                            
Osr = 10;                           
Tsi = Isr / fsw;                  
Tsv = Osr / fsw;                    

%% Set Base Values for Gains
Ibase = 1;                       % Ibase
Vbase = 1;                       % Vbase 

KiL = 1/Ibase;                      
Kvo = 1/Vbase;                      
Kvin = 1/Vbase;
Kmul = 1;         

%% Set Control System Parameters
fc_i = fsw/10;                      
fc_v = 12;                          

%% Set up Soft Start/Ramping up the reference voltage for Voltage loop 
Vref = Vout;                        
Vout_init = Vin_pk;                 
T_rise = 0.20;                    
Slope = (Vref-Vout_init)/T_rise;    

%% Current Loop Control (Inductor current Transfer function)[ IL / d]
Gid = Vout/(s*L);

%% Current loop compensator calculation
fz_i = fc_i/10;                             

Gci = (2*pi*fc_i*L)/(KiL*Vout);            

Kp_i = Gci;                        
Ki_i = Kp_i*2*pi*fz_i;

Gic_comp = Kp_i+Ki_i/s;
Gicl = Gid*Gic_comp*KiL;

%% Voltage Loop Control (Inductor current Transfer function)[ IL / d]
Gvc = (2*Kmul)/(Kvin*KiL*C*Vout*s);
%% Voltage loop compensator calculation
fz_v = 5;                                            

Gcv = (C*Vout*2*pi*fc_v*Kvin*KiL)/(Kvo*2*Kmul);     

Kp_v = Gcv;
Ki_v = Kp_v*2*pi*fz_v;

Gvc_comp = Kp_v+Ki_v/s;
Gvcl = (Gvc*Gvc_comp*Kvo);

%% System Parameters
% Set motor parameters
pmsm.model          = 'Compressor'; % Compressor , Leadshine 400

if strcmp (pmsm.model,'Compressor')

        % Compressor motor parameters
    pmsm.sn             = '654321';             %           // Manufacturer Model Number
    pmsm.p              = 3;                    %           // Pole Pairs for the motor
    pmsm.Rs             = 0.91;                 %Ohm        // Stator Resistor
    pmsm.Ld             = 0.0112;               %H          // D-axis inductance value
    pmsm.Lq             = 0.0112;               %H          // Q-axis inductance value
    pmsm.Lav            = (pmsm.Ld+pmsm.Lq)/2;  %H          // Average inductance
    pmsm.Ke             = 77;                   %Bemf Const	// Vline_peak/krpm
    pmsm.Kt             = 0.274;                %Nm/A       // Torque constant
    pmsm.J              = 4.6e-5;
    pmsm.B              = 8.74e-5;              %Kg-m2/s    // Friction Co-efficient
    pmsm.I_rated        = 5*sqrt(2);            %A      	// Rated current (phase-peak)
    pmsm.N_max          = 5000;                 %rpm        // Max speed
    pmsm.N_rated        = 4000;                 %rpm        // rated speed
    pmsm.f_rated        = (pmsm.N_rated*pmsm.p*2)/120;                %Hz    // Rated Frequency
    pmsm.w_rated_elec   = pmsm.f_rated*2*pi;                      %rad/sec    // Rated electrical speed
    pmsm.w_base_elec    = pmsm.w_rated_elec*1;                     %rad/sec    // Base electrical speed
    pmsm.FluxPM         = (pmsm.Ke)/(sqrt(3)*2*pi*1000*pmsm.p/60);    %PM flux computed from Ke
    pmsm.T_rated        = (3/2)*pmsm.p*pmsm.FluxPM*pmsm.I_rated;      %Get T_rated from I_rated
    pmsm.QEPSlits       = 1000;

end

%% Rotor Locking Parameters

% Leadshine 400
Lock_Time           = 0.5;
Lock_Time_Counts    = Lock_Time/T_pwm;
Lock_Current        = 2;
Lock_Voltage        = Lock_Current*pmsm.Rs;
Lock_CommutationAngle = 0;

%% Openloop Parameters Hurst 300

OL_Speed      = 500;          %rpm
OL_Id         = 0;
OL_Iq         = 0.5;


OL_Vd         = -0.7;
OL_Vq         = 2.69;

%% Inverter parameters 
% MCHV-230V-1.5kW Development Board 

inverter.model         = 'MCHV-230V-1.5kW';           % 		// Manufacturer Model Number
inverter.sn            = 'INV_XXXX';         		% 		// Manufacturer Serial Number
inverter.V_dc          = 380;
inverter.ISenseMax     = 22.0; 					%Amps   // Max current that can be measured
inverter.I_trip        = 10;                  		%Amps   // Max current for trip
inverter.Rds_on        = 1e-3;                      %Ohms   // Rds ON
inverter.Rshunt        = 0.003;                      %Ohms   // Rshunt
inverter.R_board       = inverter.Rds_on + inverter.Rshunt/3;  %Ohms
inverter.MaxADCCnt     = 4095;      				%Counts // ADC Counts Max Value
inverter.invertingAmp  = -1;                        % 		// Non inverting current measurement amplifier
inverter.deadtime      = 1.25e-6;                      %sec    // Deadtime for the PWM 
inverter.OpampFb_Rf    = 5e3;                    %Ohms   // Opamp Feedback resistance for current measurement
inverter.opampInput_R  = 200;                       %Ohms   // Opamp Input resistance for current measurement
inverter.opamp_Gain    = inverter.OpampFb_Rf/inverter.opampInput_R; % // Opamp Gain used for current measurement

%% Derive Characteristics
pmsm.N_base = 100;%mcb_getBaseSpeed(pmsm,inverter); %rpm // Base speed of motor at given Vdc
%% PU System details // Set base values for pu conversion
% SI_System = 0;%mcb_SetSISystem(pmsm);

%% Controller design // Get ballpark values!
% Get PI Gains
% PI_params_SI = mcb.internal.SetControllerParameters(pmsm,inverter,SI_System,T_pwm_mc,Ts_mc,Ts_speed_mc);

%Updating delays for simulation
% PI_params_SI.delay_Currents    = int32(Ts_mc/Ts_simulink_mc);
% PI_params_SI.delay_Position    = int32(Ts_mc/Ts_simulink_mc);
% PI_params_SI.delay_Speed       = int32(Ts_speed_mc/Ts_simulink_mc);

%% Serial Communication for Debugging

Ts_serialIn     = 100e-3;
Ts_serialOut    = 500e-6;

target.frameSize = 120;
target.comport = 'COM11';
target.BaudRate = 1000000;