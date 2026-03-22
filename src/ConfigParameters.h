/*
 * File: ConfigParameters.h
 *
 * Code generated for Simulink model 'ConfigParams'.
 *
 * Model version                  : 2.2
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Thu May 30 19:30:52 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_ConfigParameters_h_
#define RTW_HEADER_ConfigParameters_h_
#include "rtwtypes.h"
#include "ConfigParams_types.h"

/* Exported data declaration */

/* Declaration for custom storage class: ExportToFile */
extern struct_DWWWf6N21VtITTqdrqSn0 PI_params;/* Referenced by: '<S1>/Gain6' */

/* PI_params.Kp_id - Proportional gain for Id controller
   PI_params.Ki_id - Integral gain for Id controller
   PI_params.Kp_iq - Proportional gain for Id controller
   PI_params.Ki_iq - Integral gain for Iq controller
   PI_params.Kp_speed - Proportional gain for speed controller
   PI_params.Ki_speed - Integral gain for speed controller
   PI_params.delay_Currents - Delay in current measurement [sec]
   PI_params.delay_Speed - Delay in speed measurement [sec]
   PI_params.delay_Position - Delay in position measurement [sec] */
extern real_T T_pwm;                   /* Referenced by: '<S1>/Gain2' */

/* T_pwm - PWM period [sec] */
extern real_T Ts;                      /* Referenced by: '<S1>/Gain' */

/* Ts - Current controller sample time [sec] */
extern real_T Ts_speed;                /* Referenced by: '<S1>/Gain1' */

/* Ts_speed - Speed controller sample time [sec] */
extern struct_I5k2n9mWimERhc2OS6k9dF inverter;/* Referenced by: '<S1>/Gain4' */

/* inverter.V_dc - DC link voltage of the inverter
   inverter.ISenseMax - Maximum measurement current by the current measurement circuit
   inverter.R_board - Inverter board resistance */
extern struct_hXFaRMdP3p9q6ruHP5AkdC pmsm;/* Referenced by: '<S1>/Gain3' */

/* pmsm.P - Number of pole pairs
   pmsm.Rs - Stator resistance
   pmsm.Ld - D axis inductance
   pmsm.Lq - Q axis inductance
   pmsm.Ke - Back EMF constant
   pmsm.J - Inertia
   pmsm.B - Friction co-efficient
   pmsm.FluxPM - PM flux computed from Ke
   pmsm.I_rated - Rated motor current
   pmsm.QEPSlits - Resolution for angle conversion (used for Hall sensor angle scaling)
   pmsm.N_rated - Rated motor speed
   pmsm.T_rated - Rated motor torque
   pmsm.V_boost - Minimum voltage to spin in open loop as a ratio of rated motor voltage
   pmsm.N_base - Base speed of the motor */
#endif                                 /* RTW_HEADER_ConfigParameters_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
