/*
 * File: ConfigParameters.c
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

#include "ConfigParameters.h"
#include "rtwtypes.h"
#include "ConfigParams_types.h"

/* Exported data definition */

/* Definition for custom storage class: ExportToFile */
struct_DWWWf6N21VtITTqdrqSn0 PI_params = {
  0.30F,      // Kp_id
  0.0F,       // Ki_id = 0: P-only on Id axis prevents integrator drift from Hall angle error
  0.50F,      // Kp_iq
  0.1F,       // Ki_iq - safe proven value (increase via BLE: SET Ki_iq <val>)
  0.78F,      // Kp_speed - tuned for N_base=4750 (was 1.0)
  1.5F,       // Ki_speed - tuned for N_base=4750 (was 0.1)
  2,
  20,
  2
} ;                                    /* Referenced by: '<S1>/Gain6' */

/* PI_params.Kp_id - Proportional gain for Id controller
   PI_params.Ki_id - Integral gain for Id controller
   PI_params.Kp_iq - Proportional gain for Id controller
   PI_params.Ki_iq - Integral gain for Iq controller
   PI_params.Kp_speed - Proportional gain for speed controller
   PI_params.Ki_speed - Integral gain for speed controller
   PI_params.delay_Currents - Delay in current measurement [sec]
   PI_params.delay_Speed - Delay in speed measurement [sec]
   PI_params.delay_Position - Delay in position measurement [sec] */
real_T T_pwm = 5.0E-5;                 /* Referenced by: '<S1>/Gain2' */

/* T_pwm - PWM period [sec] */
real_T Ts = 0.0001;                    /* Referenced by: '<S1>/Gain' */

/* Ts - Current controller sample time [sec] */
real_T Ts_speed = 0.001;               /* Referenced by: '<S1>/Gain1' */

/* Ts_speed - Speed controller sample time [sec] */
struct_I5k2n9mWimERhc2OS6k9dF inverter = {
  18.0F,
  16.5F,
  0.0033F
} ;                                    /* Referenced by: '<S1>/Gain4' */

/* inverter.V_dc - DC link voltage of the inverter (18V: Makita BL1820B nominal)
   inverter.ISenseMax - Maximum measurement current by the current measurement circuit
   inverter.R_board - Inverter board resistance */
struct_hXFaRMdP3p9q6ruHP5AkdC pmsm = {
  8.0F,
  0.2865F,
  0.0001505F,
  0.0001505F,
  6.0F,
  1.81e-5F,
  0.001F,
  0.002915F,
  3.29F,
  1000.0F,
  4750.0F,
  0.134F,
  0.05F,   // V_boost reduced from 0.15 to 0.05 to reduce open-loop overspeed
  4750.0F    // N_base: match motor rated speed (651614 datasheet)
} ;                                    /* Referenced by: '<S1>/Gain3' */

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

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
