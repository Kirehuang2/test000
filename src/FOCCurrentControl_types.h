/*
 * File: FOCCurrentControl_types.h
 *
 * Code generated for Simulink model 'FOCCurrentControl'.
 *
 * Model version                  : 2.93
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Thu May 30 19:22:32 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_FOCCurrentControl_types_h_
#define RTW_HEADER_FOCCurrentControl_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_struct_DWWWf6N21VtITTqdrqSn0_
#define DEFINED_TYPEDEF_FOR_struct_DWWWf6N21VtITTqdrqSn0_

typedef struct {
  real32_T Kp_id;
  real32_T Ki_id;
  real32_T Kp_iq;
  real32_T Ki_iq;
  real32_T Kp_speed;
  real32_T Ki_speed;
  int32_T delay_Currents;
  int32_T delay_Speed;
  int32_T delay_Position;
} struct_DWWWf6N21VtITTqdrqSn0;

#endif

#ifndef DEFINED_TYPEDEF_FOR_struct_hXFaRMdP3p9q6ruHP5AkdC_
#define DEFINED_TYPEDEF_FOR_struct_hXFaRMdP3p9q6ruHP5AkdC_

typedef struct {
  real32_T p;
  real32_T Rs;
  real32_T Ld;
  real32_T Lq;
  real32_T Ke;
  real32_T J;
  real32_T B;
  real32_T FluxPM;
  real32_T I_rated;
  real32_T QEPSlits;
  real32_T N_rated;
  real32_T T_rated;
  real32_T V_boost;
  real32_T N_base;
} struct_hXFaRMdP3p9q6ruHP5AkdC;

#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM_FOCCurrentControl_T RT_MODEL_FOCCurrentControl_T;

#endif                               /* RTW_HEADER_FOCCurrentControl_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
