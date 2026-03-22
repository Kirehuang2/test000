/*
 * File: ConfigParams.c
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

#include "ConfigParams.h"
#include "rtwtypes.h"
#include "ConfigParams_types.h"
#include "ConfigParameters.h"

/* Real-time model */
static RT_MODEL_ConfigParams_T ConfigParams_M_;
RT_MODEL_ConfigParams_T *const ConfigParams_M = &ConfigParams_M_;

/* Model step function */
void ConfigParams_step(real32_T *arg_Input, real32_T *arg_Out1)
{
  /* Outport: '<Root>/Out1' incorporates:
   *  Gain: '<S1>/Gain'
   *  Gain: '<S1>/Gain1'
   *  Gain: '<S1>/Gain2'
   *  Gain: '<S1>/Gain3'
   *  Gain: '<S1>/Gain4'
   *  Gain: '<S1>/Gain5'
   *  Gain: '<S1>/Gain6'
   *  Gain: '<S1>/Gain7'
   *  Inport: '<Root>/Input'
   *  Sum: '<S1>/Add'
   */
  *arg_Out1 = (((((((real32_T)Ts * *arg_Input + (real32_T)Ts_speed * *arg_Input)
                   + (real32_T)T_pwm * *arg_Input) + pmsm.Rs * *arg_Input) +
                 inverter.V_dc * *arg_Input) + 0.0001F * *arg_Input) +
               PI_params.Kp_id * *arg_Input) + 0.1F * *arg_Input;
}

/* Model initialize function */
void ConfigParams_initialize(void)
{
  /* (no initialization code required) */
}

/* Model terminate function */
void ConfigParams_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
