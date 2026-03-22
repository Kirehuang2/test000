/*
 * File: ConfigParams.h
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

#ifndef RTW_HEADER_ConfigParams_h_
#define RTW_HEADER_ConfigParams_h_
#ifndef ConfigParams_COMMON_INCLUDES_
#define ConfigParams_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* ConfigParams_COMMON_INCLUDES_ */

#include "ConfigParams_types.h"

/* Includes for objects with custom storage classes */
#include "ConfigParameters.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Real-time Model Data Structure */
struct tag_RTM_ConfigParams_T {
  const char_T * volatile errorStatus;
};

/* Model entry point functions */
extern void ConfigParams_initialize(void);
extern void ConfigParams_terminate(void);

/* Customized model step function */
extern void ConfigParams_step(real32_T *arg_Input, real32_T *arg_Out1);

/* Real-time Model object */
extern RT_MODEL_ConfigParams_T *const ConfigParams_M;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'ConfigParams'
 * '<S1>'   : 'ConfigParams/Subsystem'
 */
#endif                                 /* RTW_HEADER_ConfigParams_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
