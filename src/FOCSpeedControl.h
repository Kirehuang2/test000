/*
 * File: FOCSpeedControl.h
 *
 * Code generated for Simulink model 'FOCSpeedControl'.
 *
 * Model version                  : 2.28
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Thu May 30 19:22:51 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_FOCSpeedControl_h_
#define RTW_HEADER_FOCSpeedControl_h_
#ifndef FOCSpeedControl_COMMON_INCLUDES_
#define FOCSpeedControl_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* FOCSpeedControl_COMMON_INCLUDES_ */

#include "FOCSpeedControl_types.h"

/* Includes for objects with custom storage classes */
#include "ConfigParameters.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T UnitDelay_DSTATE;           /* '<S58>/Unit Delay' */
  real32_T Integrator_DSTATE;          /* '<S37>/Integrator' */
  uint32_T temporalCounter_i1;         /* '<S54>/OpenLoop to ClosedLoop' */
  int8_T Integrator_PrevResetState;    /* '<S37>/Integrator' */
  uint8_T Integrator_IC_LOADING;       /* '<S37>/Integrator' */
  uint8_T is_active_c1_FOCSpeedControl;/* '<S54>/OpenLoop to ClosedLoop' */
  uint8_T is_c1_FOCSpeedControl;       /* '<S54>/OpenLoop to ClosedLoop' */
  uint8_T is_Run;                      /* '<S54>/OpenLoop to ClosedLoop' */
} DW_FOCSpeedControl_T;

/* Real-time Model Data Structure */
struct tag_RTM_FOCSpeedControl_T {
  const char_T * volatile errorStatus;
};

/* Block states (default storage) */
extern DW_FOCSpeedControl_T FOCSpeedControl_DW;

/* Model entry point functions */
extern void SpeedControl_initialize(void);
extern void FOCSpeedControl_terminate(void);

/* Customized model step function */
extern void SpeedControl_step(boolean_T arg_Enable, real32_T arg_SpeedRef_PU,
  real32_T arg_SpeedFb_PU, real32_T arg_IqFb, real32_T arg_Mode, real32_T
  arg_IdqRef_PU[2], real32_T *arg_SpeedRefOut_PU, boolean_T *arg_EnClOut);

/* Real-time Model object */
extern RT_MODEL_FOCSpeedControl_T *const FOCSpeedControl_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Gain1' : Unused code path elimination
 * Block '<Root>/Gain2' : Unused code path elimination
 * Block '<S58>/Data Type Duplicate' : Unused code path elimination
 */

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
 * '<Root>' : 'FOCSpeedControl'
 * '<S1>'   : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset'
 * '<S2>'   : 'FOCSpeedControl/Open Loop to Closed Loop'
 * '<S3>'   : 'FOCSpeedControl/Zero_Cancellation'
 * '<S4>'   : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Anti-windup'
 * '<S5>'   : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/D Gain'
 * '<S6>'   : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Filter'
 * '<S7>'   : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Filter ICs'
 * '<S8>'   : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/I Gain'
 * '<S9>'   : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Ideal P Gain'
 * '<S10>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk'
 * '<S11>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Integrator'
 * '<S12>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Integrator ICs'
 * '<S13>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/N Copy'
 * '<S14>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/N Gain'
 * '<S15>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/P Copy'
 * '<S16>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Parallel P Gain'
 * '<S17>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Reset Signal'
 * '<S18>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Saturation'
 * '<S19>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk'
 * '<S20>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Sum'
 * '<S21>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Sum Fdbk'
 * '<S22>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Tracking Mode'
 * '<S23>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum'
 * '<S24>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral'
 * '<S25>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain'
 * '<S26>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/postSat Signal'
 * '<S27>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/preSat Signal'
 * '<S28>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel'
 * '<S29>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S30>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S31>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/D Gain/Disabled'
 * '<S32>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Filter/Disabled'
 * '<S33>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Filter ICs/Disabled'
 * '<S34>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/I Gain/External Parameters'
 * '<S35>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Ideal P Gain/Passthrough'
 * '<S36>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Ideal P Gain Fdbk/Disabled'
 * '<S37>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Integrator/Discrete'
 * '<S38>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Integrator ICs/External IC'
 * '<S39>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/N Copy/Disabled wSignal Specification'
 * '<S40>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/N Gain/Disabled'
 * '<S41>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/P Copy/Disabled'
 * '<S42>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Parallel P Gain/External Parameters'
 * '<S43>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Reset Signal/External Reset'
 * '<S44>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Saturation/Enabled'
 * '<S45>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Saturation Fdbk/Disabled'
 * '<S46>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Sum/Sum_PI'
 * '<S47>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Sum Fdbk/Disabled'
 * '<S48>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Tracking Mode/Disabled'
 * '<S49>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Tracking Mode Sum/Passthrough'
 * '<S50>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Tsamp - Integral/TsSignalSpecification'
 * '<S51>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/Tsamp - Ngain/Passthrough'
 * '<S52>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/postSat Signal/Forward_Path'
 * '<S53>'  : 'FOCSpeedControl/Discrete PI Controller  with anti-windup & reset/preSat Signal/Forward_Path'
 * '<S54>'  : 'FOCSpeedControl/Open Loop to Closed Loop/Sensorless'
 * '<S55>'  : 'FOCSpeedControl/Open Loop to Closed Loop/Sensorless/OpenLoop to ClosedLoop'
 * '<S56>'  : 'FOCSpeedControl/Zero_Cancellation/IIR Filter'
 * '<S57>'  : 'FOCSpeedControl/Zero_Cancellation/IIR Filter/Low-pass'
 * '<S58>'  : 'FOCSpeedControl/Zero_Cancellation/IIR Filter/Low-pass/IIR Low Pass Filter'
 */
#endif                                 /* RTW_HEADER_FOCSpeedControl_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
