/*
 * File: FOCCurrentControl_private.h
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

#ifndef RTW_HEADER_FOCCurrentControl_private_h_
#define RTW_HEADER_FOCCurrentControl_private_h_
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "FOCCurrentControl.h"
#include "FOCCurrentControl_types.h"
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

extern void FOCCurrentControl_Accumulate(boolean_T rtu_Enable, real32_T
  rtu_Theta, real32_T rtu_Theta_e_prev, real32_T *rty_theta_e,
  B_Accumulate_FOCCurrentContro_T *localB, DW_Accumulate_FOCCurrentContr_T
  *localDW);
extern void FOCCurrentCon_IfActionSubsystem(real32_T rtu_In1, real32_T *rty_Out1);
extern void FOCCurrentCo_IfActionSubsystem1(real32_T rtu_In1, real32_T *rty_Out1);
extern void FOCCurrent_InverseParkTransform(real32_T rtu_Ds, real32_T rtu_Qs,
  real32_T rtu_sin, real32_T rtu_cos, real32_T *rty_Alpha, real32_T *rty_Beta);
extern void FOCCurrentControl_ParkTransform(real32_T rtu_Alpha, real32_T
  rtu_Beta, real32_T rtu_sine, real32_T rtu_cos, real32_T *rty_Ds, real32_T
  *rty_Qs);

#endif                             /* RTW_HEADER_FOCCurrentControl_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
