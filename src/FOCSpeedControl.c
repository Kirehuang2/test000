/*
 * File: FOCSpeedControl.c
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

#include "FOCSpeedControl.h"
#include "rtwtypes.h"
#include "FOCSpeedControl_types.h"
#include "ConfigParameters.h"

/* Named constants for Chart: '<S54>/OpenLoop to ClosedLoop' */
#define FOCSpeedCont_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define FOCSpeedContr_IN_ClosedLoopTemp ((uint8_T)2U)
#define FOCSpeedControl_IN_ClosedLoop  ((uint8_T)1U)
#define FOCSpeedControl_IN_Init        ((uint8_T)1U)
#define FOCSpeedControl_IN_OpenLoop    ((uint8_T)3U)
#define FOCSpeedControl_IN_Run         ((uint8_T)2U)

/* Block states (default storage) */
DW_FOCSpeedControl_T FOCSpeedControl_DW;

/* Real-time model */
static RT_MODEL_FOCSpeedControl_T FOCSpeedControl_M_;
RT_MODEL_FOCSpeedControl_T *const FOCSpeedControl_M = &FOCSpeedControl_M_;

/* Model step function */
void SpeedControl_step(boolean_T arg_Enable, real32_T arg_SpeedRef_PU, real32_T
  arg_SpeedFb_PU, real32_T arg_IqFb, real32_T arg_Mode, real32_T arg_IdqRef_PU[2],
  real32_T *arg_SpeedRefOut_PU, boolean_T *arg_EnClOut)
{
  real32_T rtb_Add;
  real32_T rtb_DeadZone;
  real32_T rtb_IProdOut;
  int8_T tmp;
  int8_T tmp_0;
  boolean_T rtb_AND;

  /* Chart: '<S54>/OpenLoop to ClosedLoop' incorporates:
   *  Constant: '<Root>/Constant'
   *  Inport: '<Root>/Enable'
   *  Inport: '<Root>/Mode'
   *  Inport: '<Root>/SpeedFb_PU'
   *  Inport: '<Root>/SpeedRef_PU'
   */
  if (FOCSpeedControl_DW.temporalCounter_i1 < MAX_uint32_T) {
    FOCSpeedControl_DW.temporalCounter_i1++;
  }

  if (FOCSpeedControl_DW.is_active_c1_FOCSpeedControl == 0U) {
    FOCSpeedControl_DW.is_active_c1_FOCSpeedControl = 1U;
    FOCSpeedControl_DW.is_c1_FOCSpeedControl = FOCSpeedControl_IN_Init;
    *arg_SpeedRefOut_PU = 0.0F;
    *arg_EnClOut = false;
  } else if (FOCSpeedControl_DW.is_c1_FOCSpeedControl == FOCSpeedControl_IN_Init)
  {
    if (arg_Enable && (arg_Mode >= 2.0F)) {
      FOCSpeedControl_DW.is_c1_FOCSpeedControl = FOCSpeedControl_IN_Run;
      FOCSpeedControl_DW.is_Run = FOCSpeedControl_IN_OpenLoop;
      FOCSpeedControl_DW.temporalCounter_i1 = 0U;
      *arg_SpeedRefOut_PU = arg_SpeedRef_PU;  // Pass through speed reference
      *arg_EnClOut = true;  // Enable closed-loop immediately (Hall sensor available)
    } else {
      *arg_SpeedRefOut_PU = 0.0F;
      *arg_EnClOut = false;  // Keep false until Enable
    }

    /* case IN_Run: */
  } else if (!arg_Enable) {
    FOCSpeedControl_DW.is_Run = FOCSpeedCont_IN_NO_ACTIVE_CHILD;
    FOCSpeedControl_DW.is_c1_FOCSpeedControl = FOCSpeedControl_IN_Init;
    *arg_SpeedRefOut_PU = 0.0F;
    *arg_EnClOut = false;
  } else {
    switch (FOCSpeedControl_DW.is_Run) {
     case FOCSpeedControl_IN_ClosedLoop:
      *arg_SpeedRefOut_PU = arg_SpeedRef_PU;
      *arg_EnClOut = true;
      break;

     case FOCSpeedContr_IN_ClosedLoopTemp:
      if (FOCSpeedControl_DW.temporalCounter_i1 >= 999U) {
        FOCSpeedControl_DW.is_Run = FOCSpeedControl_IN_ClosedLoop;
        *arg_SpeedRefOut_PU = arg_SpeedRef_PU;
        *arg_EnClOut = true;
      } else {
        *arg_SpeedRefOut_PU = 0.15F;
        *arg_EnClOut = true;
      }
      break;

     default:
      /* case IN_OpenLoop: */
      /* MODIFIED: Always enable closed-loop since Hall sensor is available */
      /* Original transition condition bypassed */
      *arg_SpeedRefOut_PU = arg_SpeedRef_PU;  // Pass through speed reference
      *arg_EnClOut = true;  // Always enable closed-loop
      break;
    }
  }

  /* End of Chart: '<S54>/OpenLoop to ClosedLoop' */

  /* Logic: '<Root>/AND' incorporates:
   *  Inport: '<Root>/Enable'
   */
  rtb_AND = (arg_Enable && (*arg_EnClOut));

  /* Switch: '<Root>/Switch' incorporates:
   *  Inport: '<Root>/SpeedFb_PU'
   */
  if (rtb_AND) {
    rtb_IProdOut = *arg_SpeedRefOut_PU;
  } else {
    rtb_IProdOut = arg_SpeedFb_PU;
  }

  /* Sum: '<S58>/Add1' incorporates:
   *  Constant: '<S58>/Filter_Constant'
   *  Constant: '<S58>/One'
   *  Product: '<S58>/Product'
   *  Product: '<S58>/Product1'
   *  Switch: '<Root>/Switch'
   *  UnitDelay: '<S58>/Unit Delay'
   */
  FOCSpeedControl_DW.UnitDelay_DSTATE = rtb_IProdOut * 0.01F + 0.99F *
    FOCSpeedControl_DW.UnitDelay_DSTATE;

  /* Sum: '<Root>/Sum' incorporates:
   *  Inport: '<Root>/SpeedFb_PU'
   *  UnitDelay: '<S58>/Unit Delay'
   */
  rtb_IProdOut = FOCSpeedControl_DW.UnitDelay_DSTATE - arg_SpeedFb_PU;

  /* Logic: '<Root>/Logical Operator' */
  rtb_AND = !rtb_AND;

  /* Product: '<Root>/Product' incorporates:
   *  Constant: '<Root>/Kp2'
   *  Product: '<S42>/PProd Out'
   */
  rtb_DeadZone = PI_params.Kp_speed * rtb_IProdOut;

  /* Sum: '<Root>/Add' incorporates:
   *  Inport: '<Root>/IqFb'
   *  Product: '<Root>/Product'
   */
  rtb_Add = arg_IqFb - rtb_DeadZone;

  /* DiscreteIntegrator: '<S37>/Integrator' */
  /* MODIFIED: Initialize integrator to 0 instead of rtb_Add */
  /* Original code used rtb_Add (IqFb - P_output) for bumpless transfer */
  /* Since we start in closed-loop from beginning, initialize to 0 */
  if (FOCSpeedControl_DW.Integrator_IC_LOADING != 0) {
    FOCSpeedControl_DW.Integrator_DSTATE = 0.0F;
  }

  if (rtb_AND || (FOCSpeedControl_DW.Integrator_PrevResetState != 0)) {
    FOCSpeedControl_DW.Integrator_DSTATE = 0.0F;
  }

  /* Sum: '<S46>/Sum' incorporates:
   *  DiscreteIntegrator: '<S37>/Integrator'
   */
  rtb_DeadZone += FOCSpeedControl_DW.Integrator_DSTATE;

  /* Outport: '<Root>/IdqRef_PU' incorporates:
   *  Constant: '<Root>/Id_ref'
   */
  arg_IdqRef_PU[0] = 0.0F;

  /* Saturate: '<S44>/Saturation' incorporates:
   *  DeadZone: '<S30>/DeadZone'
   */
  if (rtb_DeadZone > 0.193939403F) {
    /* Outport: '<Root>/IdqRef_PU' */
    arg_IdqRef_PU[1] = 0.193939403F;
    rtb_DeadZone -= 0.193939403F;
  } else {
    if (rtb_DeadZone < -0.193939403F) {
      /* Outport: '<Root>/IdqRef_PU' */
      arg_IdqRef_PU[1] = -0.193939403F;
    } else {
      /* Outport: '<Root>/IdqRef_PU' */
      arg_IdqRef_PU[1] = rtb_DeadZone;
    }

    if (rtb_DeadZone >= -0.193939403F) {
      rtb_DeadZone = 0.0F;
    } else {
      rtb_DeadZone -= -0.193939403F;
    }
  }

  /* End of Saturate: '<S44>/Saturation' */

  /* Product: '<S34>/IProd Out' incorporates:
   *  Constant: '<Root>/Ki1'
   */
  rtb_IProdOut *= PI_params.Ki_speed * (real32_T)Ts_speed;

  /* Update for DiscreteIntegrator: '<S37>/Integrator' */
  FOCSpeedControl_DW.Integrator_IC_LOADING = 0U;

  /* Switch: '<S28>/Switch1' incorporates:
   *  Constant: '<S28>/Clamping_zero'
   *  Constant: '<S28>/Constant'
   *  Constant: '<S28>/Constant2'
   *  RelationalOperator: '<S28>/fix for DT propagation issue'
   */
  if (rtb_DeadZone > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S28>/Switch2' incorporates:
   *  Constant: '<S28>/Clamping_zero'
   *  Constant: '<S28>/Constant3'
   *  Constant: '<S28>/Constant4'
   *  RelationalOperator: '<S28>/fix for DT propagation issue1'
   */
  if (rtb_IProdOut > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S28>/Switch' incorporates:
   *  Constant: '<S28>/Clamping_zero'
   *  Constant: '<S28>/Constant1'
   *  Logic: '<S28>/AND3'
   *  RelationalOperator: '<S28>/Equal1'
   *  RelationalOperator: '<S28>/Relational Operator'
   *  Switch: '<S28>/Switch1'
   *  Switch: '<S28>/Switch2'
   */
  if ((rtb_DeadZone != 0.0F) && (tmp == tmp_0)) {
    rtb_IProdOut = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S37>/Integrator' incorporates:
   *  Switch: '<S28>/Switch'
   */
  FOCSpeedControl_DW.Integrator_DSTATE += rtb_IProdOut;
  FOCSpeedControl_DW.Integrator_PrevResetState = (int8_T)rtb_AND;
}

/* Model initialize function */
void SpeedControl_initialize(void)
{
  /* InitializeConditions for DiscreteIntegrator: '<S37>/Integrator' */
  FOCSpeedControl_DW.Integrator_IC_LOADING = 1U;
}

/* Model terminate function */
void FOCSpeedControl_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
