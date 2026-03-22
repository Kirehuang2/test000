/*
 * File: FOCCurrentControl.c
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

#include "FOCCurrentControl.h"
#include "rtwtypes.h"
#include "FOCCurrentControl_private.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "FOCCurrentControl_types.h"
#include "ConfigParameters.h"
#include "zero_crossing_types.h"

/* Block signals (default storage) */
B_FOCCurrentControl_T FOCCurrentControl_B;

/* Block states (default storage) */
DW_FOCCurrentControl_T FOCCurrentControl_DW;

/* Previous zero-crossings (trigger) states */
PrevZCX_FOCCurrentControl_T FOCCurrentControl_PrevZCX;

/* Real-time model */
static RT_MODEL_FOCCurrentControl_T FOCCurrentControl_M_;
RT_MODEL_FOCCurrentControl_T *const FOCCurrentControl_M = &FOCCurrentControl_M_;

/*
 * Output and update for enable system:
 *    '<S35>/Accumulate'
 *    '<S285>/Accumulate'
 */
void FOCCurrentControl_Accumulate(boolean_T rtu_Enable, real32_T rtu_Theta,
  real32_T rtu_Theta_e_prev, real32_T *rty_theta_e,
  B_Accumulate_FOCCurrentContro_T *localB, DW_Accumulate_FOCCurrentContr_T
  *localDW)
{
  /* Outputs for Enabled SubSystem: '<S35>/Accumulate' incorporates:
   *  EnablePort: '<S89>/Enable'
   */
  if (rtu_Enable) {
    real32_T rtb_Add_c;
    real32_T tmp;

    /* Outputs for Enabled SubSystem: '<S89>/Subsystem' incorporates:
     *  EnablePort: '<S90>/Enable'
     */
    /* Delay: '<S89>/Delay' */
    if (localDW->Delay_DSTATE) {
      /* SignalConversion generated from: '<S90>/Input' */
      localB->Input = rtu_Theta;
    }

    /* End of Delay: '<S89>/Delay' */
    /* End of Outputs for SubSystem: '<S89>/Subsystem' */

    /* Sum: '<S89>/Add' */
    rtb_Add_c = localB->Input + rtu_Theta_e_prev;

    /* DataTypeConversion: '<S89>/Data Type Conversion' */
    tmp = floorf(rtb_Add_c);
    if (rtIsNaNF(tmp) || rtIsInfF(tmp)) {
      tmp = 0.0F;
    } else {
      tmp = fmodf(tmp, 65536.0F);
    }

    /* Sum: '<S89>/Add1' incorporates:
     *  DataTypeConversion: '<S89>/Data Type Conversion'
     *  DataTypeConversion: '<S89>/Data Type Conversion1'
     */
    *rty_theta_e = rtb_Add_c - (real32_T)(tmp < 0.0F ? (int32_T)(int16_T)
      -(int16_T)(uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);

    /* Update for Delay: '<S89>/Delay' incorporates:
     *  Constant: '<S89>/Constant'
     */
    localDW->Delay_DSTATE = true;
  }

  /* End of Outputs for SubSystem: '<S35>/Accumulate' */
}

/*
 * Output and update for action system:
 *    '<S92>/If Action Subsystem'
 *    '<S275>/If Action Subsystem'
 *    '<S302>/If Action Subsystem'
 *    '<S307>/If Action Subsystem'
 */
void FOCCurrentCon_IfActionSubsystem(real32_T rtu_In1, real32_T *rty_Out1)
{
  real32_T tmp;

  /* DataTypeConversion: '<S94>/Convert_uint16' */
  tmp = floorf(rtu_In1);
  if (rtIsNaNF(tmp) || rtIsInfF(tmp)) {
    tmp = 0.0F;
  } else {
    tmp = fmodf(tmp, 65536.0F);
  }

  /* Sum: '<S94>/Sum' incorporates:
   *  DataTypeConversion: '<S94>/Convert_back'
   *  DataTypeConversion: '<S94>/Convert_uint16'
   */
  *rty_Out1 = rtu_In1 - (real32_T)(tmp < 0.0F ? (int32_T)(int16_T)-(int16_T)
    (uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);
}

/*
 * Output and update for action system:
 *    '<S92>/If Action Subsystem1'
 *    '<S275>/If Action Subsystem1'
 *    '<S302>/If Action Subsystem1'
 *    '<S307>/If Action Subsystem1'
 */
void FOCCurrentCo_IfActionSubsystem1(real32_T rtu_In1, real32_T *rty_Out1)
{
  real32_T tmp;

  /* DataTypeConversion: '<S95>/Convert_uint16' */
  tmp = truncf(rtu_In1);
  if (rtIsNaNF(tmp) || rtIsInfF(tmp)) {
    tmp = 0.0F;
  } else {
    tmp = fmodf(tmp, 65536.0F);
  }

  /* Sum: '<S95>/Sum' incorporates:
   *  DataTypeConversion: '<S95>/Convert_back'
   *  DataTypeConversion: '<S95>/Convert_uint16'
   */
  *rty_Out1 = rtu_In1 - (real32_T)(tmp < 0.0F ? (int32_T)(int16_T)-(int16_T)
    (uint16_T)-tmp : (int32_T)(int16_T)(uint16_T)tmp);
}

/*
 * Output and update for atomic system:
 *    '<S4>/Inverse Park Transform'
 *    '<S6>/Inverse Park Transform'
 */
void FOCCurrent_InverseParkTransform(real32_T rtu_Ds, real32_T rtu_Qs, real32_T
  rtu_sin, real32_T rtu_cos, real32_T *rty_Alpha, real32_T *rty_Beta)
{
  /* Sum: '<S144>/sum_beta' incorporates:
   *  Product: '<S144>/dsin'
   *  Product: '<S144>/qcos'
   */
  *rty_Beta = rtu_Qs * rtu_cos + rtu_Ds * rtu_sin;

  /* Sum: '<S144>/sum_alpha' incorporates:
   *  Product: '<S144>/dcos'
   *  Product: '<S144>/qsin'
   */
  *rty_Alpha = rtu_Ds * rtu_cos - rtu_Qs * rtu_sin;
}

/*
 * Output and update for atomic system:
 *    '<S4>/Park Transform'
 *    '<S6>/Park Transform'
 */
void FOCCurrentControl_ParkTransform(real32_T rtu_Alpha, real32_T rtu_Beta,
  real32_T rtu_sine, real32_T rtu_cos, real32_T *rty_Ds, real32_T *rty_Qs)
{
  /* AlgorithmDescriptorDelegate generated from: '<S147>/a16' incorporates:
   *  Product: '<S147>/acos'
   *  Product: '<S147>/asin'
   *  Product: '<S147>/bcos'
   *  Product: '<S147>/bsin'
   *  Sum: '<S147>/sum_Ds'
   *  Sum: '<S147>/sum_Qs'
   */
  *rty_Ds = rtu_Alpha * rtu_cos + rtu_Beta * rtu_sine;
  *rty_Qs = rtu_Beta * rtu_cos - rtu_Alpha * rtu_sine;
}

/* Model step function */
void FOCCurrentControl_step(boolean_T arg_Enable, real32_T arg_IdqRef_PU[2],
  real32_T arg_SpeedRef_PU, boolean_T arg_EnClIn, uint16_T arg_Iab_ADC[2],
  uint16_T arg_ENCCounts, real32_T arg_HallAngle_rad, real32_T arg_Vabc_PU[3],
  real32_T *arg_SpeedFb_PU, real32_T *arg_Iq, real32_T *arg_Mode, uint16_T arg_Out1[2])
{
  int32_T rtb_Mode;
  real32_T rtb_DeadZone;
  real32_T rtb_Gain1;
  real32_T rtb_Gain_idx_0;
  real32_T rtb_Gain_idx_1;
  real32_T rtb_Gain_pt;
  real32_T rtb_Integrator;
  real32_T rtb_Product2_b;
  real32_T rtb_Sum1_oa;
  real32_T rtb_Sum2_h;
  real32_T rtb_Sum_na;
  real32_T rtb_Sum_ox;
  real32_T rtb_algDD_o1_n;
  real32_T rtb_algDD_o2_h;
  real32_T rtb_speed;
  uint16_T rtb_Get_Integer;
  uint16_T rtb_Get_Integer_p;
  int8_T tmp;
  int8_T tmp_0;
  boolean_T rtb_NOT;
  boolean_T rtb_NOT_h_tmp;
  UNUSED_PARAMETER(arg_ENCCounts);

  /* Initialize Mode to running state (bypassing offset calibration) */
  rtb_Mode = 2;  // Mode 2 = Normal operation
  rtb_NOT = !arg_Enable;
  rtb_NOT_h_tmp = !arg_EnClIn;

  /* ============================================================
   * Current Measurement (Active Code - Outside #if 0 block)
   * ============================================================
   * Convert ADC values to per-unit current
   * Using fixed ADC midpoint offset (2048) for 12-bit ADC
   * Scale factor: 1/2048 = 0.00048828125
   * Sign restored to original (no inversion)
   */
  rtb_Gain_idx_0 = (real32_T)(int16_T)((int16_T)arg_Iab_ADC[0] - (int16_T)2048) * 0.00048828125F;
  rtb_Gain_idx_1 = (real32_T)(int16_T)((int16_T)arg_Iab_ADC[1] - (int16_T)2048) * 0.00048828125F;

  /* Clarke Transform: Convert Iab to Ialpha-Ibeta */
  /* Ialpha = Ia (rtb_Gain_idx_0 remains unchanged) */
  /* Ibeta = (Ia + 2*Ib) / sqrt(3) */
  rtb_Gain_idx_1 = ((rtb_Gain_idx_0 + rtb_Gain_idx_1) + rtb_Gain_idx_1) * 0.577350259F;

#if 0  // ===== SENSORLESS ESTIMATION DISABLED (Using Hall Sensor instead) =====
  /* Step: '<S8>/Mode' */
  if (((FOCCurrentControl_M->Timing.clockTick0) * 0.0001) < 1.0) {
    rtb_Mode = 0;

    /* Outputs for Enabled SubSystem: '<S5>/Enabled Subsystem' incorporates:
     *  EnablePort: '<S280>/Enable'
     */
    /* Sum: '<S280>/Add1' incorporates:
     *  Constant: '<S280>/Constant'
     *  UnitDelay: '<S280>/Unit Delay1'
     */
    FOCCurrentControl_DW.UnitDelay1_DSTATE++;

    /* Sum: '<S280>/Add' incorporates:
     *  Gain: '<S280>/Gain'
     *  Inport: '<Root>/Iab_ADC'
     *  UnitDelay: '<S280>/Unit Delay'
     */
    rtb_Gain_idx_0 = 0.000999987125F * (real32_T)arg_Iab_ADC[0] +
      FOCCurrentControl_DW.UnitDelay_DSTATE_p[0];
    FOCCurrentControl_DW.UnitDelay_DSTATE_p[0] = rtb_Gain_idx_0;

    /* Gain: '<S280>/Gain1' incorporates:
     *  Product: '<S280>/Divide'
     *  UnitDelay: '<S280>/Unit Delay1'
     */
    rtb_Sum_ox = floorf(rtb_Gain_idx_0 / (real32_T)
                        FOCCurrentControl_DW.UnitDelay1_DSTATE * 1000.0F);
    if (rtIsNaNF(rtb_Sum_ox) || rtIsInfF(rtb_Sum_ox)) {
      rtb_Sum_ox = 0.0F;
    } else {
      rtb_Sum_ox = fmodf(rtb_Sum_ox, 65536.0F);
    }

    /* Gain: '<S280>/Gain1' */
    FOCCurrentControl_B.Gain1[0] = (uint16_T)(rtb_Sum_ox < 0.0F ? (int32_T)
      (uint16_T)-(int16_T)(uint16_T)-rtb_Sum_ox : (int32_T)(uint16_T)rtb_Sum_ox);

    /* Sum: '<S280>/Add' incorporates:
     *  Gain: '<S280>/Gain'
     *  Inport: '<Root>/Iab_ADC'
     *  UnitDelay: '<S280>/Unit Delay'
     */
    rtb_Gain_idx_0 = 0.000999987125F * (real32_T)arg_Iab_ADC[1] +
      FOCCurrentControl_DW.UnitDelay_DSTATE_p[1];
    FOCCurrentControl_DW.UnitDelay_DSTATE_p[1] = rtb_Gain_idx_0;

    /* Gain: '<S280>/Gain1' incorporates:
     *  Product: '<S280>/Divide'
     *  UnitDelay: '<S280>/Unit Delay1'
     */
    rtb_Sum_ox = floorf(rtb_Gain_idx_0 / (real32_T)
                        FOCCurrentControl_DW.UnitDelay1_DSTATE * 1000.0F);
    if (rtIsNaNF(rtb_Sum_ox) || rtIsInfF(rtb_Sum_ox)) {
      rtb_Sum_ox = 0.0F;
    } else {
      rtb_Sum_ox = fmodf(rtb_Sum_ox, 65536.0F);
    }

    /* Gain: '<S280>/Gain1' */
    FOCCurrentControl_B.Gain1[1] = (uint16_T)(rtb_Sum_ox < 0.0F ? (int32_T)
      (uint16_T)-(int16_T)(uint16_T)-rtb_Sum_ox : (int32_T)(uint16_T)rtb_Sum_ox);

    /* End of Outputs for SubSystem: '<S5>/Enabled Subsystem' */
  } else {
    rtb_Mode = 2;
  }

  /* End of Step: '<S8>/Mode' */

  /* Gain: '<S5>/Gain' incorporates:
   *  Inport: '<Root>/Iab_ADC'
   *  Sum: '<S5>/Add'
   *  Modified: Use fixed ADC midpoint offset (2048) instead of FOCCurrentControl_B.Gain1
   *            since calibration mode is bypassed (rtb_Mode = 2)
   */
  rtb_Gain_idx_0 = (real32_T)(int16_T)((int16_T)arg_Iab_ADC[0] - (int16_T)2048) * 0.00048828125F;
  rtb_Gain_idx_1 = (real32_T)(int16_T)((int16_T)arg_Iab_ADC[1] - (int16_T)2048) * 0.00048828125F;

  /* Outputs for Atomic SubSystem: '<S4>/Clarke Transform' */
  /* Gain: '<S143>/one_by_sqrt3' incorporates:
   *  Sum: '<S143>/a_plus_2b'
   */
  rtb_Gain_idx_1 = ((rtb_Gain_idx_0 + rtb_Gain_idx_1) + rtb_Gain_idx_1) *
    0.577350259F;

  /* End of Outputs for SubSystem: '<S4>/Clarke Transform' */

  /* Logic: '<S8>/NOT' incorporates:
   *  Inport: '<Root>/Enable'
   */
  rtb_NOT = !arg_Enable;

  /* Delay: '<S28>/Delay' */
  if ((((FOCCurrentControl_PrevZCX.Delay_Reset_ZCE == POS_ZCSIG) != (int32_T)
        rtb_NOT) && (FOCCurrentControl_PrevZCX.Delay_Reset_ZCE !=
                     UNINITIALIZED_ZCSIG)) || rtb_NOT) {
    /* Delay: '<S28>/Delay' */
    FOCCurrentControl_DW.Delay_DSTATE = 0.0F;
  }

  FOCCurrentControl_PrevZCX.Delay_Reset_ZCE = rtb_NOT;

  /* If: '<S92>/If' incorporates:
   *  Constant: '<S93>/Constant'
   *  RelationalOperator: '<S93>/Compare'
   */
  if (FOCCurrentControl_DW.Delay_DSTATE < 0.0F) {
    /* Outputs for IfAction SubSystem: '<S92>/If Action Subsystem' incorporates:
     *  ActionPort: '<S94>/Action Port'
     */
    FOCCurrentCon_IfActionSubsystem(FOCCurrentControl_DW.Delay_DSTATE,
      &rtb_Product2_b);

    /* End of Outputs for SubSystem: '<S92>/If Action Subsystem' */
  } else {
    /* Outputs for IfAction SubSystem: '<S92>/If Action Subsystem1' incorporates:
     *  ActionPort: '<S95>/Action Port'
     */
    FOCCurrentCo_IfActionSubsystem1(FOCCurrentControl_DW.Delay_DSTATE,
      &rtb_Product2_b);

    /* End of Outputs for SubSystem: '<S92>/If Action Subsystem1' */
  }

  /* End of If: '<S92>/If' */

  /* Gain: '<S36>/indexing' */
  rtb_Product2_b *= 800.0F;

  /* DataTypeConversion: '<S36>/Get_Integer' */
  rtb_Sum_ox = truncf(rtb_Product2_b);
  if (rtIsNaNF(rtb_Sum_ox) || rtIsInfF(rtb_Sum_ox)) {
    rtb_Sum_ox = 0.0F;
  } else {
    rtb_Sum_ox = fmodf(rtb_Sum_ox, 65536.0F);
  }

  rtb_Get_Integer_p = (uint16_T)(rtb_Sum_ox < 0.0F ? (int32_T)(uint16_T)
    -(int16_T)(uint16_T)-rtb_Sum_ox : (int32_T)(uint16_T)rtb_Sum_ox);

  /* End of DataTypeConversion: '<S36>/Get_Integer' */

  /* Sum: '<S36>/Sum2' incorporates:
   *  DataTypeConversion: '<S36>/Data Type Conversion1'
   */
  rtb_Sum2_h = rtb_Product2_b - (real32_T)rtb_Get_Integer_p;

  /* Delay: '<S9>/Delay1' incorporates:
   *  Delay: '<S28>/Delay'
   */
  if ((((FOCCurrentControl_PrevZCX.Delay1_Reset_ZCE == POS_ZCSIG) != (int32_T)
        rtb_NOT) && (FOCCurrentControl_PrevZCX.Delay1_Reset_ZCE !=
                     UNINITIALIZED_ZCSIG)) || rtb_NOT) {
    FOCCurrentControl_DW.Delay1_DSTATE = 0.0F;
  }

  FOCCurrentControl_PrevZCX.Delay1_Reset_ZCE = rtb_NOT;

  /* Gain: '<S15>/Gain' incorporates:
   *  Delay: '<S9>/Delay1'
   *  Gain: '<S14>/Gain'
   */
  rtb_algDD_o2_h = -0.0005F * FOCCurrentControl_DW.Delay1_DSTATE;

  /* Gain: '<S22>/Gain1' incorporates:
   *  Constant: '<S12>/V_PU'
   *  Delay: '<Root>/Delay5'
   *  Product: '<S12>/Product1'
   */
  rtb_Sum_ox = 13.8564062F * FOCCurrentControl_DW.Delay5_DSTATE[1] * 2000.0F;

  /* Gain: '<S22>/Gain' incorporates:
   *  Constant: '<S12>/V_PU'
   *  Delay: '<Root>/Delay5'
   *  Product: '<S12>/Product'
   */
  rtb_Sum1_oa = 13.8564062F * FOCCurrentControl_DW.Delay5_DSTATE[0] * 2000.0F;

  /* Sum: '<S23>/Sum' incorporates:
   *  Constant: '<S23>/alpha'
   *  Gain: '<S15>/Gain'
   *  Product: '<S23>/Product'
   *  Product: '<S23>/Product2'
   *  UnaryMinus: '<S23>/Unary Minus'
   */
  rtb_Sum_na = -rtb_algDD_o2_h * rtb_Sum_ox + -10.0F * rtb_Sum1_oa;

  /* Sum: '<S23>/Sum1' incorporates:
   *  Constant: '<S23>/alpha'
   *  Gain: '<S15>/Gain'
   *  Product: '<S23>/Product1'
   *  Product: '<S23>/Product3'
   */
  rtb_Sum1_oa = rtb_Sum_ox * -10.0F + rtb_Sum1_oa * rtb_algDD_o2_h;

  /* Product: '<S12>/Product3' incorporates:
   *  Constant: '<S12>/I_PU'
   *  Delay: '<Root>/Delay5'
   */
  rtb_Sum_ox = 8.25F * FOCCurrentControl_DW.Delay5_DSTATE[3];

  /* Product: '<S12>/Product2' incorporates:
   *  Constant: '<S12>/I_PU'
   *  Delay: '<Root>/Delay5'
   */
  rtb_Product2_b = 8.25F * FOCCurrentControl_DW.Delay5_DSTATE[2];

  /* Sum: '<S19>/Sum1' incorporates:
   *  Constant: '<S19>/alpha'
   *  Product: '<S19>/Product1'
   *  Product: '<S19>/Product3'
   */
  rtb_algDD_o1_n = rtb_Sum_ox * -10.0F + rtb_Product2_b * rtb_algDD_o2_h;

  /* Sum: '<S19>/Sum' incorporates:
   *  Constant: '<S19>/alpha'
   *  Product: '<S19>/Product'
   *  Product: '<S19>/Product2'
   *  UnaryMinus: '<S19>/Unary Minus'
   */
  rtb_Integrator = -rtb_algDD_o2_h * rtb_Sum_ox + -10.0F * rtb_Product2_b;

  /* Gain: '<S17>/Gain1' */
  rtb_Gain1 = 2000.0F * rtb_algDD_o1_n;

  /* Gain: '<S17>/Gain' */
  rtb_Gain_pt = 2000.0F * rtb_Integrator;

  /* Sum: '<S16>/Sum1' incorporates:
   *  Constant: '<S16>/(Ld-Lq)//Ld'
   *  Constant: '<S16>/R//Ld'
   *  Delay: '<S9>/Delay1'
   *  Product: '<S16>/Product'
   *  Product: '<S16>/Product3'
   *  Product: '<S16>/Product5'
   *  UnaryMinus: '<S16>/Unary Minus1'
   */
  rtb_speed = rtb_Product2_b * FOCCurrentControl_DW.Delay1_DSTATE * 0.0F +
    rtb_Sum_ox * -1500.0F;

  /* Sum: '<S16>/Sum' incorporates:
   *  Constant: '<S16>/(Ld-Lq)//Ld'
   *  Constant: '<S16>/R//Ld'
   *  Delay: '<S9>/Delay1'
   *  Product: '<S16>/Product1'
   *  Product: '<S16>/Product2'
   *  Product: '<S16>/Product4'
   *  UnaryMinus: '<S16>/Unary Minus'
   *  UnaryMinus: '<S16>/Unary Minus1'
   */
  rtb_Product2_b = -(rtb_Sum_ox * FOCCurrentControl_DW.Delay1_DSTATE) * 0.0F +
    rtb_Product2_b * -1500.0F;

  /* Delay: '<S10>/Delay' incorporates:
   *  Delay: '<S28>/Delay'
   */
  if ((((FOCCurrentControl_PrevZCX.Delay_Reset_ZCE_o == POS_ZCSIG) != (int32_T)
        rtb_NOT) && (FOCCurrentControl_PrevZCX.Delay_Reset_ZCE_o !=
                     UNINITIALIZED_ZCSIG)) || rtb_NOT) {
    FOCCurrentControl_DW.Delay_DSTATE_k[0] = 0.0F;
    FOCCurrentControl_DW.Delay_DSTATE_k[1] = 0.0F;
  }

  FOCCurrentControl_PrevZCX.Delay_Reset_ZCE_o = rtb_NOT;

  /* Sum: '<S10>/Sum2' incorporates:
   *  Delay: '<S10>/Delay'
   */
  rtb_Sum_ox = FOCCurrentControl_DW.Delay_DSTATE_k[0];
  rtb_DeadZone = FOCCurrentControl_DW.Delay_DSTATE_k[1];

  /* Gain: '<S10>/Gain2' incorporates:
   *  Constant: '<S20>/alpha'
   *  Constant: '<S21>/alpha'
   *  Delay: '<S10>/Delay'
   *  Delay: '<S9>/Delay1'
   *  Gain: '<S10>/Gain1'
   *  Product: '<S18>/Product'
   *  Product: '<S18>/Product1'
   *  Product: '<S20>/Product'
   *  Product: '<S20>/Product1'
   *  Product: '<S20>/Product2'
   *  Product: '<S20>/Product3'
   *  Product: '<S21>/Product'
   *  Product: '<S21>/Product1'
   *  Product: '<S21>/Product2'
   *  Product: '<S21>/Product3'
   *  Sum: '<S10>/Sum'
   *  Sum: '<S10>/Sum2'
   *  Sum: '<S14>/Sum'
   *  Sum: '<S20>/Sum'
   *  Sum: '<S20>/Sum1'
   *  Sum: '<S21>/Sum'
   *  Sum: '<S21>/Sum1'
   *  UnaryMinus: '<S17>/Unary Minus'
   *  UnaryMinus: '<S17>/Unary Minus1'
   *  UnaryMinus: '<S18>/Unary Minus'
   *  UnaryMinus: '<S20>/Unary Minus'
   *  UnaryMinus: '<S21>/Unary Minus'
   */
  FOCCurrentControl_DW.Delay_DSTATE_k[0] = ((((-(rtb_algDD_o1_n *
    FOCCurrentControl_DW.Delay1_DSTATE) - (-rtb_algDD_o2_h * -rtb_Gain1 + -10.0F
    * -rtb_Gain_pt)) - (-rtb_algDD_o2_h * rtb_speed + -10.0F * rtb_Product2_b))
    - rtb_Sum_na) * 0.0001F + rtb_Sum_ox) * 0.333333343F;
  FOCCurrentControl_DW.Delay_DSTATE_k[1] = ((((rtb_Integrator *
    FOCCurrentControl_DW.Delay1_DSTATE - (-rtb_Gain1 * -10.0F + -rtb_Gain_pt *
    rtb_algDD_o2_h)) - (rtb_speed * -10.0F + rtb_Product2_b * rtb_algDD_o2_h)) -
    rtb_Sum1_oa) * 0.0001F + rtb_DeadZone) * 0.333333343F;

  /* Sum: '<S26>/Add1' incorporates:
   *  Constant: '<S26>/Filter_Constant'
   *  Constant: '<S26>/One'
   *  Constant: '<S9>/V_PU'
   *  Delay: '<S10>/Delay'
   *  Product: '<S26>/Product'
   *  Product: '<S26>/Product1'
   *  Product: '<S9>/Product'
   *  Sum: '<S10>/Sum1'
   *  UnitDelay: '<S26>/Unit Delay'
   */
  FOCCurrentControl_DW.UnitDelay_DSTATE[0] =
    (FOCCurrentControl_DW.Delay_DSTATE_k[0] + rtb_Integrator) * 0.0721687824F *
    0.824941874F + 0.175058112F * FOCCurrentControl_DW.UnitDelay_DSTATE[0];
  FOCCurrentControl_DW.UnitDelay_DSTATE[1] =
    (FOCCurrentControl_DW.Delay_DSTATE_k[1] + rtb_algDD_o1_n) * 0.0721687824F *
    0.824941874F + 0.175058112F * FOCCurrentControl_DW.UnitDelay_DSTATE[1];

  /* Relay: '<S27>/AlphaRelay' incorporates:
   *  UnitDelay: '<S26>/Unit Delay'
   */
  FOCCurrentControl_DW.AlphaRelay_Mode =
    ((FOCCurrentControl_DW.UnitDelay_DSTATE[0] >= 0.02F) ||
     ((!(FOCCurrentControl_DW.UnitDelay_DSTATE[0] <= -0.02F)) &&
      FOCCurrentControl_DW.AlphaRelay_Mode));

  /* Relay: '<S27>/BetaRelay' incorporates:
   *  UnitDelay: '<S26>/Unit Delay'
   */
  FOCCurrentControl_DW.BetaRelay_Mode = ((FOCCurrentControl_DW.UnitDelay_DSTATE
    [1] >= 0.02F) || ((!(FOCCurrentControl_DW.UnitDelay_DSTATE[1] <= -0.02F)) &&
                      FOCCurrentControl_DW.BetaRelay_Mode));

  /* Outputs for Triggered SubSystem: '<S27>/Dir_Sense' incorporates:
   *  TriggerPort: '<S29>/Trigger'
   */
  /* Relay: '<S27>/AlphaRelay' */
  if (FOCCurrentControl_DW.AlphaRelay_Mode &&
      (FOCCurrentControl_PrevZCX.Dir_Sense_Trig_ZCE != POS_ZCSIG)) {
    /* Switch: '<S29>/Switch' incorporates:
     *  Relay: '<S27>/BetaRelay'
     */
    if (FOCCurrentControl_DW.BetaRelay_Mode) {
      /* Switch: '<S29>/Switch' incorporates:
       *  Constant: '<S29>/Constant'
       */
      FOCCurrentControl_B.Switch = -1;
    } else {
      /* Switch: '<S29>/Switch' incorporates:
       *  Constant: '<S29>/Constant1'
       */
      FOCCurrentControl_B.Switch = 1;
    }

    /* End of Switch: '<S29>/Switch' */
  }

  FOCCurrentControl_PrevZCX.Dir_Sense_Trig_ZCE =
    FOCCurrentControl_DW.AlphaRelay_Mode;

  /* End of Outputs for SubSystem: '<S27>/Dir_Sense' */

  /* If: '<S27>/If' */
  if (FOCCurrentControl_B.Switch > 0) {
    /* Outputs for IfAction SubSystem: '<S27>/Subsystem2' incorporates:
     *  ActionPort: '<S30>/Action Port'
     */
    /* UnaryMinus: '<S30>/Unary Minus' incorporates:
     *  UnitDelay: '<S26>/Unit Delay'
     */
    rtb_speed = -FOCCurrentControl_DW.UnitDelay_DSTATE[0];

    /* SignalConversion generated from: '<S30>/SigmaBeta' incorporates:
     *  UnitDelay: '<S26>/Unit Delay'
     */
    rtb_algDD_o2_h = FOCCurrentControl_DW.UnitDelay_DSTATE[1];

    /* End of Outputs for SubSystem: '<S27>/Subsystem2' */
  } else {
    /* Outputs for IfAction SubSystem: '<S27>/Subsystem3' incorporates:
     *  ActionPort: '<S31>/Action Port'
     */
    /* UnaryMinus: '<S31>/Unary Minus' incorporates:
     *  UnitDelay: '<S26>/Unit Delay'
     */
    rtb_algDD_o2_h = -FOCCurrentControl_DW.UnitDelay_DSTATE[1];

    /* SignalConversion generated from: '<S31>/SigmaAlpha' incorporates:
     *  UnitDelay: '<S26>/Unit Delay'
     */
    rtb_speed = FOCCurrentControl_DW.UnitDelay_DSTATE[0];

    /* End of Outputs for SubSystem: '<S27>/Subsystem3' */
  }

  /* End of If: '<S27>/If' */

  /* Sqrt: '<S33>/Sqrt' incorporates:
   *  Math: '<S33>/Square'
   *  Math: '<S33>/Square1'
   *  Sum: '<S33>/Sum'
   */
  rtb_Sum_ox = sqrtf(rtb_speed * rtb_speed + rtb_algDD_o2_h * rtb_algDD_o2_h);

  /* Switch: '<S33>/Switch' incorporates:
   *  Constant: '<S33>/Constant'
   */
  if (!(rtb_Sum_ox > 1.0E-6F)) {
    rtb_Sum_ox = 1.0E-6F;
  }

  /* End of Switch: '<S33>/Switch' */

  /* Product: '<S33>/Divide1' */
  rtb_DeadZone = rtb_speed / rtb_Sum_ox;

  /* Product: '<S33>/Divide' */
  rtb_Product2_b = 1.0F / rtb_Sum_ox * rtb_algDD_o2_h;

  /* Sum: '<S28>/Sum' incorporates:
   *  Constant: '<S36>/offset'
   *  Constant: '<S36>/sine_table_values'
   *  Product: '<S28>/Product'
   *  Product: '<S28>/Product1'
   *  Product: '<S91>/Product'
   *  Product: '<S91>/Product1'
   *  Selector: '<S36>/Lookup'
   *  Sum: '<S36>/Sum'
   *  Sum: '<S91>/Sum3'
   *  Sum: '<S91>/Sum4'
   *  Sum: '<S91>/Sum5'
   *  Sum: '<S91>/Sum6'
   */
  rtb_speed = ((FOCCurrentControl_ConstP.pooled12[(int32_T)(rtb_Get_Integer_p +
    201U)] - FOCCurrentControl_ConstP.pooled12[(int32_T)(rtb_Get_Integer_p +
    200U)]) * rtb_Sum2_h + FOCCurrentControl_ConstP.pooled12[(int32_T)
               (rtb_Get_Integer_p + 200U)]) * rtb_DeadZone -
    ((FOCCurrentControl_ConstP.pooled12[(int32_T)(rtb_Get_Integer_p + 1U)] -
      FOCCurrentControl_ConstP.pooled12[rtb_Get_Integer_p]) * rtb_Sum2_h +
     FOCCurrentControl_ConstP.pooled12[rtb_Get_Integer_p]) * rtb_Product2_b;

  /* DiscreteIntegrator: '<S72>/Integrator' */
  if (rtb_NOT || (FOCCurrentControl_DW.Integrator_PrevResetState != 0)) {
    FOCCurrentControl_DW.Integrator_DSTATE = 0.0F;
  }

  /* DiscreteIntegrator: '<S72>/Integrator' incorporates:
   *  Gain: '<S69>/Integral Gain'
   */
  FOCCurrentControl_DW.Integrator_DSTATE += 20.0F * rtb_speed;

  /* Outputs for IfAction SubSystem: '<S37>/OptimizedDifferentiationMethod' incorporates:
   *  ActionPort: '<S99>/Action Port'
   */
  /* SwitchCase: '<S37>/Switch Case' incorporates:
   *  Constant: '<S139>/Filter_Constant'
   *  Constant: '<S139>/One'
   *  Constant: '<S142>/Filter_Constant'
   *  Constant: '<S142>/One'
   *  Delay: '<S133>/Delay2'
   *  Delay: '<S133>/Delay3'
   *  Delay: '<S134>/Delay2'
   *  Delay: '<S134>/Delay3'
   *  Delay: '<S28>/Delay'
   *  Gain: '<S133>/Gain'
   *  Gain: '<S133>/Gain1'
   *  Gain: '<S134>/Gain'
   *  Gain: '<S134>/Gain1'
   *  Merge: '<S37>/Merge'
   *  Product: '<S139>/Product'
   *  Product: '<S139>/Product1'
   *  Product: '<S142>/Product'
   *  Product: '<S142>/Product1'
   *  Product: '<S99>/Product'
   *  Product: '<S99>/Product1'
   *  Sum: '<S133>/Sum1'
   *  Sum: '<S134>/Sum1'
   *  Sum: '<S139>/Add1'
   *  Sum: '<S142>/Add1'
   *  Sum: '<S99>/Sum'
   *  UnitDelay: '<S139>/Unit Delay'
   *  UnitDelay: '<S142>/Unit Delay'
   */
  if ((((FOCCurrentControl_PrevZCX.Delay2_Reset_ZCE == POS_ZCSIG) != (int32_T)
        rtb_NOT) && (FOCCurrentControl_PrevZCX.Delay2_Reset_ZCE !=
                     UNINITIALIZED_ZCSIG)) || rtb_NOT) {
    FOCCurrentControl_DW.Delay2_DSTATE = 0.0F;
  }

  FOCCurrentControl_PrevZCX.Delay2_Reset_ZCE = rtb_NOT;
  rtb_algDD_o2_h = FOCCurrentControl_DW.Delay2_DSTATE;
  if ((((FOCCurrentControl_PrevZCX.Delay3_Reset_ZCE == POS_ZCSIG) != (int32_T)
        rtb_NOT) && (FOCCurrentControl_PrevZCX.Delay3_Reset_ZCE !=
                     UNINITIALIZED_ZCSIG)) || rtb_NOT) {
    FOCCurrentControl_DW.Delay3_DSTATE = 0.0F;
  }

  FOCCurrentControl_PrevZCX.Delay3_Reset_ZCE = rtb_NOT;
  FOCCurrentControl_DW.UnitDelay_DSTATE_cl = rtb_DeadZone * 0.566518307F +
    0.433481693F * FOCCurrentControl_DW.UnitDelay_DSTATE_cl;
  FOCCurrentControl_DW.Delay2_DSTATE = 65345.1289F *
    FOCCurrentControl_DW.UnitDelay_DSTATE_cl;
  FOCCurrentControl_DW.Delay3_DSTATE = ((FOCCurrentControl_DW.Delay3_DSTATE +
    FOCCurrentControl_DW.Delay2_DSTATE) - rtb_algDD_o2_h) * 0.132722586F;
  if ((((FOCCurrentControl_PrevZCX.Delay2_Reset_ZCE_f == POS_ZCSIG) != (int32_T)
        rtb_NOT) && (FOCCurrentControl_PrevZCX.Delay2_Reset_ZCE_f !=
                     UNINITIALIZED_ZCSIG)) || rtb_NOT) {
    FOCCurrentControl_DW.Delay2_DSTATE_i = 0.0F;
  }

  FOCCurrentControl_PrevZCX.Delay2_Reset_ZCE_f = rtb_NOT;
  rtb_algDD_o2_h = FOCCurrentControl_DW.Delay2_DSTATE_i;
  if ((((FOCCurrentControl_PrevZCX.Delay3_Reset_ZCE_j == POS_ZCSIG) != (int32_T)
        rtb_NOT) && (FOCCurrentControl_PrevZCX.Delay3_Reset_ZCE_j !=
                     UNINITIALIZED_ZCSIG)) || rtb_NOT) {
    FOCCurrentControl_DW.Delay3_DSTATE_c = 0.0F;
  }

  FOCCurrentControl_PrevZCX.Delay3_Reset_ZCE_j = rtb_NOT;
  FOCCurrentControl_DW.UnitDelay_DSTATE_d = rtb_Product2_b * 0.566518307F +
    0.433481693F * FOCCurrentControl_DW.UnitDelay_DSTATE_d;
  FOCCurrentControl_DW.Delay2_DSTATE_i = 65345.1289F *
    FOCCurrentControl_DW.UnitDelay_DSTATE_d;
  FOCCurrentControl_DW.Delay3_DSTATE_c = ((FOCCurrentControl_DW.Delay3_DSTATE_c
    + FOCCurrentControl_DW.Delay2_DSTATE_i) - rtb_algDD_o2_h) * 0.132722586F;
  FOCCurrentControl_B.Merge = FOCCurrentControl_DW.Delay3_DSTATE *
    rtb_Product2_b - FOCCurrentControl_DW.Delay3_DSTATE_c * rtb_DeadZone;

  /* End of SwitchCase: '<S37>/Switch Case' */
  /* End of Outputs for SubSystem: '<S37>/OptimizedDifferentiationMethod' */

  /* Sum: '<S122>/Add1' incorporates:
   *  Constant: '<S122>/Filter_Constant'
   *  Constant: '<S122>/One'
   *  Product: '<S122>/Product'
   *  Product: '<S122>/Product1'
   *  UnitDelay: '<S122>/Unit Delay'
   */
  FOCCurrentControl_DW.UnitDelay_DSTATE_c = FOCCurrentControl_B.Merge *
    0.00933678076F + 0.99066323F * FOCCurrentControl_DW.UnitDelay_DSTATE_c;

  /* Sum: '<S28>/Sum1' incorporates:
   *  Gain: '<S77>/Proportional Gain'
   *  Sum: '<S81>/Sum'
   *  UnitDelay: '<S122>/Unit Delay'
   */
  rtb_speed = (250.0F * rtb_speed + FOCCurrentControl_DW.Integrator_DSTATE) +
    FOCCurrentControl_DW.UnitDelay_DSTATE_c;

  /* Outputs for Enabled SubSystem: '<S35>/Accumulate' */
  /* Gain: '<S28>/Gain' incorporates:
   *  UnitDelay: '<S35>/Unit Delay'
   */
  FOCCurrentControl_Accumulate(true, 1.59154952E-5F * rtb_speed,
    FOCCurrentControl_DW.UnitDelay_DSTATE_i,
    &FOCCurrentControl_DW.UnitDelay_DSTATE_i, &FOCCurrentControl_B.Accumulate,
    &FOCCurrentControl_DW.Accumulate);

  /* End of Outputs for SubSystem: '<S35>/Accumulate' */
#endif  // ===== END OF SENSORLESS ESTIMATION CODE =====

  /* ============================================================
   * Hall Sensor Angle Input (bypassing sensorless estimation)
   * ============================================================
   * Use external Hall sensor angle instead of EEMF Observer + PLL
   * Convert mechanical angle [rad] to electrical angle [rad]
   * NOTE: arg_HallAngle_rad is ALREADY electrical angle (not mechanical)
   * Hall sensor module outputs electrical angle directly
   * Do NOT multiply by pmsm.p
   */
  real32_T theta_e_rad = arg_HallAngle_rad;  // Already electrical angle [rad]

  // Angle offset will be applied in hal_entry.c for testing
  // theta_e_rad += 0.0f;

  /* Calculate sin and cos of electrical angle directly */
  rtb_Sum1_oa = sinf(theta_e_rad);  // sin(theta_e)
  rtb_Sum_na = cosf(theta_e_rad);   // cos(theta_e)

  /* Outputs for Atomic SubSystem: '<S4>/Park Transform' */
  /* Outputs for Atomic SubSystem: '<S4>/Clarke Transform' */
  /* AlgorithmDescriptorDelegate generated from: '<S143>/a16' */
  FOCCurrentControl_ParkTransform(rtb_Gain_idx_0, rtb_Gain_idx_1, rtb_Sum1_oa,
    rtb_Sum_na, &rtb_algDD_o1_n, &rtb_algDD_o2_h);

  /* End of Outputs for SubSystem: '<S4>/Clarke Transform' */
  /* End of Outputs for SubSystem: '<S4>/Park Transform' */

  /* Sum: '<S145>/Sum1' incorporates:
   *  Inport: '<Root>/IdqRef_PU'
   *  Sum: '<S1>/Add1'
   */
  rtb_Sum2_h = arg_IdqRef_PU[1] - rtb_algDD_o2_h;

  /* Logic: '<S145>/NOT' incorporates:
   *  Inport: '<Root>/EnClIn'
   *  Logic: '<Root>/NOT'
   */
  rtb_NOT_h_tmp = !arg_EnClIn;

  /* Outputs for Enabled SubSystem: '<Root>/OpenLoopControl' incorporates:
   *  EnablePort: '<S6>/Enable'
   */
  /* Logic: '<Root>/AND' incorporates:
   *  Concatenate: '<S6>/Vector Concatenate2'
   *  Constant: '<S286>/offset'
   *  Constant: '<S286>/sine_table_values'
   *  Constant: '<S287>/offset'
   *  Constant: '<S287>/sine_table_values'
   *  Constant: '<S6>/Constant'
   *  Gain: '<S6>/Gain'
   *  Inport: '<Root>/Enable'
   *  Product: '<S301>/Product'
   *  Product: '<S301>/Product1'
   *  Product: '<S306>/Product'
   *  Product: '<S306>/Product1'
   *  Saturate: '<S6>/Saturation'
   *  Selector: '<S286>/Lookup'
   *  Selector: '<S287>/Lookup'
   *  Sum: '<S286>/Sum'
   *  Sum: '<S287>/Sum'
   *  Sum: '<S301>/Sum3'
   *  Sum: '<S301>/Sum4'
   *  Sum: '<S301>/Sum5'
   *  Sum: '<S301>/Sum6'
   *  Sum: '<S306>/Sum3'
   *  Sum: '<S306>/Sum4'
   *  Sum: '<S306>/Sum5'
   *  Sum: '<S306>/Sum6'
   *  UnitDelay: '<S285>/Unit Delay'
   *  UnitDelay: '<S290>/Unit Delay'
   */
  if (rtb_NOT_h_tmp && arg_Enable) {
    if (!FOCCurrentControl_DW.OpenLoopControl_MODE) {
      /* InitializeConditions for UnitDelay: '<S290>/Unit Delay' */
      FOCCurrentControl_DW.UnitDelay_DSTATE_ct = 0.0F;

      /* InitializeConditions for UnitDelay: '<S285>/Unit Delay' */
      FOCCurrentControl_DW.UnitDelay_DSTATE_j = 0.0F;
      FOCCurrentControl_DW.OpenLoopControl_MODE = true;
    }

    /* If: '<S307>/If' incorporates:
     *  Constant: '<S308>/Constant'
     *  RelationalOperator: '<S308>/Compare'
     *  UnitDelay: '<S35>/Unit Delay'
     */
    if (FOCCurrentControl_DW.UnitDelay_DSTATE_i < 0.0F) {
      /* Outputs for IfAction SubSystem: '<S307>/If Action Subsystem' incorporates:
       *  ActionPort: '<S309>/Action Port'
       */
      FOCCurrentCon_IfActionSubsystem(FOCCurrentControl_DW.UnitDelay_DSTATE_i,
        &rtb_Gain1);

      /* End of Outputs for SubSystem: '<S307>/If Action Subsystem' */
    } else {
      /* Outputs for IfAction SubSystem: '<S307>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S310>/Action Port'
       */
      FOCCurrentCo_IfActionSubsystem1(FOCCurrentControl_DW.UnitDelay_DSTATE_i,
        &rtb_Gain1);

      /* End of Outputs for SubSystem: '<S307>/If Action Subsystem1' */
    }

    /* End of If: '<S307>/If' */

    /* Gain: '<S287>/indexing' */
    rtb_Gain1 *= 800.0F;

    /* DataTypeConversion: '<S287>/Get_Integer' */
    rtb_Sum_ox = truncf(rtb_Gain1);
    if (rtIsNaNF(rtb_Sum_ox) || rtIsInfF(rtb_Sum_ox)) {
      rtb_Sum_ox = 0.0F;
    } else {
      rtb_Sum_ox = fmodf(rtb_Sum_ox, 65536.0F);
    }

    rtb_Get_Integer_p = (uint16_T)(rtb_Sum_ox < 0.0F ? (int32_T)(uint16_T)
      -(int16_T)(uint16_T)-rtb_Sum_ox : (int32_T)(uint16_T)rtb_Sum_ox);

    /* End of DataTypeConversion: '<S287>/Get_Integer' */

    /* Sum: '<S290>/Add1' incorporates:
     *  Constant: '<S290>/Filter_Constant'
     *  Constant: '<S290>/One'
     *  Inport: '<Root>/SpeedRef_PU'
     *  Product: '<S290>/Product'
     *  Product: '<S290>/Product1'
     *  UnitDelay: '<S290>/Unit Delay'
     */
    FOCCurrentControl_DW.UnitDelay_DSTATE_ct = arg_SpeedRef_PU * 0.01F + 0.99F *
      FOCCurrentControl_DW.UnitDelay_DSTATE_ct;

    /* Outputs for Enabled SubSystem: '<S285>/Accumulate' */
    FOCCurrentControl_Accumulate(true, 0.0208F *
      FOCCurrentControl_DW.UnitDelay_DSTATE_ct,
      FOCCurrentControl_DW.UnitDelay_DSTATE_j,
      &FOCCurrentControl_DW.UnitDelay_DSTATE_j,
      &FOCCurrentControl_B.Accumulate_n, &FOCCurrentControl_DW.Accumulate_n);

    /* End of Outputs for SubSystem: '<S285>/Accumulate' */

    /* If: '<S302>/If' incorporates:
     *  Constant: '<S303>/Constant'
     *  Gain: '<S6>/Gain'
     *  RelationalOperator: '<S303>/Compare'
     *  UnitDelay: '<S285>/Unit Delay'
     *  UnitDelay: '<S290>/Unit Delay'
     */
    if (FOCCurrentControl_DW.UnitDelay_DSTATE_j < 0.0F) {
      /* Outputs for IfAction SubSystem: '<S302>/If Action Subsystem' incorporates:
       *  ActionPort: '<S304>/Action Port'
       */
      FOCCurrentCon_IfActionSubsystem(FOCCurrentControl_DW.UnitDelay_DSTATE_j,
        &rtb_Gain_pt);

      /* End of Outputs for SubSystem: '<S302>/If Action Subsystem' */
    } else {
      /* Outputs for IfAction SubSystem: '<S302>/If Action Subsystem1' incorporates:
       *  ActionPort: '<S305>/Action Port'
       */
      FOCCurrentCo_IfActionSubsystem1(FOCCurrentControl_DW.UnitDelay_DSTATE_j,
        &rtb_Gain_pt);

      /* End of Outputs for SubSystem: '<S302>/If Action Subsystem1' */
    }

    /* End of If: '<S302>/If' */

    /* Gain: '<S286>/indexing' */
    rtb_Gain_pt *= 800.0F;

    /* DataTypeConversion: '<S286>/Get_Integer' */
    rtb_Sum_ox = truncf(rtb_Gain_pt);
    if (rtIsNaNF(rtb_Sum_ox) || rtIsInfF(rtb_Sum_ox)) {
      rtb_Sum_ox = 0.0F;
    } else {
      rtb_Sum_ox = fmodf(rtb_Sum_ox, 65536.0F);
    }

    rtb_Get_Integer = (uint16_T)(rtb_Sum_ox < 0.0F ? (int32_T)(uint16_T)
      -(int16_T)(uint16_T)-rtb_Sum_ox : (int32_T)(uint16_T)rtb_Sum_ox);

    /* End of DataTypeConversion: '<S286>/Get_Integer' */

    /* Sum: '<S286>/Sum2' incorporates:
     *  DataTypeConversion: '<S286>/Data Type Conversion1'
     */
    rtb_Product2_b = rtb_Gain_pt - (real32_T)rtb_Get_Integer;

    /* Abs: '<S6>/Abs' incorporates:
     *  Inport: '<Root>/SpeedRef_PU'
     */
    rtb_Sum_ox = fabsf(arg_SpeedRef_PU);

    /* Saturate: '<S6>/Saturation' */
    if (rtb_Sum_ox > 0.5F) {
      rtb_Sum_ox = 0.5F;
    } else if (rtb_Sum_ox < pmsm.V_boost) {
      rtb_Sum_ox = pmsm.V_boost;
    }

    /* Outputs for Atomic SubSystem: '<S6>/Inverse Park Transform' */
    FOCCurrent_InverseParkTransform(rtb_Sum_ox, 0.0F,
      (FOCCurrentControl_ConstP.pooled12[(int32_T)(rtb_Get_Integer + 1U)] -
       FOCCurrentControl_ConstP.pooled12[rtb_Get_Integer]) * rtb_Product2_b +
      FOCCurrentControl_ConstP.pooled12[rtb_Get_Integer],
      (FOCCurrentControl_ConstP.pooled12[(int32_T)(rtb_Get_Integer + 201U)] -
       FOCCurrentControl_ConstP.pooled12[(int32_T)(rtb_Get_Integer + 200U)]) *
      rtb_Product2_b + FOCCurrentControl_ConstP.pooled12[(int32_T)
      (rtb_Get_Integer + 200U)], &rtb_Integrator, &rtb_DeadZone);

    /* End of Outputs for SubSystem: '<S6>/Inverse Park Transform' */

    /* Gain: '<S297>/one_by_two' incorporates:
     *  Constant: '<S286>/offset'
     *  Constant: '<S286>/sine_table_values'
     *  Constant: '<S6>/Constant'
     *  Product: '<S301>/Product'
     *  Product: '<S301>/Product1'
     *  Saturate: '<S6>/Saturation'
     *  Selector: '<S286>/Lookup'
     *  Sum: '<S286>/Sum'
     *  Sum: '<S301>/Sum3'
     *  Sum: '<S301>/Sum4'
     *  Sum: '<S301>/Sum5'
     *  Sum: '<S301>/Sum6'
     */
    rtb_Gain_pt = 0.5F * rtb_Integrator;

    /* Gain: '<S297>/sqrt3_by_two' */
    rtb_Sum_ox = 0.866025388F * rtb_DeadZone;

    /* Sum: '<S297>/add_b' */
    rtb_Product2_b = rtb_Sum_ox - rtb_Gain_pt;

    /* Sum: '<S297>/add_c' */
    rtb_Gain_pt = (0.0F - rtb_Gain_pt) - rtb_Sum_ox;

    /* Gain: '<S295>/one_by_two' incorporates:
     *  MinMax: '<S295>/Max'
     *  MinMax: '<S295>/Min'
     *  Sum: '<S295>/Add'
     */
    rtb_Sum_ox = (fmaxf(fmaxf(rtb_Integrator, rtb_Product2_b), rtb_Gain_pt) +
                  fminf(fminf(rtb_Integrator, rtb_Product2_b), rtb_Gain_pt)) *
      -0.5F;

    /* SignalConversion generated from: '<S6>/Vector Concatenate' incorporates:
     *  Concatenate: '<S6>/Vector Concatenate'
     *  Gain: '<S294>/Gain'
     *  Sum: '<S294>/Add3'
     */
    FOCCurrentControl_B.VectorConcatenate[0] = (rtb_Integrator + rtb_Sum_ox) *
      1.15470052F;

    /* SignalConversion generated from: '<S6>/Vector Concatenate' incorporates:
     *  Concatenate: '<S6>/Vector Concatenate'
     *  Gain: '<S294>/Gain'
     *  Sum: '<S294>/Add1'
     */
    FOCCurrentControl_B.VectorConcatenate[1] = (rtb_Product2_b + rtb_Sum_ox) *
      1.15470052F;

    /* SignalConversion generated from: '<S6>/Vector Concatenate' incorporates:
     *  Concatenate: '<S6>/Vector Concatenate'
     *  Gain: '<S294>/Gain'
     *  Sum: '<S294>/Add2'
     */
    FOCCurrentControl_B.VectorConcatenate[2] = (rtb_Sum_ox + rtb_Gain_pt) *
      1.15470052F;

    /* SignalConversion generated from: '<S6>/Vector Concatenate1' incorporates:
     *  Concatenate: '<S6>/Vector Concatenate1'
     */
    FOCCurrentControl_B.VectorConcatenate1[0] = rtb_Integrator;

    /* SignalConversion generated from: '<S6>/Vector Concatenate1' incorporates:
     *  Concatenate: '<S6>/Vector Concatenate1'
     */
    FOCCurrentControl_B.VectorConcatenate1[1] = rtb_DeadZone;

    /* Sum: '<S287>/Sum2' incorporates:
     *  DataTypeConversion: '<S287>/Data Type Conversion1'
     */
    rtb_Sum_ox = rtb_Gain1 - (real32_T)rtb_Get_Integer_p;

    /* Outputs for Atomic SubSystem: '<S6>/Park Transform' */
    FOCCurrentControl_ParkTransform(rtb_Integrator, rtb_DeadZone,
      (FOCCurrentControl_ConstP.pooled12[(int32_T)(rtb_Get_Integer_p + 1U)] -
       FOCCurrentControl_ConstP.pooled12[rtb_Get_Integer_p]) * rtb_Sum_ox +
      FOCCurrentControl_ConstP.pooled12[rtb_Get_Integer_p],
      (FOCCurrentControl_ConstP.pooled12[(int32_T)(rtb_Get_Integer_p + 201U)] -
       FOCCurrentControl_ConstP.pooled12[(int32_T)(rtb_Get_Integer_p + 200U)]) *
      rtb_Sum_ox + FOCCurrentControl_ConstP.pooled12[(int32_T)(rtb_Get_Integer_p
      + 200U)], &FOCCurrentControl_B.VectorConcatenate2[0],
      &FOCCurrentControl_B.VectorConcatenate2[1]);

    /* End of Outputs for SubSystem: '<S6>/Park Transform' */
  } else {
    FOCCurrentControl_DW.OpenLoopControl_MODE = false;
  }

  /* End of Logic: '<Root>/AND' */
  /* End of Outputs for SubSystem: '<Root>/OpenLoopControl' */

  /* Outport: '<Root>/Iq' incorporates:
   *  SignalConversion generated from: '<S4>/Vector Concatenate2'
   */
  *arg_Iq = rtb_algDD_o2_h;

  /* Sum: '<S1>/Add1' incorporates:
   *  Inport: '<Root>/IdqRef_PU'
   *  SignalConversion generated from: '<S4>/Vector Concatenate2'
   *  Sum: '<S145>/Sum'
   */
  rtb_Integrator = arg_IdqRef_PU[0] - rtb_algDD_o1_n;

  /* Product: '<S1>/Product' incorporates:
   *  Constant: '<S1>/Kp'
   *  Product: '<S205>/PProd Out'
   *  Sum: '<S1>/Add1'
   */
  rtb_algDD_o2_h = rtb_Integrator * PI_params.Kp_id;

  /* Sum: '<S1>/Add' incorporates:
   *  Constant: '<S1>/Kp'
   *  Product: '<S1>/Product'
   */
  rtb_Gain1 = FOCCurrentControl_B.VectorConcatenate2[0] - rtb_algDD_o2_h;
  rtb_Sum_ox = FOCCurrentControl_B.VectorConcatenate2[1] - rtb_Sum2_h *
    PI_params.Kp_id;

  /* DiscreteIntegrator: '<S250>/Integrator' incorporates:
   *  Logic: '<S145>/NOT'
   */
  /* MODIFIED: Initialize Iq integrator to 0 for closed-loop start */
  if (FOCCurrentControl_DW.Integrator_IC_LOADING != 0) {
    FOCCurrentControl_DW.Integrator_DSTATE_n = 0.0F;
  }

  if (rtb_NOT_h_tmp || (FOCCurrentControl_DW.Integrator_PrevResetState_e != 0))
  {
    FOCCurrentControl_DW.Integrator_DSTATE_n = 0.0F;
  }

  /* Sum: '<S259>/Sum' incorporates:
   *  Constant: '<S145>/Kp1'
   *  DiscreteIntegrator: '<S250>/Integrator'
   *  Product: '<S255>/PProd Out'
   *  Sum: '<S145>/Sum1'
   */
  rtb_Product2_b = rtb_Sum2_h * PI_params.Kp_iq +
    FOCCurrentControl_DW.Integrator_DSTATE_n;

  /* Saturate: '<S257>/Saturation' */
  if (rtb_Product2_b > 1.0F) {
    rtb_Sum_ox = 1.0F;
  } else if (rtb_Product2_b < -1.0F) {
    rtb_Sum_ox = -1.0F;
  } else {
    rtb_Sum_ox = rtb_Product2_b;
  }

  /* End of Saturate: '<S257>/Saturation' */

  /* DiscreteIntegrator: '<S200>/Integrator' incorporates:
   *  Logic: '<S145>/NOT'
   */
  /* MODIFIED: Initialize Id integrator to 0 for closed-loop start */
  if (FOCCurrentControl_DW.Integrator_IC_LOADING_k != 0) {
    FOCCurrentControl_DW.Integrator_DSTATE_l = 0.0F;
  }

  if (rtb_NOT_h_tmp || (FOCCurrentControl_DW.Integrator_PrevResetState_o != 0))
  {
    FOCCurrentControl_DW.Integrator_DSTATE_l = 0.0F;
  }

  /* Sum: '<S209>/Sum' incorporates:
   *  DiscreteIntegrator: '<S200>/Integrator'
   */
  rtb_DeadZone = rtb_algDD_o2_h + FOCCurrentControl_DW.Integrator_DSTATE_l;

  /* Saturate: '<S207>/Saturation' */
  if (rtb_DeadZone > 1.0F) {
    rtb_Gain1 = 1.0F;
  } else if (rtb_DeadZone < -1.0F) {
    rtb_Gain1 = -1.0F;
  } else {
    rtb_Gain1 = rtb_DeadZone;
  }

  /* End of Saturate: '<S207>/Saturation' */

  /* Sum: '<S156>/Sum1' incorporates:
   *  Product: '<S156>/Product'
   *  Product: '<S156>/Product1'
   */
  rtb_algDD_o2_h = rtb_Gain1 * rtb_Gain1 + rtb_Sum_ox * rtb_Sum_ox;

  /* Outputs for IfAction SubSystem: '<S150>/D-Q Equivalence' incorporates:
   *  ActionPort: '<S153>/Action Port'
   */
  /* If: '<S153>/If' incorporates:
   *  If: '<S150>/If'
   *  RelationalOperator: '<S153>/Relational Operator'
   */
  if (rtb_algDD_o2_h > 0.9025F) {
    /* Outputs for IfAction SubSystem: '<S153>/Limiter' incorporates:
     *  ActionPort: '<S157>/Action Port'
     */
    /* Product: '<S157>/Reciprocal' incorporates:
     *  Sqrt: '<S157>/Square Root'
     */
    rtb_algDD_o2_h = 1.0F / sqrtf(rtb_algDD_o2_h);

    /* Product: '<S157>/Product1' incorporates:
     *  Constant: '<S155>/Constant3'
     *  Product: '<S157>/Product'
     *  Switch: '<S155>/Switch'
     */
    rtb_Gain1 = rtb_Gain1 * 0.95F * rtb_algDD_o2_h;
    rtb_Sum_ox = rtb_Sum_ox * 0.95F * rtb_algDD_o2_h;

    /* End of Outputs for SubSystem: '<S153>/Limiter' */
  }

  /* End of If: '<S153>/If' */
  /* End of Outputs for SubSystem: '<S150>/D-Q Equivalence' */

  /* Outputs for Atomic SubSystem: '<S4>/Inverse Park Transform' */
  FOCCurrent_InverseParkTransform(rtb_Gain1, rtb_Sum_ox, rtb_Sum1_oa, rtb_Sum_na,
    &rtb_algDD_o1_n, &rtb_algDD_o2_h);

  /* End of Outputs for SubSystem: '<S4>/Inverse Park Transform' */

  /* Switch: '<S7>/Switch3' */
  if (rtb_Mode > 0) {
    /* Switch: '<S7>/Switch1' */
    if (rtb_Mode > 1) {
      /* Switch: '<S7>/Switch2' incorporates:
       *  Constant: '<S7>/Constant2'
       *  Inport: '<Root>/Enable'
       */
      if (arg_Enable) {
        /* Switch: '<S7>/Switch' incorporates:
         *  Inport: '<Root>/EnClIn'
         *  Switch: '<S7>/Switch2'
         */
        if (arg_EnClIn) {
          /* Gain: '<S272>/sqrt3_by_two' */
          rtb_Sum_na = 0.866025388F * rtb_algDD_o2_h;

          /* Gain: '<S272>/one_by_two' */
          rtb_Sum_ox = 0.5F * rtb_algDD_o1_n;

          /* Sum: '<S272>/add_c' */
          rtb_Sum1_oa = (0.0F - rtb_Sum_ox) - rtb_Sum_na;

          /* Sum: '<S272>/add_b' */
          rtb_Sum_ox = rtb_Sum_na - rtb_Sum_ox;

          /* Gain: '<S270>/one_by_two' incorporates:
           *  MinMax: '<S270>/Max'
           *  MinMax: '<S270>/Min'
           *  Sum: '<S270>/Add'
           */
          rtb_Sum_na = (fmaxf(fmaxf(rtb_algDD_o1_n, rtb_Sum_ox), rtb_Sum1_oa) +
                        fminf(fminf(rtb_algDD_o1_n, rtb_Sum_ox), rtb_Sum1_oa)) *
            -0.5F;

          /* SignalConversion generated from: '<S4>/Vector Concatenate' incorporates:
           *  Gain: '<S269>/Gain'
           *  Sum: '<S269>/Add2'
           *  Switch: '<S7>/Switch2'
           */
          arg_Vabc_PU[2] = (rtb_Sum_na + rtb_Sum1_oa) * 1.15470052F;

          /* SignalConversion generated from: '<S4>/Vector Concatenate' incorporates:
           *  Gain: '<S269>/Gain'
           *  Sum: '<S269>/Add1'
           *  Switch: '<S7>/Switch2'
           */
          arg_Vabc_PU[1] = (rtb_Sum_ox + rtb_Sum_na) * 1.15470052F;

          /* SignalConversion generated from: '<S4>/Vector Concatenate' incorporates:
           *  Gain: '<S269>/Gain'
           *  Sum: '<S269>/Add3'
           *  Switch: '<S7>/Switch2'
           */
          arg_Vabc_PU[0] = (rtb_algDD_o1_n + rtb_Sum_na) * 1.15470052F;
        } else {
          arg_Vabc_PU[0] = FOCCurrentControl_B.VectorConcatenate[0];
          arg_Vabc_PU[1] = FOCCurrentControl_B.VectorConcatenate[1];
          arg_Vabc_PU[2] = FOCCurrentControl_B.VectorConcatenate[2];
        }

        /* End of Switch: '<S7>/Switch' */
      } else {
        arg_Vabc_PU[0] = 0.0F;
        arg_Vabc_PU[1] = 0.0F;
        arg_Vabc_PU[2] = 0.0F;
      }

      /* End of Switch: '<S7>/Switch2' */

      /* Outport: '<Root>/Vabc_PU' incorporates:
       *  Constant: '<S7>/Constant'
       *  Gain: '<S7>/Gain'
       *  Sum: '<S7>/Add2'
       *  Switch: '<S7>/Switch2'
       */
      arg_Vabc_PU[0] = 0.5F * arg_Vabc_PU[0] + 0.5F;
      arg_Vabc_PU[1] = 0.5F * arg_Vabc_PU[1] + 0.5F;
      arg_Vabc_PU[2] = 0.5F * arg_Vabc_PU[2] + 0.5F;
    } else {
      /* Outport: '<Root>/Vabc_PU' incorporates:
       *  Constant: '<S7>/Position Offset Calib'
       *  Switch: '<S7>/Switch2'
       */
      arg_Vabc_PU[0] = 0.6F;
      arg_Vabc_PU[1] = 0.55F;
      arg_Vabc_PU[2] = 0.55F;
    }

    /* End of Switch: '<S7>/Switch1' */
  } else {
    /* Outport: '<Root>/Vabc_PU' incorporates:
     *  Constant: '<S7>/Current Offset Calib'
     *  Switch: '<S7>/Switch2'
     */
    arg_Vabc_PU[0] = 0.5F;
    arg_Vabc_PU[1] = 0.5F;
    arg_Vabc_PU[2] = 0.5F;
  }

  /* End of Switch: '<S7>/Switch3' */

  /* Sum: '<S40>/Add1' incorporates:
   *  Constant: '<S40>/Filter_Constant'
   *  Constant: '<S40>/One'
   *  Product: '<S40>/Product'
   *  Product: '<S40>/Product1'
   *  UnitDelay: '<S40>/Unit Delay'
   */
  FOCCurrentControl_DW.UnitDelay_DSTATE_cx = rtb_speed * 0.00933678076F +
    0.99066323F * FOCCurrentControl_DW.UnitDelay_DSTATE_cx;

  /* Outport: '<Root>/SpeedFb_PU' incorporates:
   *  Gain: '<S9>/SpeedGain'
   *  UnitDelay: '<S40>/Unit Delay'
   */
  *arg_SpeedFb_PU = 0.000765168F * FOCCurrentControl_DW.UnitDelay_DSTATE_cx;

  /* Outport: '<Root>/Mode' */
  *arg_Mode = (real32_T)rtb_Mode;

  /* Outport: '<Root>/DebugData' */
  arg_Out1[0] = FOCCurrentControl_B.Gain1[0];
  arg_Out1[1] = FOCCurrentControl_B.Gain1[1];

  /* Outputs for Atomic SubSystem: '<S4>/Clarke Transform' */
  /* SignalConversion generated from: '<S3>/Vector Concatenate2' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S143>/a16'
   *  Delay: '<Root>/Delay5'
   */
  FOCCurrentControl_DW.Delay5_DSTATE[2] = rtb_Gain_idx_0;

  /* SignalConversion generated from: '<S3>/Vector Concatenate2' incorporates:
   *  AlgorithmDescriptorDelegate generated from: '<S143>/a16'
   *  Delay: '<Root>/Delay5'
   */
  FOCCurrentControl_DW.Delay5_DSTATE[3] = rtb_Gain_idx_1;

  /* End of Outputs for SubSystem: '<S4>/Clarke Transform' */

  /* Switch: '<S3>/Switch' incorporates:
   *  Delay: '<Root>/Delay5'
   *  Inport: '<Root>/EnClIn'
   *  SignalConversion generated from: '<S4>/Vector Concatenate1'
   * */
  if (arg_EnClIn) {
    FOCCurrentControl_DW.Delay5_DSTATE[0] = rtb_algDD_o1_n;
    FOCCurrentControl_DW.Delay5_DSTATE[1] = rtb_algDD_o2_h;
  } else {
    FOCCurrentControl_DW.Delay5_DSTATE[0] =
      FOCCurrentControl_B.VectorConcatenate1[0];
    FOCCurrentControl_DW.Delay5_DSTATE[1] =
      FOCCurrentControl_B.VectorConcatenate1[1];
  }

  /* End of Switch: '<S3>/Switch' */

  /* DeadZone: '<S193>/DeadZone' */
  if (rtb_DeadZone > 1.0F) {
    rtb_DeadZone--;
  } else if (rtb_DeadZone >= -1.0F) {
    rtb_DeadZone = 0.0F;
  } else {
    rtb_DeadZone++;
  }

  /* End of DeadZone: '<S193>/DeadZone' */

  /* Product: '<S197>/IProd Out' incorporates:
   *  Constant: '<S145>/Ki'
   */
  rtb_Gain_idx_0 = PI_params.Ki_id * (real32_T)Ts * rtb_Integrator;

  /* DeadZone: '<S243>/DeadZone' */
  if (rtb_Product2_b > 1.0F) {
    rtb_Product2_b--;
  } else if (rtb_Product2_b >= -1.0F) {
    rtb_Product2_b = 0.0F;
  } else {
    rtb_Product2_b++;
  }

  /* End of DeadZone: '<S243>/DeadZone' */

  /* Product: '<S247>/IProd Out' incorporates:
   *  Constant: '<S145>/Ki1'
   *  Sum: '<S145>/Sum1'
   */
  rtb_algDD_o2_h = PI_params.Ki_iq * (real32_T)Ts * rtb_Sum2_h;

  /* Update for Delay: '<S28>/Delay' incorporates:
   *  UnitDelay: '<S35>/Unit Delay'
   */
  FOCCurrentControl_DW.Delay_DSTATE = FOCCurrentControl_DW.UnitDelay_DSTATE_i;

  /* Update for Delay: '<S9>/Delay1' incorporates:
   *  UnitDelay: '<S40>/Unit Delay'
   */
  FOCCurrentControl_DW.Delay1_DSTATE = FOCCurrentControl_DW.UnitDelay_DSTATE_cx;

  /* Update for DiscreteIntegrator: '<S72>/Integrator' */
  FOCCurrentControl_DW.Integrator_PrevResetState = (int8_T)rtb_NOT;

  /* Update for DiscreteIntegrator: '<S250>/Integrator' */
  FOCCurrentControl_DW.Integrator_IC_LOADING = 0U;

  /* Switch: '<S241>/Switch1' incorporates:
   *  Constant: '<S241>/Clamping_zero'
   *  Constant: '<S241>/Constant'
   *  Constant: '<S241>/Constant2'
   *  RelationalOperator: '<S241>/fix for DT propagation issue'
   */
  if (rtb_Product2_b > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S241>/Switch2' incorporates:
   *  Constant: '<S241>/Clamping_zero'
   *  Constant: '<S241>/Constant3'
   *  Constant: '<S241>/Constant4'
   *  RelationalOperator: '<S241>/fix for DT propagation issue1'
   */
  if (rtb_algDD_o2_h > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S241>/Switch' incorporates:
   *  Constant: '<S241>/Clamping_zero'
   *  Constant: '<S241>/Constant1'
   *  Logic: '<S241>/AND3'
   *  RelationalOperator: '<S241>/Equal1'
   *  RelationalOperator: '<S241>/Relational Operator'
   *  Switch: '<S241>/Switch1'
   *  Switch: '<S241>/Switch2'
   */
  if ((rtb_Product2_b != 0.0F) && (tmp == tmp_0)) {
    rtb_algDD_o2_h = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S250>/Integrator' incorporates:
   *  Logic: '<S145>/NOT'
   *  Switch: '<S241>/Switch'
   */
  FOCCurrentControl_DW.Integrator_DSTATE_n += rtb_algDD_o2_h;
  FOCCurrentControl_DW.Integrator_PrevResetState_e = (int8_T)rtb_NOT_h_tmp;

  /* Update for DiscreteIntegrator: '<S200>/Integrator' */
  FOCCurrentControl_DW.Integrator_IC_LOADING_k = 0U;

  /* Switch: '<S191>/Switch1' incorporates:
   *  Constant: '<S191>/Clamping_zero'
   *  Constant: '<S191>/Constant'
   *  Constant: '<S191>/Constant2'
   *  RelationalOperator: '<S191>/fix for DT propagation issue'
   */
  if (rtb_DeadZone > 0.0F) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S191>/Switch2' incorporates:
   *  Constant: '<S191>/Clamping_zero'
   *  Constant: '<S191>/Constant3'
   *  Constant: '<S191>/Constant4'
   *  RelationalOperator: '<S191>/fix for DT propagation issue1'
   */
  if (rtb_Gain_idx_0 > 0.0F) {
    tmp_0 = 1;
  } else {
    tmp_0 = -1;
  }

  /* Switch: '<S191>/Switch' incorporates:
   *  Constant: '<S191>/Clamping_zero'
   *  Constant: '<S191>/Constant1'
   *  Logic: '<S191>/AND3'
   *  RelationalOperator: '<S191>/Equal1'
   *  RelationalOperator: '<S191>/Relational Operator'
   *  Switch: '<S191>/Switch1'
   *  Switch: '<S191>/Switch2'
   */
  if ((rtb_DeadZone != 0.0F) && (tmp == tmp_0)) {
    rtb_Gain_idx_0 = 0.0F;
  }

  /* Update for DiscreteIntegrator: '<S200>/Integrator' incorporates:
   *  Logic: '<S145>/NOT'
   *  Switch: '<S191>/Switch'
   */
  FOCCurrentControl_DW.Integrator_DSTATE_l += rtb_Gain_idx_0;
  FOCCurrentControl_DW.Integrator_PrevResetState_o = (int8_T)rtb_NOT_h_tmp;

  /* Update absolute time for base rate */
  /* The "clockTick0" counts the number of times the code of this task has
   * been executed. The resolution of this integer timer is 0.0001, which is the step size
   * of the task. Size of "clockTick0" ensures timer will not overflow during the
   * application lifespan selected.
   */
  FOCCurrentControl_M->Timing.clockTick0++;
}

/* Model initialize function */
void FOCCurrentControl_initialize(void)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));
  FOCCurrentControl_PrevZCX.Delay_Reset_ZCE = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay1_Reset_ZCE = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay_Reset_ZCE_o = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Dir_Sense_Trig_ZCE = POS_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay2_Reset_ZCE_ft = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay3_Reset_ZCE_k = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay2_Reset_ZCE_fx = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay3_Reset_ZCE_m = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay1_Reset_ZCE_k = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay_Reset_ZCE_h = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay1_Reset_ZCE_e = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay_Reset_ZCE_j = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay2_Reset_ZCE = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay3_Reset_ZCE = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay2_Reset_ZCE_f = UNINITIALIZED_ZCSIG;
  FOCCurrentControl_PrevZCX.Delay3_Reset_ZCE_j = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for DiscreteIntegrator: '<S250>/Integrator' */
  FOCCurrentControl_DW.Integrator_IC_LOADING = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S200>/Integrator' */
  FOCCurrentControl_DW.Integrator_IC_LOADING_k = 1U;
}

/* Model terminate function */
void FOCCurrentControl_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
