/*
 * File: FOCCurrentControl.h
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

#ifndef RTW_HEADER_FOCCurrentControl_h_
#define RTW_HEADER_FOCCurrentControl_h_
#ifndef FOCCurrentControl_COMMON_INCLUDES_
#define FOCCurrentControl_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* FOCCurrentControl_COMMON_INCLUDES_ */

#include "FOCCurrentControl_types.h"
#include "rt_defines.h"
#include "rt_nonfinite.h"
#include "zero_crossing_types.h"

/* Includes for objects with custom storage classes */
#include "ConfigParameters.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals for system '<S35>/Accumulate' */
typedef struct {
  real32_T Input;                      /* '<S90>/Input' */
} B_Accumulate_FOCCurrentContro_T;

/* Block states (default storage) for system '<S35>/Accumulate' */
typedef struct {
  boolean_T Delay_DSTATE;              /* '<S89>/Delay' */
} DW_Accumulate_FOCCurrentContr_T;

/* Block signals (default storage) */
typedef struct {
  real32_T Merge;                      /* '<S37>/Merge' */
  real32_T VectorConcatenate2[2];      /* '<S6>/Vector Concatenate2' */
  real32_T VectorConcatenate1[2];      /* '<S6>/Vector Concatenate1' */
  real32_T VectorConcatenate[3];       /* '<S6>/Vector Concatenate' */
  uint16_T Gain1[2];                   /* '<S280>/Gain1' */
  int16_T Switch;                      /* '<S29>/Switch' */
  B_Accumulate_FOCCurrentContro_T Accumulate_n;/* '<S285>/Accumulate' */
  B_Accumulate_FOCCurrentContro_T Accumulate;/* '<S35>/Accumulate' */
} B_FOCCurrentControl_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real32_T Delay_DSTATE;               /* '<S28>/Delay' */
  real32_T Delay1_DSTATE;              /* '<S9>/Delay1' */
  real32_T Delay5_DSTATE[4];           /* '<Root>/Delay5' */
  real32_T Delay_DSTATE_k[2];          /* '<S10>/Delay' */
  real32_T UnitDelay_DSTATE[2];        /* '<S26>/Unit Delay' */
  real32_T Integrator_DSTATE;          /* '<S72>/Integrator' */
  real32_T UnitDelay_DSTATE_c;         /* '<S122>/Unit Delay' */
  real32_T UnitDelay_DSTATE_i;         /* '<S35>/Unit Delay' */
  real32_T Integrator_DSTATE_n;        /* '<S250>/Integrator' */
  real32_T Integrator_DSTATE_l;        /* '<S200>/Integrator' */
  real32_T UnitDelay_DSTATE_cx;        /* '<S40>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ct;        /* '<S290>/Unit Delay' */
  real32_T UnitDelay_DSTATE_j;         /* '<S285>/Unit Delay' */
  real32_T UnitDelay_DSTATE_p[2];      /* '<S280>/Unit Delay' */
  real32_T Delay2_DSTATE;              /* '<S133>/Delay2' */
  real32_T Delay3_DSTATE;              /* '<S133>/Delay3' */
  real32_T UnitDelay_DSTATE_cl;        /* '<S142>/Unit Delay' */
  real32_T Delay2_DSTATE_i;            /* '<S134>/Delay2' */
  real32_T Delay3_DSTATE_c;            /* '<S134>/Delay3' */
  real32_T UnitDelay_DSTATE_d;         /* '<S139>/Unit Delay' */
  uint32_T UnitDelay1_DSTATE;          /* '<S280>/Unit Delay1' */
  int8_T Integrator_PrevResetState;    /* '<S72>/Integrator' */
  int8_T Integrator_PrevResetState_e;  /* '<S250>/Integrator' */
  int8_T Integrator_PrevResetState_o;  /* '<S200>/Integrator' */
  uint8_T Integrator_IC_LOADING;       /* '<S250>/Integrator' */
  uint8_T Integrator_IC_LOADING_k;     /* '<S200>/Integrator' */
  boolean_T AlphaRelay_Mode;           /* '<S27>/AlphaRelay' */
  boolean_T BetaRelay_Mode;            /* '<S27>/BetaRelay' */
  boolean_T OpenLoopControl_MODE;      /* '<Root>/OpenLoopControl' */
  DW_Accumulate_FOCCurrentContr_T Accumulate_n;/* '<S285>/Accumulate' */
  DW_Accumulate_FOCCurrentContr_T Accumulate;/* '<S35>/Accumulate' */
} DW_FOCCurrentControl_T;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState Delay_Reset_ZCE;          /* '<S28>/Delay' */
  ZCSigState Delay1_Reset_ZCE;         /* '<S9>/Delay1' */
  ZCSigState Delay_Reset_ZCE_o;        /* '<S10>/Delay' */
  ZCSigState Delay2_Reset_ZCE;         /* '<S133>/Delay2' */
  ZCSigState Delay3_Reset_ZCE;         /* '<S133>/Delay3' */
  ZCSigState Delay2_Reset_ZCE_f;       /* '<S134>/Delay2' */
  ZCSigState Delay3_Reset_ZCE_j;       /* '<S134>/Delay3' */
  ZCSigState Delay1_Reset_ZCE_k;       /* '<S125>/Delay1' */
  ZCSigState Delay_Reset_ZCE_h;        /* '<S125>/Delay' */
  ZCSigState Delay1_Reset_ZCE_e;       /* '<S126>/Delay1' */
  ZCSigState Delay_Reset_ZCE_j;        /* '<S126>/Delay' */
  ZCSigState Delay2_Reset_ZCE_ft;      /* '<S100>/Delay2' */
  ZCSigState Delay3_Reset_ZCE_k;       /* '<S100>/Delay3' */
  ZCSigState Delay2_Reset_ZCE_fx;      /* '<S101>/Delay2' */
  ZCSigState Delay3_Reset_ZCE_m;       /* '<S101>/Delay3' */
  ZCSigState Dir_Sense_Trig_ZCE;       /* '<S27>/Dir_Sense' */
} PrevZCX_FOCCurrentControl_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: )
   * Referenced by:
   *   '<S148>/sine_table_values'
   *   '<S286>/sine_table_values'
   *   '<S287>/sine_table_values'
   *   '<S36>/sine_table_values'
   */
  real32_T pooled12[1002];
} ConstP_FOCCurrentControl_T;

/* Real-time Model Data Structure */
struct tag_RTM_FOCCurrentControl_T {
  const char_T * volatile errorStatus;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
  } Timing;
};

/* Block signals (default storage) */
extern B_FOCCurrentControl_T FOCCurrentControl_B;

/* Block states (default storage) */
extern DW_FOCCurrentControl_T FOCCurrentControl_DW;

/* Zero-crossing (trigger) state */
extern PrevZCX_FOCCurrentControl_T FOCCurrentControl_PrevZCX;

/* Constant parameters (default storage) */
extern const ConstP_FOCCurrentControl_T FOCCurrentControl_ConstP;

/* Model entry point functions */
extern void FOCCurrentControl_initialize(void);
extern void FOCCurrentControl_terminate(void);

/* Customized model step function */
extern void FOCCurrentControl_step(boolean_T arg_Enable, real32_T arg_IdqRef_PU
  [2], real32_T arg_SpeedRef_PU, boolean_T arg_EnClIn, uint16_T arg_Iab_ADC[2],
  uint16_T arg_ENCCounts, real32_T arg_HallAngle_rad, real32_T arg_Vabc_PU[3],
  real32_T *arg_SpeedFb_PU, real32_T *arg_Iq, real32_T *arg_Mode, uint16_T arg_Out1[2]);

/* Real-time Model object */
extern RT_MODEL_FOCCurrentControl_T *const FOCCurrentControl_M;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S9>/Data Type Propagation' : Unused code path elimination
 * Block '<S9>/Data Type Conversion' : Unused code path elimination
 * Block '<S9>/Data Type Conversion1' : Unused code path elimination
 * Block '<S10>/Data Type Duplicate' : Unused code path elimination
 * Block '<S10>/Data Type Propagation' : Unused code path elimination
 * Block '<S16>/Data Type Duplicate' : Unused code path elimination
 * Block '<S16>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S16>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S16>/Data Type Propagation' : Unused code path elimination
 * Block '<S16>/Data Type Propagation1' : Unused code path elimination
 * Block '<S16>/Data Type Propagation2' : Unused code path elimination
 * Block '<S16>/Data Type Propagation3' : Unused code path elimination
 * Block '<S17>/Data Type Duplicate' : Unused code path elimination
 * Block '<S17>/Data Type Propagation' : Unused code path elimination
 * Block '<S18>/Data Type Duplicate' : Unused code path elimination
 * Block '<S18>/Data Type Propagation' : Unused code path elimination
 * Block '<S14>/Data Type Propagation' : Unused code path elimination
 * Block '<S19>/Data Type Duplicate' : Unused code path elimination
 * Block '<S19>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S19>/Data Type Propagation' : Unused code path elimination
 * Block '<S20>/Data Type Duplicate' : Unused code path elimination
 * Block '<S20>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S20>/Data Type Propagation' : Unused code path elimination
 * Block '<S21>/Data Type Duplicate' : Unused code path elimination
 * Block '<S21>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S21>/Data Type Propagation' : Unused code path elimination
 * Block '<S22>/Data Type Duplicate' : Unused code path elimination
 * Block '<S22>/Data Type Propagation' : Unused code path elimination
 * Block '<S15>/Data Type Propagation' : Unused code path elimination
 * Block '<S23>/Data Type Duplicate' : Unused code path elimination
 * Block '<S23>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S23>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S23>/Data Type Propagation' : Unused code path elimination
 * Block '<S23>/Data Type Propagation1' : Unused code path elimination
 * Block '<S26>/Data Type Duplicate' : Unused code path elimination
 * Block '<S12>/Data Type Propagation' : Unused code path elimination
 * Block '<S12>/Data Type Propagation1' : Unused code path elimination
 * Block '<S28>/Data Type Duplicate' : Unused code path elimination
 * Block '<S28>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S40>/Data Type Duplicate' : Unused code path elimination
 * Block '<S33>/Data Type Duplicate' : Unused code path elimination
 * Block '<S33>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S35>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S36>/Data Type Duplicate' : Unused code path elimination
 * Block '<S36>/Data Type Propagation' : Unused code path elimination
 * Block '<S94>/Data Type Duplicate' : Unused code path elimination
 * Block '<S95>/Data Type Duplicate' : Unused code path elimination
 * Block '<S96>/Data Type Duplicate' : Unused code path elimination
 * Block '<S96>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S96>/Data Type Propagation1' : Unused code path elimination
 * Block '<S96>/Data Type Propagation2' : Unused code path elimination
 * Block '<S100>/Data Type Propagation' : Unused code path elimination
 * Block '<S100>/Data Type Propagation1' : Unused code path elimination
 * Block '<S101>/Data Type Propagation' : Unused code path elimination
 * Block '<S101>/Data Type Propagation1' : Unused code path elimination
 * Block '<S108>/Data Type Duplicate' : Unused code path elimination
 * Block '<S111>/Data Type Duplicate' : Unused code path elimination
 * Block '<S114>/Data Type Duplicate' : Unused code path elimination
 * Block '<S105>/Data Type Duplicate' : Unused code path elimination
 * Block '<S105>/Data Type Propagation' : Unused code path elimination
 * Block '<S122>/Data Type Duplicate' : Unused code path elimination
 * Block '<S98>/Data Type Duplicate' : Unused code path elimination
 * Block '<S98>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S98>/Data Type Propagation1' : Unused code path elimination
 * Block '<S129>/Data Type Duplicate' : Unused code path elimination
 * Block '<S132>/Data Type Duplicate' : Unused code path elimination
 * Block '<S125>/Data Type Propagation' : Unused code path elimination
 * Block '<S125>/Data Type Propagation1' : Unused code path elimination
 * Block '<S126>/Data Type Propagation' : Unused code path elimination
 * Block '<S126>/Data Type Propagation1' : Unused code path elimination
 * Block '<S99>/Data Type Duplicate' : Unused code path elimination
 * Block '<S133>/Data Type Propagation' : Unused code path elimination
 * Block '<S133>/Data Type Propagation1' : Unused code path elimination
 * Block '<S134>/Data Type Propagation' : Unused code path elimination
 * Block '<S134>/Data Type Propagation1' : Unused code path elimination
 * Block '<S139>/Data Type Duplicate' : Unused code path elimination
 * Block '<S142>/Data Type Duplicate' : Unused code path elimination
 * Block '<S143>/Data Type Duplicate' : Unused code path elimination
 * Block '<S144>/Data Type Duplicate' : Unused code path elimination
 * Block '<S144>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S157>/Data Type Duplicate' : Unused code path elimination
 * Block '<S164>/Data Type Duplicate' : Unused code path elimination
 * Block '<S164>/Data Type Propagation' : Unused code path elimination
 * Block '<S165>/Data Type Duplicate' : Unused code path elimination
 * Block '<S150>/Data Type Duplicate' : Unused code path elimination
 * Block '<S155>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S155>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S156>/Sqrt' : Unused code path elimination
 * Block '<S146>/Data Type Duplicate' : Unused code path elimination
 * Block '<S146>/Vc' : Unused code path elimination
 * Block '<S272>/Data Type Duplicate' : Unused code path elimination
 * Block '<S147>/Data Type Duplicate' : Unused code path elimination
 * Block '<S147>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S148>/Data Type Duplicate' : Unused code path elimination
 * Block '<S148>/Data Type Propagation' : Unused code path elimination
 * Block '<S277>/Data Type Duplicate' : Unused code path elimination
 * Block '<S278>/Data Type Duplicate' : Unused code path elimination
 * Block '<S290>/Data Type Duplicate' : Unused code path elimination
 * Block '<S282>/Data Type Duplicate' : Unused code path elimination
 * Block '<S282>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S283>/Data Type Duplicate' : Unused code path elimination
 * Block '<S283>/Vc' : Unused code path elimination
 * Block '<S297>/Data Type Duplicate' : Unused code path elimination
 * Block '<S284>/Data Type Duplicate' : Unused code path elimination
 * Block '<S284>/Data Type Duplicate1' : Unused code path elimination
 * Block '<S285>/Data Type Duplicate2' : Unused code path elimination
 * Block '<S286>/Data Type Duplicate' : Unused code path elimination
 * Block '<S286>/Data Type Propagation' : Unused code path elimination
 * Block '<S304>/Data Type Duplicate' : Unused code path elimination
 * Block '<S305>/Data Type Duplicate' : Unused code path elimination
 * Block '<S287>/Data Type Duplicate' : Unused code path elimination
 * Block '<S287>/Data Type Propagation' : Unused code path elimination
 * Block '<S309>/Data Type Duplicate' : Unused code path elimination
 * Block '<S310>/Data Type Duplicate' : Unused code path elimination
 * Block '<S9>/PositionGain' : Eliminated nontunable gain of 1
 * Block '<S28>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S28>/Data Type Conversion1' : Eliminate redundant data type conversion
 * Block '<S35>/scaleIn' : Eliminated nontunable gain of 1
 * Block '<S35>/scaleOut' : Eliminated nontunable gain of 1
 * Block '<S36>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S92>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S37>/Cast To Boolean' : Eliminate redundant data type conversion
 * Block '<S148>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S275>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S285>/scaleIn' : Eliminated nontunable gain of 1
 * Block '<S285>/scaleOut' : Eliminated nontunable gain of 1
 * Block '<S286>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S302>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S287>/Get_FractionVal' : Eliminate redundant data type conversion
 * Block '<S307>/convert_pu' : Eliminated nontunable gain of 1
 * Block '<S149>/Offset' : Unused code path elimination
 * Block '<S149>/Unary_Minus' : Unused code path elimination
 * Block '<S155>/enableInportSatLim' : Unused code path elimination
 * Block '<S155>/enableInportSatMethod' : Unused code path elimination
 * Block '<S150>/ReplaceInport_satLim' : Unused code path elimination
 * Block '<S150>/ReplaceInport_satMethod' : Unused code path elimination
 * Block '<S273>/Offset' : Unused code path elimination
 * Block '<S273>/Unary_Minus' : Unused code path elimination
 * Block '<S291>/Offset' : Unused code path elimination
 * Block '<S291>/Unary_Minus' : Unused code path elimination
 * Block '<S298>/Offset' : Unused code path elimination
 * Block '<S298>/Unary_Minus' : Unused code path elimination
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
 * '<Root>' : 'FOCCurrentControl'
 * '<S1>'   : 'FOCCurrentControl/Calculate Idq0'
 * '<S2>'   : 'FOCCurrentControl/Calculate Position and Speed'
 * '<S3>'   : 'FOCCurrentControl/Calculate VI Alpha Beta'
 * '<S4>'   : 'FOCCurrentControl/ClosedLoopControl'
 * '<S5>'   : 'FOCCurrentControl/Compensate offset'
 * '<S6>'   : 'FOCCurrentControl/OpenLoopControl'
 * '<S7>'   : 'FOCCurrentControl/OutputScaling'
 * '<S8>'   : 'FOCCurrentControl/Calculate Position and Speed/EEMF'
 * '<S9>'   : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer'
 * '<S10>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer'
 * '<S11>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/IIR Filter'
 * '<S12>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/PerUnit'
 * '<S13>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer'
 * '<S14>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer/Subsystem'
 * '<S15>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer/Subsystem2'
 * '<S16>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer/Subsystem/A11'
 * '<S17>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer/Subsystem/A12'
 * '<S18>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer/Subsystem/A22'
 * '<S19>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer/Subsystem/L'
 * '<S20>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer/Subsystem/L1'
 * '<S21>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer/Subsystem/L2'
 * '<S22>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer/Subsystem2/B'
 * '<S23>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/EEMF Observer/Subsystem2/L'
 * '<S24>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/IIR Filter/IIR Filter'
 * '<S25>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/IIR Filter/IIR Filter/Low-pass'
 * '<S26>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S27>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/Direction_Latch'
 * '<S28>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL'
 * '<S29>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/Direction_Latch/Dir_Sense'
 * '<S30>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/Direction_Latch/Subsystem2'
 * '<S31>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/Direction_Latch/Subsystem3'
 * '<S32>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/IIR Filter'
 * '<S33>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/Normalize'
 * '<S34>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller'
 * '<S35>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/Position Generator'
 * '<S36>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/Sine-Cosine Lookup1'
 * '<S37>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1'
 * '<S38>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/IIR Filter/IIR Filter'
 * '<S39>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/IIR Filter/IIR Filter/Low-pass'
 * '<S40>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S41>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Anti-windup'
 * '<S42>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/D Gain'
 * '<S43>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Filter'
 * '<S44>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Filter ICs'
 * '<S45>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/I Gain'
 * '<S46>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Ideal P Gain'
 * '<S47>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Ideal P Gain Fdbk'
 * '<S48>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Integrator'
 * '<S49>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Integrator ICs'
 * '<S50>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/N Copy'
 * '<S51>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/N Gain'
 * '<S52>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/P Copy'
 * '<S53>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Parallel P Gain'
 * '<S54>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Reset Signal'
 * '<S55>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Saturation'
 * '<S56>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Saturation Fdbk'
 * '<S57>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Sum'
 * '<S58>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Sum Fdbk'
 * '<S59>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Tracking Mode'
 * '<S60>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Tracking Mode Sum'
 * '<S61>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Tsamp - Integral'
 * '<S62>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Tsamp - Ngain'
 * '<S63>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/postSat Signal'
 * '<S64>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/preSat Signal'
 * '<S65>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Anti-windup/Passthrough'
 * '<S66>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/D Gain/Disabled'
 * '<S67>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Filter/Disabled'
 * '<S68>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Filter ICs/Disabled'
 * '<S69>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/I Gain/Internal Parameters'
 * '<S70>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Ideal P Gain/Passthrough'
 * '<S71>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S72>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Integrator/Discrete'
 * '<S73>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Integrator ICs/Internal IC'
 * '<S74>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/N Copy/Disabled wSignal Specification'
 * '<S75>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/N Gain/Disabled'
 * '<S76>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/P Copy/Disabled'
 * '<S77>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S78>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Reset Signal/External Reset'
 * '<S79>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Saturation/Passthrough'
 * '<S80>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Saturation Fdbk/Disabled'
 * '<S81>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Sum/Sum_PI'
 * '<S82>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Sum Fdbk/Disabled'
 * '<S83>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Tracking Mode/Disabled'
 * '<S84>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S85>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S86>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S87>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/postSat Signal/Forward_Path'
 * '<S88>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/PID Controller/preSat Signal/Forward_Path'
 * '<S89>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/Position Generator/Accumulate'
 * '<S90>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/Position Generator/Accumulate/Subsystem'
 * '<S91>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/Sine-Cosine Lookup1/Interpolation'
 * '<S92>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/Sine-Cosine Lookup1/WrapUp'
 * '<S93>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/Sine-Cosine Lookup1/WrapUp/Compare To Zero'
 * '<S94>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/Sine-Cosine Lookup1/WrapUp/If Action Subsystem'
 * '<S95>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/Sine-Cosine Lookup1/WrapUp/If Action Subsystem1'
 * '<S96>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod'
 * '<S97>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IIR Filter'
 * '<S98>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod'
 * '<S99>'  : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod'
 * '<S100>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/Differentiator'
 * '<S101>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/Differentiator1'
 * '<S102>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter1'
 * '<S103>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter2'
 * '<S104>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter3'
 * '<S105>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/Subsystem'
 * '<S106>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter1/IIR Filter'
 * '<S107>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter1/IIR Filter/Low-pass'
 * '<S108>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter1/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S109>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter2/IIR Filter'
 * '<S110>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter2/IIR Filter/Low-pass'
 * '<S111>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter2/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S112>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter3/IIR Filter'
 * '<S113>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter3/IIR Filter/Low-pass'
 * '<S114>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/IIR Filter3/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S115>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/Subsystem/Compare To Constant'
 * '<S116>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/Subsystem/Compare To Constant1'
 * '<S117>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/Subsystem/Compare To Constant2'
 * '<S118>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/Subsystem/Compare To Constant4'
 * '<S119>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/DifferentiationMethod/Subsystem/Compare To Constant5'
 * '<S120>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IIR Filter/IIR Filter'
 * '<S121>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IIR Filter/IIR Filter/Low-pass'
 * '<S122>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S123>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod/IIR Filter1'
 * '<S124>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod/IIR Filter3'
 * '<S125>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod/Integrator'
 * '<S126>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod/Integrator1'
 * '<S127>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod/IIR Filter1/IIR Filter'
 * '<S128>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod/IIR Filter1/IIR Filter/Low-pass'
 * '<S129>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod/IIR Filter1/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S130>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod/IIR Filter3/IIR Filter'
 * '<S131>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod/IIR Filter3/IIR Filter/Low-pass'
 * '<S132>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/IntegralMethod/IIR Filter3/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S133>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod/Differentiator'
 * '<S134>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod/Differentiator1'
 * '<S135>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod/IIR Filter1'
 * '<S136>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod/IIR Filter3'
 * '<S137>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter'
 * '<S138>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter/Low-pass'
 * '<S139>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod/IIR Filter1/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S140>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter'
 * '<S141>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter/Low-pass'
 * '<S142>' : 'FOCCurrentControl/Calculate Position and Speed/EEMF/Extended EMF Observer/Speed Observer/PLL/SpeedFeedforward1/OptimizedDifferentiationMethod/IIR Filter3/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S143>' : 'FOCCurrentControl/ClosedLoopControl/Clarke Transform'
 * '<S144>' : 'FOCCurrentControl/ClosedLoopControl/Inverse Park Transform'
 * '<S145>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers'
 * '<S146>' : 'FOCCurrentControl/ClosedLoopControl/PWM Reference Generator'
 * '<S147>' : 'FOCCurrentControl/ClosedLoopControl/Park Transform'
 * '<S148>' : 'FOCCurrentControl/ClosedLoopControl/Sine-Cosine Lookup'
 * '<S149>' : 'FOCCurrentControl/ClosedLoopControl/Inverse Park Transform/Switch_Axis'
 * '<S150>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter'
 * '<S151>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller'
 * '<S152>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1'
 * '<S153>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D-Q Equivalence'
 * '<S154>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D//Q Axis Priority'
 * '<S155>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/Inport//Dialog Selection'
 * '<S156>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/Magnitude_calc'
 * '<S157>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D-Q Equivalence/Limiter'
 * '<S158>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D-Q Equivalence/Passthrough'
 * '<S159>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant'
 * '<S160>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D//Q Axis Priority/Compare To Constant1'
 * '<S161>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D//Q Axis Priority/flipInputs'
 * '<S162>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D//Q Axis Priority/flipInputs1'
 * '<S163>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D//Q Axis Priority/limiter'
 * '<S164>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef1'
 * '<S165>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D//Q Axis Priority/limiter/limitRef2'
 * '<S166>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/DQ Limiter/D//Q Axis Priority/limiter/passThrough'
 * '<S167>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Anti-windup'
 * '<S168>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/D Gain'
 * '<S169>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Filter'
 * '<S170>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Filter ICs'
 * '<S171>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/I Gain'
 * '<S172>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Ideal P Gain'
 * '<S173>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Ideal P Gain Fdbk'
 * '<S174>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Integrator'
 * '<S175>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Integrator ICs'
 * '<S176>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/N Copy'
 * '<S177>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/N Gain'
 * '<S178>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/P Copy'
 * '<S179>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Parallel P Gain'
 * '<S180>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Reset Signal'
 * '<S181>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Saturation'
 * '<S182>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Saturation Fdbk'
 * '<S183>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Sum'
 * '<S184>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Sum Fdbk'
 * '<S185>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Tracking Mode'
 * '<S186>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Tracking Mode Sum'
 * '<S187>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Tsamp - Integral'
 * '<S188>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Tsamp - Ngain'
 * '<S189>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/postSat Signal'
 * '<S190>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/preSat Signal'
 * '<S191>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Anti-windup/Disc. Clamping Parallel'
 * '<S192>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S193>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S194>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/D Gain/Disabled'
 * '<S195>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Filter/Disabled'
 * '<S196>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Filter ICs/Disabled'
 * '<S197>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/I Gain/External Parameters'
 * '<S198>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Ideal P Gain/Passthrough'
 * '<S199>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Ideal P Gain Fdbk/Disabled'
 * '<S200>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Integrator/Discrete'
 * '<S201>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Integrator ICs/External IC'
 * '<S202>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/N Copy/Disabled wSignal Specification'
 * '<S203>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/N Gain/Disabled'
 * '<S204>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/P Copy/Disabled'
 * '<S205>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Parallel P Gain/External Parameters'
 * '<S206>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Reset Signal/External Reset'
 * '<S207>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Saturation/Enabled'
 * '<S208>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Saturation Fdbk/Disabled'
 * '<S209>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Sum/Sum_PI'
 * '<S210>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Sum Fdbk/Disabled'
 * '<S211>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Tracking Mode/Disabled'
 * '<S212>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Tracking Mode Sum/Passthrough'
 * '<S213>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S214>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/Tsamp - Ngain/Passthrough'
 * '<S215>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/postSat Signal/Forward_Path'
 * '<S216>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller/preSat Signal/Forward_Path'
 * '<S217>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Anti-windup'
 * '<S218>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/D Gain'
 * '<S219>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Filter'
 * '<S220>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Filter ICs'
 * '<S221>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/I Gain'
 * '<S222>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Ideal P Gain'
 * '<S223>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Ideal P Gain Fdbk'
 * '<S224>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Integrator'
 * '<S225>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Integrator ICs'
 * '<S226>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/N Copy'
 * '<S227>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/N Gain'
 * '<S228>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/P Copy'
 * '<S229>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Parallel P Gain'
 * '<S230>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Reset Signal'
 * '<S231>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Saturation'
 * '<S232>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Saturation Fdbk'
 * '<S233>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Sum'
 * '<S234>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Sum Fdbk'
 * '<S235>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Tracking Mode'
 * '<S236>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Tracking Mode Sum'
 * '<S237>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Tsamp - Integral'
 * '<S238>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Tsamp - Ngain'
 * '<S239>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/postSat Signal'
 * '<S240>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/preSat Signal'
 * '<S241>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Anti-windup/Disc. Clamping Parallel'
 * '<S242>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S243>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Anti-windup/Disc. Clamping Parallel/Dead Zone/Enabled'
 * '<S244>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/D Gain/Disabled'
 * '<S245>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Filter/Disabled'
 * '<S246>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Filter ICs/Disabled'
 * '<S247>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/I Gain/External Parameters'
 * '<S248>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Ideal P Gain/Passthrough'
 * '<S249>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S250>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Integrator/Discrete'
 * '<S251>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Integrator ICs/External IC'
 * '<S252>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/N Copy/Disabled wSignal Specification'
 * '<S253>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/N Gain/Disabled'
 * '<S254>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/P Copy/Disabled'
 * '<S255>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Parallel P Gain/External Parameters'
 * '<S256>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Reset Signal/External Reset'
 * '<S257>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Saturation/Enabled'
 * '<S258>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Saturation Fdbk/Disabled'
 * '<S259>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Sum/Sum_PI'
 * '<S260>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Sum Fdbk/Disabled'
 * '<S261>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Tracking Mode/Disabled'
 * '<S262>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Tracking Mode Sum/Passthrough'
 * '<S263>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S264>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/Tsamp - Ngain/Passthrough'
 * '<S265>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/postSat Signal/Forward_Path'
 * '<S266>' : 'FOCCurrentControl/ClosedLoopControl/PI Controllers/PI Controller1/preSat Signal/Forward_Path'
 * '<S267>' : 'FOCCurrentControl/ClosedLoopControl/PWM Reference Generator/Modulation method'
 * '<S268>' : 'FOCCurrentControl/ClosedLoopControl/PWM Reference Generator/Voltage Input'
 * '<S269>' : 'FOCCurrentControl/ClosedLoopControl/PWM Reference Generator/Modulation method/SVPWM'
 * '<S270>' : 'FOCCurrentControl/ClosedLoopControl/PWM Reference Generator/Modulation method/SVPWM/Half(Vmin+Vmax)'
 * '<S271>' : 'FOCCurrentControl/ClosedLoopControl/PWM Reference Generator/Voltage Input/Valphabeta'
 * '<S272>' : 'FOCCurrentControl/ClosedLoopControl/PWM Reference Generator/Voltage Input/Valphabeta/Inverse Clarke Transform'
 * '<S273>' : 'FOCCurrentControl/ClosedLoopControl/Park Transform/Switch_Axis'
 * '<S274>' : 'FOCCurrentControl/ClosedLoopControl/Sine-Cosine Lookup/Interpolation'
 * '<S275>' : 'FOCCurrentControl/ClosedLoopControl/Sine-Cosine Lookup/WrapUp'
 * '<S276>' : 'FOCCurrentControl/ClosedLoopControl/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S277>' : 'FOCCurrentControl/ClosedLoopControl/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S278>' : 'FOCCurrentControl/ClosedLoopControl/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S279>' : 'FOCCurrentControl/Compensate offset/Compare To Constant'
 * '<S280>' : 'FOCCurrentControl/Compensate offset/Enabled Subsystem'
 * '<S281>' : 'FOCCurrentControl/OpenLoopControl/IIR Filter'
 * '<S282>' : 'FOCCurrentControl/OpenLoopControl/Inverse Park Transform'
 * '<S283>' : 'FOCCurrentControl/OpenLoopControl/PWM Reference Generator'
 * '<S284>' : 'FOCCurrentControl/OpenLoopControl/Park Transform'
 * '<S285>' : 'FOCCurrentControl/OpenLoopControl/Position Generator'
 * '<S286>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup'
 * '<S287>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup1'
 * '<S288>' : 'FOCCurrentControl/OpenLoopControl/IIR Filter/IIR Filter'
 * '<S289>' : 'FOCCurrentControl/OpenLoopControl/IIR Filter/IIR Filter/Low-pass'
 * '<S290>' : 'FOCCurrentControl/OpenLoopControl/IIR Filter/IIR Filter/Low-pass/IIR Low Pass Filter'
 * '<S291>' : 'FOCCurrentControl/OpenLoopControl/Inverse Park Transform/Switch_Axis'
 * '<S292>' : 'FOCCurrentControl/OpenLoopControl/PWM Reference Generator/Modulation method'
 * '<S293>' : 'FOCCurrentControl/OpenLoopControl/PWM Reference Generator/Voltage Input'
 * '<S294>' : 'FOCCurrentControl/OpenLoopControl/PWM Reference Generator/Modulation method/SVPWM'
 * '<S295>' : 'FOCCurrentControl/OpenLoopControl/PWM Reference Generator/Modulation method/SVPWM/Half(Vmin+Vmax)'
 * '<S296>' : 'FOCCurrentControl/OpenLoopControl/PWM Reference Generator/Voltage Input/Valphabeta'
 * '<S297>' : 'FOCCurrentControl/OpenLoopControl/PWM Reference Generator/Voltage Input/Valphabeta/Inverse Clarke Transform'
 * '<S298>' : 'FOCCurrentControl/OpenLoopControl/Park Transform/Switch_Axis'
 * '<S299>' : 'FOCCurrentControl/OpenLoopControl/Position Generator/Accumulate'
 * '<S300>' : 'FOCCurrentControl/OpenLoopControl/Position Generator/Accumulate/Subsystem'
 * '<S301>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup/Interpolation'
 * '<S302>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup/WrapUp'
 * '<S303>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup/WrapUp/Compare To Zero'
 * '<S304>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup/WrapUp/If Action Subsystem'
 * '<S305>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup/WrapUp/If Action Subsystem1'
 * '<S306>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup1/Interpolation'
 * '<S307>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup1/WrapUp'
 * '<S308>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup1/WrapUp/Compare To Zero'
 * '<S309>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup1/WrapUp/If Action Subsystem'
 * '<S310>' : 'FOCCurrentControl/OpenLoopControl/Sine-Cosine Lookup1/WrapUp/If Action Subsystem1'
 */
#endif                                 /* RTW_HEADER_FOCCurrentControl_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
