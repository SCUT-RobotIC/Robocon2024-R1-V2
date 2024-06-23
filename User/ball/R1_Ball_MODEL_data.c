/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: R1_Ball_MODEL_data.c
 *
 * Code generated for Simulink model 'R1_Ball_MODEL'.
 *
 * Model version                  : 1.4
 * Simulink Coder version         : 9.9 (R2023a) 19-Nov-2022
 * C/C++ source code generated on : Thu Apr 11 23:52:48 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "R1_Ball_MODEL.h"

/* Block parameters (default storage) */
P rtP = {
  /* Variable: Kd_lift
   * Referenced by: '<S29>/Derivative Gain'
   */
  0.1,

  /* Variable: Kd_shoot_l
   * Referenced by: '<S176>/Derivative Gain'
   */
  0.1,

  /* Variable: Kd_shoot_r
   * Referenced by: '<S128>/Derivative Gain'
   */
  0.1,

  /* Variable: Kd_shoot_up
   * Referenced by: '<S80>/Derivative Gain'
   */
  0.1,

  /* Variable: Ki_lift
   * Referenced by: '<S32>/Integral Gain'
   */
  0.7,

  /* Variable: Ki_shoot_l
   * Referenced by: '<S179>/Integral Gain'
   */
  0.7,

  /* Variable: Ki_shoot_r
   * Referenced by: '<S131>/Integral Gain'
   */
  0.7,

  /* Variable: Ki_shoot_up
   * Referenced by: '<S83>/Integral Gain'
   */
  0.7,

  /* Variable: Kp_lift
   * Referenced by: '<S40>/Proportional Gain'
   */
  2.0,

  /* Variable: Kp_shoot_l
   * Referenced by: '<S187>/Proportional Gain'
   */
  2.0,

  /* Variable: Kp_shoot_r
   * Referenced by: '<S139>/Proportional Gain'
   */
  2.0,

  /* Variable: Kp_shoot_up
   * Referenced by: '<S91>/Proportional Gain'
   */
  2.0,

  /* Variable: M2006_max
   * Referenced by:
   *   '<S1>/Gain2'
   *   '<S2>/Gain'
   *   '<S42>/Saturation'
   *   '<S93>/Saturation'
   */
  14976.0,

  /* Variable: M3508_max
   * Referenced by:
   *   '<S2>/Gain1'
   *   '<S2>/Gain2'
   *   '<S141>/Saturation'
   *   '<S189>/Saturation'
   */
  8911.0,

  /* Mask Parameter: DiscretePIDController_InitialCo
   * Referenced by: '<S81>/Filter'
   */
  0.0,

  /* Mask Parameter: DiscretePIDController1_InitialC
   * Referenced by: '<S129>/Filter'
   */
  0.0,

  /* Mask Parameter: DiscretePIDController2_InitialC
   * Referenced by: '<S177>/Filter'
   */
  0.0,

  /* Mask Parameter: DiscretePIDController2_Initia_p
   * Referenced by: '<S30>/Filter'
   */
  0.0,

  /* Mask Parameter: DiscretePIDController_Initial_p
   * Referenced by: '<S86>/Integrator'
   */
  0.0,

  /* Mask Parameter: DiscretePIDController1_Initia_n
   * Referenced by: '<S134>/Integrator'
   */
  0.0,

  /* Mask Parameter: DiscretePIDController2_Initia_l
   * Referenced by: '<S182>/Integrator'
   */
  0.0,

  /* Mask Parameter: DiscretePIDController2_Initia_n
   * Referenced by: '<S35>/Integrator'
   */
  0.0,

  /* Mask Parameter: DiscretePIDController_N
   * Referenced by: '<S89>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: DiscretePIDController1_N
   * Referenced by: '<S137>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: DiscretePIDController2_N
   * Referenced by: '<S185>/Filter Coefficient'
   */
  100.0,

  /* Mask Parameter: DiscretePIDController2_N_b
   * Referenced by: '<S38>/Filter Coefficient'
   */
  100.0,

  /* Computed Parameter: Integrator_gainval
   * Referenced by: '<S86>/Integrator'
   */
  0.001,

  /* Computed Parameter: Filter_gainval
   * Referenced by: '<S81>/Filter'
   */
  0.001,

  /* Computed Parameter: Integrator_gainval_e
   * Referenced by: '<S134>/Integrator'
   */
  0.001,

  /* Computed Parameter: Filter_gainval_o
   * Referenced by: '<S129>/Filter'
   */
  0.001,

  /* Computed Parameter: Integrator_gainval_i
   * Referenced by: '<S182>/Integrator'
   */
  0.001,

  /* Computed Parameter: Filter_gainval_n
   * Referenced by: '<S177>/Filter'
   */
  0.001,

  /* Computed Parameter: Integrator_gainval_o
   * Referenced by: '<S35>/Integrator'
   */
  0.001,

  /* Computed Parameter: Filter_gainval_l
   * Referenced by: '<S30>/Filter'
   */
  0.001
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
