/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: R1_Ball_MODEL.c
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
#include "rtwtypes.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Model step function */
void R1_Ball_MODEL_step(void)
{
  real_T rtb_FilterCoefficient;
  real_T rtb_FilterCoefficient_d;
  real_T rtb_FilterCoefficient_e;
  real_T rtb_FilterCoefficient_o;
  real_T rtb_Integrator_c2;
  real_T rtb_Integrator_l;
  real_T rtb_Integrator_n;
  real_T rtb_Sum2;
  real_T u0;

  /* Sum: '<S2>/Sum' incorporates:
   *  Inport: '<Root>/shoot_up_in'
   *  Inport: '<Root>/shoot_up_tgt'
   */
  rtb_Integrator_n = rtU.shoot_up_tgt - rtU.shoot_up_in;

  /* Gain: '<S89>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S81>/Filter'
   *  Gain: '<S80>/Derivative Gain'
   *  Sum: '<S81>/SumD'
   */
  rtb_FilterCoefficient = (rtP.Kd_shoot_up * rtb_Integrator_n -
    rtDW.Filter_DSTATE) * rtP.DiscretePIDController_N;

  /* Sum: '<S95>/Sum' incorporates:
   *  DiscreteIntegrator: '<S86>/Integrator'
   *  Gain: '<S91>/Proportional Gain'
   */
  u0 = (rtP.Kp_shoot_up * rtb_Integrator_n + rtDW.Integrator_DSTATE) +
    rtb_FilterCoefficient;

  /* Saturate: '<S93>/Saturation' */
  if (u0 > rtP.M2006_max) {
    u0 = rtP.M2006_max;
  } else if (u0 < -rtP.M2006_max) {
    u0 = -rtP.M2006_max;
  }

  /* Outport: '<Root>/shoot_up_out' incorporates:
   *  Gain: '<S2>/Gain'
   *  Saturate: '<S93>/Saturation'
   */
  rtY.shoot_up_out = 16384.0 / rtP.M2006_max * u0;

  /* Sum: '<S2>/Sum1' incorporates:
   *  Inport: '<Root>/shoot_r_in'
   *  Inport: '<Root>/shoot_r_tgt'
   */
  rtb_Integrator_c2 = rtU.shoot_r_tgt - rtU.shoot_r_in;

  /* Gain: '<S137>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S129>/Filter'
   *  Gain: '<S128>/Derivative Gain'
   *  Sum: '<S129>/SumD'
   */
  rtb_FilterCoefficient_e = (rtP.Kd_shoot_r * rtb_Integrator_c2 -
    rtDW.Filter_DSTATE_b) * rtP.DiscretePIDController1_N;

  /* Sum: '<S143>/Sum' incorporates:
   *  DiscreteIntegrator: '<S134>/Integrator'
   *  Gain: '<S139>/Proportional Gain'
   */
  u0 = (rtP.Kp_shoot_r * rtb_Integrator_c2 + rtDW.Integrator_DSTATE_p) +
    rtb_FilterCoefficient_e;

  /* Saturate: '<S141>/Saturation' */
  if (u0 > rtP.M3508_max) {
    u0 = rtP.M3508_max;
  } else if (u0 < -rtP.M3508_max) {
    u0 = -rtP.M3508_max;
  }

  /* Outport: '<Root>/shoot_r_out' incorporates:
   *  Gain: '<S2>/Gain1'
   *  Saturate: '<S141>/Saturation'
   */
  rtY.shoot_r_out = 16384.0 / rtP.M3508_max * u0;

  /* Sum: '<S2>/Sum2' incorporates:
   *  Inport: '<Root>/shoot_l_in'
   *  Inport: '<Root>/shoot_l_tgt'
   */
  rtb_Integrator_l = rtU.shoot_l_tgt - rtU.shoot_l_in;

  /* Gain: '<S185>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S177>/Filter'
   *  Gain: '<S176>/Derivative Gain'
   *  Sum: '<S177>/SumD'
   */
  rtb_FilterCoefficient_o = (rtP.Kd_shoot_l * rtb_Integrator_l -
    rtDW.Filter_DSTATE_o) * rtP.DiscretePIDController2_N;

  /* Sum: '<S191>/Sum' incorporates:
   *  DiscreteIntegrator: '<S182>/Integrator'
   *  Gain: '<S187>/Proportional Gain'
   */
  u0 = (rtP.Kp_shoot_l * rtb_Integrator_l + rtDW.Integrator_DSTATE_b) +
    rtb_FilterCoefficient_o;

  /* Saturate: '<S189>/Saturation' */
  if (u0 > rtP.M3508_max) {
    u0 = rtP.M3508_max;
  } else if (u0 < -rtP.M3508_max) {
    u0 = -rtP.M3508_max;
  }

  /* Outport: '<Root>/shoot_l_out' incorporates:
   *  Gain: '<S2>/Gain2'
   *  Saturate: '<S189>/Saturation'
   */
  rtY.shoot_l_out = 16384.0 / rtP.M3508_max * u0;

  /* Sum: '<S1>/Sum2' incorporates:
   *  Inport: '<Root>/lift_in'
   *  Inport: '<Root>/lift_tgt'
   */
  rtb_Sum2 = rtU.lift_tgt - rtU.lift_in;

  /* Gain: '<S38>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S30>/Filter'
   *  Gain: '<S29>/Derivative Gain'
   *  Sum: '<S30>/SumD'
   */
  rtb_FilterCoefficient_d = (rtP.Kd_lift * rtb_Sum2 - rtDW.Filter_DSTATE_ox) *
    rtP.DiscretePIDController2_N_b;

  /* Sum: '<S44>/Sum' incorporates:
   *  DiscreteIntegrator: '<S35>/Integrator'
   *  Gain: '<S40>/Proportional Gain'
   */
  u0 = (rtP.Kp_lift * rtb_Sum2 + rtDW.Integrator_DSTATE_j) +
    rtb_FilterCoefficient_d;

  /* Saturate: '<S42>/Saturation' */
  if (u0 > rtP.M2006_max) {
    u0 = rtP.M2006_max;
  } else if (u0 < -rtP.M2006_max) {
    u0 = -rtP.M2006_max;
  }

  /* Outport: '<Root>/lift_out' incorporates:
   *  Gain: '<S1>/Gain2'
   *  Saturate: '<S42>/Saturation'
   */
  rtY.lift_out = 16384.0 / rtP.M2006_max * u0;

  /* Update for DiscreteIntegrator: '<S86>/Integrator' incorporates:
   *  Gain: '<S83>/Integral Gain'
   */
  rtDW.Integrator_DSTATE += rtP.Ki_shoot_up * rtb_Integrator_n *
    rtP.Integrator_gainval;

  /* Update for DiscreteIntegrator: '<S81>/Filter' */
  rtDW.Filter_DSTATE += rtP.Filter_gainval * rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S134>/Integrator' incorporates:
   *  Gain: '<S131>/Integral Gain'
   */
  rtDW.Integrator_DSTATE_p += rtP.Ki_shoot_r * rtb_Integrator_c2 *
    rtP.Integrator_gainval_e;

  /* Update for DiscreteIntegrator: '<S129>/Filter' */
  rtDW.Filter_DSTATE_b += rtP.Filter_gainval_o * rtb_FilterCoefficient_e;

  /* Update for DiscreteIntegrator: '<S182>/Integrator' incorporates:
   *  Gain: '<S179>/Integral Gain'
   */
  rtDW.Integrator_DSTATE_b += rtP.Ki_shoot_l * rtb_Integrator_l *
    rtP.Integrator_gainval_i;

  /* Update for DiscreteIntegrator: '<S177>/Filter' */
  rtDW.Filter_DSTATE_o += rtP.Filter_gainval_n * rtb_FilterCoefficient_o;

  /* Update for DiscreteIntegrator: '<S35>/Integrator' incorporates:
   *  Gain: '<S32>/Integral Gain'
   */
  rtDW.Integrator_DSTATE_j += rtP.Ki_lift * rtb_Sum2 * rtP.Integrator_gainval_o;

  /* Update for DiscreteIntegrator: '<S30>/Filter' */
  rtDW.Filter_DSTATE_ox += rtP.Filter_gainval_l * rtb_FilterCoefficient_d;
}

/* Model initialize function */
void R1_Ball_MODEL_initialize(void)
{
  /* InitializeConditions for DiscreteIntegrator: '<S86>/Integrator' */
  rtDW.Integrator_DSTATE = rtP.DiscretePIDController_Initial_p;

  /* InitializeConditions for DiscreteIntegrator: '<S81>/Filter' */
  rtDW.Filter_DSTATE = rtP.DiscretePIDController_InitialCo;

  /* InitializeConditions for DiscreteIntegrator: '<S134>/Integrator' */
  rtDW.Integrator_DSTATE_p = rtP.DiscretePIDController1_Initia_n;

  /* InitializeConditions for DiscreteIntegrator: '<S129>/Filter' */
  rtDW.Filter_DSTATE_b = rtP.DiscretePIDController1_InitialC;

  /* InitializeConditions for DiscreteIntegrator: '<S182>/Integrator' */
  rtDW.Integrator_DSTATE_b = rtP.DiscretePIDController2_Initia_l;

  /* InitializeConditions for DiscreteIntegrator: '<S177>/Filter' */
  rtDW.Filter_DSTATE_o = rtP.DiscretePIDController2_InitialC;

  /* InitializeConditions for DiscreteIntegrator: '<S35>/Integrator' */
  rtDW.Integrator_DSTATE_j = rtP.DiscretePIDController2_Initia_n;

  /* InitializeConditions for DiscreteIntegrator: '<S30>/Filter' */
  rtDW.Filter_DSTATE_ox = rtP.DiscretePIDController2_Initia_p;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
