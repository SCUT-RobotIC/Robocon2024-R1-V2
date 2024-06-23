/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Yaw1.c
 *
 * Code generated for Simulink model 'Yaw1'.
 *
 * Model version                  : 2.11
 * Simulink Coder version         : 23.2 (R2023b) 01-Aug-2023
 * C/C++ source code generated on : Tue Feb 27 21:58:32 2024
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "Yaw1.h"
#include "rtwtypes.h"
#include <math.h>

/* Named constants for Chart: '<S3>/Chart1' */
#define IN_ADD                         ((uint8_T)1U)
#define IN_IDLE                        ((uint8_T)2U)
#define IN_SUB                         ((uint8_T)3U)

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void Yaw1_step(void)
{
  real_T rtb_FilterCoefficient;
  real_T rtb_FilterCoefficient_j;
  real_T rtb_FilterCoefficient_n;
  real_T rtb_Sum3;
  real_T rtb_Sum_p;

  /* SwitchCase: '<S1>/Switch Case' incorporates:
   *  Inport: '<Root>/yaw_status'
   */
  switch ((int32_T)rtU.yaw_status) {
   case 1:
    /* Outputs for IfAction SubSystem: '<S1>/If Action_speed Subsystem' incorporates:
     *  ActionPort: '<S2>/Action Port'
     */
    /* Sum: '<S2>/Sum' incorporates:
     *  Inport: '<Root>/yaw_speed_rpm'
     *  Inport: '<Root>/yaw_target'
     */
    rtb_Sum_p = rtU.yaw_target - rtU.yaw_speed_rpm;

    /* Gain: '<S39>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S31>/Filter'
     *  Gain: '<S30>/Derivative Gain'
     *  Sum: '<S31>/SumD'
     */
    rtb_FilterCoefficient_n = (0.0 * rtb_Sum_p - rtDW.Filter_DSTATE_k) * 100.0;

    /* Sum: '<S45>/Sum' incorporates:
     *  DiscreteIntegrator: '<S36>/Integrator'
     *  Gain: '<S41>/Proportional Gain'
     */
    rtb_Sum3 = (0.0 * rtb_Sum_p + rtDW.Integrator_DSTATE_lm) +
      rtb_FilterCoefficient_n;

    /* Saturate: '<S43>/Saturation' */
    if (rtb_Sum3 > 16384.0) {
      /* Outport: '<Root>/yaw_SPD_OUT' */
      rtY.yaw_SPD_OUT = 16384.0;
    } else if (rtb_Sum3 < -16384.0) {
      /* Outport: '<Root>/yaw_SPD_OUT' */
      rtY.yaw_SPD_OUT = -16384.0;
    } else {
      /* Outport: '<Root>/yaw_SPD_OUT' */
      rtY.yaw_SPD_OUT = rtb_Sum3;
    }

    /* End of Saturate: '<S43>/Saturation' */

    /* Update for DiscreteIntegrator: '<S31>/Filter' */
    rtDW.Filter_DSTATE_k += 0.001 * rtb_FilterCoefficient_n;

    /* Update for DiscreteIntegrator: '<S36>/Integrator' incorporates:
     *  Gain: '<S33>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_lm += 0.0 * rtb_Sum_p * 0.001;

    /* End of Outputs for SubSystem: '<S1>/If Action_speed Subsystem' */
    break;

   case 2:
    /* Outputs for IfAction SubSystem: '<S1>/If Action_speed Subsystem1' incorporates:
     *  ActionPort: '<S3>/Action Port'
     */
    /* Sum: '<S3>/Sum' incorporates:
     *  Gain: '<S3>/Gain'
     *  Inport: '<Root>/yaw_circle'
     *  Inport: '<Root>/yaw_ecd'
     *  Inport: '<Root>/yaw_target'
     *  Sum: '<S3>/Sum2'
     */
    rtb_FilterCoefficient_n = rtU.yaw_target - (8191.0 * rtU.yaw_circle +
      rtU.yaw_ecd);

    /* Abs: '<S3>/Abs' */
    rtb_Sum_p = fabs(rtb_FilterCoefficient_n);

    /* Switch: '<S3>/Switch2' incorporates:
     *  Constant: '<S3>/Constant'
     */
    if (!(rtb_Sum_p > rtP.DeadBand)) {
      rtb_FilterCoefficient_n = 0.0;
    }

    /* End of Switch: '<S3>/Switch2' */

    /* Gain: '<S138>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S130>/Filter'
     *  Gain: '<S129>/Derivative Gain'
     *  Sum: '<S130>/SumD'
     */
    rtb_FilterCoefficient = (0.0 * rtb_FilterCoefficient_n - rtDW.Filter_DSTATE)
      * 100.0;

    /* Switch: '<S3>/Switch1' incorporates:
     *  Constant: '<S3>/Constant'
     *  Gain: '<S3>/Multiply'
     *  Inport: '<Root>/yaw_speed_rpm'
     *  Saturate: '<S142>/Saturation'
     *  Sum: '<S3>/Sum1'
     */
    if (rtb_Sum_p > rtP.DeadBand) {
      /* Sum: '<S144>/Sum' incorporates:
       *  DiscreteIntegrator: '<S135>/Integrator'
       *  Gain: '<S140>/Proportional Gain'
       */
      rtb_Sum3 = (0.0 * rtb_FilterCoefficient_n + rtDW.Integrator_DSTATE) +
        rtb_FilterCoefficient;

      /* Saturate: '<S142>/Saturation' */
      if (rtb_Sum3 > 16384.0) {
        rtb_Sum3 = 16384.0;
      } else if (rtb_Sum3 < -16384.0) {
        rtb_Sum3 = -16384.0;
      }

      rtb_Sum_p = rtP.trans * rtb_Sum3 - rtU.yaw_speed_rpm;
    } else {
      rtb_Sum_p = 0.0;
    }

    /* End of Switch: '<S3>/Switch1' */

    /* Gain: '<S90>/Filter Coefficient' incorporates:
     *  DiscreteIntegrator: '<S82>/Filter'
     *  Gain: '<S81>/Derivative Gain'
     *  Sum: '<S82>/SumD'
     */
    rtb_FilterCoefficient_j = (0.0 * rtb_Sum_p - rtDW.Filter_DSTATE_j) * 100.0;

    /* Sum: '<S96>/Sum' incorporates:
     *  DiscreteIntegrator: '<S87>/Integrator'
     *  Gain: '<S92>/Proportional Gain'
     */
    rtb_Sum3 = (0.0 * rtb_Sum_p + rtDW.Integrator_DSTATE_l) +
      rtb_FilterCoefficient_j;

    /* Saturate: '<S94>/Saturation' */
    if (rtb_Sum3 > 16384.0) {
      /* Outport: '<Root>/yaw_ANG_OUT' */
      rtY.yaw_ANG_OUT = 16384.0;
    } else if (rtb_Sum3 < -16384.0) {
      /* Outport: '<Root>/yaw_ANG_OUT' */
      rtY.yaw_ANG_OUT = -16384.0;
    } else {
      /* Outport: '<Root>/yaw_ANG_OUT' */
      rtY.yaw_ANG_OUT = rtb_Sum3;
    }

    /* End of Saturate: '<S94>/Saturation' */

    /* Sum: '<S3>/Sum3' incorporates:
     *  Inport: '<Root>/yaw_ecd'
     *  Inport: '<Root>/yaw_last_ecd'
     */
    rtb_Sum3 = rtU.yaw_ecd - rtU.yaw_last_ecd;

    /* Chart: '<S3>/Chart1' */
    if (rtDW.is_active_c1_Yaw1 == 0U) {
      rtDW.is_active_c1_Yaw1 = 1U;
      rtDW.is_c1_Yaw1 = IN_IDLE;
    } else {
      switch (rtDW.is_c1_Yaw1) {
       case IN_ADD:
        if (rtb_Sum3 <= -5000.0) {
          rtDW.is_c1_Yaw1 = IN_SUB;
          rtDW.u2 = -1.0;
        } else if ((rtb_Sum3 < 5000.0) && (rtb_Sum3 > -5000.0)) {
          rtDW.is_c1_Yaw1 = IN_IDLE;
        }
        break;

       case IN_IDLE:
        if (rtb_Sum3 >= 5000.0) {
          rtDW.is_c1_Yaw1 = IN_ADD;
          rtDW.u2 = 1.0;
        } else if (rtb_Sum3 <= -5000.0) {
          rtDW.is_c1_Yaw1 = IN_SUB;
          rtDW.u2 = -1.0;
        } else {
          rtDW.u2 = 0.0;
        }
        break;

       default:
        /* case IN_SUB: */
        if (rtb_Sum3 >= 5000.0) {
          rtDW.is_c1_Yaw1 = IN_ADD;
          rtDW.u2 = 1.0;
        } else if ((rtb_Sum3 < 5000.0) && (rtb_Sum3 > -5000.0)) {
          rtDW.is_c1_Yaw1 = IN_IDLE;
        }
        break;
      }
    }

    /* End of Chart: '<S3>/Chart1' */

    /* Outport: '<Root>/yaw_circle_out' incorporates:
     *  Inport: '<Root>/yaw_circle'
     *  Sum: '<S3>/Sum4'
     */
    rtY.yaw_circle_out = rtDW.u2 + rtU.yaw_circle;

    /* Update for DiscreteIntegrator: '<S135>/Integrator' incorporates:
     *  Gain: '<S132>/Integral Gain'
     */
    rtDW.Integrator_DSTATE += 0.0 * rtb_FilterCoefficient_n * 0.001;

    /* Update for DiscreteIntegrator: '<S130>/Filter' */
    rtDW.Filter_DSTATE += 0.001 * rtb_FilterCoefficient;

    /* Update for DiscreteIntegrator: '<S82>/Filter' */
    rtDW.Filter_DSTATE_j += 0.001 * rtb_FilterCoefficient_j;

    /* Update for DiscreteIntegrator: '<S87>/Integrator' incorporates:
     *  Gain: '<S84>/Integral Gain'
     */
    rtDW.Integrator_DSTATE_l += 0.0 * rtb_Sum_p * 0.001;

    /* End of Outputs for SubSystem: '<S1>/If Action_speed Subsystem1' */
    break;
  }

  /* End of SwitchCase: '<S1>/Switch Case' */
}

/* Model initialize function */
void Yaw1_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
