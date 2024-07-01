/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: Yaw1.h
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

#ifndef RTW_HEADER_Yaw1_h_
#define RTW_HEADER_Yaw1_h_
#ifndef Yaw1_COMMON_INCLUDES_
#define Yaw1_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* Yaw1_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#define Yaw1_M                         (rtM)

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T u2;                           /* '<S3>/Chart1' */
  real_T Integrator_DSTATE;            /* '<S135>/Integrator' */
  real_T Filter_DSTATE;                /* '<S130>/Filter' */
  real_T Filter_DSTATE_j;              /* '<S82>/Filter' */
  real_T Integrator_DSTATE_l;          /* '<S87>/Integrator' */
  real_T Filter_DSTATE_k;              /* '<S31>/Filter' */
  real_T Integrator_DSTATE_lm;         /* '<S36>/Integrator' */
  uint8_T is_active_c1_Yaw1;           /* '<S3>/Chart1' */
  uint8_T is_c1_Yaw1;                  /* '<S3>/Chart1' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T yaw_speed_rpm;                /* '<Root>/yaw_speed_rpm' */
  real_T yaw_target;                   /* '<Root>/yaw_target' */
  real_T yaw_status;                   /* '<Root>/yaw_status' */
  real_T yaw_ecd;                      /* '<Root>/yaw_ecd' */
  real_T yaw_last_ecd;                 /* '<Root>/yaw_last_ecd' */
  real_T yaw_circle;                   /* '<Root>/yaw_circle' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T yaw_SPD_OUT;                  /* '<Root>/yaw_SPD_OUT' */
  real_T yaw_ANG_OUT;                  /* '<Root>/yaw_ANG_OUT' */
  real_T yaw_circle_out;               /* '<Root>/yaw_circle_out' */
} ExtY;

/* Parameters (default storage) */
struct P_ {
  real_T DeadBand;                     /* Variable: DeadBand
                                        * Referenced by:
                                        *   '<S3>/Switch1'
                                        *   '<S3>/Switch2'
                                        */
  real_T trans;                        /* Variable: trans
                                        * Referenced by: '<S3>/Multiply'
                                        */
};

/* Parameters (default storage) */
typedef struct P_ P;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block parameters (default storage) */
extern P rtP;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void Yaw1_initialize(void);
extern void Yaw1_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Note that this particular code originates from a subsystem build,
 * and has its own system numbers different from the parent model.
 * Refer to the system hierarchy for this subsystem below, and use the
 * MATLAB hilite_system command to trace the generated code back
 * to the parent model.  For example,
 *
 * hilite_system('PID_MODEL/Yaw1')    - opens subsystem PID_MODEL/Yaw1
 * hilite_system('PID_MODEL/Yaw1/Kp') - opens and selects block Kp
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'PID_MODEL'
 * '<S1>'   : 'PID_MODEL/Yaw1'
 * '<S2>'   : 'PID_MODEL/Yaw1/If Action_speed Subsystem'
 * '<S3>'   : 'PID_MODEL/Yaw1/If Action_speed Subsystem1'
 * '<S4>'   : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1'
 * '<S5>'   : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Anti-windup'
 * '<S6>'   : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/D Gain'
 * '<S7>'   : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Filter'
 * '<S8>'   : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Filter ICs'
 * '<S9>'   : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/I Gain'
 * '<S10>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Ideal P Gain'
 * '<S11>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Ideal P Gain Fdbk'
 * '<S12>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Integrator'
 * '<S13>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Integrator ICs'
 * '<S14>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/N Copy'
 * '<S15>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/N Gain'
 * '<S16>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/P Copy'
 * '<S17>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Parallel P Gain'
 * '<S18>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Reset Signal'
 * '<S19>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Saturation'
 * '<S20>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Saturation Fdbk'
 * '<S21>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Sum'
 * '<S22>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Sum Fdbk'
 * '<S23>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Tracking Mode'
 * '<S24>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Tracking Mode Sum'
 * '<S25>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Tsamp - Integral'
 * '<S26>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Tsamp - Ngain'
 * '<S27>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/postSat Signal'
 * '<S28>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/preSat Signal'
 * '<S29>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Anti-windup/Passthrough'
 * '<S30>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/D Gain/Internal Parameters'
 * '<S31>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S32>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S33>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/I Gain/Internal Parameters'
 * '<S34>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Ideal P Gain/Passthrough'
 * '<S35>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S36>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Integrator/Discrete'
 * '<S37>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Integrator ICs/Internal IC'
 * '<S38>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/N Copy/Disabled'
 * '<S39>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/N Gain/Internal Parameters'
 * '<S40>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/P Copy/Disabled'
 * '<S41>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S42>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Reset Signal/Disabled'
 * '<S43>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Saturation/Enabled'
 * '<S44>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Saturation Fdbk/Disabled'
 * '<S45>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Sum/Sum_PID'
 * '<S46>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Sum Fdbk/Disabled'
 * '<S47>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Tracking Mode/Disabled'
 * '<S48>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S49>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S50>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S51>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/postSat Signal/Forward_Path'
 * '<S52>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem/Discrete PID Controller1/preSat Signal/Forward_Path'
 * '<S53>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Chart1'
 * '<S54>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller'
 * '<S55>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1'
 * '<S56>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Anti-windup'
 * '<S57>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/D Gain'
 * '<S58>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Filter'
 * '<S59>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Filter ICs'
 * '<S60>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/I Gain'
 * '<S61>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Ideal P Gain'
 * '<S62>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S63>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Integrator'
 * '<S64>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Integrator ICs'
 * '<S65>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/N Copy'
 * '<S66>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/N Gain'
 * '<S67>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/P Copy'
 * '<S68>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Parallel P Gain'
 * '<S69>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Reset Signal'
 * '<S70>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Saturation'
 * '<S71>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Saturation Fdbk'
 * '<S72>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Sum'
 * '<S73>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Sum Fdbk'
 * '<S74>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Tracking Mode'
 * '<S75>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Tracking Mode Sum'
 * '<S76>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Tsamp - Integral'
 * '<S77>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Tsamp - Ngain'
 * '<S78>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/postSat Signal'
 * '<S79>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/preSat Signal'
 * '<S80>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S81>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/D Gain/Internal Parameters'
 * '<S82>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S83>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Filter ICs/Internal IC - Filter'
 * '<S84>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/I Gain/Internal Parameters'
 * '<S85>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S86>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S87>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Integrator/Discrete'
 * '<S88>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S89>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/N Copy/Disabled'
 * '<S90>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/N Gain/Internal Parameters'
 * '<S91>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/P Copy/Disabled'
 * '<S92>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Parallel P Gain/Internal Parameters'
 * '<S93>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Reset Signal/Disabled'
 * '<S94>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Saturation/Enabled'
 * '<S95>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S96>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Sum/Sum_PID'
 * '<S97>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S98>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S99>'  : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S100>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S101>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S102>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S103>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller/preSat Signal/Forward_Path'
 * '<S104>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Anti-windup'
 * '<S105>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/D Gain'
 * '<S106>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Filter'
 * '<S107>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Filter ICs'
 * '<S108>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/I Gain'
 * '<S109>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Ideal P Gain'
 * '<S110>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Ideal P Gain Fdbk'
 * '<S111>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Integrator'
 * '<S112>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Integrator ICs'
 * '<S113>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/N Copy'
 * '<S114>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/N Gain'
 * '<S115>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/P Copy'
 * '<S116>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Parallel P Gain'
 * '<S117>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Reset Signal'
 * '<S118>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Saturation'
 * '<S119>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Saturation Fdbk'
 * '<S120>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Sum'
 * '<S121>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Sum Fdbk'
 * '<S122>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Tracking Mode'
 * '<S123>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Tracking Mode Sum'
 * '<S124>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Tsamp - Integral'
 * '<S125>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Tsamp - Ngain'
 * '<S126>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/postSat Signal'
 * '<S127>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/preSat Signal'
 * '<S128>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Anti-windup/Passthrough'
 * '<S129>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/D Gain/Internal Parameters'
 * '<S130>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S131>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S132>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/I Gain/Internal Parameters'
 * '<S133>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Ideal P Gain/Passthrough'
 * '<S134>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S135>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Integrator/Discrete'
 * '<S136>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Integrator ICs/Internal IC'
 * '<S137>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/N Copy/Disabled'
 * '<S138>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/N Gain/Internal Parameters'
 * '<S139>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/P Copy/Disabled'
 * '<S140>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S141>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Reset Signal/Disabled'
 * '<S142>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Saturation/Enabled'
 * '<S143>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Saturation Fdbk/Disabled'
 * '<S144>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Sum/Sum_PID'
 * '<S145>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Sum Fdbk/Disabled'
 * '<S146>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Tracking Mode/Disabled'
 * '<S147>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S148>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S149>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S150>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/postSat Signal/Forward_Path'
 * '<S151>' : 'PID_MODEL/Yaw1/If Action_speed Subsystem1/Discrete PID Controller1/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_Yaw1_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
