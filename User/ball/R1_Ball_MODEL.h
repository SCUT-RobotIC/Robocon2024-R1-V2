/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: R1_Ball_MODEL.h
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

#ifndef RTW_HEADER_R1_Ball_MODEL_h_
#define RTW_HEADER_R1_Ball_MODEL_h_
#ifndef R1_Ball_MODEL_COMMON_INCLUDES_
#define R1_Ball_MODEL_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                                 /* R1_Ball_MODEL_COMMON_INCLUDES_ */

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T Integrator_DSTATE;            /* '<S86>/Integrator' */
  real_T Filter_DSTATE;                /* '<S81>/Filter' */
  real_T Integrator_DSTATE_p;          /* '<S134>/Integrator' */
  real_T Filter_DSTATE_b;              /* '<S129>/Filter' */
  real_T Integrator_DSTATE_b;          /* '<S182>/Integrator' */
  real_T Filter_DSTATE_o;              /* '<S177>/Filter' */
  real_T Integrator_DSTATE_j;          /* '<S35>/Integrator' */
  real_T Filter_DSTATE_ox;             /* '<S30>/Filter' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T shoot_up_tgt;                 /* '<Root>/shoot_up_tgt' */
  real_T shoot_up_in;                  /* '<Root>/shoot_up_in' */
  real_T shoot_r_tgt;                  /* '<Root>/shoot_r_tgt' */
  real_T shoot_r_in;                   /* '<Root>/shoot_r_in' */
  real_T shoot_l_tgt;                  /* '<Root>/shoot_l_tgt' */
  real_T shoot_l_in;                   /* '<Root>/shoot_l_in' */
  real_T lift_tgt;                     /* '<Root>/lift_tgt' */
  real_T lift_in;                      /* '<Root>/lift_in' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T shoot_up_out;                 /* '<Root>/shoot_up_out' */
  real_T shoot_r_out;                  /* '<Root>/shoot_r_out' */
  real_T shoot_l_out;                  /* '<Root>/shoot_l_out' */
  real_T lift_out;                     /* '<Root>/lift_out' */
} ExtY;

/* Parameters (default storage) */
struct P_ {
  real_T Kd_lift;                      /* Variable: Kd_lift
                                        * Referenced by: '<S29>/Derivative Gain'
                                        */
  real_T Kd_shoot_l;                   /* Variable: Kd_shoot_l
                                        * Referenced by: '<S176>/Derivative Gain'
                                        */
  real_T Kd_shoot_r;                   /* Variable: Kd_shoot_r
                                        * Referenced by: '<S128>/Derivative Gain'
                                        */
  real_T Kd_shoot_up;                  /* Variable: Kd_shoot_up
                                        * Referenced by: '<S80>/Derivative Gain'
                                        */
  real_T Ki_lift;                      /* Variable: Ki_lift
                                        * Referenced by: '<S32>/Integral Gain'
                                        */
  real_T Ki_shoot_l;                   /* Variable: Ki_shoot_l
                                        * Referenced by: '<S179>/Integral Gain'
                                        */
  real_T Ki_shoot_r;                   /* Variable: Ki_shoot_r
                                        * Referenced by: '<S131>/Integral Gain'
                                        */
  real_T Ki_shoot_up;                  /* Variable: Ki_shoot_up
                                        * Referenced by: '<S83>/Integral Gain'
                                        */
  real_T Kp_lift;                      /* Variable: Kp_lift
                                        * Referenced by: '<S40>/Proportional Gain'
                                        */
  real_T Kp_shoot_l;                   /* Variable: Kp_shoot_l
                                        * Referenced by: '<S187>/Proportional Gain'
                                        */
  real_T Kp_shoot_r;                   /* Variable: Kp_shoot_r
                                        * Referenced by: '<S139>/Proportional Gain'
                                        */
  real_T Kp_shoot_up;                  /* Variable: Kp_shoot_up
                                        * Referenced by: '<S91>/Proportional Gain'
                                        */
  real_T M2006_max;                    /* Variable: M2006_max
                                        * Referenced by:
                                        *   '<S1>/Gain2'
                                        *   '<S2>/Gain'
                                        *   '<S42>/Saturation'
                                        *   '<S93>/Saturation'
                                        */
  real_T M3508_max;                    /* Variable: M3508_max
                                        * Referenced by:
                                        *   '<S2>/Gain1'
                                        *   '<S2>/Gain2'
                                        *   '<S141>/Saturation'
                                        *   '<S189>/Saturation'
                                        */
  real_T DiscretePIDController_InitialCo;
                              /* Mask Parameter: DiscretePIDController_InitialCo
                               * Referenced by: '<S81>/Filter'
                               */
  real_T DiscretePIDController1_InitialC;
                              /* Mask Parameter: DiscretePIDController1_InitialC
                               * Referenced by: '<S129>/Filter'
                               */
  real_T DiscretePIDController2_InitialC;
                              /* Mask Parameter: DiscretePIDController2_InitialC
                               * Referenced by: '<S177>/Filter'
                               */
  real_T DiscretePIDController2_Initia_p;
                              /* Mask Parameter: DiscretePIDController2_Initia_p
                               * Referenced by: '<S30>/Filter'
                               */
  real_T DiscretePIDController_Initial_p;
                              /* Mask Parameter: DiscretePIDController_Initial_p
                               * Referenced by: '<S86>/Integrator'
                               */
  real_T DiscretePIDController1_Initia_n;
                              /* Mask Parameter: DiscretePIDController1_Initia_n
                               * Referenced by: '<S134>/Integrator'
                               */
  real_T DiscretePIDController2_Initia_l;
                              /* Mask Parameter: DiscretePIDController2_Initia_l
                               * Referenced by: '<S182>/Integrator'
                               */
  real_T DiscretePIDController2_Initia_n;
                              /* Mask Parameter: DiscretePIDController2_Initia_n
                               * Referenced by: '<S35>/Integrator'
                               */
  real_T DiscretePIDController_N;     /* Mask Parameter: DiscretePIDController_N
                                       * Referenced by: '<S89>/Filter Coefficient'
                                       */
  real_T DiscretePIDController1_N;   /* Mask Parameter: DiscretePIDController1_N
                                      * Referenced by: '<S137>/Filter Coefficient'
                                      */
  real_T DiscretePIDController2_N;   /* Mask Parameter: DiscretePIDController2_N
                                      * Referenced by: '<S185>/Filter Coefficient'
                                      */
  real_T DiscretePIDController2_N_b;
                                   /* Mask Parameter: DiscretePIDController2_N_b
                                    * Referenced by: '<S38>/Filter Coefficient'
                                    */
  real_T Integrator_gainval;           /* Computed Parameter: Integrator_gainval
                                        * Referenced by: '<S86>/Integrator'
                                        */
  real_T Filter_gainval;               /* Computed Parameter: Filter_gainval
                                        * Referenced by: '<S81>/Filter'
                                        */
  real_T Integrator_gainval_e;       /* Computed Parameter: Integrator_gainval_e
                                      * Referenced by: '<S134>/Integrator'
                                      */
  real_T Filter_gainval_o;             /* Computed Parameter: Filter_gainval_o
                                        * Referenced by: '<S129>/Filter'
                                        */
  real_T Integrator_gainval_i;       /* Computed Parameter: Integrator_gainval_i
                                      * Referenced by: '<S182>/Integrator'
                                      */
  real_T Filter_gainval_n;             /* Computed Parameter: Filter_gainval_n
                                        * Referenced by: '<S177>/Filter'
                                        */
  real_T Integrator_gainval_o;       /* Computed Parameter: Integrator_gainval_o
                                      * Referenced by: '<S35>/Integrator'
                                      */
  real_T Filter_gainval_l;             /* Computed Parameter: Filter_gainval_l
                                        * Referenced by: '<S30>/Filter'
                                        */
};

/* Parameters (default storage) */
typedef struct P_ P;

/* Block parameters (default storage) */
extern P rtP;

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY rtY;

/* Model entry point functions */
extern void R1_Ball_MODEL_initialize(void);
extern void R1_Ball_MODEL_step(void);

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
 * '<Root>' : 'R1_Ball_MODEL'
 * '<S1>'   : 'R1_Ball_MODEL/lift_speed_loop'
 * '<S2>'   : 'R1_Ball_MODEL/shoot_speed_loop'
 * '<S3>'   : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2'
 * '<S4>'   : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Anti-windup'
 * '<S5>'   : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/D Gain'
 * '<S6>'   : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Filter'
 * '<S7>'   : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Filter ICs'
 * '<S8>'   : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/I Gain'
 * '<S9>'   : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Ideal P Gain'
 * '<S10>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Ideal P Gain Fdbk'
 * '<S11>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Integrator'
 * '<S12>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Integrator ICs'
 * '<S13>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/N Copy'
 * '<S14>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/N Gain'
 * '<S15>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/P Copy'
 * '<S16>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Parallel P Gain'
 * '<S17>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Reset Signal'
 * '<S18>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Saturation'
 * '<S19>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Saturation Fdbk'
 * '<S20>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Sum'
 * '<S21>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Sum Fdbk'
 * '<S22>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Tracking Mode'
 * '<S23>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Tracking Mode Sum'
 * '<S24>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Tsamp - Integral'
 * '<S25>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Tsamp - Ngain'
 * '<S26>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/postSat Signal'
 * '<S27>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/preSat Signal'
 * '<S28>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Anti-windup/Passthrough'
 * '<S29>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/D Gain/Internal Parameters'
 * '<S30>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Filter/Disc. Forward Euler Filter'
 * '<S31>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Filter ICs/Internal IC - Filter'
 * '<S32>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/I Gain/Internal Parameters'
 * '<S33>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Ideal P Gain/Passthrough'
 * '<S34>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S35>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Integrator/Discrete'
 * '<S36>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Integrator ICs/Internal IC'
 * '<S37>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/N Copy/Disabled'
 * '<S38>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/N Gain/Internal Parameters'
 * '<S39>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/P Copy/Disabled'
 * '<S40>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Parallel P Gain/Internal Parameters'
 * '<S41>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Reset Signal/Disabled'
 * '<S42>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Saturation/Enabled'
 * '<S43>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Saturation Fdbk/Disabled'
 * '<S44>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Sum/Sum_PID'
 * '<S45>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Sum Fdbk/Disabled'
 * '<S46>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Tracking Mode/Disabled'
 * '<S47>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S48>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Tsamp - Integral/TsSignalSpecification'
 * '<S49>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S50>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/postSat Signal/Forward_Path'
 * '<S51>'  : 'R1_Ball_MODEL/lift_speed_loop/Discrete PID Controller2/preSat Signal/Forward_Path'
 * '<S52>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller'
 * '<S53>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1'
 * '<S54>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2'
 * '<S55>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Anti-windup'
 * '<S56>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/D Gain'
 * '<S57>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Filter'
 * '<S58>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Filter ICs'
 * '<S59>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/I Gain'
 * '<S60>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Ideal P Gain'
 * '<S61>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Ideal P Gain Fdbk'
 * '<S62>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Integrator'
 * '<S63>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Integrator ICs'
 * '<S64>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/N Copy'
 * '<S65>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/N Gain'
 * '<S66>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/P Copy'
 * '<S67>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Parallel P Gain'
 * '<S68>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Reset Signal'
 * '<S69>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Saturation'
 * '<S70>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Saturation Fdbk'
 * '<S71>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Sum'
 * '<S72>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Sum Fdbk'
 * '<S73>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Tracking Mode'
 * '<S74>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Tracking Mode Sum'
 * '<S75>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Tsamp - Integral'
 * '<S76>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Tsamp - Ngain'
 * '<S77>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/postSat Signal'
 * '<S78>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/preSat Signal'
 * '<S79>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Anti-windup/Passthrough'
 * '<S80>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/D Gain/Internal Parameters'
 * '<S81>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Filter/Disc. Forward Euler Filter'
 * '<S82>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Filter ICs/Internal IC - Filter'
 * '<S83>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/I Gain/Internal Parameters'
 * '<S84>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Ideal P Gain/Passthrough'
 * '<S85>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S86>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Integrator/Discrete'
 * '<S87>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Integrator ICs/Internal IC'
 * '<S88>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/N Copy/Disabled'
 * '<S89>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/N Gain/Internal Parameters'
 * '<S90>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/P Copy/Disabled'
 * '<S91>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Parallel P Gain/Internal Parameters'
 * '<S92>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Reset Signal/Disabled'
 * '<S93>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Saturation/Enabled'
 * '<S94>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Saturation Fdbk/Disabled'
 * '<S95>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Sum/Sum_PID'
 * '<S96>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Sum Fdbk/Disabled'
 * '<S97>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Tracking Mode/Disabled'
 * '<S98>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Tracking Mode Sum/Passthrough'
 * '<S99>'  : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S100>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/Tsamp - Ngain/Passthrough'
 * '<S101>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/postSat Signal/Forward_Path'
 * '<S102>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller/preSat Signal/Forward_Path'
 * '<S103>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Anti-windup'
 * '<S104>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/D Gain'
 * '<S105>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Filter'
 * '<S106>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Filter ICs'
 * '<S107>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/I Gain'
 * '<S108>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Ideal P Gain'
 * '<S109>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Ideal P Gain Fdbk'
 * '<S110>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Integrator'
 * '<S111>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Integrator ICs'
 * '<S112>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/N Copy'
 * '<S113>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/N Gain'
 * '<S114>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/P Copy'
 * '<S115>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Parallel P Gain'
 * '<S116>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Reset Signal'
 * '<S117>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Saturation'
 * '<S118>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Saturation Fdbk'
 * '<S119>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Sum'
 * '<S120>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Sum Fdbk'
 * '<S121>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Tracking Mode'
 * '<S122>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Tracking Mode Sum'
 * '<S123>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Tsamp - Integral'
 * '<S124>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Tsamp - Ngain'
 * '<S125>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/postSat Signal'
 * '<S126>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/preSat Signal'
 * '<S127>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Anti-windup/Passthrough'
 * '<S128>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/D Gain/Internal Parameters'
 * '<S129>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Filter/Disc. Forward Euler Filter'
 * '<S130>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Filter ICs/Internal IC - Filter'
 * '<S131>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/I Gain/Internal Parameters'
 * '<S132>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Ideal P Gain/Passthrough'
 * '<S133>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S134>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Integrator/Discrete'
 * '<S135>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Integrator ICs/Internal IC'
 * '<S136>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/N Copy/Disabled'
 * '<S137>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/N Gain/Internal Parameters'
 * '<S138>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/P Copy/Disabled'
 * '<S139>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S140>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Reset Signal/Disabled'
 * '<S141>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Saturation/Enabled'
 * '<S142>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Saturation Fdbk/Disabled'
 * '<S143>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Sum/Sum_PID'
 * '<S144>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Sum Fdbk/Disabled'
 * '<S145>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Tracking Mode/Disabled'
 * '<S146>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S147>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S148>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S149>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/postSat Signal/Forward_Path'
 * '<S150>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller1/preSat Signal/Forward_Path'
 * '<S151>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Anti-windup'
 * '<S152>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/D Gain'
 * '<S153>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Filter'
 * '<S154>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Filter ICs'
 * '<S155>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/I Gain'
 * '<S156>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Ideal P Gain'
 * '<S157>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Ideal P Gain Fdbk'
 * '<S158>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Integrator'
 * '<S159>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Integrator ICs'
 * '<S160>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/N Copy'
 * '<S161>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/N Gain'
 * '<S162>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/P Copy'
 * '<S163>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Parallel P Gain'
 * '<S164>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Reset Signal'
 * '<S165>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Saturation'
 * '<S166>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Saturation Fdbk'
 * '<S167>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Sum'
 * '<S168>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Sum Fdbk'
 * '<S169>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Tracking Mode'
 * '<S170>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Tracking Mode Sum'
 * '<S171>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Tsamp - Integral'
 * '<S172>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Tsamp - Ngain'
 * '<S173>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/postSat Signal'
 * '<S174>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/preSat Signal'
 * '<S175>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Anti-windup/Passthrough'
 * '<S176>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/D Gain/Internal Parameters'
 * '<S177>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Filter/Disc. Forward Euler Filter'
 * '<S178>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Filter ICs/Internal IC - Filter'
 * '<S179>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/I Gain/Internal Parameters'
 * '<S180>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Ideal P Gain/Passthrough'
 * '<S181>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Ideal P Gain Fdbk/Disabled'
 * '<S182>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Integrator/Discrete'
 * '<S183>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Integrator ICs/Internal IC'
 * '<S184>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/N Copy/Disabled'
 * '<S185>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/N Gain/Internal Parameters'
 * '<S186>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/P Copy/Disabled'
 * '<S187>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Parallel P Gain/Internal Parameters'
 * '<S188>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Reset Signal/Disabled'
 * '<S189>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Saturation/Enabled'
 * '<S190>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Saturation Fdbk/Disabled'
 * '<S191>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Sum/Sum_PID'
 * '<S192>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Sum Fdbk/Disabled'
 * '<S193>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Tracking Mode/Disabled'
 * '<S194>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Tracking Mode Sum/Passthrough'
 * '<S195>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Tsamp - Integral/TsSignalSpecification'
 * '<S196>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/Tsamp - Ngain/Passthrough'
 * '<S197>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/postSat Signal/Forward_Path'
 * '<S198>' : 'R1_Ball_MODEL/shoot_speed_loop/Discrete PID Controller2/preSat Signal/Forward_Path'
 */
#endif                                 /* RTW_HEADER_R1_Ball_MODEL_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
