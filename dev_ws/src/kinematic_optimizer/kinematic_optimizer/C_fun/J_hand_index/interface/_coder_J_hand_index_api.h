/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_J_hand_index_api.h
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 24-Dec-2021 15:14:07
 */

#ifndef _CODER_J_HAND_INDEX_API_H
#define _CODER_J_HAND_INDEX_API_H

/* Include Files */
#include "emlrt.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus

extern "C" {

#endif

  /* Function Declarations */
  void J_hand_index(real_T L_index_middle, real_T L_index_distal, real_T
                    L_index_proximal, real_T alpha_1, real_T alpha_2, real_T
                    q_index_1, real_T q_index_2, real_T q_index_3, real_T
                    q_index_4, real_T J_07[24]);
  void J_hand_index_api(const mxArray * const prhs[9], const mxArray *plhs[1]);
  void J_hand_index_atexit(void);
  void J_hand_index_initialize(void);
  void J_hand_index_terminate(void);
  void J_hand_index_xil_shutdown(void);
  void J_hand_index_xil_terminate(void);

#ifdef __cplusplus

}
#endif
#endif

/*
 * File trailer for _coder_J_hand_index_api.h
 *
 * [EOF]
 */
