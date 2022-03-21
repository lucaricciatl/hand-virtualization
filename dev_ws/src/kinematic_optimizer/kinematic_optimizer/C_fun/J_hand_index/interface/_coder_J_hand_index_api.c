/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: _coder_J_hand_index_api.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 24-Dec-2021 15:14:07
 */

/* Include Files */
#include "_coder_J_hand_index_api.h"
#include "_coder_J_hand_index_mex.h"

/* Variable Definitions */
emlrtCTX emlrtRootTLSGlobal = NULL;
emlrtContext emlrtContextGlobal = { true,/* bFirstTime */
  false,                               /* bInitialized */
  131595U,                             /* fVersionInfo */
  NULL,                                /* fErrorFunction */
  "J_hand_index",                      /* fFunctionName */
  NULL,                                /* fRTCallStack */
  false,                               /* bDebugMode */
  { 2045744189U, 2170104910U, 2743257031U, 4284093946U },/* fSigWrd */
  NULL                                 /* fSigMem */
};

/* Function Declarations */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId);
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId);
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *L_index_middle, const char_T *identifier);
static const mxArray *emlrt_marshallOut(const real_T u[24]);

/* Function Definitions */
/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *u
 *                const emlrtMsgIdentifier *parentId
 * Return Type  : real_T
 */
static real_T b_emlrt_marshallIn(const emlrtStack *sp, const mxArray *u, const
  emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = c_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *src
 *                const emlrtMsgIdentifier *msgId
 * Return Type  : real_T
 */
static real_T c_emlrt_marshallIn(const emlrtStack *sp, const mxArray *src, const
  emlrtMsgIdentifier *msgId)
{
  static const int32_T dims = 0;
  real_T ret;
  emlrtCheckBuiltInR2012b(sp, msgId, src, "double", false, 0U, &dims);
  ret = *(real_T *)emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

/*
 * Arguments    : const emlrtStack *sp
 *                const mxArray *L_index_middle
 *                const char_T *identifier
 * Return Type  : real_T
 */
static real_T emlrt_marshallIn(const emlrtStack *sp, const mxArray
  *L_index_middle, const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  real_T y;
  thisId.fIdentifier = (const char_T *)identifier;
  thisId.fParent = NULL;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(L_index_middle), &thisId);
  emlrtDestroyArray(&L_index_middle);
  return y;
}

/*
 * Arguments    : const real_T u[24]
 * Return Type  : const mxArray *
 */
static const mxArray *emlrt_marshallOut(const real_T u[24])
{
  static const int32_T iv[2] = { 0, 0 };

  static const int32_T iv1[2] = { 6, 4 };

  const mxArray *m;
  const mxArray *y;
  y = NULL;
  m = emlrtCreateNumericArray(2, &iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, iv1, 2);
  emlrtAssign(&y, m);
  return y;
}

/*
 * Arguments    : const mxArray * const prhs[9]
 *                const mxArray *plhs[1]
 * Return Type  : void
 */
void J_hand_index_api(const mxArray * const prhs[9], const mxArray *plhs[1])
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  real_T (*J_07)[24];
  real_T L_index_distal;
  real_T L_index_middle;
  real_T L_index_proximal;
  real_T alpha_1;
  real_T alpha_2;
  real_T q_index_1;
  real_T q_index_2;
  real_T q_index_3;
  real_T q_index_4;
  st.tls = emlrtRootTLSGlobal;
  J_07 = (real_T (*)[24])mxMalloc(sizeof(real_T [24]));

  /* Marshall function inputs */
  L_index_middle = emlrt_marshallIn(&st, emlrtAliasP(prhs[0]), "L_index_middle");
  L_index_distal = emlrt_marshallIn(&st, emlrtAliasP(prhs[1]), "L_index_distal");
  L_index_proximal = emlrt_marshallIn(&st, emlrtAliasP(prhs[2]),
    "L_index_proximal");
  alpha_1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[3]), "alpha_1");
  alpha_2 = emlrt_marshallIn(&st, emlrtAliasP(prhs[4]), "alpha_2");
  q_index_1 = emlrt_marshallIn(&st, emlrtAliasP(prhs[5]), "q_index_1");
  q_index_2 = emlrt_marshallIn(&st, emlrtAliasP(prhs[6]), "q_index_2");
  q_index_3 = emlrt_marshallIn(&st, emlrtAliasP(prhs[7]), "q_index_3");
  q_index_4 = emlrt_marshallIn(&st, emlrtAliasP(prhs[8]), "q_index_4");

  /* Invoke the target function */
  J_hand_index(L_index_middle, L_index_distal, L_index_proximal, alpha_1,
               alpha_2, q_index_1, q_index_2, q_index_3, q_index_4, *J_07);

  /* Marshall function outputs */
  plhs[0] = emlrt_marshallOut(*J_07);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void J_hand_index_atexit(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  J_hand_index_xil_terminate();
  J_hand_index_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void J_hand_index_initialize(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, 0);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void J_hand_index_terminate(void)
{
  emlrtStack st = { NULL,              /* site */
    NULL,                              /* tls */
    NULL                               /* prev */
  };

  st.tls = emlrtRootTLSGlobal;
  emlrtLeaveRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

/*
 * File trailer for _coder_J_hand_index_api.c
 *
 * [EOF]
 */
