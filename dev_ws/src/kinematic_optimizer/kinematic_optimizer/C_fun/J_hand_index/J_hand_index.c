/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: J_hand_index.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 24-Dec-2021 15:14:07
 */

/* Include Files */
#include "J_hand_index.h"
#include <math.h>

/* Function Definitions */
/*
 * J_HAND_INDEX
 *     J_07 = J_HAND_INDEX(L_INDEX_MIDDLE,L_INDEX_DISTAL,L_INDEX_PROXIMAL,ALPHA_1,ALPHA_2,Q_INDEX_1,Q_INDEX_2,Q_INDEX_3,Q_INDEX_4)
 * Arguments    : double L_index_middle
 *                double L_index_distal
 *                double L_index_proximal
 *                double alpha_1
 *                double alpha_2
 *                double q_index_1
 *                double q_index_2
 *                double q_index_3
 *                double q_index_4
 *                double J_07[24]
 * Return Type  : void
 */
void J_hand_index(double L_index_middle, double L_index_distal, double
                  L_index_proximal, double alpha_1, double alpha_2, double
                  q_index_1, double q_index_2, double q_index_3, double
                  q_index_4, double J_07[24])
{
  double t12;
  double t13;
  double t16;
  double t17;
  double t2;
  double t21;
  double t22;
  double t25;
  double t26;
  double t3;
  double t4;
  double t40;
  double t42;
  double t43;
  double t43_tmp;
  double t44;
  double t47;
  double t5;
  double t50;
  double t51;
  double t54;
  double t55;
  double t6;
  double t60;
  double t62;
  double t63;
  double t65;
  double t68;
  double t69;
  double t7;
  double t71;
  double t72;
  double t72_tmp;
  double t8;
  double t9;

  /*     This function was generated by the Symbolic Math Toolbox version 8.6. */
  /*     24-Dec-2021 15:07:46 */
  t2 = cos(q_index_1);
  t3 = cos(q_index_2);
  t4 = cos(q_index_3);
  t5 = cos(q_index_4);
  t6 = sin(q_index_1);
  t7 = sin(q_index_2);
  t8 = sin(q_index_3);
  t9 = sin(q_index_4);
  t12 = t3 * t4;
  t13 = t3 * t8;
  t4 *= t7;
  t8 *= t7;
  t16 = L_index_middle * t12;
  t17 = L_index_middle * t8;
  t21 = t2 * t13;
  t22 = t2 * t4;
  t25 = t6 * t13;
  t26 = t6 * t4;
  t40 = t9 * (t13 + t4);
  t4 = t21 + t22;
  t42 = t25 + t26;
  t43_tmp = L_index_distal * alpha_2;
  t43 = t43_tmp * t40;
  t44 = t5 * (t12 + -t8);
  t13 = t2 * t12 + t2 * -t8;
  t47 = t6 * t8 + -(t6 * t12);
  t12 = t5 * t4;
  t50 = t9 * t4;
  t51 = t5 * t42;
  t54 = t5 * t13;
  t4 = t9 * t13;
  t8 = t9 * t47;
  t68 = -L_index_distal * (alpha_2 - 1.0) * (t40 - t44);
  t55 = -(t43_tmp * t44);
  t60 = -(t43_tmp * t12);
  t62 = -(t43_tmp * t51);
  t63 = t43_tmp * t8;
  t69 = t54 + -t50;
  t71 = -t51 + t8;
  t65 = -(t43_tmp * t4);
  t72_tmp = L_index_distal * (alpha_2 - 1.0);
  t72 = t72_tmp * (t12 + t4);
  t12 = t72_tmp * (t51 + -t8);
  t40 = L_index_proximal * alpha_1;
  t44 = t40 * t3;
  t51 = L_index_proximal * t3;
  J_07[0] = (((((t6 * t17 + t6 * -t16) - t72_tmp * (t9 * t42 + t5 * t47)) + t51 *
               t6 * (alpha_1 - 1.0)) + t43_tmp * t9 * t42) + t43_tmp * t5 * t47)
    - t44 * t6;
  t8 = L_index_proximal * t2;
  t13 = t40 * t2;
  J_07[1] = (((((t2 * t16 - t2 * t17) + t43_tmp * t54) + t43_tmp * -t50) +
              t72_tmp * (t50 - t54)) - t8 * t3 * (alpha_1 - 1.0)) + t13 * t3;
  J_07[2] = 0.0;
  J_07[3] = 0.0;
  J_07[4] = t71;
  J_07[5] = t69;
  t4 = (((-(L_index_middle * t21) + -(L_index_middle * t22)) + t60) + t65) + t72;
  J_07[6] = (t4 + t8 * t7 * (alpha_1 - 1.0)) - t13 * t7;
  t8 = (((-(L_index_middle * t25) + -(L_index_middle * t26)) + t62) + t63) + t12;
  J_07[7] = (t8 + L_index_proximal * t6 * t7 * (alpha_1 - 1.0)) - t40 * t6 * t7;
  t40 = (((t17 + -t16) + t43) + t55) + t68;
  J_07[8] = (t40 - t44) + t51 * (alpha_1 - 1.0);
  J_07[9] = 0.0;
  J_07[10] = t69;
  J_07[11] = t71;
  J_07[12] = t4;
  J_07[13] = t8;
  J_07[14] = t40;
  J_07[15] = 0.0;
  J_07[16] = t69;
  J_07[17] = t71;
  J_07[18] = (t60 + t65) + t72;
  J_07[19] = (t62 + t63) + t12;
  J_07[20] = (t43 + t55) + t68;
  J_07[21] = 0.0;
  J_07[22] = t69;
  J_07[23] = t71;
}

/*
 * File trailer for J_hand_index.c
 *
 * [EOF]
 */
