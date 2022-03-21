/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 * File: main.c
 *
 * MATLAB Coder version            : 5.1
 * C/C++ source code generated on  : 24-Dec-2021 15:14:07
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "J_hand_index.h"
#include "J_hand_index_terminate.h"

/* Function Declarations */
static double argInit_real_T(void);
static void main_J_hand_index(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : double
 */
static double argInit_real_T(void)
{
  return 0.0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
static void main_J_hand_index(void)
{
  double J_07[24];
  double L_index_middle_tmp;

  /* Initialize function 'J_hand_index' input arguments. */
  L_index_middle_tmp = argInit_real_T();

  /* Call the entry-point 'J_hand_index'. */
  J_hand_index(L_index_middle_tmp, L_index_middle_tmp, L_index_middle_tmp,
               L_index_middle_tmp, L_index_middle_tmp, L_index_middle_tmp,
               L_index_middle_tmp, L_index_middle_tmp, L_index_middle_tmp, J_07);
}

/*
 * Arguments    : int argc
 *                const char * const argv[]
 * Return Type  : int
 */
int main(int argc, const char * const argv[])
{
  (void)argc;
  (void)argv;

  /* The initialize function is being called automatically from your entry-point function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
     You can call entry-point functions multiple times. */
  main_J_hand_index();

  /* Terminate the application.
     You do not need to do this more than one time. */
  J_hand_index_terminate();
  return 0;
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
