/* $Date:  Temmuz 16, 2019$
 * $Revision: 0.99 $
 * $Author: MVK $
 *
 * File : controller.c 
 * Based: IMIdyn.c 
 *
 * Abstract:
 *      S-Function for the adaptive controller
 *
 *
 * Copyright 2019-inf Mesih Veysi Kilinc.
 * $Revision: 0.99 $
 */

#define S_FUNCTION_NAME  controller
#define S_FUNCTION_LEVEL 2

#include "simstruc.h"
#include <math.h>


/*====================*
 * S-function methods *
 *====================*/

/* Function: mdlInitializeSizes ===============================================
 * Abstract:
 *    The sizes information is used by Simulink to determine the S-function
 *    block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
	/* a, k, g1, g2, b_o, m_o */
	ssSetNumSFcnParams(S, 6);
	for(int i = 0; i<6; i++){
		ssSetSFcnParamNotTunable(S, i);
	}

	/* input port information */
	if (!ssSetNumInputPorts(S, 5)) return;

	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortWidth(S, 0, 2);  /* q, qdot  */

	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortWidth(S, 1, 2);  /* e, edot */

	ssSetInputPortDirectFeedThrough(S, 2, 1);
	ssSetInputPortWidth(S, 2, 1);  /* qd_ddot */

	ssSetInputPortDirectFeedThrough(S, 3, 1);
	ssSetInputPortWidth(S, 3, 1);  /* b */

	ssSetInputPortDirectFeedThrough(S, 4, 1);
	ssSetInputPortWidth(S, 4, 1);  /* m */


	/* output port information	 */
	if (!ssSetNumOutputPorts(S,3)) return;

	ssSetOutputPortWidth(S, 0, 1); /* qdoubledot */

	ssSetOutputPortWidth(S, 1, 1); /* bdot */

	ssSetOutputPortWidth(S, 2, 1); /* mdot */
	ssSetNumSampleTimes(S, 1);

	/* Take care when specifying exception free code - see sfuntmpl_doc.c */
	ssSetOptions(S, (SS_OPTION_EXCEPTION_FREE_CODE));

} /* end mdlInitializeSizes */


/* Function: mdlInitializeSampleTimes =========================================
 * Abstract:
 *    S-function is comprised of only continuous sample time elements
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
	ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
	ssSetOffsetTime(S, 0, 0.0);
}


/* Function: mdlOutputs =======================================================
 * Abstract:
 *      calculate the kinematic control output v(t) from the 
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{

	/* input channels  */
	InputRealPtrsType qPtr  = ssGetInputPortRealSignalPtrs(S,0); 
	InputRealPtrsType ePtr  = ssGetInputPortRealSignalPtrs(S,1);  
	InputRealPtrsType qdPtr = ssGetInputPortRealSignalPtrs(S,2);  
	InputRealPtrsType bPtr  = ssGetInputPortRealSignalPtrs(S,3);  
	InputRealPtrsType mPtr  = ssGetInputPortRealSignalPtrs(S,4);  

	real_T   *u        = ssGetOutputPortRealSignal(S,0); 
	real_T   *bdot     = ssGetOutputPortRealSignal(S,1); 
	real_T   *mdot     = ssGetOutputPortRealSignal(S,2); 

	/* inputs */
	real_T q	= *(qPtr[0]);
	real_T qdot	= *(qPtr[1]);
	real_T e	= *(ePtr[0]);
	real_T edot	= *(ePtr[1]);
	real_T qddot2	= *(qdPtr[0]);
	real_T b	= *(bPtr[0]);
	real_T m	= *(mPtr[0]);

	/* parameters */
	real_T a  = mxGetPr(ssGetSFcnParam(S, 0))[0];
	real_T k  = mxGetPr(ssGetSFcnParam(S, 1))[0]; 
	real_T g1 = mxGetPr(ssGetSFcnParam(S, 2))[0];
	real_T g2 = mxGetPr(ssGetSFcnParam(S, 3))[0];
	real_T b0 = mxGetPr(ssGetSFcnParam(S, 4))[0]; 
	real_T m0 = mxGetPr(ssGetSFcnParam(S, 5))[0]; 

	/* helper */
	real_T r = edot + a*e;

	/* outputs */
	bdot[0] = (g1*q*qdot*r)/(1+q*q);
	mdot[0] = (g2*qddot2*r);
	u[0] = b*q*qdot + m*(1+q*q)*qddot2 + (1+q*q)*(k*r + a*edot); 
}


/* Function: mdlTerminate =====================================================
 * Abstract:
 *    No termination needed, but we are required to have this routine.
 */
static void mdlTerminate(SimStruct *S)
{

}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif



/* END function Outher Loop Control */
