/* $Date:  Temmuz 16, 2019$
 * $Revision: 0.99 $
 * $Author: MVK $
 *
 * File : model.c 
 * Based: IMIdyn.c 
 *
 * Abstract:
 *      S-Function for the simulation model
 *
 *
 * Copyright 2019-inf Mesih Veysi Kilinc.
 * $Revision: 0.99 $
 */

#define S_FUNCTION_NAME  model
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
	ssSetNumSFcnParams(S, 2);
	ssSetSFcnParamNotTunable(S, 0);
	ssSetSFcnParamNotTunable(S, 1);

	/* input port information */
	if (!ssSetNumInputPorts(S, 3)) return;

	ssSetInputPortDirectFeedThrough(S, 0, 1);
	ssSetInputPortWidth(S, 0, 1);  /* q  */

	ssSetInputPortDirectFeedThrough(S, 1, 1);
	ssSetInputPortWidth(S, 1, 1);  /* dq */

	ssSetInputPortDirectFeedThrough(S, 2, 1);
	ssSetInputPortWidth(S, 2, 1);  /* Tau */


	/* output port information	 */
	if (!ssSetNumOutputPorts(S,1)) return;

	ssSetOutputPortWidth(S, 0, 1); /* qdoubledot */
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
	InputRealPtrsType posPtr = ssGetInputPortRealSignalPtrs(S,0); 
	InputRealPtrsType velPtr = ssGetInputPortRealSignalPtrs(S,1);  
	InputRealPtrsType tauPtr = ssGetInputPortRealSignalPtrs(S,2);  

	real_T   *qdoubledot     = ssGetOutputPortRealSignal(S,0); 

	real_T q	= *(posPtr[0]);
	real_T qd	= *(velPtr[0]);
	real_T tau	= *(tauPtr[0]);
	real_T b = mxGetPr(ssGetSFcnParam(S, 0))[0];
	real_T m = mxGetPr(ssGetSFcnParam(S, 1))[0]; 

	qdoubledot[0] = (tau - (b*q*qd)) / (m*(1+q*q));


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
