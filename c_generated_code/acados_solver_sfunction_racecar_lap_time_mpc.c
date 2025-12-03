/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#define S_FUNCTION_NAME acados_solver_sfunction_racecar_lap_time_mpc
#define S_FUNCTION_LEVEL 2

#define MDL_START

// acados
// #include "acados/utils/print.h"
#include "acados_c/sim_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "acados_solver_racecar_lap_time_mpc.h"



#include "simstruc.h"

#define SAMPLINGTIME 0.01


  
  
  
  
  
  
  
  
  
  
  
  
  
  



static void mdlInitializeSizes (SimStruct *S)
{
    // specify the number of continuous and discrete states
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    int N = 50;

  

  

  

  

  

    // specify the number of input ports
    if ( !ssSetNumInputPorts(S, 7) )
        return;

    // specify the number of output ports
    if ( !ssSetNumOutputPorts(S, 6) )
        return;

    // specify dimension information for the input ports
    // lbx_0
    ssSetInputPortVectorDimension(S, 0, 7);
    // ubx_0
    ssSetInputPortVectorDimension(S, 1, 7);
    // parameter_traj
    ssSetInputPortVectorDimension(S, 2, 357);


    // lbx
    ssSetInputPortVectorDimension(S, 3, 49);
    // ubx
    ssSetInputPortVectorDimension(S, 4, 49);
    // lbu
    ssSetInputPortVectorDimension(S, 5, 100);
    // ubu
    ssSetInputPortVectorDimension(S, 6, 100);/* specify dimension information for the OUTPUT ports */
    ssSetOutputPortVectorDimension(S, 0, 2 );
    ssSetOutputPortVectorDimension(S, 1, 1 );
    ssSetOutputPortVectorDimension(S, 2, 1 );
    ssSetOutputPortVectorDimension(S, 3, 7 ); // state at shooting node 1
    ssSetOutputPortVectorDimension(S, 4, 1);
    ssSetOutputPortVectorDimension(S, 5, 1 );// specify the direct feedthrough status
    // should be set to 1 for all inputs used in mdlOutputs
    ssSetInputPortDirectFeedThrough(S, 0, 1);
    ssSetInputPortDirectFeedThrough(S, 1, 1);
    ssSetInputPortDirectFeedThrough(S, 2, 1);
    ssSetInputPortDirectFeedThrough(S, 3, 1);
    ssSetInputPortDirectFeedThrough(S, 4, 1);
    ssSetInputPortDirectFeedThrough(S, 5, 1);
    ssSetInputPortDirectFeedThrough(S, 6, 1);

    // one sample time
    ssSetNumSampleTimes(S, 1);
}


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

#endif /* MATLAB_MEX_FILE */


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
    racecar_lap_time_mpc_solver_capsule *capsule = racecar_lap_time_mpc_acados_create_capsule();
    racecar_lap_time_mpc_acados_create(capsule);

    ssSetUserData(S, (void*)capsule);
}


static void mdlOutputs(SimStruct *S, int_T tid)
{
    racecar_lap_time_mpc_solver_capsule *capsule = ssGetUserData(S);
    ocp_nlp_config *nlp_config = racecar_lap_time_mpc_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = racecar_lap_time_mpc_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = racecar_lap_time_mpc_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out = racecar_lap_time_mpc_acados_get_nlp_out(capsule);
    ocp_nlp_solver *nlp_solver = racecar_lap_time_mpc_acados_get_nlp_solver(capsule);

    InputRealPtrsType in_sign;

    int N = 50;



    // local buffer
    double buffer[357];
    double tmp_double;
    int tmp_offset, tmp_int;

    /* go through inputs */
    // lbx_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 0);
    for (int i = 0; i < 7; i++)
        buffer[i] = (double)(*in_sign[i]);

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", buffer);
    // ubx_0
    in_sign = ssGetInputPortRealSignalPtrs(S, 1);
    for (int i = 0; i < 7; i++)
        buffer[i] = (double)(*in_sign[i]);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", buffer);
    // parameter_traj
    in_sign = ssGetInputPortRealSignalPtrs(S, 2);
    // update value of parameters
    tmp_offset = 0;
    for (int stage = 0; stage <= N; stage++)
    {
        tmp_int = ocp_nlp_dims_get_from_attr(nlp_config, nlp_dims, nlp_out, stage, "p");
        for (int jj = 0; jj < tmp_int; jj++)
        {
            buffer[jj] = (double)(*in_sign[tmp_offset+jj]);
        }
        racecar_lap_time_mpc_acados_update_params(capsule, stage, buffer, tmp_int);
        tmp_offset += tmp_int;
    }

  


  

  
    // lbx
    in_sign = ssGetInputPortRealSignalPtrs(S, 3);
    tmp_offset = 0;
    for (int stage = 1; stage < N; stage++)
    {
        tmp_int = ocp_nlp_dims_get_from_attr(nlp_config, nlp_dims, nlp_out, stage, "lbx");
        for (int jj = 0; jj < tmp_int; jj++)
            buffer[jj] = (double)(*in_sign[tmp_offset+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, stage, "lbx", (void *) buffer);
        tmp_offset += tmp_int;
    }
    // ubx
    in_sign = ssGetInputPortRealSignalPtrs(S, 4);
    tmp_offset = 0;
    for (int stage = 1; stage < N; stage++)
    {
        tmp_int = ocp_nlp_dims_get_from_attr(nlp_config, nlp_dims, nlp_out, stage, "ubx");
        for (int jj = 0; jj < tmp_int; jj++)
            buffer[jj] = (double)(*in_sign[tmp_offset+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, stage, "ubx", (void *) buffer);
        tmp_offset += tmp_int;
    }
    // lbu
    in_sign = ssGetInputPortRealSignalPtrs(S, 5);
    tmp_offset = 0;
    for (int stage = 0; stage < N; stage++)
    {
        tmp_int = ocp_nlp_dims_get_from_attr(nlp_config, nlp_dims, nlp_out, stage, "lbu");
        for (int jj = 0; jj < tmp_int; jj++)
            buffer[jj] = (double)(*in_sign[tmp_offset+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, stage, "lbu", (void *) buffer);
        tmp_offset += tmp_int;
    }
    // ubu
    in_sign = ssGetInputPortRealSignalPtrs(S, 6);
    tmp_offset = 0;
    for (int stage = 0; stage < N; stage++)
    {
        tmp_int = ocp_nlp_dims_get_from_attr(nlp_config, nlp_dims, nlp_out, stage, "ubu");
        for (int jj = 0; jj < tmp_int; jj++)
            buffer[jj] = (double)(*in_sign[tmp_offset+jj]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, stage, "ubu", (void *) buffer);
        tmp_offset += tmp_int;
    }

    int ignore_inits = 0;
    // ssPrintf("ignore_inits = %d\n", ignore_inits);

    if (ignore_inits == 0)
    {
    }/* call solver */
    int acados_status = racecar_lap_time_mpc_acados_solve(capsule);
    // get time
    ocp_nlp_get(nlp_solver, "time_tot", (void *) buffer);
    tmp_double = buffer[0];

    /* set outputs */
    double *out_ptr;
    out_ptr = ssGetOutputPortRealSignal(S, 0);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", (void *) out_ptr);

  

  

  

  
    out_ptr = ssGetOutputPortRealSignal(S, 1);
    *out_ptr = (double) acados_status;
    out_ptr = ssGetOutputPortRealSignal(S, 2);
    *out_ptr = (double) nlp_out->inf_norm_res;
    out_ptr = ssGetOutputPortRealSignal(S, 3);
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", (void *) out_ptr);
    out_ptr = ssGetOutputPortRealSignal(S, 4);
    out_ptr[0] = tmp_double;
    out_ptr = ssGetOutputPortRealSignal(S, 5);
    // get sqp iter
    ocp_nlp_get(nlp_solver, "sqp_iter", (void *) &tmp_int);
    *out_ptr = (double) tmp_int;

  

}

static void mdlTerminate(SimStruct *S)
{
    racecar_lap_time_mpc_solver_capsule *capsule = ssGetUserData(S);

    racecar_lap_time_mpc_acados_free(capsule);
    racecar_lap_time_mpc_acados_free_capsule(capsule);
}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
