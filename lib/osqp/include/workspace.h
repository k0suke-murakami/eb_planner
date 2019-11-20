#ifndef WORKSPACE_H
#define WORKSPACE_H

/*
 * This file was autogenerated by OSQP-Python on November 20, 2019 at 20:41:47.
 * 
 * This file contains the prototypes for all the workspace variables needed
 * by OSQP. The actual data is contained inside workspace.c.
 */

#include "types.h"
#include "qdldl_interface.h"

// Data structure prototypes
extern csc Pdata;
extern csc Adata;
extern c_float qdata[300];
extern c_float ldata[300];
extern c_float udata[300];
extern OSQPData data;

// Settings structure prototype
extern OSQPSettings settings;

// Scaling structure prototypes
extern c_float Dscaling[300];
extern c_float Dinvscaling[300];
extern c_float Escaling[300];
extern c_float Einvscaling[300];
extern OSQPScaling scaling;

// Prototypes for linsys_solver structure
extern csc linsys_solver_L;
extern c_float linsys_solver_Dinv[600];
extern c_int linsys_solver_P[600];
extern c_float linsys_solver_bp[600];
extern c_float linsys_solver_sol[600];
extern c_float linsys_solver_rho_inv_vec[300];
extern qdldl_solver linsys_solver;

// Prototypes for solution
extern c_float xsolution[300];
extern c_float ysolution[300];

extern OSQPSolution solution;

// Prototype for info structure
extern OSQPInfo info;

// Prototypes for the workspace
extern c_float work_rho_vec[300];
extern c_float work_rho_inv_vec[300];
extern c_float work_x[300];
extern c_float work_y[300];
extern c_float work_z[300];
extern c_float work_xz_tilde[600];
extern c_float work_x_prev[300];
extern c_float work_z_prev[300];
extern c_float work_Ax[300];
extern c_float work_Px[300];
extern c_float work_Aty[300];
extern c_float work_delta_y[300];
extern c_float work_Atdelta_y[300];
extern c_float work_delta_x[300];
extern c_float work_Pdelta_x[300];
extern c_float work_Adelta_x[300];
extern c_float work_D_temp[300];
extern c_float work_D_temp_A[300];
extern c_float work_E_temp[300];

extern OSQPWorkspace workspace;

#endif // ifndef WORKSPACE_H
