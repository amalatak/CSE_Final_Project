/************************************************************************
PURPOSE: (Print the final orbiter state.)
*************************************************************************/
#include <stdio.h>
#include "../include/orbiter.hh"
#include "../include/quaternion.hh"
#include "../include/ATTITUDE.hh"
#include "trick/exec_proto.h"

int orbit_system_shutdown( ORBIT_SYSTEM* C) {
    double t = exec_get_sim_time();
    printf( "========================================\n");
    printf( "        Target State at Shutdown        \n");
    printf( "t = %g\n", t);
    printf( "pos = [%.9f, %.9f, %.9f]\n", C->target_pos[0], C->target_pos[1], C->target_pos[2]);
    printf( "vel = [%.9f, %.9f, %.9f]\n", C->target_vel[0], C->target_vel[1], C->target_vel[2]);
    printf( "qer = [%.9f, %.9f, %.9f]\n", C->target.estimator.euler_error_est[0], 
                                          C->target.estimator.euler_error_est[1], 
                                          C->target.estimator.euler_error_est[2]);
    printf( "dqr = [%.9f, %.9f, %.9f]\n", C->target.estimator.euler_error_est_rate[0], 
                                          C->target.estimator.euler_error_est_rate[1], 
                                          C->target.estimator.euler_error_est_rate[2]);
    printf( "========================================\n");
    printf( "        Chaser State at Shutdown        \n");
    printf( "t = %g\n", t);
    printf( "pos = [%.9f, %.9f, %.9f]\n", C->chaser_pos[0], C->chaser_pos[1], C->chaser_pos[2]);
    printf( "vel = [%.9f, %.9f, %.9f]\n", C->chaser_vel[0], C->chaser_vel[1], C->chaser_vel[2]);
    printf( "qer = [%.9f, %.9f, %.9f]\n", C->chaser.estimator.euler_error_est[0], 
                                          C->chaser.estimator.euler_error_est[1], 
                                          C->chaser.estimator.euler_error_est[2]);
    printf( "dqr = [%.9f, %.9f, %.9f]\n", C->chaser.estimator.euler_error_est_rate[0], 
                                          C->chaser.estimator.euler_error_est_rate[1], 
                                          C->chaser.estimator.euler_error_est_rate[2]);
    printf( "========================================\n");
    return 0 ;
}
