/*********************************************************************
  PURPOSE: ( Trick numeric )
*********************************************************************/
#include <stddef.h>
#include <math.h>
#include <iostream>
#include "trick/integrator_c_intf.h"
#include "../include/orbiter_numeric.hh"
#include "../include/ATTITUDE.hh"

using namespace std; 

/********************************************************************
ORBIT PROPAGATION VALIDATED WITH MATLAB 
********************************************************************/

int orbit_system_deriv(ORBIT_SYSTEM* C) {
    
    double chaser_norm = sqrt(C->chaser_pos[0]*C->chaser_pos[0] + 
                              C->chaser_pos[1]*C->chaser_pos[1] + 
                              C->chaser_pos[2]*C->chaser_pos[2]);

    C->chaser_acc[0] = -C->mu*C->chaser_pos[0]/(chaser_norm*chaser_norm*chaser_norm);
    C->chaser_acc[1] = -C->mu*C->chaser_pos[1]/(chaser_norm*chaser_norm*chaser_norm);
    C->chaser_acc[2] = -C->mu*C->chaser_pos[2]/(chaser_norm*chaser_norm*chaser_norm);


    double target_norm = sqrt(C->target_pos[0]*C->target_pos[0] + 
                              C->target_pos[1]*C->target_pos[1] + 
                              C->target_pos[2]*C->target_pos[2]);

    C->target_acc[0] = -C->mu*C->target_pos[0]/(target_norm*target_norm*target_norm);
    C->target_acc[1] = -C->mu*C->target_pos[1]/(target_norm*target_norm*target_norm);
    C->target_acc[2] = -C->mu*C->target_pos[2]/(target_norm*target_norm*target_norm);

    return(0);
}

int orbit_system_integ(ORBIT_SYSTEM* C) {
    int ipass;

    load_state(
        &C->chaser_pos[0] ,
        &C->chaser_pos[1] ,
        &C->chaser_pos[2] ,
        &C->chaser_vel[0] ,
        &C->chaser_vel[1] ,
        &C->chaser_vel[2] ,
        &C->chaser.w_body_b[0] ,
        &C->chaser.w_body_b[1] ,
        &C->chaser.w_body_b[2] ,
        &C->chaser.q_state[0] ,
        &C->chaser.q_state[1] ,
        &C->chaser.q_state[2] ,
        &C->chaser.q_state[3] ,
        &C->target_pos[0] ,
        &C->target_pos[1] ,
        &C->target_pos[2] ,
        &C->target_vel[0] ,
        &C->target_vel[1] ,
        &C->target_vel[2] ,
        &C->target.w_body_b[0] ,
        &C->target.w_body_b[1] ,
        &C->target.w_body_b[2] ,
        &C->target.q_state[0] ,
        &C->target.q_state[1] ,
        &C->target.q_state[2] ,
        &C->target.q_state[3] ,
        NULL);

    load_deriv(
        &C->chaser_vel[0] ,
        &C->chaser_vel[1] ,
        &C->chaser_vel[2] ,
        &C->chaser_acc[0] ,
        &C->chaser_acc[1] ,
        &C->chaser_acc[2] ,
        &C->chaser.wdot_b_b[0] ,
        &C->chaser.wdot_b_b[1] ,
        &C->chaser.wdot_b_b[2] ,
        &C->chaser.q_dot[0] ,
        &C->chaser.q_dot[1] ,
        &C->chaser.q_dot[2] ,
        &C->chaser.q_dot[3] ,
        &C->target_vel[0] ,
        &C->target_vel[1] ,
        &C->target_vel[2] ,
        &C->target_acc[0] ,
        &C->target_acc[1] ,
        &C->target_acc[2] ,
        &C->target.wdot_b_b[0] ,
        &C->target.wdot_b_b[1] ,
        &C->target.wdot_b_b[2] ,
        &C->target.q_dot[0] ,
        &C->target.q_dot[1] ,
        &C->target.q_dot[2] ,
        &C->target.q_dot[3] ,
        NULL);

    ipass = integrate();


    C->target.target_dynamics_update(C->target_pos, C->target_vel);
    C->chaser.chaser_dynamics_update(C->chaser_pos, C->chaser_vel, C->target_pos, C->target_vel, C->target.target_i, C->target.w_body_b);
    


    unload_state(
        &C->chaser_pos[0] ,
        &C->chaser_pos[1] ,
        &C->chaser_pos[2] ,
        &C->chaser_vel[0] ,
        &C->chaser_vel[1] ,
        &C->chaser_vel[2] ,
        &C->chaser.w_body_b[0] ,
        &C->chaser.w_body_b[1] ,
        &C->chaser.w_body_b[2] ,
        &C->chaser.q_state[0] ,
        &C->chaser.q_state[1] ,
        &C->chaser.q_state[2] ,
        &C->chaser.q_state[3] ,
        &C->target_pos[0] ,
        &C->target_pos[1] ,
        &C->target_pos[2] ,
        &C->target_vel[0] ,
        &C->target_vel[1] ,
        &C->target_vel[2] ,
        &C->target.w_body_b[0] ,
        &C->target.w_body_b[1] ,
        &C->target.w_body_b[2] ,
        &C->target.q_state[0] ,
        &C->target.q_state[1] ,
        &C->target.q_state[2] ,
        &C->target.q_state[3] ,
        NULL );

    return(ipass);
}
