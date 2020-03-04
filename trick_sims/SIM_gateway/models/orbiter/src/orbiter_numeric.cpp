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
    C->time = get_integ_time();

    /*
        Target Loop
    */

    // Set desired target values as f(r, v)
    C->target.calculate_LVLH_i(C->target_pos, C->target_vel, C->target.LVLH);
    C->target.controller.set_qdes_center_pointing(C->target.LVLH, C->target.controller.q_des);
    C->target.controller.set_wdes_center_pointing(C->target_pos, C->target_vel, C->target.q_state, C->target.controller.w_des);

    // Measure attitude with star trackers
    C->target.quat_util.quat2DCM(C->target.q_state, C->target.body_i);
    C->target.sensor.star_tracker(C->target.body_i, C->target.sensor.DCM_measured);
    C->target.quat_util.DCM2quat(C->target.sensor.DCM_measured, C->target.estimator.q_estimate); // no estimation, q_meas = q_est
    
    // Estimate error
    C->target.quat_util.calculate_quaternion_error(C->target.estimator.q_estimate, C->target.controller.q_des, C->target.estimator.q_error);
    C->target.quat_util.calculate_euler_error(C->target.estimator.q_error, C->target.estimator.euler_error_est);
    C->target.quat_util.calculate_euler_error_rate(C->target.w_body_b, 
                                                   C->target.controller.w_des, 
                                                   C->target.estimator.euler_error_est, 
                                                   C->target.estimator.euler_error_est_rate);

    // Controller
    C->target.controller.attitude_control(C->target.estimator.euler_error_est, 
                                          C->target.estimator.euler_error_est_rate, 
                                          C->target.physical_properties.Jmat,
                                          C->target.controller.torque_out);

    // Actuators go here
    // Dynamics
    C->target.calculate_wdot_body_bwrti(C->target.controller.torque_out, 
                                        C->target.physical_properties.Jmat, 
                                        C->target.physical_properties.Jmat_inv, 
                                        C->target.w_body_b, 
                                        C->target.wdot_b_b);
    C->target.quat_util.calculate_qdot(C->target.w_body_b, C->target.q_state, C->target.q_dot);

    /* 
        Chaser Loop
    */

    /* Desired Values as f(r_t, v_t, q_t, r_c, v_c, q_c) */
    C->chaser.estimator.calculate_r_camera_to_dock(C->target.q_state, C->target_pos, C->chaser.q_state, C->chaser_pos, C->chaser.r_camera_to_dock);
    C->chaser.estimator.calculate_r_camera_to_dock_rate(C->target_vel, C->target.q_state, C->target.w_body_b,
                                                        C->chaser_vel, C->chaser.q_state, C->chaser.w_body_b, 
                                                        C->chaser.r_camera_to_dock_rate);
    
    C->chaser.calculate_chaser_frame(C->chaser.r_camera_to_dock, C->chaser_vel, C->chaser.Chaser_Frame);
    C->chaser.controller.set_qdes_target_pointing(C->chaser.Chaser_Frame, 
                                                  C->chaser.controller.q_des);
    C->chaser.controller.set_wdes_target_pointing(C->chaser.r_camera_to_dock, 
                                                  C->chaser.r_camera_to_dock_rate, 
                                                  C->chaser.q_state, 
                                                  C->chaser.controller.w_des);

    /* Make measurements */
    C->chaser.estimator.discretized_chaser_estimate(C->time, 
                                                    C->chaser_pos, 
                                                    C->chaser.q_state, 
                                                    C->chaser.controller.q_des, 
                                                    C->chaser.controller.w_des, 
                                                    C->chaser.w_body_b, 
                                                    C->chaser.r_camera_to_dock);

    /* Control System */
    C->chaser.controller.attitude_control(C->chaser.estimator.euler_error_est, 
                                          C->chaser.estimator.euler_error_est_rate, 
                                          C->chaser.physical_properties.Jmat, 
                                          C->chaser.controller.torque_out);

    /* Dynamics */
    C->chaser.calculate_wdot_body_bwrti(C->chaser.controller.torque_out, 
                              C->chaser.physical_properties.Jmat, 
                              C->chaser.physical_properties.Jmat_inv, 
                              C->chaser.w_body_b, 
                              C->chaser.wdot_b_b);
    C->chaser.quat_util.calculate_qdot(C->chaser.w_body_b, C->chaser.q_state, C->chaser.q_dot);

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
