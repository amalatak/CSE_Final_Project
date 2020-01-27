#include "../include/orbiter.hh"
#include "../include/quaternion.hh"
#include "../include/ATTITUDE.hh"
#include "../include/estimation.hh"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

#include "../../../../dependencies/Eigen/Dense"
#include "../../../../dependencies/Eigen/Eigen"
using std::cout;
using std::endl;

using namespace Eigen;

/*
TO DO:
    Sensor geometry
*/

void ATTITUDE::print_qerr() {
    printf( "qer = [%.9f, %.9f, %.9f]\n", eul_er_est[0], eul_er_est[1], eul_er_est[2]);
    printf( "dqr = [%.9f, %.9f, %.9f]\n", eul_er_rate_est[0], eul_er_rate_est[1], eul_er_rate_est[2]);
}

/********************************************************************
CONSTANTS
********************************************************************/

void ATTITUDE::set_Jmat(double J_00, double J_01, double J_02, 
                        double J_10, double J_11, double J_12, 
                        double J_20, double J_21, double J_22)
    {
    Jmat[0][0] = J_00; Jmat[0][1] = J_01; Jmat[0][2] = J_02;
    Jmat[1][0] = J_10; Jmat[1][1] = J_11; Jmat[1][2] = J_12;
    Jmat[2][0] = J_20; Jmat[2][1] = J_21; Jmat[2][2] = J_22;

}

void ATTITUDE::set_Jmat_inv() {
    /*
    Store the inverse of the J matrix for speed
    
    *** VERIFIED AGAINST MATLAB ***

    */

    Matrix3d J_inter, J_inter_inv;
    J_inter << Jmat[0][0], Jmat[0][1], Jmat[0][2],
               Jmat[1][0], Jmat[1][1], Jmat[1][2],
               Jmat[2][0], Jmat[2][1], Jmat[2][2];  

    if (J_inter.determinant() <= 0.0001) {
        // check if determinant of J matrix is less than some value to ensure proper inversion
        cout << "J matrix is not invertible" << endl;
        exit(1);
    }

    J_inter_inv = J_inter.inverse(); 

    Jmat_inv[0][0] = J_inter_inv(0, 0); Jmat_inv[0][1] = J_inter_inv(0, 1); Jmat_inv[0][2] = J_inter_inv(0, 2);
    Jmat_inv[1][0] = J_inter_inv(1, 0); Jmat_inv[1][1] = J_inter_inv(1, 1); Jmat_inv[1][2] = J_inter_inv(1, 2);
    Jmat_inv[2][0] = J_inter_inv(2, 0); Jmat_inv[2][1] = J_inter_inv(2, 1); Jmat_inv[2][2] = J_inter_inv(2, 2);

}

/********************************************************************
DCM OPERATIONS
********************************************************************/
void ATTITUDE::set_body_i(double body_0_0, double body_0_1, double body_0_2, 
                          double body_1_0, double body_1_1, double body_1_2, 
                          double body_2_0, double body_2_1, double body_2_2) 
    {
    /*
        Initialize or set i2b matrix
    */
    body_i[0][0] = body_0_0; body_i[0][1] = body_0_1; body_i[0][2] = body_0_2;
    body_i[1][0] = body_1_0; body_i[1][1] = body_1_1; body_i[1][2] = body_1_2;
    body_i[2][0] = body_2_0; body_i[2][1] = body_2_1; body_i[2][2] = body_2_2; 
}


/********************************************************************
KINEMATICS
********************************************************************/
void ATTITUDE::set_rv(double pos[], double vel[]) {
    rv[0] = pos[0]; rv[1] = pos[1]; rv[2] = pos[2]; 
    rv[3] = vel[0]; rv[4] = vel[1]; rv[5] = vel[2];

    position[0] = pos[0]; position[1] = pos[1]; position[2] = pos[2];
    velocity[0] = vel[0]; velocity[1] = vel[1]; velocity[2] = vel[2];
}

void ATTITUDE::set_pos_vel_rel(double pos2[], double vel2[]) {
    /*
        This function finds the relative position of the 
        chaser to the target. It sets the relative vector
        to be rel_r = pos_target - pos_chaser
    */
    pos_rel[0] = pos2[0] - position[0];
    pos_rel[1] = pos2[1] - position[1];
    pos_rel[2] = pos2[2] - position[2];

    vel_rel[0] = vel2[0] - velocity[0];
    vel_rel[1] = vel2[1] - velocity[1];
    vel_rel[2] = vel2[2] - velocity[2];
}


void ATTITUDE::calculate_wdot_body_bwrti() {
    /*
    Calculate rate of change of angular velocity of a spacecraft body with respect to the inertial frame
    coordinatized in the body frame 

    wdot*J = sum(M), so
    wdot = J^(-1)*(sum(M) - cross(w_body_bwrti, Jmat*w_body_bwrti))

    */

    double w_cross[3], Jw[3], t_sub_wcross[3];

    utility.matvecmul(Jmat, w_body_b, Jw);
    utility.cross(w_body_b, Jw, w_cross);

    t_sub_wcross[0] = torques[0] - w_cross[0];
    t_sub_wcross[1] = torques[1] - w_cross[1];
    t_sub_wcross[2] = torques[2] - w_cross[2];

    utility.matvecmul(Jmat_inv, t_sub_wcross, wdot_b_b);

}

/********************************************************************
TOP LEVEL SCRIPTS
********************************************************************/


void ATTITUDE::target_initialize() {
    /* 
        This script assumes the following variables have been set:
         * Position
         * Velocity
         * J
        
        And will initialize the following variables:
         * qest
         * LVLH_i
         * w_body
         * wdot_body
         * External Torques
    */
    double offset_eul_mag[3], offset_eul_axis[3];
    offset_eul_mag[0] = 1.0;
    offset_eul_mag[1] = 1.0;
    offset_eul_mag[2] = 1.0;
    utility.norm(offset_eul_mag, offset_eul_axis);

    utility.set_vec(0.0, 0.0, 0.0, wdot_b_b);
    utility.set_vec(0.0, 0.0, 0.0, w_body_b);
    utility.set_vec(0.0, 0.0, 0.0, torques);
    sensor.set_star_tracker_error(0.00001);

    dcm.calculate_LVLH_i(position, velocity, LVLH_i);
    quat_util.set_q(LVLH_i, 0.0, offset_eul_axis, q_state);
    quat_util.set_q(LVLH_i, 0.0, offset_eul_axis, q_est);

}

void ATTITUDE::chaser_initialize() {
    /* 
        This script assumes the following variables have been set:
         * Position
         * Velocity
         * J
        
        And will initialize the following variables:
         * qest
         * chaser_i
         * w_body
         * wdot_body
         * External Torques
    */

    
    double offset_eul_mag[3], offset_eul_axis[3];
    utility.set_vec(1.0, 1.0, 1.0, offset_eul_mag);
    utility.norm(offset_eul_mag, offset_eul_axis);
    utility.set_vec(0.0, 0.0, 0.0, wdot_b_b);
    utility.set_vec(0.0, 0.0, 0.0, w_body_b);
    utility.set_vec(0.0, 0.0, 0.0, torques);

    /* sensors */
    sensor.set_horizon_sensor_location(1.0, -3.0, -1.0);
    sensor.set_horizon_sensor_error(0.0);  // deg

    sensor.set_camera_location(5.0, 1.0, 1.0);
    sensor.set_camera_error(0.0000);       // rad

    utility.set_vec(-2.0, -20, 3, docking_port_location); // in the Target body frame

    dcm.calculate_chaser_frame(pos_rel, velocity, chaser_i);
    quat_util.set_q(chaser_i, 0.01, offset_eul_axis, q_state);
    quat_util.set_q(chaser_i, 0.01, offset_eul_axis, q_est);

}

void ATTITUDE::target_dynamics_update(double pos[], double vel[]) {
    /*
        This script is to update the dynamics of the target
        assuming it is set to reference the LVLH frame and moves
        independent of the chaser. 
    */
    double body_i_meas[3][3];
    set_rv(pos, vel);
    dcm.calculate_LVLH_i(position, velocity, LVLH_i);
    calculate_wdot_body_bwrti();
    quat_util.calculate_qdot(w_body_b, q_state, q_dot); 

    controller.set_qdes_center_pointing(LVLH_i, q_des);
    controller.set_wdes_center_pointing(position, velocity, q_state, w_ref_b);

    quat_util.quat2DCM(q_state, target_i);
    sensor.star_tracker(target_i, body_i_meas);
    quat_util.DCM2quat(body_i_meas, q_est);

    quat_util.calculate_quaternion_error(q_est, q_des, q_err);
    quat_util.calculate_euler_error(q_err, eul_er_est);
    quat_util.calculate_euler_error_rate(w_body_b, w_ref_b, eul_er_est, eul_er_rate_est);

    controller.attitude_control(eul_er_est, eul_er_rate_est, Jmat, torques);
}

void ATTITUDE::estimate_attitude() {
    /* ATTITUDE DETERMINATION */
    double y_hat[3], body_chaser_estimate[3][3], body_chaser_estimate_t[3][3], dq_est[4];

    sensor.horizon_sensor(body_i, position, y_hat);
    sensor.camera(pos_rel, body_i, camera_sensor);
    estimator.TRIAD(camera_sensor, y_hat, body_chaser_estimate_t);
    utility.transpose(body_chaser_estimate_t, body_chaser_estimate);
    quat_util.DCM2quat(body_chaser_estimate, dq_est);
    quat_util.calculate_euler_error(dq_est, eul_er_est);
    quat_util.calculate_euler_error_rate(w_body_b, w_ref_b, eul_er_est, eul_er_rate_est);
}

void ATTITUDE::chaser_dynamics_update(double pos[], double vel[], double target_pos[], double target_vel[], double target_attitude[3][3]) {
    /*
        This script is to update the dynamics of the chaser
        assuming it is set to reference the defined chaser frame and moves
        relative to the target. 
    */
    double port_i[3];
    sensor.set_target_location(target_attitude, docking_port_location, target_pos, port_i);

    set_rv(pos, vel);
    set_pos_vel_rel(target_pos, target_vel);
    
    dcm.calculate_chaser_frame(pos_rel, velocity, chaser_i);
    calculate_wdot_body_bwrti();
    quat_util.calculate_qdot(w_body_b, q_state, q_dot);
    quat_util.quat2DCM(q_state, body_i);
    dcm.calculate_body_chaser(chaser_i, body_i, body_chaser);
    controller.set_qdes_target_pointing(chaser_i, q_des);
    controller.set_wdes_target_pointing(pos_rel, vel_rel, q_state, w_ref_b);
    
    estimate_attitude();

    controller.attitude_control(eul_er_est, eul_er_rate_est, Jmat, torques);
}