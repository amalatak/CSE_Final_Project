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


void ATTITUDE::print_qerr() {
    printf( "qer = [%.9f, %.9f, %.9f]\n", eul_er_est[0], eul_er_est[1], eul_er_est[2]);
    printf( "dqr = [%.9f, %.9f, %.9f]\n", eul_er_rate_est[0], eul_er_rate_est[1], eul_er_rate_est[2]);
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

    position[0] = pos[0]; position[1] = pos[1]; position[2] = pos[2];
    velocity[0] = vel[0]; velocity[1] = vel[1]; velocity[2] = vel[2];
}

void ATTITUDE::set_target_w_b(double w_target_body_b[3]) {
    target_w_b[0] = w_target_body_b[0];
    target_w_b[1] = w_target_body_b[1];
    target_w_b[2] = w_target_body_b[2];
}


void ATTITUDE::calculate_wdot_body_bwrti() {
    /*
    Calculate rate of change of angular velocity of a spacecraft body with respect to the inertial frame
    coordinatized in the body frame 

    wdot*J = sum(M), so
    wdot = J^(-1)*(sum(M) - cross(w_body_bwrti, Jmat*w_body_bwrti))

    */

    double w_cross[3], Jw[3], t_sub_wcross[3];

    utility.matvecmul(physical_properties.Jmat, w_body_b, Jw);
    utility.cross(w_body_b, Jw, w_cross);

    t_sub_wcross[0] = torques[0] - w_cross[0];
    t_sub_wcross[1] = torques[1] - w_cross[1];
    t_sub_wcross[2] = torques[2] - w_cross[2];

    utility.matvecmul(physical_properties.Jmat_inv, t_sub_wcross, wdot_b_b);

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
    utility.set_vec(0.0, 0.0, 0.000001, w_body_b);
    utility.set_vec(0.0, 0.0, 0.0, torques);

    estimator.calculate_LVLH_i(position, velocity, LVLH_i);
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
    utility.set_vec(0.0, 0.0, 0.00001, w_body_b);
    utility.set_vec(0.0, 0.0, 0.0, torques);

    /* sensors */
    last_measure_time = -1.0;

    estimator.calculate_chaser_frame(r_camera_to_dock, velocity, chaser_i);
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
    estimator.calculate_LVLH_i(position, velocity, LVLH_i);
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

    controller.attitude_control(eul_er_est, eul_er_rate_est, physical_properties.Jmat, torques);
}

void ATTITUDE::estimate_attitude() {
    /* ATTITUDE DETERMINATION */
    double body_chaser_estimate[3][3], body_chaser_estimate_t[3][3];
    double q_extrapolate[4], q_des_2_i[4], dq[4], delta_q[4];

    // Account for sensor discretization
    // If sensor update, read in sensor values
    if (sensor.sensor_rate*(time - last_measure_time) >= 1.0) {
        last_measure_time = time;

        sensor.IMU(time, w_body_b, w_body_b_est);                        // measure angular velocity
        sensor.horizon_sensor(body_i, position, y_hat);                  // measure y-dir
        sensor.camera(r_camera_to_dock, body_i, x_hat);                  // measure x-dir

        estimator.TRIAD(x_hat, y_hat, body_chaser_estimate_t);           // estimate transformation error
        utility.transpose(body_chaser_estimate_t, body_chaser_estimate); // body to chaser frame transformation estimate
        quat_util.DCM2quat(body_chaser_estimate, dq_est);                // quaternion dq

        dq_est_for_interpolation[0] = dq_est[0]; 
        dq_est_for_interpolation[1] = dq_est[1];
        dq_est_for_interpolation[2] = dq_est[2]; 
        dq_est_for_interpolation[3] = dq_est[3];

        q_des_for_inertpolation[0] = q_des[0];
        q_des_for_inertpolation[1] = q_des[1];
        q_des_for_inertpolation[2] = q_des[2];
        q_des_for_inertpolation[3] = q_des[3];

        dq[0] = dq_est[0];
        dq[1] = dq_est[1];
        dq[2] = dq_est[2];
        dq[3] = dq_est[3];

        
    } else {

        /* 
            Interpolation Algorithm is as follows 
            q_i2b = q_bnom2b x q_i2bnom

        */
        double wmag;
        wmag = sqrt(w_body_b_est[0]*w_body_b_est[0] + w_body_b_est[1]*w_body_b_est[1] + w_body_b_est[2]*w_body_b_est[2]);

        quat_util.qmult(dq_est_for_interpolation, q_des_for_inertpolation, q_ib);

        delta_q[0] = sin(wmag*(time - last_measure_time)/2)*(w_body_b_est[0]/wmag);
        delta_q[1] = sin(wmag*(time - last_measure_time)/2)*(w_body_b_est[1]/wmag);
        delta_q[2] = sin(wmag*(time - last_measure_time)/2)*(w_body_b_est[2]/wmag);
        delta_q[3] = cos(wmag*(time - last_measure_time)/2);

        quat_util.qmult(delta_q, q_ib, q_extrapolate);
    
        quat_util.q_conjugate(q_des_for_inertpolation, q_des_2_i);
        quat_util.qmult(q_extrapolate, q_des_2_i, dq);


    }

    quat_util.calculate_euler_error(dq, eul_er_est);
    quat_util.calculate_euler_error_rate(w_body_b_est, w_ref_b, eul_er_est, eul_er_rate_est);

}

void ATTITUDE::chaser_dynamics_update(double pos[], double vel[], double target_pos[], double target_vel[], 
                                      double target_attitude[4], double w_target_b[3], double sim_time) {
    /*
        This script is to update the dynamics of the chaser
        assuming it is set to reference the defined chaser frame and moves
        relative to the target. 
    */
    time = sim_time;
    set_rv(pos, vel);
    set_target_w_b(w_target_b);
    estimator.calculate_r_camera_to_dock(target_attitude, target_pos, q_state, pos, r_camera_to_dock);
    estimator.calculate_r_camera_to_dock_rate(target_vel, target_attitude, w_target_b, vel, q_state, w_body_b, r_camera_to_dock_rate);
    
    estimator.calculate_chaser_frame(r_camera_to_dock, velocity, chaser_i);
    calculate_wdot_body_bwrti();
    quat_util.calculate_qdot(w_body_b, q_state, q_dot);
    quat_util.quat2DCM(q_state, body_i);
    

    controller.set_qdes_target_pointing(chaser_i, q_des);
    controller.set_wdes_target_pointing(r_camera_to_dock, r_camera_to_dock_rate, q_state, w_ref_b);
    
    estimator.discretized_chaser_estimate(time, pos, q_state, q_des, w_ref_b, w_body_b, r_camera_to_dock);

    controller.attitude_control(estimator.euler_error_est, estimator.euler_error_est_rate, physical_properties.Jmat, torques);
    utility.print_vec(estimator.euler_error_est);
}

