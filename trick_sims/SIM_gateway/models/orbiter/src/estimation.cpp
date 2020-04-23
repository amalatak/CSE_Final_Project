#include "../include/estimation.hh"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

using std::cout;
using std::endl;


void estimation::set_camera_location(double camera_loc[3]) {
    camera_location[0] = camera_loc[0];
    camera_location[1] = camera_loc[1];
    camera_location[2] = camera_loc[2];
}

void estimation::set_docking_port_location(double port_loc[3]) {
    docking_port_location[0] = port_loc[0];
    docking_port_location[1] = port_loc[1];
    docking_port_location[2] = port_loc[2];
}

/* ESTIMATION */
void estimation::TRIAD(double vec1[3], double vec2[3], double frame_est[3][3]) {
    /* 
        This algorithm calculates a reference frame given two measurement vectors
        by establishing a primary vector, vec1 and a plane by crossing the two to 
        get a third vector normal. Then the second vector is crossed with the third
        to get the completed frame which is created as such
        vx = v1
        vz = v1xv2
        vy = vzxvx
        DCM = [vx'
               vy'
               vz']
    */

    double vx[3], vy[3], vz[3], vz_not_norm[3];

    vx[0] = vec1[0]/sqrt(vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2]) ;
    vx[1] = vec1[1]/sqrt(vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2]) ;
    vx[2] = vec1[2]/sqrt(vec1[0]*vec1[0] + vec1[1]*vec1[1] + vec1[2]*vec1[2]) ;

    utility.cross(vx, vec2, vz_not_norm);
    vz[0] = vz_not_norm[0]/sqrt(vz_not_norm[0]*vz_not_norm[0] + vz_not_norm[1]*vz_not_norm[1] + vz_not_norm[2]*vz_not_norm[2]) ;
    vz[1] = vz_not_norm[1]/sqrt(vz_not_norm[0]*vz_not_norm[0] + vz_not_norm[1]*vz_not_norm[1] + vz_not_norm[2]*vz_not_norm[2]) ;
    vz[2] = vz_not_norm[2]/sqrt(vz_not_norm[0]*vz_not_norm[0] + vz_not_norm[1]*vz_not_norm[1] + vz_not_norm[2]*vz_not_norm[2]) ;

    utility.cross(vz, vx, vy);
    frame_est[0][0] = vx[0]; frame_est[0][1] = vx[1]; frame_est[0][2] = vx[2];
    frame_est[1][0] = vy[0]; frame_est[1][1] = vy[1]; frame_est[1][2] = vy[2];
    frame_est[2][0] = vz[0]; frame_est[2][1] = vz[1]; frame_est[2][2] = vz[2];
}

void estimation::calculate_r_camera_to_dock(double q_t[4], double target_pos[3], double q_c[4], double chaser_pos[3], double r_camera_2_dock_i[3]) {
    /*
        Given:
            r_chaser - inertial frame
            r_target - inertial frame
            r_camera - chaser body frame
            r_docking_port - target body frame
        Output:
            r_camera_to_target_dock - inertial frame

        1 - Use target attitude and port position in target frame to find port in inertial frame
        2 - Use chaser attitude and camera position in chaser frame to find camera in inertial frame
        3 - Add these vectors to the r_cg2cg vector
    */
    double target_i2b[3][3], target_b2i[3][3], chaser_i2b[3][3], chaser_b2i[3][3], 
           r_dock_i_rel_to_cg[3], r_cam_i_rel_to_cg[3], 
           port_i[3], cam_i[3];

    quat_util.quat2DCM(q_c, chaser_i2b);
    quat_util.quat2DCM(q_t, target_i2b);

    // get docking port location in inertial frame
    utility.transpose(target_i2b, target_b2i);                                      // Calculate Tb2i
    utility.matvecmul(target_b2i, docking_port_location, r_dock_i_rel_to_cg);       // Calculate Tb2i*r_port_b
    utility.addition(target_pos, r_dock_i_rel_to_cg, port_i);


    // get camera location in inertial frame
    utility.transpose(chaser_i2b, chaser_b2i);
    utility.matvecmul(chaser_b2i, camera_location, r_cam_i_rel_to_cg);
    utility.addition(chaser_pos, r_dock_i_rel_to_cg, cam_i);

    // find r_cam_to_dock
    utility.subtract(port_i, cam_i, r_camera_2_dock_i);
}

void estimation::calculate_r_camera_to_dock_rate(double target_vel[3], double q_t[4], double w_target_b[3], 
                                                 double chaser_vel[3], double q_c[4], double w_chaser_b[3],
                                                 double r_camera_to_dock_rate_i[3]) {
    /* 
    
        Calculate velocity of docking port in the inertial frame

        vd = vcg + wxrd

        Calculate velocity of the camera in the inertial frame
        relative velocity is vd - vc
    */

    double wxr_target_b[3], wxr_target_i[3], target_DCM[3][3], target_attiutde_transpose[3][3], v_docking_port[3],
           wxr_chaser_b[3], wxr_chaser_i[3], chaser_DCM[3][3], chaser_attitude_transpose[3][3], v_camera[3];


    quat_util.quat2DCM(q_t, target_DCM);
    quat_util.quat2DCM(q_c, chaser_DCM);

    // Find camera velocity in inertial frame
    utility.cross(w_chaser_b, camera_location, wxr_chaser_b);
    utility.transpose(chaser_DCM, chaser_attitude_transpose);
    utility.matvecmul(chaser_attitude_transpose, wxr_chaser_b, wxr_chaser_i);

    utility.addition(chaser_vel, wxr_chaser_i, v_camera);

    // Find port velocity in inertial frame
    utility.cross(w_target_b, docking_port_location, wxr_target_b);
    utility.transpose(target_DCM, target_attiutde_transpose);
    utility.matvecmul(chaser_attitude_transpose, wxr_target_b, wxr_target_i);

    utility.addition(target_vel, wxr_target_i, v_docking_port);
    
    // THIS IS NOT WORKING
    utility.subtract(target_vel, chaser_vel, r_camera_to_dock_rate_i);
}

void estimation::estimate_chaser_attitude(double time, double pos[3], double q_c[4], double w_body_b[3], 
                                          double r_camera2dock[3], double DCM_est[3][3]) {
    /* 
        Estimate Chaser Attitude using TRIAS 
    */
    double body_chaser_estimate_t[3][3], body_i[3][3];

    quat_util.quat2DCM(q_c, body_i);
    sensor.IMU(time, w_body_b, w_body_b_est);                        // measure angular velocity
    sensor.horizon_sensor(body_i, pos, y_hat);                       // measure y-dir
    sensor.camera(r_camera2dock, body_i, x_hat);                     // measure x-dir

    TRIAD(x_hat, y_hat, body_chaser_estimate_t);                     // estimate transformation error
    utility.transpose(body_chaser_estimate_t, DCM_est);              // body to chaser frame transformation estimate
}

void estimation::discretized_chaser_estimate(double time, double pos[3], double q_c[4], double qdes[4], double wdes[3], double w_body_b[3], double r_camera2dock[3]) {
    /* 
            ATTITUDE DETERMINATION 

        Inputs:
         * Simulation time
         * Angular velocity in the body frame
         * MCI Position
         * Body DCM
         * Camera to docking port radius
        Outputs:
         * Angular velocity estimate
         * Delta quaternion estimate
        
         
        Interpolation Algorithm is as follows 
        q_i2b = q_bnom2b x q_i2bnom

    */
    double wmag, q_extrapolate[4], q_des_star[4], delta_q[4];
    double test_dq[4];

    // Account for sensor discretization
    // If sensor update, read in sensor values
    if (time != last_time) {
        last_time = time;
        if (sensor.sensor_rate*(time - last_measure_time) >= 1.0) {
            last_measure_time = time; 

            estimate_chaser_attitude(time, pos, q_c, w_body_b, r_camera2dock, DCM_body_estimate);
            quat_util.DCM2quat(DCM_body_estimate, dq_est); 
            quat_util.qmult(dq_est, qdes, q_ib0);
            
        }
        /* Test region */
        estimate_chaser_attitude(time, pos, q_c, w_body_b, r_camera2dock, DCM_body_estimate);
        quat_util.DCM2quat(DCM_body_estimate, test_dq);

        wmag = sqrt(w_body_b_est[0]*w_body_b_est[0] + w_body_b_est[1]*w_body_b_est[1] + w_body_b_est[2]*w_body_b_est[2]);
        delta_q[0] = sin(wmag*(time - last_measure_time)/2)*(w_body_b_est[0]/wmag);
        delta_q[1] = sin(wmag*(time - last_measure_time)/2)*(w_body_b_est[1]/wmag);
        delta_q[2] = sin(wmag*(time - last_measure_time)/2)*(w_body_b_est[2]/wmag);
        delta_q[3] = cos(wmag*(time - last_measure_time)/2);
        
        quat_util.qmult(delta_q, q_ib0, q_extrapolate);   
        quat_util.q_conjugate(qdes, q_des_star);             
        quat_util.qmult(q_extrapolate, q_des_star, dq_est);  

        quat_util.calculate_euler_error(dq_est, euler_error_est);
        quat_util.calculate_euler_error_rate(w_body_b_est, wdes, euler_error_est, euler_error_est_rate);
    }
}