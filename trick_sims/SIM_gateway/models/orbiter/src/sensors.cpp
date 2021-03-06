#include "../include/sensors.hh"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>
#include <stdlib.h>

using std::cout;
using std::endl;

/* *** HOW SHOULD I GENERATE NOISE *** */

void sensors::set_horizon_sensor_error(double deg_error) {
    /*  
        Set the horizon sensor error
    */
    horizon_sensor_error = M_PI*deg_error/180.0;
}

void sensors::set_camera_error(double error_mag) {
    /*  
        Set the camera sensor error
    */
    camera_error = error_mag;
}
void sensors::set_star_tracker_error(double error_mag) {
    /*  
        Set the star sensor error
    */
    star_tracker_error = M_PI*error_mag/180.0;
}

void sensors::set_gyro_errors(double bias, double angular_random_walk) {
    /*
        Set the gyro bias and angular random walk
    */

    gyro_bias = bias*(M_PI/180.0)/3600.0;          // rad/s
    gyro_walk = angular_random_walk*(M_PI/180)/60; // rad/s
    generate_noise_vec(1.0, gyro_walk_direction);  // --
}

void sensors::generate_noise_mat(double error, double noise_mat[3][3]) {
    /*  
        This function generates a noise transformation matrix for small signal rotations
        The generated error is a normal random error so error is gaussian.

        Generates the error by rotating by random small errors (RSE)s

            random_error = T3(RSE)*T1(RSE)*T3(RSE)

        Input  : sensor angle read error
        Output : Noise transformation matrix
    */

    std::normal_distribution<double> distribution(0.0, 1.0);   // Distribution w/ mean 0 and std 1

    double T3_error_1[3][3], T1_error_2[3][3], T3_error_3[3][3];
    double error_3_1[3][3];

    dcm.T3(error*distribution(generator), T3_error_1);
    dcm.T1(error*distribution(generator), T1_error_2);
    dcm.T3(error*distribution(generator), T3_error_3);

    utility.matmul(T3_error_1, T1_error_2, error_3_1);
    utility.matmul(error_3_1, T3_error_3, noise_mat);

}
void sensors::generate_noise_vec(double error, double noise_vec[3]) {
    /*  
        Generate a noise vector
    */

    std::normal_distribution<double> distribution(0.0, 1.0);   // Distribution w/ mean 0 and std 1

    noise_vec[0] = error*distribution(generator);
    noise_vec[1] = error*distribution(generator);
    noise_vec[2] = error*distribution(generator);
}

double sensors::generate_noise_scalar(double error) {
    /*  
        Generate a noise scalar
    */
    std::normal_distribution<double> distribution(0.0, 1.0);   // Distribution w/ mean 0 and std 1
    return error*distribution(generator);
}

/* GEOMETRY */

void sensors::set_horizon_sensor_location(double sensor_x_b, double sensor_y_b, double sensor_z_b) {
    /* 
        Set the position of the horizon sensor in the body frame; it
        should be below the spacecraft, i.e. in the negative y direction
        of the chaser frame.
    */
    horizon_sensor_location[0] = sensor_x_b;
    horizon_sensor_location[1] = sensor_y_b;
    horizon_sensor_location[2] = sensor_z_b;
}

void sensors::set_camera_location(double camera_x_b, double camera_y_b, double camera_z_b) {
    /* 
        Set the position of the camera in the body frame; it should be
        in front of the spacecraft, i.e. in the positive x direction of
        the chaser frame.
    */
    camera_location[0] = camera_x_b;
    camera_location[1] = camera_y_b;
    camera_location[2] = camera_z_b;
}


/***************************************************************************

HARDWARE MODELLING

***************************************************************************/

void sensors::horizon_sensor(double Ti2b[3][3], double position[3], double meas_dir[3]) {
    /*  
        Determine the horizon sensor output to the center of the planet
        
        First, generate orthogonal random error matrix
        Use that random error to perturb the "real" inertial to body transformation matrix

        Inputs: (1) Inertial to body transformation matrix (Attitude)
                (2) Inertial position
    
        Output: Normalized direction to the center of the inertial system (in this case
                the center of the Earth)
    */
    double noise_mat[3][3], noisy_Ti2b[3][3], pos_dir[3];  
    generate_noise_mat(horizon_sensor_error, noise_mat);
    utility.norm(position, pos_dir);

    utility.matmul(noise_mat, Ti2b, noisy_Ti2b);
    utility.matvecmul(noisy_Ti2b, pos_dir, meas_dir);


}

void sensors::camera(double rel_pos[3], double Ti2b[3][3], double target_dir_meas[3]) {
    /*  
        This function simulates a camera viewing a target
        The body axes are nominal as such
            x along the target-chaser position vector
            y completes the right handed system along the position vector
            z points out of plane 

        We find the z-component of rel_pos by dotting the direction vector with the body z-axis.
        We find the y-component of rel_pos by dotting the direction vector with the body y-axis.

        Now, measure camera itself relative to target instead of cg relative to target.
        r_cam2target = Ti2b
    */
    
    double noise_y = generate_noise_scalar(camera_error);
    double noise_z = generate_noise_scalar(camera_error);

    double rel_pos_b[3], u_c2t_b[3], zb[3], yb[3];
    double zmeas, ymeas, meas_vec[3];

    utility.matvecmul(Ti2b, rel_pos, rel_pos_b);
    utility.norm(rel_pos_b, u_c2t_b);

    yb[0] = 0.0; yb[1] = 1.0; yb[2] = 0.0;
    zb[0] = 0.0; zb[1] = 0.0; zb[2] = 1.0; 

    zmeas = utility.dot(zb, u_c2t_b);
    ymeas = utility.dot(yb, u_c2t_b);

    meas_vec[0] = sqrt(1 - ymeas*ymeas - zmeas*zmeas);
    meas_vec[1] = ymeas + noise_y;
    meas_vec[2] = zmeas + noise_z;
    
    utility.norm(meas_vec, target_dir_meas);

}

void sensors::star_tracker(double DCM_i[3][3], double DCM_i_meas[3][3]) {
    /*
        This takes in some frame of reference and adds noise, with more noise around one axis 
        to model star trackers
    */
    double noise_mat[3][3];
    generate_noise_mat(star_tracker_error, noise_mat);
    utility.matmul(noise_mat, DCM_i, DCM_i_meas);
}


void sensors::IMU(double time, double w_body[3], double w_meas[3]) {
    /*
        Measure angular velocity of a satellite body

        can use the vector [.8147; .9058; .1270] for testing 
        random walk directions
    */
    double vgyro, perturbations[3];
    vgyro = gyro_walk*gyro_walk;
    
    perturbations[0] = gyro_bias + time*vgyro*gyro_walk_direction[0];
    perturbations[1] = gyro_bias + time*vgyro*gyro_walk_direction[1];
    perturbations[2] = gyro_bias + time*vgyro*gyro_walk_direction[2];
    
    w_meas[0] = w_body[0] + perturbations[0];
    w_meas[1] = w_body[1] + perturbations[1];
    w_meas[2] = w_body[2] + perturbations[2];
}

