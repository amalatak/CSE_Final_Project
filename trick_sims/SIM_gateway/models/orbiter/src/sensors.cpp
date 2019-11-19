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
    star_tracker_error = error_mag;
}

void sensors::horizon_sensor(double Ti2b[3][3], double position[3], double center_dir[3]) {
    /*  
        Determine the horizon sensor output to the center of the planet
    */
    double noise_mat[3][3], noise_body[3][3], meas_dir[3], pos_dir[3];  

    generate_noise_mat(horizon_sensor_error, noise_mat);
    utility.matmul(noise_mat, Ti2b, noise_body);

    pos_dir[0] = position[0]/sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2]);
    pos_dir[1] = position[1]/sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2]);
    pos_dir[2] = position[2]/sqrt(position[0]*position[0] + position[1]*position[1] + position[2]*position[2]);

    utility.matvecmul(noise_body, pos_dir, meas_dir);

    center_dir[0] = meas_dir[0]/sqrt(meas_dir[0]*meas_dir[0] + meas_dir[1]*meas_dir[1] + meas_dir[2]*meas_dir[2]);
    center_dir[1] = meas_dir[1]/sqrt(meas_dir[0]*meas_dir[0] + meas_dir[1]*meas_dir[1] + meas_dir[2]*meas_dir[2]);
    center_dir[2] = meas_dir[2]/sqrt(meas_dir[0]*meas_dir[0] + meas_dir[1]*meas_dir[1] + meas_dir[2]*meas_dir[2]);
    
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

    */
    double rel_pos_b[3], u_c2t_b[3], zb[3], yb[3];
    double zmeas, ymeas, meas_vec[3], meas_dir[3], target_dir[3];
    double noise_mat[3][3];

    utility.matvecmul(Ti2b, rel_pos, rel_pos_b);
    utility.norm(rel_pos_b, u_c2t_b);

    yb[0] = 0.0; yb[1] = 1.0; yb[2] = 0.0;
    zb[0] = 0.0; zb[1] = 0.0; zb[2] = 1.0; 

    zmeas = utility.dot(zb, u_c2t_b);
    ymeas = utility.dot(yb, u_c2t_b);

    meas_vec[0] = 1.0;
    meas_vec[1] = ymeas;
    meas_vec[2] = zmeas;

    utility.norm(meas_vec, target_dir);

    generate_noise_mat(camera_error, noise_mat);
    utility.matvecmul(noise_mat, target_dir, meas_dir);

    target_dir_meas[0] = meas_dir[0]/sqrt(meas_dir[0]*meas_dir[0] + meas_dir[1]*meas_dir[1] + meas_dir[2]*meas_dir[2]);
    target_dir_meas[1] = meas_dir[1]/sqrt(meas_dir[0]*meas_dir[0] + meas_dir[1]*meas_dir[1] + meas_dir[2]*meas_dir[2]);
    target_dir_meas[2] = meas_dir[2]/sqrt(meas_dir[0]*meas_dir[0] + meas_dir[1]*meas_dir[1] + meas_dir[2]*meas_dir[2]);

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

void sensors::gyroscope() {
    /*
        TBD
    */
}

void sensors::generate_noise_mat(double error, double noise_mat[3][3]) {
    /*  
        Generate a noise matrix
        Eventually replace with rotation matrix?
    */
    std::normal_distribution<double> distribution(0.0, 1.0);   // Distribution w/ mean 0 and std 1

    noise_mat[0][0] = 1.0 - error*distribution(generator);
    noise_mat[0][1] = error*distribution(generator);
    noise_mat[0][2] = error*distribution(generator);

    noise_mat[1][0] = error*distribution(generator);
    noise_mat[1][1] = 1.0 - error*distribution(generator);
    noise_mat[1][2] = error*distribution(generator);

    noise_mat[2][0] = error*distribution(generator);
    noise_mat[2][1] = error*distribution(generator);;
    noise_mat[2][2] = 1.0 - error*distribution(generator);;

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