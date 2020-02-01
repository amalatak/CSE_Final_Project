/*************************************************************************
PURPOSE: ( Sensor Model )
**************************************************************************/
#ifndef SENSORS_H
#define SENSORS_H

#include "estimation.hh"
#include "sensors.hh"
#include "UTILITIES.hh"
#include "DCM.hh"
#include <iostream>
#include <vector>
#include <math.h>
#include <random>
#include <stdlib.h>


#ifdef __cplusplus
extern "C" {
#endif


class sensors {
private:
    UTILITIES utility;
    DCM dcm;

    double camera_location[3];
    double horizon_sensor_location[3];

    double horizon_sensor_error;       /* rad horizon sensor error */
    double camera_error;
    double star_tracker_error;
    std::default_random_engine generator;

public:
    // Set errors
    void set_horizon_sensor_error(double deg_error);
    void set_camera_error(double error_mag);
    void set_star_tracker_error(double error_mag);

    // Geometry
    void set_horizon_sensor_location(double sensor_x_b, double sensor_y_b, double sensor_z_b);
    void set_camera_location(double camera_x_b, double camera_y_b, double camera_z_b);

    // Generate noise values
    void generate_noise_vec(double error, double noise_vec[3]);
    void generate_noise_mat(double error, double noise_mat[3][3]);
    double generate_noise_scalar(double error);

    // Hardware
    void horizon_sensor(double Ti2b[3][3], double position[3], double center_dir[3]);
    void camera(double rel_pos[3], double Ti2b[3][3], double target_dir[3]);
    void star_tracker(double DCM_i[3][3], double DCM_i_meas[3][3]);

};

#ifdef __cplusplus
}
#endif

#endif

