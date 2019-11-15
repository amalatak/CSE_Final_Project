/*************************************************************************
PURPOSE: ( Sensor Model )
**************************************************************************/
#ifndef SENSORS_H
#define SENSORS_H

#include "estimation.hh"
#include "sensors.hh"
#include "UTILITIES.hh"
#include <iostream>
#include <vector>
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif


class sensors {
private:
    UTILITIES utility;
    double horizon_sensor_error;       /* rad horizon sensor error */

public:
    void set_horizon_sensor_error(double deg_error);
    void generate_noise_vec(double error, double noise_vec[3]);
    void generate_noise_mat(double error, double noise_mat[3][3]);
    void horizon_sensor(double Ti2b[3][3], double position[3], double center_dir[3]);
    void camera(double rel_pos[3], double Ti2b[3][3], double target_dir[3]);
    void star_tracker();
    void gyroscope();

};

#ifdef __cplusplus
}
#endif

#endif

