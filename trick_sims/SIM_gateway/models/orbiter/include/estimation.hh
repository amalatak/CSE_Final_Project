/*************************************************************************
PURPOSE: ( Orbiter Attitude Model )
**************************************************************************/
#ifndef ESTIMATION_H
#define ESTIMATION_H

#include <iostream>
#include <vector>
#include <math.h>
#include "UTILITIES.hh"
#include "sensors.hh"
#include "quaternion.hh"
#include "mass_geometry.hh"


#ifdef __cplusplus
extern "C" {
#endif


class estimation {
private:
    UTILITIES utility;
    quaternion quat_util;

    double last_time;
    
    double docking_port_location[3];
    double camera_location[3];
    double DCM_body_estimate[3][3];
    double dq_est[4];
    double q_ib0[4];

    double x_hat[3];
    double y_hat[3];
    double z_hat[3];

    double discretization_enable;

public:
    sensors sensor;
    double last_measure_time;
    double r_cam2dock[3];
    double r_cam2dock_rate[3];

    double DCM_estimate[3][3];
    double q_estimate[4];
    double q_error[4];
    
    double w_body_b_est[3];
    double euler_error_est[3];
    double euler_error_est_rate[3];

    estimation();
    void enable_discretization();
    void disable_discretization();

    void set_camera_location(double camera_loc[3]);
    void set_docking_port_location(double port_loc[3]);

    void calculate_r_camera_to_dock(double q_t[4], double target_pos[3], double q_c[4], double chaser_pos[3], double r_camera_2_dock_i[3]);
    void calculate_r_camera_to_dock_rate(double target_vel[3], double q_t[4], double w_target_b[3], 
                                         double chaser_vel[3], double q_c[4], double w_chaser_b[3],
                                         double r_camera_to_dock_rate_i[3]);

    /* ESTIMATION */
    void TRIAD(double vec1[3], double vec2[3], double frame_est[3][3]);
    void estimate_chaser_attitude(double time, double pos[3], double q_c[4], double w_body_b[3], double r_camera2dock[3], double DCM_est[3][3]);
    void discretized_chaser_estimate(double time, double pos[3], double q_c[4], double qdes[4], double wdes[3], double w_body_b[3], double r_camera2dock[3]);
};

#ifdef __cplusplus
}
#endif

#endif

