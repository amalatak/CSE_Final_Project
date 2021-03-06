/*************************************************************************
PURPOSE: ( Orbiter Attitude Model )
**************************************************************************/
#ifndef CONTROL_H
#define CONTROL_H

#include <iostream>
#include <vector>
#include <math.h>
#include "quaternion.hh"
#include "UTILITIES.hh"


#ifdef __cplusplus
extern "C" {
#endif


class control {
private:
    // Control Values
    double Kp;                        /* -- Proportional Gain */
    double Ki;                        /* -- Integral Gain */
    double Kd;                        /* -- Derivative Gain */
    double pp_w_lim;                  /* rad/s angular velocity limit for phase plane control */
    double pp_angle_lim;              /* rad angular limit for phase plane control */
    double jet_thrust;                /* N thrust force from vernier jets */

    double w_ref_b[3];

    // CONTROL VALUES
    double w_ref_i[3];
    double att_ref[3][3]; 
    double control_out[3];            /* -- Controller output */

    quaternion quat_util;
    UTILITIES utility;

public:
    double r_camera_to_dock[3];
    double r_camera_to_dock_rate[3];
    double q_des[4];
    double w_des[3];
    double torque_out[3];

    /* CONTROLS */
    void set_gains(double K_p, double K_i, double K_d);
    void set_qdes_center_pointing(double LVLH_i[3][3], double qdes[4]);
    void set_wdes_center_pointing(double position[3], double velocity[3], double quat_body[4], double wdes_b[3]);

    void set_qdes_target_pointing(double chaser_frame[3][3], double qdes[4]);
    void set_wdes_target_pointing(double pos_rel[3], double vel_rel[3], double quat_body[4], double wdes_b[3]);
    void set_phase_plane_bounds(double err_omega, double err_angle);
    void set_jet_thrust(double thrust);

    void PD(double eul_err[3], double eul_err_rate[3], double desired_out[3]);
    void Phase_Plane(double eul_err[3], double eul_err_rate[3], double desired_out[3]);
    void phase_plane_control(double eul_err[3], double eul_err_rate[3], double Tout[3]);
    void attitude_control(double eul_err[3], double eul_err_rate[3], double Jmat[3][3], double Tout[3]);

};

#ifdef __cplusplus
}
#endif

#endif

