/******************************* TRICK HEADER ****************************
PURPOSE: (Set the initial data values)
*************************************************************************/

/* Model Include files */
#include <math.h>
#include <stdio.h>
#include <iostream>
#include "../include/orbiter.hh"

/* utility include file */
#include "../include/ATTITUDE.hh"

using namespace std;

/* default data job */
int orbit_system_default_data( ORBIT_SYSTEM* C ) {

    C->mu = 4.9048695e12;
    C->time = 0.0 ;
    C->r_mag = 1900.0e3 ; // system radii
    C->off_angle = .0001;
    C->chaser.estimator.last_measure_time = -1.0; // for system discretization

    C->target_pos0[0] = C->r_mag ; 
    C->target_pos0[1] = 0.0 ;
    C->target_pos0[2] = 0.0 ;

    C->chaser_pos0[0] = C->r_mag*cos(C->off_angle) ; // chaser lives just behind target
    C->chaser_pos0[1] =-C->r_mag*sin(C->off_angle) ;
    C->chaser_pos0[2] = 0.0 ;

    C->utility.eye(C->target.body_i);
    C->utility.eye(C->chaser.body_i); 

    C->chaser.physical_properties.set_Jmat(100.0, 0.0, 0.0,
                                           0.0, 200.0, 0.0,
                                           0.0, 0.0, 100.0);

    C->target.physical_properties.set_Jmat(1000.0, 0.0, 0.0,
                                           0.0, 2000.0, 0.0,
                                           0.0, 0.0, 1000.0);

    C->utility.set_vec(5.0, -1.0, 1.0, C->chaser.physical_properties.camera_location);         // in the chaser body frame
    C->utility.set_vec(-2.0, -20.0, 3.0, C->chaser.physical_properties.docking_port_location); // in the Target body frame

    C->chaser.sensor.set_horizon_sensor_error(0.01);  // deg
    C->chaser.sensor.set_camera_error(0.00001);       // rad
    C->chaser.sensor.set_gyro_errors(0.01, 0.022);    // deg/hr, deg/hr
    C->chaser.estimator.sensor.sensor_rate = 10.0;    // Hz
    C->target.sensor.set_star_tracker_error(0.00001);

    C->target.controller.set_gains(0.01, 0.0, 0.1);
    C->chaser.controller.set_gains(0.01, 0.0, 0.1);

    C->chaser.sensor.set_horizon_sensor_error(0.0);  // deg
    C->chaser.sensor.set_camera_error(0.0000);       // rad
    C->chaser.sensor.set_gyro_errors(0.0, 0.0);      // deg/hr, deg/hr

    return 0 ;
}

/* initialization job */
int orbit_system_init( ORBIT_SYSTEM* C ) {

    C->chaser.estimator.set_camera_location(C->chaser.physical_properties.camera_location);
    C->chaser.estimator.set_docking_port_location(C->chaser.physical_properties.docking_port_location);
    C->utility.subtract(C->target_pos0, C->chaser_pos0, C->relative_pos0);
    C->chaser.physical_properties.set_Jmat_inv();
    C->target.physical_properties.set_Jmat_inv();

    /* TARGET INITIAL VALUES */
   
    C->target_pos[0] = C->target_pos0[0] ; 
    C->target_pos[1] = C->target_pos0[1] ;
    C->target_pos[2] = C->target_pos0[2] ;

    C->target_vel0[0] = 0.0;
    C->target_vel0[1] = sqrt(C->mu/(C->r_mag));
    C->target_vel0[2] = 0.0;

    C->target_vel[0] = C->target_vel0[0] ; 
    C->target_vel[1] = C->target_vel0[1] ; 
    C->target_vel[2] = C->target_vel0[2] ; 

    C->target_acc[0] = -C->mu*C->target_pos[0]/sqrt(C->target_pos[0]*C->target_pos[0] + 
                                                    C->target_pos[1]*C->target_pos[1] + 
                                                    C->target_pos[2]*C->target_pos[2]);

    C->target_acc[1] = -C->mu*C->target_pos[1]/sqrt(C->target_pos[0]*C->target_pos[0] + 
                                                    C->target_pos[1]*C->target_pos[1] + 
                                                    C->target_pos[2]*C->target_pos[2]);

    C->target_acc[2] = -C->mu*C->target_pos[2]/sqrt(C->target_pos[0]*C->target_pos[0] + 
                                                    C->target_pos[1]*C->target_pos[1] + 
                                                    C->target_pos[2]*C->target_pos[2]);

    C->utility.set_vec(0.0, 0.0, 0.000001, C->target.w_body_b);
    C->utility.set_vec(0.0, 0.0, 0.0, C->target.wdot_b_b);
    C->utility.set_vec(0.0, 0.0, 0.0, C->target.controller.torque_out);

    C->target.calculate_LVLH_i(C->target_pos0, C->target_vel0, C->target.LVLH);
    C->target.quat_util.set_q_from_DCM(C->target.LVLH, 1.0, 1.0, 1.0, 0.001, C->target.q_state);
    C->target.quat_util.set_q_from_DCM(C->target.LVLH, 1.0, 1.0, 1.0, 0.001, C->target.estimator.q_estimate);


    /* CHASER INITIAL VALUES */
   
    C->chaser_pos[0] = C->chaser_pos0[0] ; 
    C->chaser_pos[1] = C->chaser_pos0[1] ;
    C->chaser_pos[2] = C->chaser_pos0[2] ;

    C->chaser_vel0[0] = sqrt(C->mu/(C->r_mag))*sin(C->off_angle);
    C->chaser_vel0[1] = sqrt(C->mu/(C->r_mag))*cos(C->off_angle);
    C->chaser_vel0[2] = 0.0;

    C->chaser_vel[0] = C->chaser_vel0[0] ; 
    C->chaser_vel[1] = C->chaser_vel0[1] ; 
    C->chaser_vel[2] = C->chaser_vel0[2] ; 

    C->chaser_acc[0] = -C->mu*C->chaser_pos[0]/sqrt(C->chaser_pos[0]*C->chaser_pos[0] + 
                                                    C->chaser_pos[1]*C->chaser_pos[1] + 
                                                    C->chaser_pos[2]*C->chaser_pos[2]);

    C->chaser_acc[1] = -C->mu*C->chaser_pos[1]/sqrt(C->chaser_pos[0]*C->chaser_pos[0] + 
                                                    C->chaser_pos[1]*C->chaser_pos[1] + 
                                                    C->chaser_pos[2]*C->chaser_pos[2]);

    C->chaser_acc[2] = -C->mu*C->chaser_pos[2]/sqrt(C->chaser_pos[0]*C->chaser_pos[0] + 
                                                    C->chaser_pos[1]*C->chaser_pos[1] + 
                                                    C->chaser_pos[2]*C->chaser_pos[2]);

    C->utility.set_vec(0.0, 0.0, 0.000001, C->chaser.w_body_b);
    C->utility.set_vec(0.0, 0.0, 0.0, C->chaser.wdot_b_b);
    C->utility.set_vec(0.0, 0.0, 0.0, C->chaser.controller.torque_out);

    C->chaser.calculate_chaser_frame(C->relative_pos0, C->chaser_vel, C->chaser.Chaser_Frame);
    C->chaser.quat_util.set_q_from_DCM(C->chaser.Chaser_Frame, 1.0, 1.0, 1.0, 0.01, C->chaser.q_state);
    C->chaser.quat_util.set_q_from_DCM(C->chaser.Chaser_Frame, 1.0, 1.0, 1.0, 0.01, C->chaser.estimator.q_estimate);

    return 0 ; 
}

