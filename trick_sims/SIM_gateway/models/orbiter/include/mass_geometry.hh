/*************************************************************************
PURPOSE: ( Orbiter Mass Model )
**************************************************************************/
#ifndef MASS_GEOMETRY_HH
#define MASS_GEOMETRY_HH

#include <iostream>
#include <vector>
#include <math.h>
#include "quaternion.hh"
#include "UTILITIES.hh"


#ifdef __cplusplus
extern "C" {
#endif


class mass_geometry {
private:

    quaternion quat_util;
    UTILITIES utility;

public:
    double Jmat[3][3];         // Inertial matrix
    double Jmat_inv[3][3];     // Inverse of the inertial matrix
    double IMU_location[3];
    double camera_location[3];
    double star_tracker_location[3];
    double docking_port_location[3];
    
    void set_Jmat(double J_00, double J_01, double J_02, 
                  double J_10, double J_11, double J_12, 
                  double J_20, double J_21, double J_22);
    void set_Jmat_inv();

};

#ifdef __cplusplus
}
#endif

#endif

