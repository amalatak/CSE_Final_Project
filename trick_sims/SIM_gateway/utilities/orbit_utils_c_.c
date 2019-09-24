#include "orbit_utils.h"
#include <stddef.h>
#include <math.h>

/*
double * cross(double u[], double v[]) {
    // calculate u x v
    static double product[3];

    product[0] = u[1]*v[2] - v[1]*u[2];
    product[1] = v[0]*u[2] - u[0]*v[2];
    product[2] = u[0]*v[1] - v[0]*u[1];

    return product ;
}
*/

double * rv2oe(double rv []) {
    static double oe_arr[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    double pnorm, vnorm, pos[3], vel[3];
    //double * h;

    pos[0] = rv[0];
    pos[1] = rv[1];
    pos[2] = rv[2];
    vel[0] = rv[3];
    vel[1] = rv[4];
    vel[2] = rv[5];

    pnorm = sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]);
    vnorm = sqrt(rv[3]*rv[3] + rv[4]*rv[4] + rv[5]*rv[5]);

    //h = cross(pos, vel);

    oe_arr[0] = pnorm;

    return oe_arr ;
}