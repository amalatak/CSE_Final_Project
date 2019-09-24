#include "orbit_utils.h"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

double * cross(double u[], double v[]) {
    // calculate u x v
    static double product[3];

    product[0] = u[1]*v[2] - v[1]*u[2];
    product[1] = v[0]*u[2] - u[0]*v[2];
    product[2] = u[0]*v[1] - v[0]*u[1];

    return product ;
}

double * divide_arr_by_const(double arr[], double const_value) {
    // divide a len 3 array by constant
    static double divisor[3];

    divisor[0] = arr[0]/const_value;
    divisor[1] = arr[1]/const_value;
    divisor[2] = arr[2]/const_value;

    return divisor;
}

double * array_subtraction(double u[], double v[]) {
    // subtract two length 3 arrays
    static double result[3];
    result[0] = u[0] - v[0];
    result[1] = u[1] - v[1];
    result[2] = u[2] - v[2];
    return result;
}

double inner_product(double u[], double v[]) {
    // calculate inner product of 3x1 u and v
    double product;
    product = u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
    return product;
}

vector<double> test_func(vector<double> vect) {
    vector<double> ret_vec(3);
    ret_vec[0] = vect[0];
    return ret_vec;
}


double * rv2oe(double rv [], double mu) {
    static double oe_arr[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; 
    double pnorm, vnorm, emag, semi_major_axis, RAAN, inclination, arg_of_p, true_anomaly;
    double pos[3], vel[3], z[3] = {0.0, 0.0, 1.0};
    double * h, *n_unitized, *evec, *n_vec;
    vector<double> angular_mom[3];

    pos[0] = rv[0];
    pos[1] = rv[1];
    pos[2] = rv[2];
    vel[0] = rv[3];
    vel[1] = rv[4];
    vel[2] = rv[5];

    pnorm = sqrt(rv[0]*rv[0] + rv[1]*rv[1] + rv[2]*rv[2]);
    vnorm = sqrt(rv[3]*rv[3] + rv[4]*rv[4] + rv[5]*rv[5]);

    h = cross(pos, vel);
    n_vec = cross(z, h);
    n_unitized = divide_arr_by_const(n_vec, sqrt(inner_product(n_vec, n_vec))) ;
    printf("%.9f, %.9f, %.9f\n", n_unitized[0], n_unitized[1], n_unitized[2]);
    divide_arr_by_const(pos, pnorm);
    printf("%.9f, %.9f, %.9f\n", n_unitized[0], n_unitized[1], n_unitized[2]);

    evec = array_subtraction(divide_arr_by_const(cross(vel, h), mu), divide_arr_by_const(pos, pnorm) ) ;
    printf("%.9f, %.9f, %.9f\n", n_unitized[0], n_unitized[1], n_unitized[2]);
    emag = sqrt(inner_product(evec, evec)) ;

    semi_major_axis = -mu/(2*(vnorm*vnorm/2 - mu/pnorm));
    RAAN = acos(n_vec[0]);

    if (n_vec[1] < 0) {
        RAAN = 2*M_PI - RAAN;
    }
    inclination = acos(inner_product(z, h)/sqrt(inner_product(h, h)));

    arg_of_p = acos(inner_product(n_unitized, evec)/emag) ;
    if (evec[2] < 0) {
        arg_of_p = 2*M_PI - arg_of_p;
    }

    true_anomaly = acos(inner_product(pos, evec)/(emag*pnorm));
	if (inner_product(pos, vel) < 0) {
		true_anomaly = 2*M_PI - true_anomaly;
    }

    oe_arr[0] = semi_major_axis;
    oe_arr[1] = emag;
    oe_arr[2] = inclination;
    oe_arr[3] = arg_of_p;
    oe_arr[4] = RAAN;
    oe_arr[5] = true_anomaly;

    return oe_arr ;
}