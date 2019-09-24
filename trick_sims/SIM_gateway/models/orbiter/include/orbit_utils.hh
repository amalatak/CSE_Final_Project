#ifndef ORBIT_UTILS_H
#define ORBIT_UTILS_H


#ifdef __cplusplus
extern "C" {
#endif
#include "orbiter.hh"
#include <stddef.h>
#include <stdio.h>
#include <iostream>
#include <vector>
#include <math.h>

using namespace std;

class ORBIT {
    private:
        double rv_init[6];
    public:
        int rv2oe(ORBITER*);
        void test();
}
//double * rv2oe(double rv[], double mu) ;

#ifdef __cplusplus
}
#endif

#endif

