/*********************************************************************
  PURPOSE: ( Trick numeric )
*********************************************************************/
#include <stddef.h>
#include <math.h>
#include <iostream>
#include "trick/integrator_c_intf.h"
#include "../include/orbiter_numeric.hh"
#include "../include/ATTITUDE.hh"

using namespace std; 

int orbiter_deriv(ORBITER* C) {
    
    double norm = sqrt(C->pos[0]*C->pos[0] + C->pos[1]*C->pos[1] + C->pos[2]*C->pos[2]);

    C->acc[0] = -C->mu*C->pos[0]/(norm*norm*norm);
    C->acc[1] = -C->mu*C->pos[1]/(norm*norm*norm);
    C->acc[2] = -C->mu*C->pos[2]/(norm*norm*norm);

    return(0);
}

int orbiter_integ(ORBITER* C) {
    int ipass;

    load_state(
        &C->pos[0] ,
        &C->pos[1] ,
        &C->pos[2] ,
        &C->vel[0] ,
        &C->vel[1] ,
        &C->vel[2] ,
        NULL);

    load_deriv(
        &C->vel[0] ,
        &C->vel[1] ,
        &C->vel[2] ,
        &C->acc[0] ,
        &C->acc[1] ,
        &C->acc[2] ,
        NULL);

    ipass = integrate();
    
    C->attitude.set_rv(C->pos, C->vel);
    C->attitude.calculate_LVLH_i();
    C->attitude.calculate_wdot_body_bwrti();
    C->attitude.calculate_body_LVLH();


    unload_state(
        &C->pos[0] ,
        &C->pos[1] ,
        &C->pos[2] ,
        &C->vel[0] ,
        &C->vel[1] ,
        &C->vel[2] ,
        NULL );

    return(ipass);
}
