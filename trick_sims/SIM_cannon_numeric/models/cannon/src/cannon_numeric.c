/*********************************************************************
  PURPOSE: ( Trick numeric )
*********************************************************************/
#include <stddef.h>
#include <stdio.h>
#include "trick/integrator_c_intf.h"
#include "../include/cannon_numeric.h"

int cannon_deriv(CANNON* C) {

    if (!C->impact) {
        C->acc[0] = 0.0 ;
        C->acc[1] = -9.81 ;
    } else {
        C->acc[0] = 0.0 ;
        C->acc[1] = 0.0 ;
    }
    return(0);
}

int cannon_integ(CANNON* C) {
    int ipass;

    load_state(
        &C->pos[0] ,
        &C->pos[1] ,
        &C->vel[0] ,
        &C->vel[1] ,
        NULL);

    load_deriv(
        &C->vel[0] ,
        &C->vel[1] ,
        &C->acc[0] ,
        &C->acc[1] ,
        NULL);

    ipass = integrate();

    unload_state(
        &C->pos[0] ,
        &C->pos[1] ,
        &C->vel[0] ,
        &C->vel[1] ,
        NULL );

    return(ipass);
}
