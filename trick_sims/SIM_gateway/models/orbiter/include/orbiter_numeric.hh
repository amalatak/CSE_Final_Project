/*************************************************************************
PURPOSE: ( Orbiter Numeric Model )
**************************************************************************/

#ifndef ORBIT_SYSTEM_NUMERIC_H
#define ORBIT_SYSTEM_NUMERIC_H

#include "orbiter.hh"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

int orbit_system_integ(ORBIT_SYSTEM*) ;
int orbit_system_deriv(ORBIT_SYSTEM*) ;

#ifdef __cplusplus
}
#endif
#endif
