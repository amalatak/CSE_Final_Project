/*************************************************************************
PURPOSE: ( Orbiter Numeric Model )
**************************************************************************/

#ifndef ORBITER_NUMERIC_H
#define ORBITER_NUMERIC_H

#include "orbiter.hh"
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif
int orbiter_integ(ORBITER*) ;
int orbiter_deriv(ORBITER*) ;
#ifdef __cplusplus
}
#endif
#endif
