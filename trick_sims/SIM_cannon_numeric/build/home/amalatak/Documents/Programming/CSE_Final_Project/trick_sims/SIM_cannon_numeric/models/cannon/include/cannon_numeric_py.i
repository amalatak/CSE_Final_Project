%module md90de8bdc9357de96137da391049d8b6

%include "trick/swig/trick_swig.i"


%insert("begin") %{
#include <Python.h>
#include <cstddef>
%}

%{
#include "/home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/models/cannon/include/cannon_numeric.h"
%}





#ifndef CANNON_NUMERIC_H
#define CANNON_NUMERIC_H

%import "build/home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/models/cannon/include/cannon_py.i"

#ifdef __cplusplus
extern "C" {
#endif
int cannon_integ(CANNON*) ;
int cannon_deriv(CANNON*) ;
#ifdef __cplusplus
}
#endif
#endif

