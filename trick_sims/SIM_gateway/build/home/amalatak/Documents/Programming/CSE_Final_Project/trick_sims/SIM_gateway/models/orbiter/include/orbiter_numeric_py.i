%module m4e2ba54134d357f6443bf075f8a67623

%include "trick/swig/trick_swig.i"


%insert("begin") %{
#include <Python.h>
#include <cstddef>
%}

%{
#include "/home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter_numeric.hh"
%}





#ifndef ORBITER_NUMERIC_H
#define ORBITER_NUMERIC_H

%import "build/home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter_py.i"

#ifdef __cplusplus
extern "C" {
#endif
int orbiter_integ(ORBITER*) ;
int orbiter_deriv(ORBITER*) ;
#ifdef __cplusplus
}
#endif
#endif

