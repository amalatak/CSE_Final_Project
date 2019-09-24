%module md0bdaa7eb6a286abc5bedec9685b5b98

%include "trick/swig/trick_swig.i"


%insert("begin") %{
#include <Python.h>
#include <cstddef>
%}

%{
#include "/home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter.hh"
%}

%trick_swig_class_typemap(ORBITER, ORBITER)



#ifndef ORBITER_H
#define ORBITER_H

typedef struct {

    double vel0[3] ;    

    double pos0[3] ;    


    double acc[3] ;     

    double vel[3] ;     

    double pos[3] ;     


    double time;        

    double mu;          


} ORBITER ;

#ifdef __cplusplus
extern "C" {
#endif
    int orbiter_default_data(ORBITER*) ;
    int orbiter_init(ORBITER*) ;
    int orbiter_shutdown(ORBITER*) ;
#ifdef __cplusplus
}
#endif

#endif

#ifdef TRICK_SWIG_DEFINED_ORBITER
%trick_cast_as(ORBITER, ORBITER)
#endif
