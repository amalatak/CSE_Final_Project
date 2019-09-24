from pkgutil import extend_path
__path__ = extend_path(__path__, __name__)
import sys
import os
sys.path.append(os.getcwd() + "/trick")

import _sim_services
from sim_services import *

# create "all_cvars" to hold all global/static vars
all_cvars = new_cvar_list()
combine_cvars(all_cvars, cvar)
cvar = None

# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/S_source.hh
import _md38a7ef8dc1d324ace74a19a83805998
combine_cvars(all_cvars, cvar)
cvar = None

# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/models/cannon/include/cannon.h
import _ma2404a1fbe417ddcf7e1f2dcd4bd85a3
combine_cvars(all_cvars, cvar)
cvar = None

# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/models/cannon/include/cannon_numeric.h
import _md90de8bdc9357de96137da391049d8b6
combine_cvars(all_cvars, cvar)
cvar = None

# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/S_source.hh
from md38a7ef8dc1d324ace74a19a83805998 import *
# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/models/cannon/include/cannon.h
from ma2404a1fbe417ddcf7e1f2dcd4bd85a3 import *
# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_cannon_numeric/models/cannon/include/cannon_numeric.h
from md90de8bdc9357de96137da391049d8b6 import *

# S_source.hh
import _md38a7ef8dc1d324ace74a19a83805998
from md38a7ef8dc1d324ace74a19a83805998 import *

import _top
import top

import _swig_double
import swig_double

import _swig_int
import swig_int

import _swig_ref
import swig_ref

from shortcuts import *

from exception import *

cvar = all_cvars

