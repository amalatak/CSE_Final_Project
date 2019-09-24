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

# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/S_source.hh
import _m4fbc2b835aa824420d037a47086569f0
combine_cvars(all_cvars, cvar)
cvar = None

# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter.hh
import _md0bdaa7eb6a286abc5bedec9685b5b98
combine_cvars(all_cvars, cvar)
cvar = None

# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter_numeric.hh
import _m4e2ba54134d357f6443bf075f8a67623
combine_cvars(all_cvars, cvar)
cvar = None

# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/S_source.hh
from m4fbc2b835aa824420d037a47086569f0 import *
# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter.hh
from md0bdaa7eb6a286abc5bedec9685b5b98 import *
# /home/amalatak/Documents/Programming/CSE_Final_Project/trick_sims/SIM_gateway/models/orbiter/include/orbiter_numeric.hh
from m4e2ba54134d357f6443bf075f8a67623 import *

# S_source.hh
import _m4fbc2b835aa824420d037a47086569f0
from m4fbc2b835aa824420d037a47086569f0 import *

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

