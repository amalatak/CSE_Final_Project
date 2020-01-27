# This file is for automated processing of simulation data and generating plots

import numpy as np 
import argparse 
import os
import glob
import subprocess


'''*************************************************
    CONSTANTS
*************************************************'''

DEFAULT_SYSTEM_NAME = "dyn"
DEFAULT_OBJECT_RECORD_NAME = "orbit_system"
DEFAULT_OBJECT_PATH = '.'.join([DEFAULT_SYSTEM_NAME, DEFAULT_OBJECT_RECORD_NAME])
DEFAULT_CHASER_OBJECT_PATH = ".".join([DEFAULT_OBJECT_PATH, "chaser"])
DEFAULT_TARGET_OBJECT_PATH = ".".join([DEFAULT_OBJECT_PATH, "target"])
DEFAULT_SIM_RUNTIME = 200
CLEANUP_COMMAND = "make spotless"
COMPILE_COMMAND = "trick-CP"
FILE_ID = "S_main"
RELATIVE_PATH_TO_DATA = "RUN_test/log_{}.csv".format(DEFAULT_OBJECT_RECORD_NAME)


'''*************************************************
    Command Line Arguments
*************************************************'''

parser = argparse.ArgumentParser(description = "Process trick simulation data")

parser.add_argument('--path_2_Sdef',
                   help="""Absolute or relative path to S_define file, if no arg, assumes this
                           is being run in the S_define directory""")
parser.add_argument('--sim_time', default=200,
                   help='Simulated run time, default is 200 seconds')
parser.add_argument('--rc', default=0, nargs='?', const=1,
                   help='Restart the compilation')

# scalar variables
parser.add_argument('--var',  nargs='+',
                   help='Simulation object variable name')
parser.add_argument('--full_path_var',  nargs='+',
                   help='Simulation object variable name, specify full object path')
parser.add_argument('--chaser_var',  nargs='+',
                   help='simulation object variable name, specific to chaser variables')
parser.add_argument('--target_var',  nargs='+',
                   help='simulation object variable name, specific to target variables')

# vector variables, input as a list
parser.add_argument('--var_vec',  nargs='+',
                   help='Simulation object vector variable name with number of elements')
parser.add_argument('--full_path_var_vec',  nargs='+',
                   help='Simulation object vector variable name with number of elements, specify full object path')
parser.add_argument('--chaser_var_vec',  nargs='+',
                   help='simulation object vector variable name with number of elements, specific to chaser variables')
parser.add_argument('--target_var_vec',  nargs='+',
                   help='simulation object vector variable name with number of elements, specific to target variables')
args = parser.parse_args()

'''*************************************************
   Extract and format simulation variables
*************************************************'''
varlist = []

if args.var:
    for var in args.var:
        varlist.append(".".join([DEFAULT_OBJECT_PATH, var]))

if args.chaser_var:
    for var in args.chaser_var:
        varlist.append(".".join([DEFAULT_CHASER_OBJECT_PATH, var]))

if args.target_var:
    for var in args.target_var:
        varlist.append(".".join([DEFAULT_TARGET_OBJECT_PATH, var]))

if args.full_path_var:
    for full_var in args.full_path_var:
        varlist.append(full_var)


''' change to S_define directory if necessary '''
if args.path_2_Sdef:
    os.chdir(args.path_2_Sdef)

'''*************************************************
   Write Trick Input File
*************************************************'''

try:
    input_file = open("RUN_test/automate_input.py","w+")
except:
    print("\nERROR: Could not create temportary input.py file\n")
    raise


stop_command = "trick.stop({0:.1f})\n".format(args.sim_time)
input_file.write(stop_command)

if varlist: # .or. if vectorvarlist
    first_line = "file_data = trick.DRAscii('{}')\n".format(DEFAULT_OBJECT_RECORD_NAME)
    input_file.write(first_line)

    # add variables to input file
    for sim_var in varlist:
        variable_lines = 'file_data.add_variable("{}")\n'.format(sim_var)
        input_file.write(variable_lines)
    
    # add final lines
    freq_line = "file_data.freq = trick.DR_Always\n"
    input_file.write(freq_line)
    record_group_line = "trick.add_data_record_group(file_data, trick.DR_Buffer)\n"
    input_file.write(record_group_line)
    last_line = "file_data.thisown = 0\n"
    input_file.write(last_line)
    
input_file.close()


print('input file contents:')
subprocess.Popen("cat RUN_test/automate_input.py", shell=True)


""" Get executable name for differenct machines """
for file in glob.glob("*.exe"):
    if FILE_ID in file:
        EXECUTABLE = file
    else:
        print("No executable found, where am I???")
        exit()

RUN_COMMAND = "./{} RUN_test/automate_input.py".format(EXECUTABLE)


""" Recompile """
if args.rc:
    os.system(CLEANUP_COMMAND)
os.system(COMPILE_COMMAND)
os.system(RUN_COMMAND)

