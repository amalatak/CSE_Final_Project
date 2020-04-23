import numpy as np 
import argparse 
import re
import os
import glob
import subprocess
import csv
import matplotlib.pyplot as plt


'''*************************************************
    CONSTANTS
*************************************************'''

DEFAULT_SYSTEM_NAME = "dyn"
DEFAULT_OBJECT_RECORD_NAME = "orbit_system"
DEFAULT_OBJECT_PATH = '.'.join([DEFAULT_SYSTEM_NAME, DEFAULT_OBJECT_RECORD_NAME])
DEFAULT_CHASER_PATH = '.'.join([DEFAULT_OBJECT_PATH, "chaser"])
DEFAULT_TARGET_PATH = '.'.join([DEFAULT_OBJECT_PATH, "target"])
DEFAULT_DATA_FOLDER = "RUN_test"
DEFAULT_SIM_RUNTIME = 200
CLEANUP_COMMAND = "make spotless"
COMPILE_COMMAND = "trick-CP"
FILE_ID = "S_main"
RELATIVE_PATH_TO_DATA = os.path.join(DEFAULT_DATA_FOLDER, "log_{}.csv".format(DEFAULT_OBJECT_RECORD_NAME))


'''*************************************************
    Command Line Arguments
*************************************************'''

parser = argparse.ArgumentParser(description = "Process trick simulation data")

parser.add_argument('--path_2_Sdef',
                   help="""Absolute or relative path to S_define file, if no arg, assumes this
                           is being run in the S_define directory""")
parser.add_argument('--sim_time', default=200,
                   help='Simulated run time, default is {} seconds'.format(DEFAULT_SIM_RUNTIME))
parser.add_argument('--rc', default=0, nargs='?', const=1,
                   help='Restart the compilation')
parser.add_argument('--show_plots', default=0, nargs='?', const=1,
                   help='Show plots of given variables')

# scalar variables
# scalar variables
parser.add_argument('--var',  nargs='+',
                   help='Simulation object variable name')
parser.add_argument('--full_path_var',  nargs='+',
                   help='Simulation object variable name, specify full object path')
parser.add_argument('--chaser_var',  nargs='+',
                   help='Simulation object variable name')
parser.add_argument('--target_var',  nargs='+',
                   help='Simulation object variable name, specify full object path')

# vector variables, input as a list
parser.add_argument('--var_vec',  nargs='+',
                   help='Simulation object vector variable name with number of elements. \
                   Input in variable[n_elements] format')
parser.add_argument('--full_path_var_vec',  nargs='+',
                   help="Simulation object vector variable name with number of elements, \
                    specify full object path. Input in variable[n_elements] format")
parser.add_argument('--chaser_var_vec',  nargs='+',
                   help='Simulation object vector variable name with number of elements. \
                   Input in variable[n_elements] format')
parser.add_argument('--target_var_vec',  nargs='+',
                   help="Simulation object vector variable name with number of elements, \
                    specify full object path. Input in variable[n_elements] format")

args = parser.parse_args()

'''*************************************************
   Extract and format simulation variables
*************************************************'''
varlist = []

if args.var:
    for var in args.var:
        varlist.append(".".join([DEFAULT_OBJECT_PATH, var]))

if args.full_path_var:
    for full_var in args.full_path_var:
        varlist.append(full_var)

if args.chaser_var:
    for var in args.chaser_var:
        varlist.append(".".join([DEFAULT_CHASER_PATH, var]))

if args.target_var:
    for var in args.target_var:
        varlist.append(".".join([DEFAULT_TARGET_PATH, var]))

vector_vars = []
pattern = re.compile(r"\[(\d+)\]")

if args.var_vec:
    for var in args.var_vec:
        # seperate vector terms and the number of terms from the input
        n_elements = int(pattern.findall(var)[0])
        variable_name = var.strip("[{}]".format(str(n_elements)))
        vector_vars.append([".".join([DEFAULT_OBJECT_PATH, variable_name]), n_elements])

if args.full_path_var_vec:
    for full_var in args.full_path_var_vec:
        # seperate vector terms and the number of terms from the input
        n_elements = int(pattern.findall(full_var)[0])
        variable_name = full_var.strip("[{}]".format(str(n_elements)))
        vector_vars.append([variable_name, n_elements])

if args.chaser_var_vec:
    for var in args.chaser_var_vec:
        # seperate vector terms and the number of terms from the input
        n_elements = int(pattern.findall(var)[0])
        variable_name = var.strip("[{}]".format(str(n_elements)))
        vector_vars.append([".".join([DEFAULT_CHASER_PATH, variable_name]), n_elements])

if args.target_var_vec:
    for var in args.target_var_vec:
        # seperate vector terms and the number of terms from the input
        n_elements = int(pattern.findall(var)[0])
        variable_name = var.strip("[{}]".format(str(n_elements)))
        vector_vars.append([".".join([DEFAULT_TARGET_PATH, variable_name]), n_elements])


''' change to S_define directory if necessary '''
if args.path_2_Sdef:
    os.chdir(args.path_2_Sdef)

'''*************************************************
   Write Trick Input File
*************************************************'''

# open input file
try:
    input_file = open(os.path.join(DEFAULT_DATA_FOLDER, "automate_input.py"), "w+")
except:
    # When opening the input file, the automate_input file path may not exist, which will
    # raise an error
    print("\nERROR: Could not create input.py file {} \n".format(input_file))
    raise

# write simulation stop command
stop_command = "trick.stop({0:.1f})\n".format(float(args.sim_time))
input_file.write(stop_command)

# if variables are input, write them to the simulation
if varlist or vector_vars:
    # initialize data saving
    first_line = "file_data = trick.DRAscii('{}')\n".format(DEFAULT_OBJECT_RECORD_NAME)
    input_file.write(first_line)

    # add variables to input file
    for sim_var in varlist:
        variable_lines = 'file_data.add_variable("{}")\n'.format(sim_var)
        input_file.write(variable_lines)

    for data_vec in vector_vars:
        for i in range(data_vec[1]):
            variable_lines = 'file_data.add_variable("{0}[{1}]")\n'.format(data_vec[0], i)
            input_file.write(variable_lines)
    
    # add final lines
    freq_line = "file_data.freq = trick.DR_Always\n"
    input_file.write(freq_line)
    record_group_line = "trick.add_data_record_group(file_data, trick.DR_Buffer)\n"
    input_file.write(record_group_line)
    last_line = "file_data.thisown = 0\n"
    input_file.write(last_line)

    
# Close the file
input_file.close()

""" Recompile """
if args.rc:
    os.system(CLEANUP_COMMAND)
os.system(COMPILE_COMMAND)


"""***********************************************
RUN THE SIMULATION
***********************************************"""
""" Get executable name for differenct machines """
for file in glob.glob("*.exe"):
    if FILE_ID in file:
        EXECUTABLE = file
    else:
        print("No executable found, where am I???")
        exit()

# Get the simulation run command i.e. ./executable path/to/input/file
RUN_COMMAND = "./{0} {1}/automate_input.py".format(EXECUTABLE, DEFAULT_DATA_FOLDER)

os.system(RUN_COMMAND)

# if no data processing
if not (varlist or vector_vars):
    exit()

"""***********************************************
PROCESS THE DATA
***********************************************"""

# Get data
csv_data = []
with open(RELATIVE_PATH_TO_DATA, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    for row in csvreader:
        csv_data.append(row)

# check which variables were actually collected
collected_variables = csv_data[0]
processing_variables = []

# make sure csv recorded time
time_name = "none"
if "time" in collected_variables[0]:
    processing_variables.append(collected_variables[0])
    time_name = collected_variables[0]

pattern = re.compile(r"\{(.*)\}")
varflag = 0
for variable in varlist:
    varflag = 0
    for collected in collected_variables:
        if variable in collected:
            varflag = 1
            processing_variables.append(variable)
    # check that you collect the data you want
    # want varflag to be 1
    if not varflag:
        print('Variable {} was not logged'.format(variable))



for variable in vector_vars:
    for i in range(variable[1]):
        vec_var = "{0}[{1}]".format(variable[0], i)
        varflag = 0
        for collected in collected_variables:
            if vec_var in collected:
                varflag = 1
                processing_variables.append(vec_var)
        # check that you collect the data you want
        # want varflag to be 1
        if not varflag:
            print('Variable {} was not logged'.format(vec_var))

# initialize all variables with an empty list
Data_dict = {}
pattern = re.compile(r"\{(.*)\}")

for variable in processing_variables:
    dict_key = variable
    Data_dict[dict_key] = []

# loop through all collected data and add to Data_dict 
for row in csv_data[1:]:
    for i, col in enumerate(row): 
        Data_dict[processing_variables[i]].append(float(col))

# Check that plot directory exists and attempt to create if not

plot_directory = os.path.join(os.getcwd(), DEFAULT_DATA_FOLDER, "plot_data")
if not os.path.isdir(plot_directory):
    print("File path {} doesn't exist, creating it ... \n".format(plot_directory))
    os.mkdir(plot_directory)
    if os.path.isdir(plot_directory):
        print("Path created successfully\n")
    elif not os.path.isdir(plot_directory):
        print("Path couldn't be created, exiting...")
        exit()

num_plots = 0
for variable in varlist:
    if not variable in Data_dict:
        print("Vector {} wasn't in data".format(variable))  
        continue

    num_plots += 1
    plt.figure(num_plots)
    plt.plot(Data_dict[time_name], Data_dict[variable])
    plt.xlabel(time_name)
    plt.ylabel(variable)
    plt.title("Simulation Data for {}".format(variable))
    plt.savefig(os.path.join(os.getcwd(), DEFAULT_DATA_FOLDER, "plot_data", variable.split('.')[-1]))
    
for variable in vector_vars:
    if not "{0}[{1}]".format(variable[0], "0") in Data_dict:
        print("Vector {} wasn't in data".format(variable[0]))  
        continue
    num_plots += 1
    plt.figure(num_plots)
    plt.xlabel(time_name)
    plt.ylabel(variable[0])
    plt.title("Simulation Data for {}".format(variable[0]))
    key = []
    for i in range(variable[1]):
        key.append(variable[0].split('.')[-1] + "[{}]".format(i))
    
    for i in range(variable[1]):
        plt.plot(Data_dict[time_name], Data_dict["{0}[{1}]".format(variable[0], str(i))])
    plt.legend(key)
    plt.savefig(os.path.join(os.getcwd(), DEFAULT_DATA_FOLDER, "plot_data", variable[0].split('.')[-1]))

if args.show_plots:
    plt.show()