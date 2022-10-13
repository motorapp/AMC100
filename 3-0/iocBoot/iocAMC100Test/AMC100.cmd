## uncomment to see every command sent to AMC100
#epicsEnvSet("AMC100_DEBUG_FILE", "amc100_debug.txt")

#Load motor records for real motors
#Motor record version 6-10 and up
dbLoadTemplate("$(TOP)/AMC100TestApp/Db/amc100_motors.substitutions")

#Load DMC controller features (eg. message consoles)
dbLoadTemplate("$(TOP)/AMC100TestApp/Db/amc100_dmc_ctrl.substitutions")

# AMC100CreateController command parameters are:
#
# 1. Const char *portName 	- The name of the asyn port that will be created for this controller
# 2. Const char *address 	- The address of the controller
# 3. double updatePeriod	- The time in ms between poll
#                       	- Recommend 50ms or less for ethernet

# Create a AMC100 controller
AMC100CreateController("AMC100", "10.108.2.52:9090", 50)

# AMC100CreateAxes command parameters are:
#
# 1. char *portName Asyn port for controller

# Create the axes
AMC100CreateAxes("AMC100")

