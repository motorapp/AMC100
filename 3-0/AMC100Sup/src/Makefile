TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#==================================================
# Build an IOC support library

LIBRARY_IOC += AMC100Support

# motorRecord.h will be created from motorRecord.dbd
# install devMotorSoft.dbd into <top>/dbd
DBD += AMC100Support.dbd

#Require C++ 2011 standard compatibility
USR_CXXFLAGS_Linux += -std=c++11

# For sCalcPostfix.h
USR_INCLUDES += -I$(CALC)/calcApp/src

# The following are compiled and added to the Support library
AMC100Support_SRCS += AMC100Controller.cpp AMC100Axis.cpp AMC100Connector.cpp AMC100Poller.cpp

AMC100Support_LIBS += asyn motor calc sscan autosave busy
AMC100Support_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
