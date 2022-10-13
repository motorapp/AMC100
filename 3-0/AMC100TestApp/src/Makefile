TOP=../..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

#=============================
# build an ioc application

PROD_IOC = AMC100Test

# <name>.dbd will be created from <name>Include.dbd
DBD += AMC100Test.dbd
AMC100Test_DBD += base.dbd
AMC100Test_DBD += asyn.dbd
AMC100Test_DBD += drvAsynSerialPort.dbd
AMC100Test_DBD += drvAsynIPPort.dbd
AMC100Test_DBD += motorSupport.dbd
AMC100Test_DBD += calcSupport.dbd
AMC100Test_DBD += sscanSupport.dbd
AMC100Test_DBD += asSupport.dbd
AMC100Test_DBD += busySupport.dbd
AMC100Test_DBD += AMC100Support.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
AMC100Test_SRCS += AMC100Test_registerRecordDeviceDriver.cpp
AMC100Test_SRCS_DEFAULT += AMC100TestMain.cpp

#add a definition for each support application used by this application
AMC100Test_LIBS += AMC100Support
AMC100Test_LIBS += motor busy asyn calc sscan autosave
ifdef SNCSEQ
AMC100Test_LIBS += seq pv
endif
AMC100Test_LIBS += $(EPICS_BASE_IOC_LIBS)

#===========================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
