//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// Licence as published by the Free Software Foundation; either
// version 2.1 of the Licence, or (at your option) any later version.
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public Licence for more details.
//
// You should have received a copy of the GNU Lesser General Public
// Licence along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA.
//
// Contact details:
// cliftm@ansto.gov.au
// 800 Blackburn Road, Clayton, Victoria 3168, Australia.

// Change log:
// 16/09/14 M.Clift First release

#include <stdio.h>
#include <math.h>
#include <float.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#if defined _WIN32 || _WIN64
#include <process.h>
#else
#include <unistd.h>
#endif /* _WIN32 */
#include <iostream>  //cout
#include <fstream>   //ifstream
#include <sstream> //format source keys, string stream
#include <iomanip> //format source keys
#include <algorithm> //std::remove_if

#include <epicsString.h>
#include <iocsh.h>
#include <epicsThread.h>
#include <epicsExit.h>
#include <errlog.h>
#include <initHooks.h>
#include <shareLib.h>
#include <drvAsynIPPort.h>
#include <drvAsynSerialPort.h>

#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>
#include <asynOctet.h>
#include <asynShellCommands.h>

#include "AMC100Controller.h"
#include <epicsExport.h>

static const char *driverName = "AMC100Controller";
static const char *driverVersion = "3-0-0";

//Block read functions during Iocinit
//Prevent normal behaviour of output records getting/reading initial value at iocInit from driver
//Instead output records will write their db default or autosave value just after iocInit
//This change in behaviour is anticipated by the asyn record device layer and causes no error mesgs
static bool dbInitialized = false;

//Static count of AMC100 controllers.  Used to derive communications port name L(controller num)
static int controller_num = 0;

//Signal handler setup for SIGTERM, and SIGINT
static bool signalHandlerSetup = false;

//Convenience functions
#ifndef MAX
#define MAX(a,b) ((a)>(b)? (a): (b))
#endif
#ifndef MIN
#define MIN(a,b) ((a)<(b)? (a): (b))
#endif

//C++ "To String with Precision" static function template
template <typename T>
string tsp(const T a_value, const int n)
{
    ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

//Signal handler for SIGTERM, and SIGINT
static void signalHandler(int sig)
{
   switch (sig) {
      case SIGINT:
         epicsExit (128 + sig);
         break;
      case SIGTERM:
         epicsExit (128 + sig);
         break;
   }
}

//EPICS shutdown handler
static void shutdownCallback(void *pPvt)
{
  AMC100Controller *pC_ = (AMC100Controller *)pPvt;
  //IOC shutdown is in progress
  pC_->shutdownController();
}

//EPICS iocInit status
extern "C" void myHookFunction(initHookState state)
{
  //Update dbInitialized status for all AMC100Controller instances
  if (state >= initHookAfterInitDatabase)
	dbInitialized = true;
}

//Connection status
void connectCallback(asynUser *pasynUser, asynException exception)
{
   AMC100Controller* pC_ = (AMC100Controller*)pasynUser->userData;
   int connected;           //Asyn connected flag
   string mesg;              //Controller mesg

   //Update connected status
   if (exception == asynExceptionConnect)
       {
       pasynManager->isConnected(pasynUser, &connected);
       //Check that device will actually respond when asyn connected = 1
       if (connected)
          epicsEventSignal(pC_->connectEvent_);  //AMC100Connector will now check for response to query
       else
          {
          //Inform user of disconnect only if AMC100Controller connected_ is true
          if (pC_->connected_)
             {
             //Update connection status pv
             pC_->setIntegerParam(pC_->AMC100CommunicationError_, 1);
             //If asyn connected = 0 device wont respond so go ahead and set AMC100Controller connected_ false
             pC_->connected_ = false;
             //Give disconnect message
             pC_->lock();
             mesg = "Disconnected from " + pC_->model_ + " at " + pC_->address_;
             pC_->setCtrlError(mesg);
             pC_->unlock();
             }
          }
       }
}

/** Creates a new AMC100Controller object.
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] address      	 The name or address to provide to AMC100 communication library 
  * \param[in] updatePeriod  	 The time between polls when any axis is moving
                                 If (updatePeriod < 0), polled/synchronous at abs(updatePeriod) is done regardless of bus type 
  */
AMC100Controller::AMC100Controller(const char *portName, const char *address, double updatePeriod)
  :  asynMotorController(portName, (int)(MAX_ADDRESS), (int)NUM_AMC100_PARAMS,
                         (int)(asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynOctetMask | asynDrvUserMask), 
                         (int)(asynInt32Mask | asynFloat64Mask | asynUInt32DigitalMask | asynOctetMask),
                         (int)(ASYN_CANBLOCK | ASYN_MULTIDEVICE),
                         (int)1, // autoconnect
                         (int)0, (int)0),  // Default priority and stack size
  numAxes_(0)
{
  string mesg;              //Controller mesg

  // Create controller-specific parameters
  createParam(AMC100DriverString, asynParamOctet, &AMC100Driver_);
  createParam(AMC100AddressString, asynParamOctet, &AMC100Address_);
  createParam(AMC100ModelString, asynParamOctet, &AMC100Model_);
  createParam(AMC100CtrlErrorString, asynParamOctet, &AMC100CtrlError_);
  createParam(AMC100AxisString, asynParamInt32, &AMC100Axis_);

  createParam(AMC100EthAddrString, asynParamOctet, &AMC100EthAddr_);
  createParam(AMC100SerialNumString, asynParamOctet, &AMC100SerialNum_);

//Add new parameters here

  createParam(AMC100CommunicationErrorString, asynParamInt32, &AMC100CommunicationError_);

  //Not connected to controller yet
  connected_ = false;
  //Store address
  address_ = address;
  //Default model
  model_ = "AMC100";
  //IOC is not shutting down yet
  shuttingDown_ = false;
  //We have not recieved a timeout yet
  consecutive_timeouts_ = 0;
  //Store absolute period in ms between data records
  updatePeriod_ = fabs(updatePeriod);
  //Limit maximum update period
  if (updatePeriod_ > MAX_UPDATE_PERIOD) {
     mesg = "Capping UpdatePeriod to " + tsp(MAX_UPDATE_PERIOD) + "ms maximum";
     setCtrlError(mesg);
     updatePeriod_ = MAX_UPDATE_PERIOD;
  }

  //Deferred moves off at start-up
  movesDeferred_ = false;
  //Store the controller number for later use
  controller_number_ = controller_num;

  //Set defaults in Paramlist before connect
  setParamDefaults();

  //Register for iocInit state updates, so we can keep track of iocInit status
  initHookRegister(myHookFunction);

  //Create the event that wakes up the AMC100Connector thread
  connectEvent_ = epicsEventMustCreate(epicsEventEmpty);

  //Create connector thread that manages connection status flags
  connector_ = new AMC100Connector(this);

  //Thread to acquire datarecord for a single AMC100Controller
  //We write our own because communications with controller is rather unique
  poller_ = new AMC100Poller(this);

  //Put, and wait until poller is in sleep mode
  //Also stop async records from controller
  poller_->sleepPoller();

  //Establish the initial connection to controller
  connect();

  //Static count of controllers.  Used to derive communications port names
  controller_num++;

  //Setup signal handler to catch SIGTERM, and SIGINT
  if (!signalHandlerSetup) {
     signal(SIGTERM, signalHandler);
     signal(SIGINT, signalHandler);
     signalHandlerSetup = true;
  }
}

//Called by AMC100Controller at start up once only
//Asyn does connection management for us, and gives callbacks at status change
void AMC100Controller::connect(void)
{
  asynInterface *pasynInterface;	//To retrieve required asyn interfaces
  string address_string;		//Temporary address string used to setup communications
  int sync_connected;			//Is the synchronous communication socket connected according to asyn
  string address = address_;		//Convert address into string for easy inspection

  //Set default timeout at connect
  timeout_ = DEFAULT_TIMEOUT;

  //Construct the asyn port name that will be used for synchronous communication
  sprintf(syncPort_, "AMC100SYNC%d", controller_number_);
  //Open Synchronous ethernet connection
  //If no port number is specified in the provided address, append Telnet port and TCP directive
  //Otherwise just append the TCP directive
  if (address.find(":") == string::npos) {
     address_string = address + ":23 TCP";
  } else {
     address_string = address + " TCP";
  }

  //Connect to the device, we don't want end of string processing
  drvAsynIPPortConfigure(syncPort_, address_string.c_str(), epicsThreadPriorityMedium, 0, 1);
  //Connect to synchronous communications port created above and return pasynUser for sync communication
  pasynOctetSyncIO->connect(syncPort_, 0, &pasynUserSyncAMC100_, NULL);
  //Store AMC100Controller instance in asynUser for later access
  pasynUserSyncAMC100_->userData = this;
  //Add asyn exception callback to capture asyn connected change of state
  pasynManager->exceptionCallbackAdd(pasynUserSyncAMC100_, connectCallback);
  //Retrieve asyn common interface for forced disconnect on synchronous communications
  pasynInterface = pasynManager->findInterface(pasynUserSyncAMC100_, asynCommonType, 1);
  //Store the asyn common interface for sync in AMC100Controller instance for use during forced disconnect
  pasynCommon_ = (asynCommon *)pasynInterface->pinterface;
  pcommonPvt_ = pasynInterface->drvPvt;
  //Retrieve sync octet interface for reading the data record
  pasynInterface = pasynManager->findInterface(pasynUserSyncAMC100_, asynOctetType, 1);
  //Store the asyn octet interface for sync connection in AMC100Controller
  pSyncOctet_ = (asynOctet *)pasynInterface->pinterface;
  pSyncOctetPvt_ = pasynInterface->drvPvt;
  //Retrieve synchronous connection status from asyn
  pasynManager->isConnected(pasynUserSyncAMC100_, &sync_connected);
  //For tracing/debugging sync communications
  //asynSetTraceMask(syncPort_,-1,0xFF);
  //asynSetTraceIOMask(syncPort_,-1,0xFF);

  //Set an EPICS exit handler that will shut down polling before exit
  //Must be done after drvAsyn port configure so our exit handler is called before asyn one at epics exit
  epicsAtExit(shutdownCallback, this);

  //AMC100Connector will now check for response to query and set connection status flag
  if (sync_connected)
     epicsEventSignal(connectEvent_);

  //Pause long enough for AMC100Connector to update connection status flags, and show connection message
  epicsThreadSleep(.5);
}

//Called when async poll fails
//Forces asyn to disconnect/reconnect
void AMC100Controller::disconnect(void)
{
   int connected;

   //Retrieve asyn connected status
   pasynManager->isConnected(pasynUserSyncAMC100_, &connected);
   //Disconnect asyn from device if asyn connected = 1
   if (connected)
      {
      //Inform asyn we are going to disconnect
      pasynManager->exceptionDisconnect(pasynUserSyncAMC100_);
      //Disconnect from the controller.  This will cause connectCallback
      pasynCommon_->disconnect(pcommonPvt_, pasynUserSyncAMC100_);
      }
}

void AMC100Controller::shutdownController()
{
   unsigned i;
   AMC100Axis *pAxis;

   //Set shutdown flag
   shuttingDown_ = true;

   //Destroy the poller thread
   if (poller_ != NULL)
      {
      delete poller_;
      poller_ = NULL;
      }

   //Destroy the connector thread
   if (connector_ != NULL)
      {
      delete connector_; 
      connector_ = NULL;
      }

   //Destroy events
   epicsEventDestroy(connectEvent_);
   //Sleep to preempt this thread, and give time
   //for profile, and array upload thread to exit
   epicsThreadSleep(.002);

   //Free any AMC100Axis
   for (i = 0; i < MAX_AMC100_AXES; i++)
      {
      if (i < MAX_AMC100_AXES)
         {
         pAxis = getAxis(i);
         if (pAxis)
            delete pAxis;
         }
      }

   //Burn parameters, and cleanup
   if (connected_) {
      //Asyn exit handler will disconnect sync connection from here
      //We just print message to tell user Asyn epicsAtExit callback is running (next) and will close connection
      cout << "Disconnecting from " << model_ << " at " << address_ << endl;
   }
}

AMC100Controller::~AMC100Controller()
{
}

void AMC100Controller::setParamDefaults(void)
{ 
  //Set defaults in Paramlist before connected
  //Set driver version
  setStringParam(AMC100Driver_, driverVersion);
  //Pass address string provided by AMC100CreateController to upper layers
  setStringParam(AMC100Address_, address_);
  //Set default model string
  setStringParam(AMC100Model_, model_);
  //Communication status
  setIntegerParam(AMC100CommunicationError_, 1);
  //Deferred moves off 
  setIntegerParam(motorDeferMoves_, 0);
  setStringParam(AMC100SerialNum_, "");
  setStringParam(AMC100EthAddr_, "");
  //Default controller error message to null string
  setStringParam(0, AMC100CtrlError_, "");
}

// extract the controller ethernet address from the output of the galil TH command
// return 00-00-00-00-00-00 if unable to parse input string
std::string AMC100Controller::extractEthAddr(const char* str)
{
	//Result string
	std::string mac;
	//Search string
	static const std::string eth("ETHERNET ADDRESS");
	//Result from TH command
	std::string th(str);
	//Find start and end of substring containing mac address
	size_t start = th.find(eth);
	start = start + eth.size() + 1;
	size_t end = start + 17;
	//copy portion of the string containing the MAC addr
	if ((start < 0) || (end >= th.size()) || (start >= end))
		mac = std::string("00-00-00-00-00-00");  //tdebug
	else
		mac = std::string(th.substr(start, end - start));
	//Trim final result to mac address length
	mac.resize(17);
	return mac;
}

//Anything that should be done once connection established
//Read controller details, stop all motors and threads
void AMC100Controller::connected(void)
{
  //static const char *functionName = "connected";
  AMC100Axis *pAxis;
  unsigned i;
  string mesg = "";              //Connected mesg

  //Flag connected as true
  connected_ = true;
  setIntegerParam(AMC100CommunicationError_, 0);

  //Wake the poller
  poller_->wakePoller();

  //Give connect message
  mesg = "Connected to " + model_ + " at " + address_;
  setCtrlError(mesg);
 
  //No timeout errors
  consecutive_timeouts_ = 0;

  callParamCallbacks();
}

//Acquire data record from controller
void AMC100Controller::acquireData(void)
{
  //const char *functionName="acquireData";
  int status = 0;               //Status   
  //epicsTimeStamp endt_;	//Used for debugging, and tracking overall performance
  //epicsTimeStamp startt_;	//Used for debugging, and tracking overall performance
  //double time_taken;		//Used for debugging, and tracking overall performance

  if (connected_) {
     //Get acquisition start time
     //epicsTimeGetCurrent(&startt_);

     //Get axes positions
     strcpy(cmd_, getAllPositions.dump().c_str());
     status = sync_writeReadController();
     //Check response
     if (!status) {
        try {
           positions_ = json::parse(resp_);
           if (positions_["error"] != nullptr || positions_["result"][0] != 0) {
              status = asynError;
           }
        }
        catch (...) {
        }
     }

     //Get axes move status
     strcpy(cmd_, getAllMoveStatus.dump().c_str());
     status |= sync_writeReadController();
     //Check response
     if (!status) {
        try {
           moveStatus_ = json::parse(resp_);
           if (moveStatus_["error"] != nullptr || moveStatus_["result"][0] != 0) {
              status = asynError;
           }
        }
        catch (...) {
        }
     }

     //Store read status for later use
     recstatus_ = (asynStatus)status;

     //Get acquisition end time
     //epicsTimeGetCurrent(&endt_);
     //Calculate acquistion time
     //time_taken = epicsTimeDiffInSeconds(&endt_, &startt_);
     //if (time_taken > 0.01)
     //printf("%s GalilController::acquire %2.6lfs stat %d\n", model_, time_taken, recstatus_);
  } //connected_

  //Track timeouts
  if (!status) {
     consecutive_timeouts_ = 0;
  }
  else {
     consecutive_timeouts_++;
  }

  //Force disconnect if any errors
  if (consecutive_timeouts_ > ALLOWED_TIMEOUTS)
     disconnect();
}

//Acquire data from controller
//Called by GalilPoller::run
asynStatus AMC100Controller::poller(void)
{
	//Acquire the data, store in AMC100Controller for now
	acquireData();

	//Return value is not monitored by asynMotorController
	return asynSuccess;
}

/** Reports on status of the driver
  * \param[in] fp The file pointer on which report information will be written
  * \param[in] level The level of report detail desired
  *
  * If details > 0 then information is printed about each axis.
  * After printing controller-specific information calls asynMotorController::report()
  */
void AMC100Controller::report(FILE *fp, int level)
{
  //int axis;
  //AMC100Axis *pAxis;

  fprintf(fp, "AMC100 motor driver %s, numAxes=%d, moving poll period=%f, idle poll period=%f\n", 
    this->portName, numAxes_, movingPollPeriod_, idlePollPeriod_);
  /*
  if (level > 0) {
    for (axis=0; axis<numAxes_; axis++) {
      pAxis = getAxis(axis);
      fprintf(fp, "  axis %d\n"
              "    pulsesPerUnit_ = %f\n"
              "    encoder position=%f\n"
              "    theory position=%f\n"
              "    limits=0x%x\n"
              "    flags=0x%x\n", 
              pAxis->axisNo_, pAxis->pulsesPerUnit_, 
              pAxis->encoderPosition_, pAxis->theoryPosition_,
              pAxis->currentLimits_, pAxis->currentFlags_);
    }
  }*/

  // Call the base class method
  asynMotorController::report(fp, level);
}

/** Returns a pointer to an AMC100MotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] pasynUser asynUser structure that encodes the axis index number. */
AMC100Axis* AMC100Controller::getAxis(asynUser *pasynUser)
{
  //For real motors
  return static_cast<AMC100Axis*>(asynMotorController::getAxis(pasynUser));
}

/** Returns a pointer to an AMC100MotorAxis object.
  * Returns NULL if the axis number encoded in pasynUser is invalid.
  * \param[in] axisNo Axis index number. */
AMC100Axis* AMC100Controller::getAxis(int axisNo)
{
  //For real motors
  return static_cast<AMC100Axis*>(asynMotorController::getAxis(axisNo));
}

/**
 * Process deferred moves for a controller
 * @return motor driver status code.
 */
asynStatus AMC100Controller::setDeferredMoves(bool deferMoves)
{
  //const char *functionName = "AMC100Controller::setDeferredMoves";

  //If we are not ending deferred moves then return
  if (deferMoves || !movesDeferred_)
     {
     movesDeferred_ = true;
     return asynSuccess;
     }

  //We are ending deferred moves.  So process them
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
              "Processing deferred moves on AMC100: %s\n", this->portName);

  //Ending deferred moves
  //Clear movesDeferred_ flag
  movesDeferred_ = false;

  return asynSuccess;
}

/** Attempts to read value from controller, returns last value set if fails.  
  ** Called by GaLilController::readInt32()
  * \param[in] cmd to send to controller
  * \param[out] value Address of the value to read. 
  * \param[in] axisNo is asyn Param list number 0 - 7.  Controller wide values use list 0 */
asynStatus AMC100Controller::get_integer(int function, epicsInt32 *value, int axisNo = 0)
{
  asynStatus status;				 //Communication status.
	
  if ((status = sync_writeReadController()) == asynSuccess)
     *value = (epicsInt32)atoi(resp_);
  else    //Comms error, return last ParamList value set using setIntegerParam
     getIntegerParam(axisNo, function, value);
  return status;
}

/** Called when asyn clients call pasynInt32->read().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[out] value Address of the value to read. */
asynStatus AMC100Controller::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
   asynStatus status;				//Status
   AMC100Axis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
   
   //Just return if shutting down
   if (shuttingDown_)
      return asynSuccess;

   //If provided addr does not return an AMC100Axis instance, then return asynError
   if (!pAxis) return asynError;

   //We dont retrieve values for records at iocInit.  
   //For output records autosave, or db defaults are pushed to hardware instead
   if (!dbInitialized) return asynError;
    
   status = asynPortDriver::readInt32(pasynUser, value);

   //Always return success. Dont need more error mesgs
   return asynSuccess;
}

/** Attempts to read value from controller, returns last good or default if fails.  
  ** Called by AMC100Controller::readFloat64()
  * \param[in] cmd to send to controller
  * \param[in] asyn Param function number
  * \param[out] value Address of the value to read. 
  * \param[in] axisNo is asyn Param list number 0 - 7.  Controller wide values use list 0 */
asynStatus AMC100Controller::get_double(int function, epicsFloat64 *value, int axisNo = 0)
{
  asynStatus status;				 //Communication status.

  if ((status = sync_writeReadController()) == asynSuccess)
     *value = (epicsFloat64)atof(resp_);
  else    //Comms error, return last ParamList value set using setDoubleParam
     getDoubleParam(axisNo, function, value);
  return status;
}

/** Called when asyn clients call pasynFloat64->read().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Address of the value to read. */
asynStatus AMC100Controller::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
  int function = pasynUser->reason;		//function requested
  AMC100Axis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
  asynStatus status;				//Asyn status
  int addr;					//Address

  //Just return if shutting down
  if (shuttingDown_)
     return asynSuccess;

  //Retrieve address.  Used for analog IO
  status = getAddress(pasynUser, &addr); 
  if (status != asynSuccess) return(status);

  //We dont retrieve values for records at iocInit.  
  //For output records autosave, or db defaults are pushed to hardware instead
  if (!dbInitialized) return asynError;

  asynPortDriver::readFloat64(pasynUser, value);

  //Always return success. Dont need more error mesgs
  return asynSuccess;	
}

/** Called when asyn clients call pasynInt32->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * If the function is motorSetClosedLoop_ then it turns the drive power on or off.
  * For all other functions it calls asynMotorController::writeInt32.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value     Value to write. */
asynStatus AMC100Controller::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int function = pasynUser->reason;		//Function requested
  int addr = 0;				        //Address requested
  int status;					//Used to work out communication_error_ status.  asynSuccess always returned
  AMC100Axis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
  string mesg = "";				//Controller mesg

  //Just return if shutting down
  if (shuttingDown_)
     return asynSuccess;

  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess) return((asynStatus)status);

  //Check axis instance
  if (addr < MAX_AMC100_AXES)
     if (!pAxis) return asynError;

  /* Set the parameter and readback in the parameter library.  This may be overwritten when we read back the
   * status at the end, but that's OK */
  status = setIntegerParam(addr, function, value);
  
  /* Call base class method */
  status = asynMotorController::writeInt32(pasynUser, value);

  //Always return success. Dont need more error mesgs
  return (asynStatus)asynSuccess;
}

/** Called when asyn clients call pasynFloat64->write().
  * Extracts the function and axis number from pasynUser.
  * Sets the value in the parameter library.
  * Calls any registered callbacks for this pasynUser->reason and address.  
  * For all other functions it calls asynMotorController::writeFloat64.
  * \param[in] pasynUser asynUser structure that encodes the reason and address.
  * \param[in] value Value to write. */
asynStatus AMC100Controller::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
  int function = pasynUser->reason;		//Function requested
  asynStatus status;				//Used to work out communication_error_ status.  asynSuccess always returned
  AMC100Axis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
  int addr=0;					//Address requested

  //Just return if shutting down
  if (shuttingDown_)
     return asynSuccess;

  //Retrieve address.  Used for analog IO
  status = getAddress(pasynUser, &addr);
  if (status != asynSuccess) return(status);

  /* Set the parameter and readback in the parameter library. */
  status = setDoubleParam(addr, function, value);
     
  /* Call base class method */
  status = asynMotorController::writeFloat64(pasynUser, value);

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

//  Overriden from asynMotorController as we need to support CSAxis profiles
/** Called when asyn clients call pasynFloat64Array->write().
  * \param[in] pasynUser pasynUser structure that encodes the reason and address.
  * \param[in] value Pointer to the array to write.
  * \param[in] nElements Number of elements to write. */
asynStatus AMC100Controller::writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value,
                                                  size_t nElements)
{
  int function = pasynUser->reason;		//Reason this method was called
  AMC100Axis *pAxis = getAxis(pasynUser);	//Retrieve the axis instance
  asynStatus status;				//Status
  int addr;					//address

  //Just return if shutting down
  if (shuttingDown_)
     return asynSuccess;

  //Retrieve address of caller
  status = getAddress(pasynUser, &addr); 
  if (status != asynSuccess) return(status);

  //Check axis instance the easy way since no RIO commands in writeFloat64Array
  if (addr < MAX_AMC100_AXES)
     {
     if (!pAxis) return asynError;
     }
  
  if (nElements > maxProfilePoints_) nElements = maxProfilePoints_;
   
  if (function == profileTimeArray_) {
    memcpy(profileTimes_, value, nElements*sizeof(double));
  } 
  else if (function == profilePositions_) {
    if (addr < MAX_AMC100_AXES)
       pAxis->defineProfile(value, nElements);
  }
  else  //Call parent class method 
     asynMotorController::writeFloat64Array(pasynUser, value, nElements);
  return asynSuccess;
}

//Below function supplied for Cygwin, MingGw
bool AMC100Controller::my_isascii(int c)
{
   if (c == 10 || c == 13 || (c >= 48 && c <= 57) || (c >= 65 && c <= 90) ||
       (c >= 97 && c <= 122) || c == 32 || c == 46)
      return true;
   else
      return false;
}

/** Writes a string to the AMC100Controller controller and reads the response using synchronous communications
  * Calls sync_writeReadController() with default locations of the input and output strings
  * and AMC100Controller timeout. */ 
asynStatus AMC100Controller::sync_writeReadController(bool testQuery, bool logCommand)
{
  const char *functionName="sync_writeReadController";
  size_t nread;
  int status;
  size_t len;
  static const char* debug_file_name = macEnvExpand("$(AMC100_DEBUG_FILE=)");
  static FILE* debug_file = ( (debug_file_name != NULL && strlen(debug_file_name) > 0) ? fopen(debug_file_name, "at") : NULL);

  //Simply return asynSuccess if not connected
  //Asyn module corrupts ram if we try write/read with no connection
  if (!connected_ && !testQuery) {
     strcpy(resp_, "");
     return asynSuccess;
  }

  /*asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, 
          "%s: controller=\"%s\" command=\"%s\"\n", functionName, address_, cmd_);*/

  //Append carriage return to provided cmd
  len = strlen(cmd_);
  if (len < MAX_AMC100_STRING_SIZE - 2) {
     cmd_[len] = '\r';
     cmd_[len+1] = '\0';
  }
  else //Command too long
     return asynError;

  //Write command, and retrieve response
  status = sync_writeReadController(cmd_, resp_, MAX_AMC100_STRING_SIZE, &nread, timeout_);

  //Remove trailing \r we added earlier
  cmd_[len] = '\0';

  //Remove any unwanted characters
  string resp = resp_;
  //Remove unwanted trailing characters
  resp.erase(resp.find_last_not_of(" \n\r\t:")+1);
  strcpy(resp_, resp.c_str());

  //Reset consecutive timeout counter if no read error
  if (!status) {
     consecutive_timeouts_ = 0;
  }

  //Force disconnect if any errors
  if (consecutive_timeouts_ > ALLOWED_TIMEOUTS)
     disconnect();

  //Debugging
  /*asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER, 
         "%s: controller=\"%s\" command=\"%s\", response=\"%s\", status=%s\n", 
	      functionName, address_, cmd_, resp_, (status == asynSuccess ? "OK" : "ERROR"));*/

  if (debug_file != NULL && logCommand) {
     time_t now;
     //Use line buffering, then flush
     setvbuf(debug_file, NULL, _IOLBF, BUFSIZ);
     time(&now);
     char time_buffer[64];
     strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", localtime(&now));
     fprintf(debug_file, "%s (%d) %s: controller=\"%s\" command=\"%s\", response=\"%s\", status=%s\n", 
	      time_buffer, getpid(), functionName, address_.c_str(), cmd_, resp_, (status == asynSuccess ? "OK" : "ERROR"));
  }

  return (asynStatus)status;
}

/** Writes a string to the controller and reads a response.
  * \param[in] output Pointer to the output string.
  * \param[out] input Pointer to the input string location.
  * \param[in] maxChars Size of the input buffer.
  * \param[out] nread Number of characters read.
  * \param[out] timeout Timeout before returning an error.*/
asynStatus AMC100Controller::sync_writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout)
{
  unsigned i = 0;	//Number of raw bytes received, general counting
  size_t nwrite;	//Bytes written
  asynStatus status = asynSuccess;//Asyn status
  int eomReason;	//End of message reason
  char buf[MAX_AMC100_STRING_SIZE] = "";	//Receive buffer
  int target_terminators = 1;   //Expected terminator characters in response
  int found_terminators = 0;	//Terminator characters found so far
  bool done = false;		//Read complete?

  //Null user supplied input buffer
  strcpy(input, "");
  //Set timeout for Sync connection
  pasynUserSyncAMC100_->timeout = timeout_;
  //Write the command
  status = pSyncOctet_->write(pSyncOctetPvt_, pasynUserSyncAMC100_, output, strlen(output), &nwrite);
  //If write ok
  if (!status) {
     while (!done) {
        //Read any response
        status = pSyncOctet_->read(pSyncOctetPvt_, pasynUserSyncAMC100_, buf, MAX_AMC100_STRING_SIZE, nread, &eomReason);
        //If read successful, search for terminator characters
        if (!status && *nread > 0) {
           //Search for terminating characters
           for (i = 0; i < *nread; i++) {
              if (buf[i] == 13 && buf[i+1] == 10) {
                 found_terminators++;
              }
           }
           //If received all expected terminators, read is complete
           if (found_terminators == target_terminators) {
              //Don't attempt any more reads
              done = true;
              //stop searching this read, and return the resp
              break;
           }
        }
        else { //Stop read if any asyn error
           if (status == 1) {
              consecutive_timeouts_++;
           }
           return asynError;
        }
     }//while (!done)
     //Copy response into user supplied buffer
     strcpy(input, buf);
  }//write ok
  return status;
}

/** Record an error message, and also display to ioc window
  * \param[in] mesg      	 Error message
  */
void AMC100Controller::setCtrlError(string mesg)
{
   if (mesg.size() > 0)
      std::cout << mesg << std::endl;
   setStringParam(0, AMC100CtrlError_, mesg.c_str());
   callParamCallbacks();
}

//IocShell functions

/** Creates a new AMC100Controller object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] address      	 The name or address to provide to AMC100 communication library
  * \param[in] updatePeriod	 The time in ms between datarecords.  Async if controller + bus supports it, otherwise is polled/synchronous.
  */
extern "C" int AMC100CreateController(const char *portName, const char *address, int updatePeriod)
{
  new AMC100Controller(portName, address, updatePeriod);
  return(asynSuccess);
}

/** Creates a new AMC100Axis object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that has already been created for this driver
  */
extern "C" asynStatus AMC100CreateAxes(const char *portName)        	/*specify which controller by port name */
{
  int axis;
  AMC100Controller *pC;
  static const char *functionName = "AMC100CreateAxes";

  //Retrieve the asynPort specified
  pC = (AMC100Controller*) findAsynPortDriver(portName);

  if (!pC) {
    printf("%s:%s: Error port %s not found\n", driverName, functionName, portName);
    return asynError;
  }
  
  pC->lock();

  for (axis = 0; axis < MAX_AMC100_AXES; axis++) {
     new AMC100Axis(pC, axis);
  }

  pC->unlock();
  return asynSuccess;
}

//Register the above IocShell functions
//AMC100CreateController iocsh function
static const iocshArg AMC100CreateControllerArg0 = {"Controller Port name", iocshArgString};
static const iocshArg AMC100CreateControllerArg1 = {"IP address", iocshArgString};
static const iocshArg AMC100CreateControllerArg2 = {"update period (ms)", iocshArgInt};
static const iocshArg * const AMC100CreateControllerArgs[] = {&AMC100CreateControllerArg0,
                                                             &AMC100CreateControllerArg1,
                                                             &AMC100CreateControllerArg2};
                                                             
static const iocshFuncDef AMC100CreateControllerDef = {"AMC100CreateController", 3, AMC100CreateControllerArgs};

static void AMC100CreateContollerCallFunc(const iocshArgBuf *args)
{
  AMC100CreateController(args[0].sval, args[1].sval, args[2].ival);
}

//AMC100CreateAxis iocsh function
static const iocshArg AMC100CreateAxesArg0 = {"Controller Port name", iocshArgString};

static const iocshArg * const AMC100CreateAxesArgs[] =  {&AMC100CreateAxesArg0};

static const iocshFuncDef AMC100CreateAxesDef = {"AMC100CreateAxes", 1, AMC100CreateAxesArgs};

static void AMC100CreateAxesCallFunc(const iocshArgBuf *args)
{
  AMC100CreateAxes(args[0].sval);
}

//Construct AMC100Controller iocsh function register
static void AMC100SupportRegister(void)
{
  iocshRegister(&AMC100CreateControllerDef, AMC100CreateContollerCallFunc);
  iocshRegister(&AMC100CreateAxesDef, AMC100CreateAxesCallFunc);
}

//Finally do the registration
extern "C" {
epicsExportRegistrar(AMC100SupportRegister);
}

