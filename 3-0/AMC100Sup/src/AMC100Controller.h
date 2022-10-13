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
// Mark Clift
// padmoz@tpg.com.au

#ifndef AMC100Controller_H
#define AMC100Controller_H

#if defined _WIN32 || _WIN64
#define rint(x) floor((x)+0.5)
#define lrint(x) floor((x)+0.5)
#define finite(x) _finite(x)
#endif /* _WIN32/_WIN64 */

#define BEGIN_TIMEOUT 0.5
#define AASCII 65
#define IASCII 73
#define QASCII 81
#define ZEROASCII 48
//Communication timeout
#define DEFAULT_TIMEOUT 2
//Number of communication retries
#define ALLOWED_TIMEOUTS 2
#define MAX_UPDATE_PERIOD 200
#define MAX_AMC100_STRING_SIZE 768
#define MAX_AMC100_AXES 3
//Number of parameter tables created
//Meaning records can have address values 0-63
#define MAX_ADDRESS 64

//Time base
#define DEFAULT_TIME 1000.0

//Nearest integer
#define NINT(f) (long)((f)>0 ? (f)+0.5 : (f)-0.5)

#include "macLib.h"
#include "AMC100Axis.h"
#include "AMC100Connector.h"
#include "AMC100Poller.h"
#include "epicsMessageQueue.h"

#include <vector>

//Json support
#include <nlohmann/json.hpp>

//Namespaces
using namespace std; //cout ostringstream vector string
using ordered_json = nlohmann::ordered_json;
using json = nlohmann::json;

//Often used commands
static ordered_json getAllPositions = {
                 {"jsonrpc", "2.0"},
                 {"method", "com.attocube.amc.control.getPositionsAndVoltages"},
                 {"params", {0}},
                 {"id", 0},
                 {"api", 2}};

static ordered_json getAllMoveStatus = {
                 {"jsonrpc", "2.0"},
                 {"method", "com.attocube.amc.control.getStatusMovingAllAxes"},
                 {"params", {0}},
                 {"id", 0},
                 {"api", 2}};

// drvInfo strings for extra parameters that the Galil controller supports
#define AMC100DriverString		"CONTROLLER_DRIVER"
#define AMC100AddressString		"CONTROLLER_ADDRESS"
#define AMC100ModelString		"CONTROLLER_MODEL"
#define AMC100CtrlErrorString		"CONTROLLER_ERROR"
#define AMC100CommunicationErrorString	"CONTROLLER_COMMERR"

#define AMC100AxisString		"MOTOR_AXIS"

#define AMC100EthAddrString	  	"CONTROLLER_ETHADDR"
#define AMC100SerialNumString	  	"CONTROLLER_SERIALNUM"

//C++ "To String with Precision" static function template
template <typename T>
string tsp(const T a_value, const int n = 6);

class AMC100Controller : public asynMotorController {
public:
  //These variables need to be accessible from static callbacks
  epicsEventId connectEvent_;		//Connection event
  int connected_;			//Is the synchronous communication socket connected according to asyn.  Async UDP is connectionless

  string address_;			//address string
  string model_;			//model string

  //Class constructor
  AMC100Controller(const char *portName, const char *address, double updatePeriod);

  asynStatus async_writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout);
  asynStatus async_writeReadController(void);

  asynStatus sync_writeReadController(const char *output, char *input, size_t maxChars, size_t *nread, double timeout);
  asynStatus sync_writeReadController(bool testQuery = false, bool logCommand = true);

  bool my_isascii(int c);
  
  /* These are the methods that we override from asynMotorController */
  asynStatus setDeferredMoves(bool deferMoves);
  asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  asynStatus writeFloat64Array(asynUser *pasynUser, epicsFloat64 *value, size_t nElements);
  asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
  asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
  void report(FILE *fp, int level);

  //Real motors
  AMC100Axis* getAxis(asynUser *pasynUser);
  AMC100Axis* getAxis(int axisNo);

  /* These are the methods that are new to this class */
  asynStatus poller(void);
  void connect(void);
  void disconnect(void);
  void connected(void);
  void acquireData(void);
  void getStatus(void);
  void setParamDefaults(void);
  asynStatus get_integer(int function, epicsInt32 *value, int axisNo);
  asynStatus get_double(int function, epicsFloat64 *value, int axisNo);
  bool axesEventMonitor(const char *axes, unsigned requestedEvent = 0);

  static std::string extractEthAddr(const char* str);
  void setCtrlError(string mesg);

  void shutdownController();
  virtual ~AMC100Controller();

protected:
  #define FIRST_AMC100_PARAM AMC100Driver_
  int AMC100Driver_;
  int AMC100Address_;
  int AMC100Model_;
  int AMC100CtrlError_;
  int AMC100Axis_;
  int AMC100EthAddr_;
  int AMC100SerialNum_;
//Add new parameters here

  int AMC100CommunicationError_;
  #define LAST_AMC100_PARAM AMC100CommunicationError_

private:

  char cmd_[MAX_AMC100_STRING_SIZE];	//holds the assembled cmd string
  char resp_[MAX_AMC100_STRING_SIZE];	//Response from controller
  ordered_json moveStatus_;
  ordered_json positions_;

  AMC100Poller *poller_;		//AMC100Poller to acquire a datarecord
  AMC100Connector *connector_;		//AMC100Connector to manage connection status flags

  bool shuttingDown_;			//IOC exit in progress
			
  int consecutive_timeouts_;		//Used for connection management
  double updatePeriod_;			//Period between data records in ms

  bool movesDeferred_;			//Should moves be deferred for this controller

  asynStatus recstatus_;		//Status of last record acquisition
  unsigned numAxes_;			//Number of axes requested by developer

  int timeout_;				//Timeout for communications
  int controller_number_;		//The controller number as counted in GalilCreateController
  
  char syncPort_[MAX_AMC100_STRING_SIZE];	//The name of the asynPort created for synchronous communication with controller
  char syncHandle_;				//Handle on controller used for synchronous communication (ie. tcp or serial)
  asynUser *pasynUserSyncAMC100_;		//Asyn user for synchronous communication
  asynCommon *pasynCommon_;			//asynCommon interface for synchronous communication
  void *pcommonPvt_;				//asynCommon drvPvt for synchronous communication
  asynOctet *pSyncOctet_;			//Asyn octet interface for synchronous communication
  void *pSyncOctetPvt_;				//Asyn octet private data for synchronous communication

  friend class AMC100Axis;
  friend class AMC100Poller;
  friend class AMC100Connector;
  friend void connectCallback(asynUser *pasynUser, asynException exception);
};
#define NUM_AMC100_PARAMS (&LAST_AMC100_PARAM - &FIRST_AMC100_PARAM + 1)
#endif  // AMC100Controller_H
