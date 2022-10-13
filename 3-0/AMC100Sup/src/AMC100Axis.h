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
//

#ifndef AMC100Axis_H
#define AMC100Axis_H

#include "asynMotorController.h"
#include "asynMotorAxis.h"
#include "epicsMessageQueue.h"

#define KPMAX			1023.875
#define KDMAX			4095.875
#define HOMING_TIMEOUT		3.5

//Home type allowed
#define HOME_NONE 0
#define HOME_REV 1
#define HOME_FWD 2
#define HOME_BOTH 3

//limitState enum used in wrong limit protection
typedef enum limitsState
   {
   unknown, consistent, not_consistent
   } limitsState;

class AMC100Axis : public asynMotorAxis
{
public:

  AMC100Axis(class AMC100Controller *pC, 
	    int axis);			//Axis 0-2

  //These are the methods that are new to this class
  //Poller for axis
  asynStatus poller(void);
  //Store settings, and implement defaults
  void setDefaults(void);
  //Extract axis data from AMC100Controller data record
  asynStatus getStatus(void);
  //Set poller status variables bassed on AMC100Controller data record info
  void setStatus(bool *moving);
  //Sets time motor has been stopped for in AMC100Axis::stoppedTime_
  void setStopTime(void);
  //Wait for event or timeout
  asynStatus axisEventMonitor(epicsEventId requestedEvent);
  //Send axis events
  void sendAxisEvents(void);
  //Thread to receive axis events as they occur (eg. start, stop)
  void eventMonitorThread();

  /* These are the methods we override from the base class */
  asynStatus move(double position, int relative, double minVelocity, double maxVelocity, double acceleration);
  asynStatus moveVelocity(double minVelocity, double maxVelocity, double acceleration);
  asynStatus home(double minVelocity, double maxVelocity, double acceleration, int forwards);
  asynStatus stop(double acceleration);
  asynStatus setClosedLoop(bool closedLoop);

  virtual ~AMC100Axis();

private:

  void axisStatusShutdown(void);        //Function to shutdown axis status thread.

  AMC100Controller *pC_;      		/**< Pointer to the asynMotorController to which this axis belongs.
                                	*   Abbreviated because it is used very frequently */
  char axisName_;			//The axis letter A-H

  double highLimit_;			//High soft limit
  double lowLimit_;			//Low soft limit
  double encmratio_;			//Encoder/motor ratio
  double deferredAcceleration_;		//Coordinate system acceleration
  double deferredVelocity_;		//Coordinate system velocity
  double deferredPosition_;		//Deferred move position
  int deferredRelative_;		//Deferred move is relative or absolute
  bool deferredMove_;			//Has a deferred move been set
  int deferredMode_;			//Sync start and stop, or sync start only
  bool axisReady_;			//Have motor record fields been pushed into driver

  epicsTimeStamp pestall_nowt_;		//Used to track length of time encoder has been stalled for
  epicsTimeStamp pestall_begint_;	//Time when possible encoder stall first detected

  int ueip_;				//motorRecord ueip.  User wants to read main encoder if true, aux if false
  int enc_tol_;                         //encoder tolerance. Used for determining encoder direction in setStatus()
  double motor_position_;		//aux encoder or step count register
  double encoder_position_;		//main encoder register
  double last_encoder_position_;	//main encoder register stored from previous poll.  Used to detect movement.
  int direction_;			//Movement direction
  bool inmotion_;			//Axis in motion status from controller
  bool fwd_;				//Forward limit status
  bool rev_;				//Reverse limit status

  bool signalCaller_;			//Should axisEventMonitor signal caller when done ?
  bool requestedEventSent_;		//Poller has sent the requested event
  double requestedTimeout_;		//Event timeout
  epicsEventId beginEvent_;		//Axis begin event
  epicsEventId stopEvent_;		//Axis stop event
  epicsEventId callerEvent_;		//Axis event monitor received event signal to interested threads
  epicsEventId requestedEvent_;		//Event requested
  epicsEventId eventMonitorStart_;	//Tell event monitor thread to start monitoring
  epicsEventId eventMonitorDone_;	//Event monitor signals done to other threads
  epicsEventWaitStatus eventResult_;	//Event received or timeout
 
  bool home_;				//Home switch raw status direct from data record
  int done_;				//Motor done status passed to motor record
  int last_done_;			//Backup of done status at end of each poll.  Used to detect stop
  bool homing_;				//Is motor homing now
  int stop_reason_;			//Reason axis stop requested
  epicsTimeStamp stop_nowt_;		//Used to track length of time motor stopped for.
  epicsTimeStamp stop_begint_;		//Used to track length of time motor stopped for.
  double stoppedTime_;			//Time motor has been stopped for
  bool resetStoppedTime_;		//Request poll thread reset stopped time if done true
  epicsEventId stoppedTimeReset_;	//Signal that poller has completed reset stop time request
  bool encDirOk_;			//Encoder direction ok flag
  bool encoderMove_;			//Encoder move status
  bool pestall_detected_;		//Possible encoder stall detected flag
  bool stopSent_;			//Has motor stop mesg been sent to pollServices thread due to encoder stall or wrong limit protection
  bool homedSent_;			//Homed message sent to pollServices
  bool homedExecuted_;			//Homed message has been executed by pollServices
  bool cancelHomeSent_;			//Cancel home process message sent to pollServices
  bool encoderSwapped_;			//Have the main, and auxiliary encoders been swapped by DFx=1

  bool shuttingDown_;			//Flag to axis shutdown

friend class AMC100Controller;
};

#endif   // AMC100Axis_H

