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
//

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#if defined _WIN32 || _WIN64
#include <windows.h>
#endif /* _WIN32 */
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <algorithm> //std::replace

#include <epicsString.h>
#include <iocsh.h>
#include <epicsThread.h>
#include <errlog.h>
#include <shareLib.h>

#include <asynOctetSyncIO.h>

#include "AMC100Controller.h"
#include <epicsExport.h>

static void eventMonitorThreadC(void *pPvt);

// These are the AMC100Axis methods

/** Creates a new AMC100Axis object.
  * \param[in] pC Pointer to the AMC100Controller to which this axis belongs. 
  * \param[in] Axis 0-2
  */
AMC100Axis::AMC100Axis(class AMC100Controller *pC, //Pointer to controller instance
		     int axis)			//Axis 0-2
  : asynMotorAxis(pC, axis),
    pC_(pC)
{
  //Initial default for stoppedTime
  epicsTimeGetCurrent(&stop_begint_);
  stop_nowt_ = stop_begint_;
 
  //Increment internal axis counter
  pC_->numAxes_++;

  //Create stop time reset event
  stoppedTimeReset_ = epicsEventMustCreate(epicsEventEmpty);
  //Create begin motion event
  beginEvent_ = epicsEventMustCreate(epicsEventEmpty);
  //Create stop motion event
  stopEvent_ = epicsEventMustCreate(epicsEventEmpty);
  //Create caller event
  callerEvent_ = epicsEventMustCreate(epicsEventEmpty);
  //Create event monitor start event
  eventMonitorStart_ = epicsEventMustCreate(epicsEventEmpty);
  //Create event monitor done event
  eventMonitorDone_ = epicsEventMustCreate(epicsEventEmpty);

  //store settings, and set defaults
  setDefaults();
 
  // Create the event monitor thread
  epicsThreadCreate("eventMonitor", 
                    epicsThreadPriorityMax,
                    epicsThreadGetStackSize(epicsThreadStackMedium),
                    (EPICSTHREADFUNC)eventMonitorThreadC, (void *)this);

  // We assume motor with encoder
  setIntegerParam(pC->motorStatusGainSupport_, 1);
  setIntegerParam(pC->motorStatusHasEncoder_, 1);
  //Default stall/following error status
  setIntegerParam(pC_->motorStatusSlip_, 0);
  callParamCallbacks();
}

//AMC100Axis destructor
AMC100Axis::~AMC100Axis()
{
  //Set flag axis shut down in progress
  shuttingDown_ = true;
  //Now flag set, send eventMonitor thread to shutdown
  epicsEventSignal(eventMonitorStart_);

  if (profilePositions_ != NULL)
     free(profilePositions_);

  //Destroy events
  //Sleep to preempt this thread, and give time
  //for event Monitor thread to exit
  epicsThreadSleep(.002);
  epicsEventDestroy(beginEvent_);
  epicsEventDestroy(stopEvent_);
  epicsEventDestroy(callerEvent_);
  epicsEventDestroy(eventMonitorStart_);
  epicsEventDestroy(eventMonitorDone_);
  epicsEventDestroy(stoppedTimeReset_);
}

/*--------------------------------------------------------------------------------*/
/* Store settings, set defaults for motor */
/*--------------------------------------------------------------------------------*/

void AMC100Axis::setDefaults(void)
{
   //const char *functionName = "AMC100Axis::setDefaults";

   //Five polls considered minimum to detect move start, stop
   double multiplier = 5.0 / (BEGIN_TIMEOUT / (pC_->updatePeriod_ / 1000.0));

   //Min multiplier is 1
   multiplier = (multiplier > 1) ? multiplier : 1;

   //Not shutting done
   shuttingDown_ = false;

   //Move method should not signal caller by default
   signalCaller_ = false;

   //Tell poller we dont want signal when events occur
   requestedEventSent_ = true;

   //Default event timeout
   requestedTimeout_ = BEGIN_TIMEOUT * multiplier;
      
   //This axis is not performing a deferred move
   deferredMove_ = false;

   //Store axis in ParamList
   setIntegerParam(pC_->AMC100Axis_, axisNo_);

   //Give default readback values for positions, movement direction
   motor_position_ = 0;
   last_encoder_position_ = 0;
   encoder_position_ = 0;
   direction_ = 1;

   //Pass default step count/aux encoder value to motorRecord
   setDoubleParam(pC_->motorPosition_, motor_position_);
   //Pass default encoder value to motorRecord
   setDoubleParam(pC_->motorEncoderPosition_, encoder_position_);
   //Pass default direction value to motorRecord
   setIntegerParam(pC_->motorStatusDirection_, direction_);

   //Axis not ready until necessary motor record fields have been pushed into driver
   axisReady_ = false;
}

/** Move the motor to an absolute location or by a relative amount.
  * \param[in] position  The absolute position to move to (if relative=0) or the relative distance to move 
  * by (if relative=1). Units=steps.
  * \param[in] relative  Flag indicating relative move (1) or absolute move (0).
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec*/
asynStatus AMC100Axis::move(double position, int relative, double minVelocity, double maxVelocity, double acceleration)
{
  //static const char *functionName = "AMC100Axis::move";
  asynStatus status = asynError;
  ordered_json resp;
  asynStatus result;

  //Assemble setControlTargetPosition command
  ordered_json setControlTargetPosition = {
                 {"jsonrpc", "2.0"},
                 {"method", "com.attocube.amc.move.setControlTargetPosition"},
                 {"params", {axisNo_, position}},
                 {"id", 0},
                 {"api", 2}};

  //Are moves to be deferred ?
  if (pC_->movesDeferred_ != 0) {
     //Moves are deferred
     status = asynSuccess;
  }
  else {
     //Moves are not deferred
     //Set position target
     strcpy(pC_->cmd_, setControlTargetPosition.dump().c_str());
     status = pC_->sync_writeReadController();
     //Check response
     if (!status) {
        resp = json::parse(pC_->resp_);
        if (resp["error"] != nullptr || resp["result"][0] != 0) {
           status = asynError;
        }
     }
     if (!status) {
        //Wait until axis moving true status delivered to mr, or timeout
        result = axisEventMonitor(beginEvent_);
     }
  }

  //Return status
  return status;
}

/** Move the motor to the home position.
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec.
  * \param[in] forwards  Flag indicating to move the motor in the forward direction(1) or reverse direction(0).
  *                      Some controllers need to be told the direction, others know which way to go to home. */
asynStatus AMC100Axis::home(double minVelocity, double maxVelocity, double acceleration, int forwards)
{
  //static const char *functionName = "AMC100Axis::home";
  asynStatus status = asynError;
  ordered_json resp;
  //Assemble searchReferencePosition command
  ordered_json searchReferencePosition = {
                 {"jsonrpc", "2.0"},
                 {"method", "com.attocube.amc.control.searchReferencePosition"},
                 {"params", {axisNo_}},
                 {"id", 0},
                 {"api", 2}};

  //Home axis
  strcpy(pC_->cmd_, searchReferencePosition.dump().c_str());
  status = pC_->sync_writeReadController();
  //Check response
  if (!status) {
     resp = json::parse(pC_->resp_);
     if (resp["error"] != nullptr || resp["result"][0] != 0) {
        status = asynError;
     }
  }

  //Return status
  return status;
}

/** Move the motor at a fixed velocity until told to stop.
  * \param[in] minVelocity The initial velocity, often called the base velocity. Units=steps/sec.
  * \param[in] maxVelocity The maximum velocity, often called the slew velocity. Units=steps/sec.
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus AMC100Axis::moveVelocity(double minVelocity, double maxVelocity, double acceleration)
{
  static const char *functionName = "moveVelocity";
  asynStatus status = asynError;
  
  //Return status
  return status;
}

/** Stop the motor.  Called by motor record
  * \param[in] acceleration The acceleration value. Units=steps/sec/sec. */
asynStatus AMC100Axis::stop(double acceleration)
{
  //static const char *functionName = "AMC100Axis::stop";
  asynStatus status = asynError;

  //Always return success. Dont need more error mesgs
  return asynSuccess;
}

/** Set the motor closed loop status. 
  * \param[in] closedLoop true = close loop, false = open loop. */
asynStatus AMC100Axis::setClosedLoop(bool closedLoop)
{
  asynStatus status;
  ordered_json resp;

  //Assemble setControlAutoReset command
  ordered_json setControlAutoReset = {
                 {"jsonrpc", "2.0"},
                 {"method", "com.attocube.amc.control.setControlAutoReset"},
                 {"params", {axisNo_, 1}},
                 {"id", 0},
                 {"api", 2}};

  //Assemble setControlOutput command
  ordered_json setControlOutput = {
                 {"jsonrpc", "2.0"},
                 {"method", "com.attocube.amc.control.setControlOutput"},
                 {"params", {axisNo_, (int)(closedLoop)}},
                 {"id", 0},
                 {"api", 2}};

  //Assemble setControlMove command
  ordered_json setControlMove = {
                 {"jsonrpc", "2.0"},
                 {"method", "com.attocube.amc.control.setControlMove"},
                 {"params", {axisNo_, (int)(closedLoop)}},
                 {"id", 0},
                 {"api", 2}};

  //Set control auto-reset
  //This resets the axis readback to 0 at location where referene detected
  strcpy(pC_->cmd_, setControlAutoReset.dump().c_str());
  status = pC_->sync_writeReadController();

  //Longer timeout required for setControlOutput
  pC_->timeout_ = 4;

  //Set control output
  //Enable the axis
  strcpy(pC_->cmd_, setControlOutput.dump().c_str());
  status = pC_->sync_writeReadController();

  //Default timeout
  pC_->timeout_ = DEFAULT_TIMEOUT;

  //Set control move
  //Enable move to setpoint
  strcpy(pC_->cmd_, setControlMove.dump().c_str());
  status = pC_->sync_writeReadController();

  return asynSuccess;
}

//Extract axis data from AMC100Controller data record and
//store in AMC100Axis (motorRecord attributes) or asyn ParamList (other record attributes)
asynStatus AMC100Axis::getStatus(void)
{
   int inmotion;			//Local copy of inmotion
   int status = asynError;		//Return status
   int homed;				//axis homed status
   ordered_json resp;			//Controller response parsed to json
   bool ctrlOutput = false;		//Axis control output enabled?
   bool ctrlMove = false;		//Axis control move enabled?
   int closedLoop = 0;			//Closed loop status to pass to asynParam

   ordered_json getControlOutput = {
                 {"jsonrpc", "2.0"},
                 {"method", "com.attocube.amc.control.getControlOutput"},
                 {"params", {axisNo_}},
                 {"id", 0},
                 {"api", 2}};

   ordered_json getControlMove = {
                 {"jsonrpc", "2.0"},
                 {"method", "com.attocube.amc.control.getControlMove"},
                 {"params", {axisNo_}},
                 {"id", 0},
                 {"api", 2}};

   //Assemble getReferencePosition command
   ordered_json getStatusReference = {
                 {"jsonrpc", "2.0"},
                 {"method", "com.attocube.amc.status.getStatusReference"},
                 {"params", {axisNo_}},
                 {"id", 0},
                 {"api", 2}};

   //If data record query success in AMC100Controller::acquireData
   if (pC_->recstatus_ == asynSuccess) {
      //extract relevant axis data from AMC100Controller data-record, store in AMC100Axis
      //If connected, then proceed
      if (pC_->connected_) {
        //Get motor, encoder position from AMC100Controller
        motor_position_ = encoder_position_ = pC_->positions_["result"][axisNo_ + 1];
        //Get move status from AMC100Controller
        inmotion = pC_->moveStatus_["result"][axisNo_ + 1];
        inmotion_ = (bool)inmotion;

        //Get axis reference status by query to controller
        strcpy(pC_->cmd_, getStatusReference.dump().c_str());
        status = pC_->sync_writeReadController();
        //Check response
        if (!status) {
           try {
              resp = json::parse(pC_->resp_);
              if (resp["error"] == nullptr || resp["result"][0] == 0) {
                 //Response is ok
                 homed = (resp["result"][1] == true) ? 1 : 0;
                 setIntegerParam(pC_->motorStatusHomed_, homed);
              }
           }
           catch (...) {
           }
        }

        //Get axis closed loop status, part 1
        //Get axis ControlOutput status by query to controller
        strcpy(pC_->cmd_, getControlOutput.dump().c_str());
        status = pC_->sync_writeReadController();
        //Check response
        if (!status) {
           try {
              resp = json::parse(pC_->resp_);
              if (resp["error"] == nullptr || resp["result"][0] == 0) {
                 //Response is ok
                 ctrlOutput = (resp["result"][1] == true) ? 1 : 0;
              }
           }
           catch (...) {
           }
        }

        //Get axis closed loop status, part 2
        //Get axis ControlMove status by query to controller
        strcpy(pC_->cmd_, getControlMove.dump().c_str());
        status = pC_->sync_writeReadController();
        //Check response
        if (!status) {
           try {
              resp = json::parse(pC_->resp_);
              if (resp["error"] == nullptr || resp["result"][0] == 0) {
                 //Response is ok
                 ctrlMove = (resp["result"][1] == true) ? 1 : 0;
              }
              //Pass motor closed loop status to asynParam list
              setIntegerParam(pC_->motorStatusPowerOn_, (int)(ctrlMove && ctrlOutput));
           }
           catch (...) {
           }
        }
      }
   }

   //Return status
   return (asynStatus)status;
}

//Set poller internal status variables based on data record info
//Called by poll
void AMC100Axis::setStatus(bool *moving)
{
  int encoder_direction;	//Determined encoder move direction

  //Encoder move status
  encoderMove_ = false;
  if (ueip_) {
     //Check encoder move
     if (last_encoder_position_ > (encoder_position_ + enc_tol_)) {
        encoder_direction = 0;
        encoderMove_ = true;
     }
     if (last_encoder_position_ < (encoder_position_ - enc_tol_)) {
        encoder_direction = 1;
        encoderMove_ = true;
     }
     //Encoder not moving
     if (fabs(last_encoder_position_ - encoder_position_) <= enc_tol_)
        encoder_direction = direction_;

     //Encoder direction ok flag
     encDirOk_ = (encoder_direction == direction_) ? true : false;
  }

   //Determine move status
   //Motors with deferred moves pending set to status moving
   if (inmotion_ || deferredMove_) {
      //Set moving flags
      *moving = true;
      done_ = 0;
   }
}

//Called by poll
//Sets motor stop time
void AMC100Axis::setStopTime(void)
{   
   //Reset stopped time if moving
   if (!done_)
      stoppedTime_ = 0.0;

   //Any request for stopped time to be reset?
   if (resetStoppedTime_ && done_)
      {
      //Get time stop first detected
      epicsTimeGetCurrent(&stop_begint_);
      //Request completed
      resetStoppedTime_ = false;
      //Signal requesting thread that stopped time has been reset
      epicsEventSignal(stoppedTimeReset_);
      }
   if (done_ && !last_done_)
      {
      //Get time stop first detected
      epicsTimeGetCurrent(&stop_begint_);
      }
   if (done_ && last_done_)
      {
      //Get time stopped for
      epicsTimeGetCurrent(&stop_nowt_);
      //Result
      stoppedTime_ = epicsTimeDiffInSeconds(&stop_nowt_, &stop_begint_);
      }
}

/** Delay until event or timeout
  * May signal interested caller thread if signalCaller_ true and event received successfully
  * \param[in] requestedEvent Requested event (ie. beginEvent_, stopEvent_) */
asynStatus AMC100Axis::axisEventMonitor(epicsEventId requestedEvent) {
   asynStatus result = asynSuccess; //Default result

   //Set the requested event
   requestedEvent_ = requestedEvent;
   //Signal event monitor to start monitoring
   epicsEventSignal(eventMonitorStart_);
   //Tell poller to signal when event occurs
   requestedEventSent_ = false;

   //Poller sends the signal we wait on
   //Release lock so sync poller can get lock
   pC_->unlock();
   //Wait for event monitor to finish
   epicsEventWait(eventMonitorDone_);
   //Check result
   if (eventResult_ != epicsEventWaitOK)
      result = asynError; //Timeout, or error
   else {
      //Inform interested caller thread that eventMonitor got event
      if (signalCaller_) {
         //Signal caller if eventMonitor got event
         epicsEventSignal(callerEvent_);
         //Caller has been signaled
         signalCaller_ = false;
      }
   }
   //Retake lock
   pC_->lock();

   //Return result
   return result;
}

/** Send axis events to waiting threads
  * Called by poller without lock after passing
  * motor status to motor record */
void AMC100Axis::sendAxisEvents(void) {

   //Check for work
   if (requestedEventSent_) return; //Nothing to do

   //Motion begin event = confirmed moving status true delivered to MR
   if (requestedEvent_ == beginEvent_)
      if (inmotion_) {
         //Axis is really moving
         //Begin event
         epicsEventSignal(beginEvent_);
         //Tell poller we dont want signal when events occur
         requestedEventSent_ = true;
      }

   //Motion stop event = confirmed moving status false delivered to MR
   if (requestedEvent_ == stopEvent_)
      if (!inmotion_) {
         //Axis is really stopped
         //Stop event
         epicsEventSignal(stopEvent_);
         //Tell poller we dont want signal when events occur
         requestedEventSent_ = true;
      }
}

/** Polls the axis.
  * This function reads the controller position, encoder position, the limit status, the moving status, 
  * and the drive power-on status.  It does not current detect following error, etc. but this could be
  * added.
  * It calls setIntegerParam() and setDoubleParam() for each item that it polls,
  * \param[out] moving A flag that is set indicating that the axis is moving (1) or done (0). */
asynStatus AMC100Axis::poller(void)
{
   //static const char *functionName = "AMC100Axis::poll";
   bool moving;			//Moving status
   int home;			//Home status to give to motorRecord
   int status;			//Communication status with controller

   //Default communication status
   status = asynError;
   //Default home status
   home = 0;
   //Default moving status
   done_ = 1;
   moving = false;
   
   //Skip if data read was bad
   if (pC_->recstatus_) goto skip;
   //Extract axis motion data, and load into AMC100Axis instance
   getStatus();

   //Set poll variables in AMC100Axis based on data record info
   setStatus(&moving);

   //Set motor stop time
   setStopTime();

skip:
   //Save encoder position, and done for next poll cycle
   last_encoder_position_ = encoder_position_;
   last_done_ = done_;

   //Set status
   //Pass step count/aux encoder info to motorRecord
   setDoubleParam(pC_->motorPosition_, motor_position_);
   //Pass encoder value to motorRecord
   setDoubleParam(pC_->motorEncoderPosition_, encoder_position_);
   //Pass home status to motorRecord
   setIntegerParam(pC_->motorStatusAtHome_, home);
   setIntegerParam(pC_->motorStatusHome_, home);
   //Pass direction to motorRecord
   setIntegerParam(pC_->motorStatusDirection_, direction_);

   //Dont show limits whilst homing otherwise mr may interrupt custom routines
   if (homing_) {
      //Dont show reverse limit when homing
      rev_ = 0;
      //Dont show forward limit when homing
      fwd_ = 0;
   }

   //Pass limit status to motorRecord
   setIntegerParam(pC_->motorStatusLowLimit_, rev_);
   setIntegerParam(pC_->motorStatusHighLimit_, fwd_);
   //Pass moving status to motorRecord
   setIntegerParam(pC_->motorStatusDone_, done_);
   setIntegerParam(pC_->motorStatusMoving_, moving);
   //Pass comms status to motorRecord
   setIntegerParam(pC_->motorStatusCommsError_, pC_->recstatus_ ? 1:0);
   //Update motor status fields in upper layers using asynMotorAxis->callParamCallbacks
   callParamCallbacks();
   //Status delivered to MR, now send events to waiting threads
   sendAxisEvents();
   //Always return success. Dont need more error mesgs
   return asynSuccess;
}

/* C Function which runs the event monitor thread */ 
static void eventMonitorThreadC(void *pPvt)
{
  AMC100Axis *pC = (AMC100Axis*)pPvt;
  pC->eventMonitorThread();
}

//Event monitor runs in its own thread
//Used to synchronize threads to events sent by poller
void AMC100Axis::eventMonitorThread()
{ 
  while (true) {
    //Wait for request
    epicsEventWait(eventMonitorStart_);
    //Check for shutdown request
    if (shuttingDown_)
       return; //Exit the thread
    //Wait for requested event
    eventResult_ = epicsEventWaitWithTimeout(requestedEvent_, requestedTimeout_);
    if (eventResult_ != epicsEventWaitOK) {
       //Timeout, or error occurred
       //Tell poller we dont want signal when events occur
       requestedEventSent_ = true;
    }
    //Signal waiting thread begin monitor done
    epicsEventSignal(eventMonitorDone_);
  }
}

