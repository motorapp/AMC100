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
// Thread to acquire datarecord for a single AMC100Controller
// We write our own because epicsEventWaitWithTimeout in asynMotorController::asynMotorPoller waits, we dont want that.
// Instead either Async speed determines poller frequency or simple epicsThreadSleep in Sync mode

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <epicsThread.h>
#include <shareLib.h>

#include "AMC100Controller.h"

AMC100Poller::AMC100Poller(AMC100Controller *pcntrl)
   :  thread(*this,"AMC100Poller",epicsThreadGetStackSize(epicsThreadStackMedium),epicsThreadPriorityMax)
{
   //Store the AMC100Controller we poll for
   pC_ = pcntrl;
   //Create poller sleep event
   pollerSleepEventId_ = epicsEventMustCreate(epicsEventEmpty);
   //Create poller wake event
   pollerWakeEventId_ = epicsEventMustCreate(epicsEventEmpty);
   //Poller awake at start
   pollerSleep_ = false;
   //Start AMC100Poller thread
   shutdownPoller_ = false;
   thread.start();
}

//AMC100Poller thread
//Acquire data record for a single AMC100Controller
//Replaces asynMotorController::asynMotorPoller
void AMC100Poller::run(void)
{
   unsigned i;
   AMC100Axis *pAxis; 	//Axis structure
   double time_taken;	//Time taken last poll cycle
   double sleep_time;	//Calculated time to sleep in synchronous mode

   //Loop until shutdown
   while (true) {
      //Dont poll in sleep mode or when shutting down
      if (!pollerSleep_ && !shutdownPoller_) {
         //Wake mode
         //Poll only if connected
         if (pC_->connected_) {
            //Obtain the lock
            pC_->lock();

            //Read start time
            epicsTimeGetCurrent(&pollstartt_);

            //Get the data record, update controller related information in AMC100Controller, and ParamList.  callBacks not called
            pC_->poller();

            //Update the AMC100Axis status, using datarecord from AMC100Controller
            //Do callbacks for AMC100Controller, AMC100Axis records
            //Do all ParamLists/axis whether user called AMC100CreateAxis or not
            //because analog/binary IO data are stored/organized in ParamList just same as axis data 
            for (i=0; i<MAX_AMC100_AXES; i++) {
               if (i < MAX_AMC100_AXES) {
                  //Retrieve AMC100Axis instance i
                  pAxis = pC_->getAxis(i);
                  if (!pAxis) {
                     //User did not call AMC100CreateAxis for this axis number
                     //Ensure callbacks are called to update upper layer analog/binary
                     //Do this for first 8 addresses only to cover analog/binary
                     //Cant call AMC100Axis->poller
                     pC_->callParamCallbacks(i);
                  }
                  else {
                     //Update AMC100Axis, and upper layers, using retrieved datarecord
                     //Update records with analog/binary data
                     pAxis->poller();
                  }
               }
            } //For

            //Release the lock
            pC_->unlock();

            //Read end time
            epicsTimeGetCurrent(&pollendt_);
            //Calculate cycle time
            time_taken = epicsTimeDiffInSeconds(&pollendt_, &pollstartt_);

            //No async, so wait updatePeriod_ rather than relying on async record delivery frequency
            //Adjust sleep time according to time_taken last poll cycle
            sleep_time = pC_->updatePeriod_/1000.0 - time_taken;
            //Must sleep in synchronous mode to release lock for other threads
            sleep_time = (sleep_time < 0.000) ? 0.001 : sleep_time;
            if (sleep_time >= 0.001)
               epicsThreadSleep(sleep_time);
         } //Connected_
         else //Not connected so sleep a little
            epicsThreadSleep(.1);
      }
      else if (pollerSleep_ && !shutdownPoller_) {
         //Sleep mode
         //Inform blocking thread poller has now entered sleep mode
         epicsEventSignal(pollerSleepEventId_);
         //Sleep until signalled
         epicsEventWait(pollerWakeEventId_);
      }

      //Kill loop as IOC is shuttingDown
      if (shutdownPoller_)  //Break from loop
         break;
   }//while
}

void AMC100Poller::shutdownPoller()
{
   //Send poller to sleep, so we know where the thread is
   sleepPoller();
   //Tell poller to shutdown
   shutdownPoller_ = true;
   //Pause
   epicsThreadSleep(.01);
   //Wake poller and send it to shutdown
   wakePoller();
   //Wait till poller thread exits
   thread.exitWait();
}

AMC100Poller::~AMC100Poller()
{
   //Shutdown poller thread
   shutdownPoller();
   //Destroy events
   epicsEventDestroy(pollerSleepEventId_);
   epicsEventDestroy(pollerWakeEventId_);
}

//Put poller in sleep mode, and stop async records if needed
//Must be called without lock so sync poller can be put to sleep
void AMC100Poller::sleepPoller(void)
{
   //Only if poller awake now
   if (!pollerSleep_) {
      //Tell poller to sleep
      pollerSleep_ = true;
      //Wait until AMC100Poller is sleeping
      epicsEventWait(pollerSleepEventId_);
   }
}

//Wake poller and re-start async records if desired
void AMC100Poller::wakePoller(void)
{
   int status;

   //Only if poller sleeping now
   if (pollerSleep_) {
      //Wake up poller
      pollerSleep_ = false;
      epicsEventSignal(pollerWakeEventId_);
   }
}
