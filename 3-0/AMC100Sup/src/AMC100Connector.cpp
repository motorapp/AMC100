// Copyright (c) 2014  Australian Synchrotron
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
// Thread to verify device actually responds when asyn connected true
// We maintain our own connected_ flag outside of asyn

#include <string.h>
#include <iostream>  //cout
#include <sstream>   //ostringstream istringstream
#include <epicsThread.h>
#include <shareLib.h>
#include <asynOctetSyncIO.h>
#include <asynCommonSyncIO.h>
#include <algorithm> //std::count

#include "AMC100Controller.h"

//Constructor
AMC100Connector::AMC100Connector(AMC100Controller *pcntrl)
   :  thread(*this,"AMC100Connector",epicsThreadGetStackSize(epicsThreadStackMedium),epicsThreadPriorityLow)
{
	//Store the AMC100Controller instance that created this AMC100Connector instance
	pC_ = pcntrl;
	//Flag not connected at startup
	pC_->connected_ = false;
	//Flag AMC100Connector thread not shutting down
	shutDownConnector_ = false;
	//Start AMC100Connector thread
	thread.start();
}

//Destructor
AMC100Connector::~AMC100Connector()
{
	//Flag to AMC100Connector run thread that IOC is shutting down
	shutDownConnector_ = true;
	//Wake AMC100Connector thread now shutdown flag is set
	epicsEventSignal(pC_->connectEvent_);
	//Wait for run thread to exit
	thread.exitWait();
}

//AMC100Connector thread
//Thread to verify device actually responds when asyn connected true
//This is needed due to Moxa use
//We maintain our own connected_ flag outside of asyn
void AMC100Connector::run(void)
{
   int sync_status;		//Synchronous communication status
   //string resp;		//For checking controller response

   //Check if AMC100 actually responds to query
   while (true) {
      //Wait for connect event signal
      epicsEventWait(pC_->connectEvent_);
      if (shutDownConnector_) {
         //Thread shutdown requested
         break;
      }
      else {
         pC_->lock();
         //Check AMC100Controller for response
         strcpy(pC_->cmd_, getAllPositions.dump().c_str());
         sync_status = pC_->sync_writeReadController(true);
         //Work out what to do
         if (!sync_status) {
            //Response received for synchronous connection
            //Do whats required for AMC100Controller once connection established
            pC_->connected();
         }
         else if (!pC_->shuttingDown_) {
            //No response
            //IOC isn't shutting down
            //Continue to force disconnect until device responds
            pC_->disconnect();
         }
      pC_->unlock();
      }
   } //while true
}

