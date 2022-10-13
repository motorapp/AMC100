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
// Thread to acquire datarecord for a single AMC100Controller
// We write our own because epicsEventWaitWithTimeout in asynMotorController::asynMotorPoller calls sleep, we dont want that.
// Needed better performance

class AMC100Poller: public epicsThreadRunable {
public:
  AMC100Poller(AMC100Controller *pcntrl);
  void wakePoller(void);
  void sleepPoller(void);
  virtual void run();
  epicsThread thread;
  ~AMC100Poller();

private:
  AMC100Controller *pC_;		//The AMC100Controller we poll for
  bool pollerSleep_;			//Tell poller to sleep
  epicsTimeStamp pollstartt_;		//Used to calculate sleep time in synchronous mode
  epicsTimeStamp pollendt_;		//Used to calculate sleep time in synchronous mode

  epicsEventId pollerSleepEventId_;    	//Poller sleep event
  epicsEventId pollerWakeEventId_;    	//Poller Wake event
  bool shutdownPoller_;
  void shutdownPoller();
};
