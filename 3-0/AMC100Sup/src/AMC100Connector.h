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
// Thread to manage disconnect/connect for a AMC100Controller

class AMC100Connector: public epicsThreadRunable {
public:
  AMC100Connector(class AMC100Controller *pcntrl);
  virtual void run();
  epicsThread thread;
  ~AMC100Connector();

private:
  bool shutDownConnector_;
  class AMC100Controller *pC_;
};

