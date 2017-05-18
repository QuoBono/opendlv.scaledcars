/**
 * OpenDaVINCI - Tutorial.
 * Copyright (C) 2015 Christian Berger
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */


#ifndef SOFTWARE_SERIALPORT_H
#define SOFTWARE_SERIALPORT_H

#include <stdint.h>
#include <iostream>

#include <string>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"


#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
#include <opendavinci/odcore/io/StringListener.h>

//#include <automotivedata/GeneratedHeaders_AutomotiveData.h>
//#include "odvdscaledcarsdatamodel/GeneratedHeaders_ODVDScaledCarsDataModel.h"


//#include "Proxy.h"


//This is to organize the namespaces in order to get the containers.
namespace odcore { namespace io { namespace conference { class ContainerConference; } } }

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace automotive::miniature;

        using namespace odcore;
        using namespace odcore::data;

        using namespace odcore::io::conference;

        // This class will handle the bytes received via a serial link. With the StringListener pre-built interface.
        class SerialReceiveBytes : public odcore::io::StringListener {

            //explain
        private:
            SerialReceiveBytes (const SerialReceiveBytes &) = delete;
            SerialReceiveBytes &operator=(const SerialReceiveBytes & ) = delete;

            //
        public:
            SerialReceiveBytes (odcore::io::conference::ContainerConference &); //that's why we organized namespaces.
            ContainerConference &m_conference;

            virtual void nextString(const string &s);
        };
    }
}

#endif
