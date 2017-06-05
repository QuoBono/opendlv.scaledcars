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

#include <stdint.h>
#include <iostream>

#include <memory>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "SerialReceiveBytes.hpp"

//for converting the string to double
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h> /* strtod */

//For Mapping the sensors
#include <map>

#include <string>
#include <cstring>
#include "SerialReceiveBytes.hpp"

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include <Proxy.h>

namespace odcore { namespace io { namespace conference { class ContainerConference; }}}

namespace automotive {
    namespace miniature {

        using namespace std;

        using namespace odcore;
        using namespace odcore::wrapper;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odcore::io::conference;

        using namespace automotive;
        using namespace automotive::miniature;


        SerialReceiveBytes::SerialReceiveBytes(odcore::io::conference::ContainerConference &conference) :
                m_conference(conference) {}

        void SerialReceiveBytes::nextString(const string &s) {

            cerr << "this is the string " << s << endl;
            cerr << "this is the string " << s << endl;
            cerr << "this is the string " << s << endl;
            cerr << "this is the string " << s << endl;
            cerr << "this is the string " << s << endl;
            cerr << "this is the string " << s << endl;

            int serialStart = s.find("{");
            int serialEnd = s.find("}");

            if (s.length() > 10 && !s.empty() && serialStart != -1 && serialEnd != -1) {

                serialStart = s.find("{");


                string tmp;
                serialEnd = 0;

                if ((int) s.length() > (int) serialStart && (int) s.length() > 10) {

                    tmp = s.substr(serialStart, s.length());
                    serialEnd = tmp.find("}");

                }

                if ((int) serialEnd > (int) serialStart && (int) tmp.length() > 10) {
                    string serialValues = tmp.substr(1, serialEnd - 1);
                    cerr << "Received new" << serialValues.length() << " bytes containing '" << serialValues << "'"
                         << endl;

                    //we create a temporal string
                    string sendSensortmp = serialValues;

                    //
                    string sensor0 = sendSensortmp.substr(0, sendSensortmp.find(" "));

                    double sensorZERO = atof(sensor0.c_str());
                    cerr << "this is sensorZERO " << sensorZERO << endl;

                    //cut the string
                    sendSensortmp = sendSensortmp.substr(sendSensortmp.find(" ") + 1, sendSensortmp.length());
                    cerr << "THIS IS sendSensortmp " << sendSensortmp << endl;
                    //
                    string sensor1 = sendSensortmp.substr(0, sendSensortmp.find(" "));
                    double sensorONE = atof(sensor1.c_str());
                    cerr << "this is sensorONE " << sensorONE << endl;

                    //cut the string
                    sendSensortmp = sendSensortmp.substr(sendSensortmp.find(" ") + 1, sendSensortmp.length());
                    cerr << "TMP2 IS sendSensortmp " << sendSensortmp << endl;
                    //
                    string sensor2 = sendSensortmp.substr(0, sendSensortmp.find(" "));
                    double sensorTWO = atof(sensor2.c_str());
                    cerr << "this is sensorTWO " << sensorTWO << endl;

                    //cut the string
                    sendSensortmp = sendSensortmp.substr(sendSensortmp.find(" ") + 1, sendSensortmp.length());
                    cerr << "TMP2 IS sendSensortmp " << sendSensortmp << endl;
                    //
                    string sensor3 = sendSensortmp.substr(0, sendSensortmp.find(" "));
                    double sensorTHREE = atof(sensor3.c_str());
                    cerr << "this is sensorTHREE " << sensorTHREE << endl;

                    //cut the string
                    sendSensortmp = sendSensortmp.substr(sendSensortmp.find(" ") + 1, sendSensortmp.length());
                    cerr << "TMP2 IS sendSensortmp " << sendSensortmp << endl;
                    //
                    string sensor4 = sendSensortmp.substr(0, sendSensortmp.find(" "));
                    double sensorFOUR = atof(sensor4.c_str());
                    cerr << "this is sensorFOUR " << sensorFOUR << endl;

                    //cut the string
                    sendSensortmp = sendSensortmp.substr(sendSensortmp.find(" ") + 1, sendSensortmp.length());
                    //
                    string sensor5 = sendSensortmp.substr(0, sendSensortmp.find(" "));
                    double sensorFIVE = atof(sensor5.c_str());
                    cerr << "this is sensorFIVE " << sensorFIVE << endl;

                    //USED FOR DEBUGGING
                    //Map the values for sending the sensors values.
//		map<int,double> sensorsMap;
//        sensorsMap.insert (pair<int,double>(0,sensorZERO) );
//        sensorsMap.insert (pair<int,double>(1,sensorONE) );
//        sensorsMap.insert (pair<int,double>(2,sensorTWO) );
//        sensorsMap.insert (pair<int,double>(3,sensorTHREE) );
//        sensorsMap.insert (pair<int,double>(4,sensorFOUR) );

                    // showing contents:
//        cerr << "sensorsMap contains:\n";
//        map<int,double>::iterator it;
//        for (it=sensorsMap.begin(); it!=sensorsMap.end(); ++it)
//            cerr << it->first << " => " << it->second << endl;

                    automotive::miniature::SensorBoardData sensorsData;
                    sensorsData.setNumberOfSensors(6);

                    sensorsData.putTo_MapOfDistances(0, sensorZERO);
                    sensorsData.putTo_MapOfDistances(1, sensorONE);
                    sensorsData.putTo_MapOfDistances(2, sensorTWO);
                    sensorsData.putTo_MapOfDistances(3, sensorTHREE);
                    sensorsData.putTo_MapOfDistances(4, sensorFOUR);
                    sensorsData.putTo_MapOfDistances(5, sensorFIVE);

                    Container sensors(sensorsData);
                    m_conference.send(sensors);

                }

            }

        }


    }
}