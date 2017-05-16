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
#include <string>
#include <memory>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>

#include "SerialReceiveBytes.hpp"

//for converting the string to double
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h> /* strtod */

using namespace std;

void SerialReceiveBytes::nextString(const string &s) {
	//cout << "original" << s << endl;

    int serialStart = s.find("{");
    int serialEnd = s.find("}");

    if(s.length() > 10 && !s.empty() && serialStart != -1 && serialEnd != -1){

 	serialStart = s.find("{");


	string tmp;
	serialEnd = 0;

	if((int) s.length() > (int) serialStart && (int) s.length() > 10){

            tmp = s.substr(serialStart, s.length());
            serialEnd = tmp.find("}");

	}

	if((int) serialEnd > (int) serialStart && (int) tmp.length() > 10){
	string serialValues = tmp.substr (1, serialEnd -1);
	cerr << "Received new" << serialValues.length() << " bytes containing '" << serialValues << "'" << endl;

		//we create this object in order to send the sensor data to the conference "handles the containers"
		//SensorBoardData sendData;

		//we create a temporal string
		string sendSensortmp = serialValues;

		//
		string sensor0 = sendSensortmp.substr(0, sendSensortmp.find(" "));

		//double sensorZERO = atof(sensor0);
		cerr << "this is sensorZERO" << sensor0 << endl;
		//cut the string
        string tmp2 = sendSensortmp.substr(sendSensortmp.find(" "), sendSensortmp.length());
        cerr << "TMP2 IS THIS " << tmp2 << endl;
		//sendSensortmp = sendSensortmp.substr(sendSensortmp.find(" "), sendSensortmp.length());
		//
		string sensor1 = tmp2.substr(0, tmp2.find(" "));
		//double sensorONE = atof(sensor1);
		cerr << "this is sensorONE" << sensor1 << endl;
		//cut the string
		sendSensortmp = sendSensortmp.substr(sendSensortmp.find(" "), sendSensortmp.length());
		//
		string sensor2 = sendSensortmp.substr(0, sendSensortmp.find(" "));
		//double sensorTWO = atof(sensorTWO);
		cerr << "this is sensorTWO" << sensor2 << endl;
		//cut the string
		sendSensortmp = sendSensortmp.substr(sendSensortmp.find(" "), sendSensortmp.length());
		//
		string sensor3 = sendSensortmp.substr(0, sendSensortmp.find(" "));
		//double sensorTHREE = atof(sensor3);
		cerr << "this is sensorTHREE" << sensor3 << endl;
		//cut the string
		sendSensortmp = sendSensortmp.substr(sendSensortmp.find(" "), sendSensortmp.length());
		//
		string sensor4 = sendSensortmp.substr(0, sendSensortmp.find(" "));
		//double sensorFOUR = atof(sensor4);
		cerr << "this is sensorFOUR" << sensor4 << endl;

		//Map dataMap;


	}

	}

    //cerr << "REAL ONE" << s.length() << " bytes containing '" << s << "'" << endl;
	
}


