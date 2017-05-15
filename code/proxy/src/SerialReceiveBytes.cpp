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
	}

	}

    //cerr << "REAL ONE" << s.length() << " bytes containing '" << s << "'" << endl;
	
}


