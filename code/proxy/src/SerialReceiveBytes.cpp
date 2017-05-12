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

	if(s.length() > 13 && !s.empty()){


	//s.erase(remove(s.begin(), s.end(), '\n'), s.end());

 	int serialStart = s.find("{");
	
	
	string tmp;
	int serialEnd = 0;

	if((int) s.length() > (int) serialStart && (int) s.length() > 12){
	tmp = s.substr(serialStart, s.length());
	serialEnd = tmp.find("}");
	}

	
	//cout << "bigger than 10 " << endl;

	if((int) serialEnd > (int) serialStart && (int) tmp.length() > 12){
	string serialValues = tmp.substr (1, serialEnd -1);
	cerr << "Received new" << serialValues.length() << " bytes containing '" << serialValues << "'" << endl;
	}
	//cout << "Received " << s.length() << " bytes containing '" << s << "'" << endl;
	}
	
}


