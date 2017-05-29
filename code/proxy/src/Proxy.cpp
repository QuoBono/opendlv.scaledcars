/**
 * proxy - Sample application to encapsulate HW/SW interfacing with embedded systems.
 * Copyright (C) 2012 - 2015 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#include <ctype.h>
#include <cstring>
#include <cmath>
#include <iostream>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/data/TimeStamp.h"

//this is for the serial communication
#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
//end for the serial communication

//serial test
#include <iostream>
#include <fstream>
//end for serial test


#include "OpenCVCamera.h"

#ifdef HAVE_UEYE
    #include "uEyeCamera.h"
#endif

#include "Proxy.h"

//this is the string listener.
#include <opendavinci/odcore/io/StringListener.h>
#include <SerialReceiveBytes.hpp>

struct CarData {
    unsigned char speedOfCar : 3;
    unsigned char steeringOfCar : 5;
} arduinoCar;

namespace automotive {
    namespace miniature {
        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odtools::recorder;

        // We add some of OpenDaVINCI's namespaces for the sake of readability.
        using namespace odcore;
        using namespace odcore::wrapper;

        bool serialBool = false;//boolean for the beginning of the connection

        std::shared_ptr<SerialPort> serial; //used to create the serial


        Proxy::Proxy(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "proxy"),
            m_recorder(),
            m_camera(),
            serialReceiveBytes(getConference())
        {}

        Proxy::~Proxy() {
        }

        void Proxy::setUp() {
            // This method will be call automatically _before_ running body().
            if (getFrequency() < 20) {
                cerr << endl << endl << "Proxy: WARNING! Running proxy with a LOW frequency (consequence: data updates are too seldom and will influence your algorithms in a negative manner!) --> suggestions: --freq=20 or higher! Current frequency: " << getFrequency() << " Hz." << endl << endl << endl;
            }

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();

            // Create built-in recorder.
            const bool useRecorder = kv.getValue<uint32_t>("proxy.useRecorder") == 1;
            if (useRecorder) {
                // URL for storing containers.
                stringstream recordingURL;
                recordingURL << "file://" << "proxy_" << TimeStamp().getYYYYMMDD_HHMMSS_noBlankNoColons() << ".rec";
                // Size of memory segments.
                const uint32_t MEMORY_SEGMENT_SIZE = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.memorySegmentSize");
                // Number of memory segments.
                const uint32_t NUMBER_OF_SEGMENTS = getKeyValueConfiguration().getValue<uint32_t>("global.buffer.numberOfMemorySegments");
                // Run recorder in asynchronous mode to allow real-time recording in background.
                const bool THREADING = true;
                // Dump shared images and shared data?
                const bool DUMP_SHARED_DATA = getKeyValueConfiguration().getValue<uint32_t>("proxy.recorder.dumpshareddata") == 1;

                m_recorder = unique_ptr<Recorder>(new Recorder(recordingURL.str(), MEMORY_SEGMENT_SIZE, NUMBER_OF_SEGMENTS, THREADING, DUMP_SHARED_DATA));
            }

            // Create the camera grabber.
            const string NAME = getKeyValueConfiguration().getValue<string>("proxy.camera.name");
            string TYPE = getKeyValueConfiguration().getValue<string>("proxy.camera.type");
            std::transform(TYPE.begin(), TYPE.end(), TYPE.begin(), ::tolower);
            const uint32_t ID = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.id");
            const uint32_t WIDTH = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.width");
            const uint32_t HEIGHT = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.height");
            const uint32_t BPP = getKeyValueConfiguration().getValue<uint32_t>("proxy.camera.bpp");

            if (TYPE.compare("opencv") == 0) {
                m_camera = unique_ptr<Camera>(new OpenCVCamera(NAME, ID, WIDTH, HEIGHT, BPP));
            }
            if (TYPE.compare("ueye") == 0) {
#ifdef HAVE_UEYE
                m_camera = unique_ptr<Camera>(new uEyeCamera(NAME, ID, WIDTH, HEIGHT, BPP));
#endif
            }

            if (m_camera.get() == NULL) {
                cerr << "No valid camera type defined." << endl;
            }

            //get the serial PortNumber and serial BaudRate from the configuration file.
            const string Port = getKeyValueConfiguration().getValue<string>("proxy.Sensor.SerialPort");
            const uint32_t SerialSpeed = getKeyValueConfiguration().getValue<uint32_t>("proxy.Sensor.SerialSpeed");

            //make the serial connection. wait a second to make it work and start the serial
            try {
                if(!serialBool){

                    //create the serial port and wait one second for the connection to succeed
                    serial = std::shared_ptr<SerialPort>(SerialPortFactory::createSerialPort(Port, SerialSpeed));
                    const uint32_t ONE_SECOND = 1000 * 1000;
                    odcore::base::Thread::usleepFor(10 * ONE_SECOND);

                    //read received values from the serial.
                    serial->setStringListener(&serialReceiveBytes);

                    // Start receiving bytes.
                    serial->start();

                    //Serial Connection is true
                    serialBool = true;
                }

                cerr << "Setup with SERIAL_PORT: " << Port << ", BAUD_RATE = " << SerialSpeed << endl;

            }
            catch(string &exception) {
                cerr << "Set up error Serial port could not be created: " << exception << endl;
            }





        }

        void Proxy::tearDown() {
            // This method will be call automatically _after_ return from body().
            //stop the serial connection
            if (serialBool){
                    serial -> stop();
                    serial->setStringListener(NULL);
                    cerr << "Proxy stopped - Serial Closed" << endl;

            }

        }

        void Proxy::distribute(Container c) {
            // Store data to recorder.
            if (m_recorder.get() != NULL) {
                // Time stamp data before storing.
                c.setReceivedTimeStamp(TimeStamp());
                m_recorder->store(c);

            }
            // Share data.
            getConference().send(c);

        }

        void Proxy::readContainer(odcore::data::Container &c) {
            //Original Vehicle Data stored from other components e.g. lanefollower
            VehicleControl vc = c.getData<VehicleControl> ();
            //cerr << "THIS IS THE ANGLE " << vc.getSteeringWheelAngle() << endl;

            //Here we transform from Radians to Degrees.
            int carSteering = (int) (vc.getSteeringWheelAngle()*(180/M_PI));
            //Increment 70 for the servo, because 0 degrees is is 70
            carSteering += 70;


            if(carSteering < 10){
                carSteering = 10;
            }

            if(carSteering > 90){
                carSteering = 90;
            }

            carSteering = carSteering / 6; //because we can only take up to 30 in 5 bits


            string steer = to_string(carSteering);



            //cerr << "THIS IS THE our made up ANGLE " << steer << " In degrees is: " << carSteering <<  endl;


            //Here we get the Vehicle Speed.
            int carSpeed = (int) (vc.getSpeed());

            string speed = to_string(carSpeed);
            //cerr << "THIS IS THE speed: " << speed << endl;

            //add the speed and steering
            string serialValues = "T" + speed + "Q" + steer;

            //cerr << "This is the bytes "  << serialValues.length() << endl;
            cerr << serialValues << endl;




            //ALTERNATIVE Version Sending Bytes.
            uint8_t bytes[3]; //utf16 characters
            unsigned int cSteer = carSteering;
            unsigned int cSpeed = carSpeed;


            //This way we assign the steering to one byte as a hexadecimal.
            bytes[0] = cSteer & 0xFF;
            bytes[1] = cSpeed & 0xFF;

            //This way we include two integers in a unsigned 16 bit integer.
            bytes[3] = (cSteer << 5);
            bytes[3] |= cSpeed;

            //Used for debugging.
            //printf("Steering in hexadecimal is %x\n", bytes[0]);
            //printf("Speed in hexadecimal is %x\n", bytes[1]);

            //Here we read the values for the steering. NOTE   : structs help read and help organize the bits
            arduinoCar.steeringOfCar =  (bytes[3]>>5);
            //printf("Steering in another way with struct is %d\n", arduinoCar.steeringOfCar);
            //Here we read the values for the speed.
            arduinoCar.speedOfCar =  bytes[3];
            //printf("Speed in another way is %d\n", arduinoCar.speedOfCar);



            //Here we send the wheel steering angle to the arduino/serial.
            if(serialBool) {

                try {

                    //Version for sending as a string.
                    //serial->send(serialValues + "\r\n");

                    //Version for sending two separate bytes one for speed another for steering
                    //serial->send(string(1, bytes[0]) + string(1, bytes[1]) + "\r\n");

                    //Version for sending in one byte the speed and steering. (use structs to read from the arduino)
                    serial->send(string(1, bytes[3]) + "\n");


                } catch (string &exception) {
                    cerr << "Serial port could not be created: " << exception << endl;

                }



            }
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Proxy::body() {
            uint32_t captureCounter = 0;
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // Capture frame.
                if (m_camera.get() != NULL) {
                    odcore::data::image::SharedImage si = m_camera->capture();

                    Container c(si);
                    distribute(c);
                    captureCounter++;
                }


                //read the vehicle container
                Container car = getKeyValueDataStore().get(automotive::VehicleControl::ID());
                readContainer(car);


            }



            cout << "Proxy: Captured " << captureCounter << " frames." << endl;

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    } //miniature
} // automotive::miniature

