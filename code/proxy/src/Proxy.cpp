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

#include "OpenCVCamera.h"

#ifdef HAVE_UEYE
    #include "uEyeCamera.h"
#endif

#include "Proxy.h"


namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odtools::recorder;

        // We add some of OpenDaVINCI's namespaces for the sake of readability.
        using namespace odcore;
        using namespace odcore::wrapper;

        //the serial communication
//        const string SERIAL_PORT = "/dev/ttyACM0"; //port that we will send -> arduino
//        const uint32_t BAUD_RATE = 9600;
        bool serialBool = false;
        std::shared_ptr<SerialPort> serial;

        //FOR TESTING WITH A FAKE SERIAL PORT

        //const string SERIAL_PORT = "/dev/pts/2";
        //const uint32_t BAUD_RATE = 19200;


        Proxy::Proxy(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "proxy"),
            m_recorder(),
            m_camera()
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

            //get the serial PortNumber and serial BaudRate
            const string Port = getKeyValueConfiguration().getValue<string>("proxy.Sensor.SerialPort");
            const uint32_t SerialSpeed = getKeyValueConfiguration().getValue<uint32_t>("proxy.Sensor.SerialSpeed");

            //make the serial connection. wait a second to make it work and start the serial
            try {
                if(!serialBool){

                    serial = std::shared_ptr<SerialPort>(SerialPortFactory::createSerialPort(Port, SerialSpeed));
                    const uint32_t ONE_SECOND = 1000 * 1000;
                    odcore::base::Thread::usleepFor(10 * ONE_SECOND);
                    // Start receiving bytes.
                    serial->start();
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
            if(serialBool){
                serial -> stop();
            }
        }

        void Proxy::sendSerial(string &number){

            try {
                //cerr << "Sending to SERIAL_PORT: " << Port << ", BAUD_RATE = " << SerialSpeed << endl;

                serial->send(number + "\r\n");

            }
            catch(string &exception) {
                cerr << "Serial port could not be created: " << exception << endl;
            }

        }

        void Proxy::distributeSerial(Container c) {
            // Store data to recorder.
            if (serialBool) {
                // Time stamp data before storing.
                c.setReceivedTimeStamp(TimeStamp());
                serial->store(c);
            }

            // Share data.
            getConference().send(c);
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

                // Get sensor data from IR/US.
            }

            cout << "Proxy: Captured " << captureCounter << " frames." << endl;

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature

