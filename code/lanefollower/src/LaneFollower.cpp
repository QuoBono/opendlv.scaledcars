 /**
 * lanefollower - Sample application for following lane markings.
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

#include <iostream>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "opendavinci/odcore/base/KeyValueConfiguration.h"
#include "opendavinci/odcore/base/Lock.h"
#include "opendavinci/odcore/data/Container.h"
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/wrapper/SharedMemoryFactory.h"

#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"

 //this is for the serial communication
#include <stdint.h>
#include <iostream>
#include <string>
#include <memory>
#include <opendavinci/odcore/base/Thread.h>
#include <opendavinci/odcore/wrapper/SerialPort.h>
#include <opendavinci/odcore/wrapper/SerialPortFactory.h>
 //end for the serial communication

 //for MAT
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

 //for the serial value converter
#include <string>
#include <math.h>

#include "LaneFollower.h"



 //serial test
#include <iostream>
#include <fstream>

 // Used for debugging
#include <opendavinci/odtools/player/Player.h>




namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace odcore::data::image;
        using namespace automotive;
        using namespace automotive::miniature;

        // We add some of OpenDaVINCI's namespaces for the sake of readability.
        using namespace odcore;
        using namespace odcore::wrapper;


        cv::Mat m_image_black; //added grey image matrix
        cv::Mat m_image_black_new;


        LaneFollower::LaneFollower(const int32_t &argc, char **argv) : TimeTriggeredConferenceClientModule(argc, argv,
                                                                                                           "lanefollower"),
                                                                       m_hasAttachedToSharedImageMemory(false),
                                                                       m_sharedImageMemory(),
                                                                       m_image(NULL),
                                                                       m_debug(false),
                                                                       m_simulation(false), //added
                                                                       m_font(),
                                                                       m_previousTime(),
                                                                       m_eSum(0),
                                                                       m_eOld(0),
                                                                       proportionalGain(1.30), //added
                                                                       integralGain(0.01), //added
                                                                       derivativeGain(0.10), //added
                                                                       m_vehicleControl() {}

        LaneFollower::~LaneFollower() {}


        // set up before running body()
        void LaneFollower::setUp() {


            // This method will be call automatically _before_ running body().
            if (m_debug) {
                // Create an OpenCV- Debug window.
                cvNamedWindow("Debug screen", CV_WINDOW_AUTOSIZE); //Fixed size of debug screen
            }


        }

        void LaneFollower::tearDown() {
            // This method will be call automatically _after_ return from body().
            if (m_image != NULL) {
                cvReleaseImage(&m_image);
                m_image_black.deallocate();

                if (m_simulation) {
                    m_image_black_new.deallocate();
                }
            }

            if (m_debug) {
                cvDestroyWindow("Debug screen");
                cvDestroyWindow("Test screen");
            }


        }

        bool LaneFollower::readSharedImage(Container &c) {

            bool retVal = false;

            if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                SharedImage si = c.getData<SharedImage>();

                // Check if we have already attached to the shared memory.
                if (!m_hasAttachedToSharedImageMemory) {
                    m_sharedImageMemory = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(si.getName());
                }


                // Check if we could successfully attach to the shared memory.
                if (m_sharedImageMemory->isValid()) {

                    // Lock the memory region to gain exclusive access using a scoped lock.
                    Lock l(m_sharedImageMemory);
                    const uint32_t numberOfChannels = 3;

                    // For example, simply show the image.
                    if (m_image == NULL) {
                        m_image = cvCreateImage(cvSize(si.getWidth(), si.getHeight()), IPL_DEPTH_8U, numberOfChannels);
                    }


                    if (m_simulation) {
                        // if the simulation camera is true we need to mirror the image
                        if (m_image != NULL) {
                            memcpy(m_image->imageData,
                                   m_sharedImageMemory->getSharedMemory(),
                                   si.getWidth() * si.getHeight() * numberOfChannels);
                            cv::Mat m_image_temp = cv::cvarrToMat(m_image);
                            cv::cvtColor(m_image_temp, m_image_black, cv::COLOR_BGR2GRAY);
                            Canny(m_image_black, m_image_black, 100, 200, 3);

                            // Mirror the image.
                            cv::flip(m_image_black, m_image_black_new, -1);
                            m_image_black = m_image_black_new.clone();
                        }
                    } else {
                        //
                        if (m_image != NULL) {
                            memcpy(m_image->imageData,
                                   m_sharedImageMemory->getSharedMemory(),
                                   si.getWidth() * si.getHeight() * numberOfChannels);
                            cv::Mat m_image_temp = cv::cvarrToMat(m_image);
                            cv::cvtColor(m_image_temp, m_image_black, cv::COLOR_BGR2GRAY);
                            Canny(m_image_black, m_image_black, 100, 200, 3);
                        }
                    }


                    retVal = true;
                }
            }
            return retVal;
        }

        void LaneFollower::processImage(Container &containerVehicleControl) {

            //boolean for the right lane marking
            static bool useRightLaneMarking = true;

            //Cross Track Error
            double e = 0;
            const int32_t distance = 280; //280

            double rightPixelAverage = 0;
            double leftPixelAverage = 0;

            double leftPixelResult = 0;
            double rightPixelResult = 0;

            int counterAverageLeft = 0;
            int counterAverageRight = 0;

            TimeStamp beforeImageProcessing;

            TimeStamp currentTime;
            double timeStep = (currentTime.toMicroseconds() - m_previousTime.toMicroseconds()) / (1000.0 * 1000.0);
            m_previousTime = currentTime;


            //Check the pixels fromm the top of the car
            uchar pixelTopLeft;
            cv::Point leftTop;
            leftTop.y = 445;
            leftTop.x = m_image_black.cols / 2 + 55; //x position of the straight line

            uchar pixelTopRight;
            cv::Point rightTop;
            rightTop.y = 445;
            rightTop.x = m_image_black.cols / 2 - 55; //x position of the straight line

            //Find the pixel in the straight line from starting from the middle-ish
            for (int y = 445; y > 0; y--) {
                pixelTopLeft = m_image_black.at<uchar>(cv::Point(m_image_black.cols / 2 , y));
                if (pixelTopLeft > 200) {
                    leftTop.y = y;
                    break;
                }
            }


            //Find the pixel in the straight line from starting from the middle-ish
            for (int y = 445; y > 0; y--) {
                pixelTopRight = m_image_black.at<uchar>(cv::Point(m_image_black.cols / 2 , y));
                if (pixelTopRight > 200) {
                    rightTop.y = y;
                    break;
                }
            }

            if (leftTop.y > 0) {
                //Draw the line
                cv::Scalar white = CV_RGB(255, 255, 255);
                line(m_image_black, cv::Point(leftTop.x, m_image_black.rows - 40), leftTop, white);

                //text and value of the line.
                stringstream sstr;
                sstr << (leftTop.y - 445);
                putText(m_image_black, sstr.str().c_str(), cv::Point(leftTop.x, leftTop.y), cv::FONT_HERSHEY_PLAIN,
                        0.5, white);
            }

            if (rightTop.y > 0) {
                //Draw the line
                cv::Scalar white = CV_RGB(255, 255, 255);
                line(m_image_black, cv::Point(rightTop.x, m_image_black.rows - 40), rightTop, white);

                //text and value of the line.
                stringstream sstr;
                sstr << (rightTop.y - 445);
                putText(m_image_black, sstr.str().c_str(), cv::Point(rightTop.x, rightTop.y), cv::FONT_HERSHEY_PLAIN,
                        0.5, white);
            }



            //This for loop will iterate for the scanline (each Y is the y of the line we draw)
            for (int32_t y = m_image_black.rows - 10; y > 445; y -= 4) {
                //cerr << "this is y: " << y << endl;
                uchar pixelLeft;
                cv::Point left;
                left.y = y;
                left.x = -1;

                //Check the middle-left side of lane.
                for (int x = m_image_black.cols / 2; x > 0; x--) {
                    pixelLeft = m_image_black.at<uchar>(cv::Point(x, y));
                    if (pixelLeft >= 200) {
                        left.x = x;
                        //Increment the counter for the average
                        counterAverageLeft++;
                        //Add the value of the left pixel to the average variable
                        leftPixelAverage = leftPixelAverage + left.x;
                        //Create the average result
                        leftPixelResult = leftPixelAverage / counterAverageLeft;
                        break;
                    }
                }


                uchar pixelRight;
                cv::Point right;
                right.y = y;
                right.x = -1;

                //Check the middle-right side of lane.
                for (int x = m_image_black.cols / 2; x < m_image_black.cols; x++) {
                    pixelRight = m_image_black.at<uchar>(cv::Point(x, y));
                    if (pixelRight >= 200) {
                        right.x = x;
                        //Increment the counter for the average
                        counterAverageRight++;
                        //Add the value of the left pixel to the average variable
                        rightPixelAverage = rightPixelAverage + right.x;
                        //Create the average result
                        rightPixelResult = rightPixelAverage / counterAverageRight;

                        break;
                    }
                }




                //draw line for the the left and right lane if debug is true
                if (m_debug) {

                    if (left.x > 0) {
                        //Draw the line
                        cv::Scalar white = CV_RGB(255, 255, 255);
                        cv::Scalar pink = CV_RGB(204, 0, 102);
                        line(m_image_black, cv::Point(m_image_black.cols / 2, y), left, white);

                        //text and value of the line. (Font is pink, white will cause problems when detecting white pixel)
                        stringstream sstr;
                        sstr << (m_image_black.cols / 2 - left.x);
                        putText(m_image_black, sstr.str().c_str(), cv::Point(m_image_black.cols / 2 - 100, y - 1),
                                cv::FONT_HERSHEY_PLAIN,
                                0.4, pink);
                    }

                    if (right.x > 0) {
                        //Draw the Line
                        cv::Scalar pink = CV_RGB(204, 0, 102);
                        line(m_image_black, cv::Point(m_image_black.cols / 2, y), right, pink);

                        //text and value of the line.
                        stringstream sstr;
                        sstr << (right.x - m_image_black.cols / 2);
                        putText(m_image_black, sstr.str().c_str(), cv::Point(m_image_black.cols / 2 + 100, y - 2),
                                cv::FONT_HERSHEY_PLAIN,
                                0.4, pink);
                    }
                }


            }

            cerr << "This is the average right pixel" << rightPixelResult << endl;
            cerr << "This is the average left pixel" << leftPixelResult << endl;




            // Shift the whole perception of the image 30px to the right,
            // this helps with keeping the right lane marking in picture.
            if (rightPixelResult > 0) rightPixelResult += 30;

            // Calculate the deviation error.
            if (rightPixelResult > 30) {
                if (!useRightLaneMarking) {
                    m_eSum = 0;
                    m_eOld = 0;
                }
                //double pixSum = (secondright.x + right.x) / 2.0;
                e = ((rightPixelResult - m_image_black.cols / 2.0) - distance) / distance;

                useRightLaneMarking = true;

            } else if (leftPixelResult > 0) {
                if (useRightLaneMarking) {
                    m_eSum = 0;
                    m_eOld = 0;
                }

                e = (distance - (m_image_black.cols / 2.0 - leftPixelResult)) / distance;

                useRightLaneMarking = false;
            } else {
                m_eSum = 0;
                m_eOld = 0;

            }

            TimeStamp afterImageProcessing;
            //cerr << "Processing time: " << (afterImageProcessing.toMicroseconds() - beforeImageProcessing.toMicroseconds())/1000.0 << "ms." << endl;

            // Show resulting features.
            if (m_debug) {
                if (m_image != NULL) {
                    imshow("Camera Original Image", m_image_black);
                    cv::waitKey(10);
                    cvWaitKey(10); //we need a wait key
                }
            }

            //used for the Algorithm
            if (fabs(e) < 1e-100) {
                m_eSum = 0;
            } else {
                m_eSum += e;
            }


            // The following values have been determined by Twiddle algorithm

            const double p = proportionalGain * e;
            const double i = integralGain * timeStep * m_eSum;
            const double d = derivativeGain * (e - m_eOld) / timeStep;
            m_eOld = e;

            double y = p + i + d; //before y

            double desiredSteering = 0;

            //If the car doesn't detect any border then the car will go straight.
            if (rightPixelResult < 0 && leftPixelResult < 0) {
                desiredSteering = 0;

                //If the car detects a stopline then the car should stop
                if (fabs(leftTop.y - rightTop.y) < 5 && rightTop.y > 300 && leftTop.y > 300) {

                    m_vehicleControl.setSpeed(0);
                    m_vehicleControl.setSteeringWheelAngle(0);
                    cerr << "STOPLINE DETECTED" << endl;

                }

            }

            // If the absolute value of the Cross TRack Error 'e' is bigger than 0.002 then we use the PID for steering
            if (fabs(e) > 1e-100) {
                desiredSteering = y; //before y
            }


            cerr << "PID: " << "e = " << e << ", eSum = " << m_eSum << ", desiredSteering = " << desiredSteering
                 << ", y = " << y << endl;

            // Go forward.
            //for SIM
//            if(m_simulation){
//                if(fabs(leftTop.y - rightTop.y) < 5 && rightTop.y > 300 && leftTop.y > 300){
//
//                    if(sleep(3)){
//
//                        if (m_debug) {
//                            if (m_image != NULL) {
//
//                                imshow("Camera Original Image", m_image_black);
//                                cv::waitKey(10);
//                                cvWaitKey(10); //we need a wait key
//                            }
//                        }
//                        m_vehicleControl.setSpeed(0);
//                        m_vehicleControl.setSteeringWheelAngle(0);
//
//                    }
//
//                }
//
//            }

            // Get the  Vehicle control container. We use it to communicate with Overtaker.
            VehicleControl vc1 = containerVehicleControl.getData<VehicleControl>();


            //Here we detect the stopline and tell he car to stop.

//                if (fabs(leftTop.y - rightTop.y) < 5 && rightTop.y > 300 && leftTop.y > 300) {
//
//                    m_vehicleControl.setSpeed(0);
//                    m_vehicleControl.setSteeringWheelAngle(0);
//                    cerr << "STOPLINE DETECTED" << endl;
//
//                }

                m_vehicleControl.setSteeringWheelAngle(desiredSteering);
                m_vehicleControl.setSpeed(1);


//            else {
//
//                m_vehicleControl.setSteeringWheelAngle(vd.getSteeringWheelAngle());
//                m_vehicleControl.setSpeed(vd.getSpeed());
//
//                cerr << "this is overtaker" << lane << endl;
//            }

    }//end of process Image


        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode LaneFollower::body() {

            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();
            //Get Debug Screen
            m_debug = kv.getValue<int32_t> ("lanefollower.debug") == 1;

            //Choose if it's simulation cam or real cam
            m_simulation = getKeyValueConfiguration().getValue<uint32_t>("lanefollower.Simulation") == 1;

            // Initialize fonts.
            const double hscale = 0.4;
            const double vscale = 0.3;
            const double shear = 0.2;
            const int thickness = 1;
            const int lineType = 6;

            cvInitFont(&m_font, CV_FONT_HERSHEY_DUPLEX, hscale, vscale, shear, thickness, lineType);


            // Overall state machine handler.
            //while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
            Container containerVehicleControl = getKeyValueDataStore().get(automotive::VehicleControl::ID());


            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                bool has_next_frame = false;

                // Get the most recent available container for a SharedImage.
                Container c = getKeyValueDataStore().get(odcore::data::image::SharedImage::ID());

                // Get most recent data from Vehicle-Control (for communication with overtake)
                containerVehicleControl = getKeyValueDataStore().get(automotive::VehicleControl::ID());


                //PLAYING WITH SENSORS
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                double sensor0 = sbd.getValueForKey_MapOfDistances(0);
                cerr << "THIS IS FROM LANEFOLLOWER THE SENSOR IS " << sensor0 << endl;
                double sensor1 = sbd.getValueForKey_MapOfDistances(1);
                cerr << "THIS IS FROM LANEFOLLOWER THE SENSOR IS " << sensor1 << endl;
                double sensor2 = sbd.getValueForKey_MapOfDistances(2);
                cerr << "THIS IS FROM LANEFOLLOWER THE SENSOR IS " << sensor2 << endl;
                double sensor3 = sbd.getValueForKey_MapOfDistances(3);
                cerr << "THIS IS FROM LANEFOLLOWER THE SENSOR IS " << sensor3 << endl;
                double sensor4 = sbd.getValueForKey_MapOfDistances(4);
                cerr << "THIS IS FROM LANEFOLLOWER THE SENSOR IS " << sensor4 << endl;
                double sensor5 = sbd.getValueForKey_MapOfDistances(5);
                cerr << "THIS IS FROM LANEFOLLOWER THE SENSOR IS " << sensor5 << endl;
                //END OF PLAYING WITH SENSORS

                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData>();
                double isLaneFollower = vd.getHeading();
                cerr << "THE HEADING IS " << isLaneFollower << endl;



                if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                    // Example for processing the received container.
                    has_next_frame = readSharedImage(c);
                }

                // Process the read image and calculate regular lane following set values for control algorithm.
                if (true == has_next_frame) {
                    processImage(containerVehicleControl);
                }
                // Create container for finally sending the set values for the control algorithm.
                Container c2(m_vehicleControl);
                // Send container.
                //if(vd.getHeading()>0){
                    cerr << "Lanefollower is ON" << endl;
                    getConference().send(c2);
                //} else {
                //    cerr << "Lanefollower is OFF" << endl;
                //}
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature

