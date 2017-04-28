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


//for MAT
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


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


        LaneFollower::LaneFollower(const int32_t &argc, char **argv) : TimeTriggeredConferenceClientModule(argc, argv, "lanefollower"),
                                                                       m_hasAttachedToSharedImageMemory(false),
                                                                       m_sharedImageMemory(),
                                                                       m_image(NULL),
                                                                       m_debug(false),
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

                //Test stuff
                //cvNamedWindow("Test screen", CV_WINDOW_AUTOSIZE);
                //cvMoveWindow("Test screen", 1000 + m_image->width + 5, 100);

            }


        }

        void LaneFollower::tearDown() {
            // This method will be call automatically _after_ return from body().
            if (m_image != NULL) {
                cvReleaseImage(&m_image);
                m_image_black.deallocate();
                m_image_black_new.deallocate();
            }

            if (m_debug) {
                cvDestroyWindow("Debug screen");
                cvDestroyWindow("Test screen");


            }

        }

        bool LaneFollower::readSharedImage(Container &c) {

            bool retVal = false;

            if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                SharedImage si = c.getData<SharedImage> ();

                // Check if we have already attached to the shared memory.
                if (!m_hasAttachedToSharedImageMemory) {
                    m_sharedImageMemory
                            = odcore::wrapper::SharedMemoryFactory::attachToSharedMemory(
                            si.getName());
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

                    // Copying the image data is very expensive...
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



                    retVal = true;
                }
            }
            return retVal;
        }

        void LaneFollower::processImage() {

            //boolean for the right lane marking
            static bool useRightLaneMarking = true;

            //Cross Track Error
            double e = 0;

            const int32_t CONTROL_SCANLINE = 460; //462 calibrated length to right: 280px
            //const int32_t SECOND_CONTROL_SCANLINE = 450;
            const int32_t distance = 280; //280

            TimeStamp beforeImageProcessing;



            //scanline= 222; you can use this value for the stop line detection

            //complexity 0^2 not good
            //This for loop will iterate for the scanline (each Y is the y of the line we draw
            for(int32_t y = m_image_black.rows; y > 220; y-= 10) {
                cerr << "this is y: " << y << endl;
                uchar pixelLeft;
                cv::Point left;
                left.y = y;
                left.x = -1;
                for(int x = m_image_black.cols /2; x > 0; x--){
                    pixelLeft = m_image_black.at<uchar>(cv::Point(x, y));
                    if (pixelLeft > 200) {
                        left.x = x;
                        break;
                    }
                }

//                cv::Point secondLeft;
//                secondLeft.y = y;
//                secondLeft.x = -1;
//
//                if (y == SECOND_CONTROL_SCANLINE){
//                    if(left.x > 0){
//                        secondLeft.x = left.x;
//                    }
//                }


                // Search from middle to the right:
                //CvScalar pixelRight;
                //CvPoint right;

                uchar pixelRight;
                cv::Point right;
                right.y = y;
                right.x = -1;

                //check right
                for(int x = m_image_black.cols/2; x < m_image_black.cols; x++) {
                    pixelRight = m_image_black.at<uchar>(cv::Point(x, y));
                    if (pixelRight > 200) {
                        right.x = x;
                        break;
                    }
                }
//
//                cv::Point secondright;
//                secondright.y = y;
//                secondright.x = -1;
//
//                if (y == SECOND_CONTROL_SCANLINE){
//                    if(right.x > 0){
//                        secondright.x = right.x;
//                    }
//                }



                //draw line for the the left and right lane if debug is true
                if (m_debug) {
//                //draw line from the middle to left pixel
//                if (left.x > 0) {
//                    cv::Scalar white = CV_RGB(255, 255, 255);
//                    line(m_image_black, cv::Point(m_image_black.cols/ 2, y), left, white);
//
//                    //text and value of the line to the
//                    stringstream sstr;
//                    sstr << (m_image_black.cols / 2 - left.x);
//                    putText(m_image_black, sstr.str().c_str(), cv::Point(m_image_black.cols/2 - 100, y - 2), cv::FONT_HERSHEY_PLAIN,
//                            0.5, white);
//                }
//
//                if (right.x > 0) {
//                    cv::Scalar pink = CV_RGB(204, 0, 102);
//                    line(m_image_black, cv::Point(m_image_black.cols/2, y), right, pink);
//
//                    stringstream sstr;
//                    sstr << (right.x - m_image_black.cols/2);
//                    putText(m_image_black, sstr.str().c_str(), cv::Point(m_image_black.cols/2 + 100, y - 2), cv::FONT_HERSHEY_PLAIN,
//                            0.5, pink);
//                }

                    if(y == CONTROL_SCANLINE){
                        if (left.x > 0) {
                            cv::Scalar white = CV_RGB(255, 255, 255);
                            line(m_image_black, cv::Point(m_image_black.cols/ 2, y), left, white);

                            //text and value of the line to the
                            stringstream sstr;
                            sstr << (m_image_black.cols / 2 - left.x);
                            putText(m_image_black, sstr.str().c_str(), cv::Point(m_image_black.cols/2 - 100, y - 2), cv::FONT_HERSHEY_PLAIN,
                                    0.5, white);
                        }

                        if (right.x > 0) {
                            cv::Scalar pink = CV_RGB(204, 0, 102);
                            line(m_image_black, cv::Point(m_image_black.cols/2, y), right, pink);

                            stringstream sstr;
                            sstr << (right.x - m_image_black.cols/2);
                            putText(m_image_black, sstr.str().c_str(), cv::Point(m_image_black.cols/2 + 100, y - 2), cv::FONT_HERSHEY_PLAIN,
                                    0.5, pink);
                        }

                    }
//
//                    if (secondLeft.x > 0) {
//                        cv::Scalar white = CV_RGB(255, 255, 255);
//                        line(m_image_black, cv::Point(m_image_black.cols/ 2, y), secondLeft, white);
//
//                        //text and value of the line to the
//                        stringstream sstr;
//                        sstr << (m_image_black.cols / 2 - secondLeft.x);
//                        putText(m_image_black, sstr.str().c_str(), cv::Point(m_image_black.cols/2 - 100, y - 2), cv::FONT_HERSHEY_PLAIN,
//                                0.5, white);
//                    }
//
//                    if (secondright.x > 0) {
//                        cv::Scalar pink = CV_RGB(204, 0, 102);
//                        line(m_image_black, cv::Point(m_image_black.cols/2, y), secondright, pink);
//
//                        stringstream sstr;
//                        sstr << (secondright.x - m_image_black.cols/2);
//                        putText(m_image_black, sstr.str().c_str(), cv::Point(m_image_black.cols/2 + 100, y - 2), cv::FONT_HERSHEY_PLAIN,
//                                0.5, pink);
//                    }
                }



                //If the loop is currently checking at the height of each iteration line (each line that we see).
                if (y == CONTROL_SCANLINE) {
                    // Calculate the deviation error.
                    if (right.x > 0) {
                        if (!useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                        }

                        e = ((right.x - m_image_black.cols/2.0) - distance)/distance;

                        useRightLaneMarking = true;
                    }
                    else if (left.x > 0) {
                        if (useRightLaneMarking) {
                            m_eSum = 0;
                            m_eOld = 0;
                            e = ((left.x - m_image_black.cols/2.0) - distance)/distance;
                        }

                        e = (distance - (m_image_black.cols/2.0 - left.x))/distance;

                        useRightLaneMarking = false;
                    } else {
//                    {
//                        // If no measurements are available, reset PID controller.
//                        if(secondright.x){
//                            if (!useRightLaneMarking) {
//                                m_eSum = 0;
//                                m_eOld = 0;
//                            }
//
//                            e = ((secondright.x - m_image_black.cols/2.0) - distance)/distance;
//
//                            useRightLaneMarking = true;
//                        } else if (secondLeft.x > 0) {
//                            if (useRightLaneMarking) {
//                                m_eSum = 0;
//                                m_eOld = 0;
//                            }
//
//                            e = (distance - (m_image_black.cols/2.0 - secondLeft.x))/distance;
//
//                            useRightLaneMarking = false;
//                        } else
//                        {
                            m_eSum = 0;
                            m_eOld = 0;
//                        }
                    }


                }







            } //commented for now

            TimeStamp afterImageProcessing;
            cerr << "Processing time: " << (afterImageProcessing.toMicroseconds() - beforeImageProcessing.toMicroseconds())/1000.0 << "ms." << endl;

            // Show resulting features.
            if (m_debug) {
                if (m_image != NULL) {

                    imshow("Camera Original Image", m_image_black);
                    cv::waitKey(10);
                    cvWaitKey(10); //we need a wait key
                }
            }

            TimeStamp currentTime;
            double timeStep = (currentTime.toMicroseconds() - m_previousTime.toMicroseconds()) / (1000.0 * 1000.0);
            m_previousTime = currentTime;

            //used for the Algorithm
            if (fabs(e) < 1e-100) {
                m_eSum = 0;
            }
            else {
                m_eSum += e;
            }


            // The following values have been determined by Twiddle algorithm

            const double p = proportionalGain * e;
            const double i = integralGain * timeStep * m_eSum;
            const double d = derivativeGain * (e - m_eOld)/timeStep;
            m_eOld = e;

            const double y = p + i + d; //before y

            double desiredSteering = 0;

            // If the absolute value of the Cross TRack Error 'e' is bigger than 0.002 then we use the PID for steering
            if (fabs(e) > 1e-100) {
                desiredSteering = y; //before y
            }

            if (desiredSteering > 25.0) {
                desiredSteering = 25.0;
            }

            if (desiredSteering < -25.0) {
                desiredSteering = -25.0;
            }
            cerr << "PID: " << "e = " << e << ", eSum = " << m_eSum << ", desiredSteering = " << desiredSteering << ", y = " << y << endl;
            // We are using OpenDaVINCI's std::shared_ptr to automatically
            // release any acquired resources.


            // Go forward.
            //for SIM
            m_vehicleControl.setSpeed(10);
            m_vehicleControl.setSteeringWheelAngle(desiredSteering);





        }

        // This method will do the main data processing job.
        // Therefore, it tries to open the real camera first. If that fails, the virtual camera images from camgen are used.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode LaneFollower::body() {
            // Get configuration data.
            KeyValueConfiguration kv = getKeyValueConfiguration();
            m_debug = kv.getValue<int32_t> ("lanefollower.debug") == 1;

            // Initialize fonts.
            const double hscale = 0.4;
            const double vscale = 0.3;
            const double shear = 0.2;
            const int thickness = 1;
            const int lineType = 6;

            cvInitFont(&m_font, CV_FONT_HERSHEY_DUPLEX, hscale, vscale, shear, thickness, lineType);

            // Parameters for overtaking.
            const int32_t ULTRASONIC_FRONT_CENTER = 3;
            const int32_t ULTRASONIC_FRONT_RIGHT = 4;
            const int32_t INFRARED_FRONT_RIGHT = 0;
            const int32_t INFRARED_REAR_RIGHT = 2;

            const double OVERTAKING_DISTANCE = 5.5;
            const double HEADING_PARALLEL = 0.04;

            // Overall state machines for moving and measuring.
            enum StateMachineMoving { FORWARD, TO_LEFT_LANE_LEFT_TURN, TO_LEFT_LANE_RIGHT_TURN, CONTINUE_ON_LEFT_LANE, TO_RIGHT_LANE_RIGHT_TURN, TO_RIGHT_LANE_LEFT_TURN };
            enum StateMachineMeasuring { DISABLE, FIND_OBJECT_INIT, FIND_OBJECT, FIND_OBJECT_PLAUSIBLE, HAVE_BOTH_IR, HAVE_BOTH_IR_SAME_DISTANCE, END_OF_OBJECT };

            StateMachineMoving stageMoving = FORWARD;
            StateMachineMeasuring stageMeasuring = FIND_OBJECT_INIT;

            // State counter for dynamically moving back to right lane.
            int32_t stageToRightLaneRightTurn = 0;
            int32_t stageToRightLaneLeftTurn = 0;

            // Distance variables to ensure we are overtaking only stationary or slowly driving obstacles.
            double distanceToObstacle = 0;
            double distanceToObstacleOld = 0;

            // Overall state machine handler.
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                bool has_next_frame = false;

                // Get the most recent available container for a SharedImage.
                Container c = getKeyValueDataStore().get(odcore::data::image::SharedImage::ID());

                if (c.getDataType() == odcore::data::image::SharedImage::ID()) {
                    // Example for processing the received container.
                    has_next_frame = readSharedImage(c);
                }

                // Process the read image and calculate regular lane following set values for control algorithm.
                if (true == has_next_frame) {
                    processImage();
                }


                // Overtaking part.
                {
                    // 1. Get most recent vehicle data:
                    Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                    VehicleData vd = containerVehicleData.getData<VehicleData> ();

                    // 2. Get most recent sensor board data:
                    Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                    SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                    // Moving state machine.
                    if (stageMoving == FORWARD) {
                        // Use m_vehicleControl data from image processing.

                        stageToRightLaneLeftTurn = 0;
                        stageToRightLaneRightTurn = 0;
                    }
                    else if (stageMoving == TO_LEFT_LANE_LEFT_TURN) {
                        // Move to the left lane: Turn left part until both IRs see something.
                        m_vehicleControl.setSpeed(1);
                        m_vehicleControl.setSteeringWheelAngle(-25);

                        // State machine measuring: Both IRs need to see something before leaving this moving state.
                        stageMeasuring = HAVE_BOTH_IR;

                        stageToRightLaneRightTurn++;
                    }
                    else if (stageMoving == TO_LEFT_LANE_RIGHT_TURN) {
                        // Move to the left lane: Turn right part until both IRs have the same distance to obstacle.
                        m_vehicleControl.setSpeed(1);
                        m_vehicleControl.setSteeringWheelAngle(25);

                        // State machine measuring: Both IRs need to have the same distance before leaving this moving state.
                        stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE;

                        stageToRightLaneLeftTurn++;
                    }
                    else if (stageMoving == CONTINUE_ON_LEFT_LANE) {
                        // Move to the left lane: Passing stage.

                        // Use m_vehicleControl data from image processing.

                        // Find end of object.
                        stageMeasuring = END_OF_OBJECT;
                    }
                    else if (stageMoving == TO_RIGHT_LANE_RIGHT_TURN) {
                        // Move to the right lane: Turn right part.
                        m_vehicleControl.setSpeed(1.5);
                        m_vehicleControl.setSteeringWheelAngle(25);

                        stageToRightLaneRightTurn--;
                        if (stageToRightLaneRightTurn == 0) {
                            stageMoving = TO_RIGHT_LANE_LEFT_TURN;
                        }
                    }
                    else if (stageMoving == TO_RIGHT_LANE_LEFT_TURN) {
                        // Move to the left lane: Turn left part.
                        m_vehicleControl.setSpeed(.9);
                        m_vehicleControl.setSteeringWheelAngle(-25);

                        stageToRightLaneLeftTurn--;
                        if (stageToRightLaneLeftTurn == 0) {
                            // Start over.
                            stageMoving = FORWARD;
                            stageMeasuring = FIND_OBJECT_INIT;

                            distanceToObstacle = 0;
                            distanceToObstacleOld = 0;

                            // Reset PID controller.
                            m_eSum = 0;
                            m_eOld = 0;
                        }
                    }

                    // Measuring state machine.
                    if (stageMeasuring == FIND_OBJECT_INIT) {
                        distanceToObstacleOld = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                        stageMeasuring = FIND_OBJECT;
                    }
                    else if (stageMeasuring == FIND_OBJECT) {
                        distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);

                        // Approaching an obstacle (stationary or driving slower than us).
                        if (  (distanceToObstacle > 0) && (((distanceToObstacleOld - distanceToObstacle) > 0) || (fabs(distanceToObstacleOld - distanceToObstacle) < 1e-2)) ) {
                            // Check if overtaking shall be started.                        
                            stageMeasuring = FIND_OBJECT_PLAUSIBLE;
                        }

                        distanceToObstacleOld = distanceToObstacle;
                    }
                    else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) {
                        if (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) < OVERTAKING_DISTANCE) {
                            stageMoving = TO_LEFT_LANE_LEFT_TURN;

                            // Disable measuring until requested from moving state machine again.
                            stageMeasuring = DISABLE;
                        }
                        else {
                            stageMeasuring = FIND_OBJECT;
                        }
                    }
                    else if (stageMeasuring == HAVE_BOTH_IR) {
                        // Remain in this stage until both IRs see something.
                        if ( (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0) ) {
                            // Turn to right.
                            stageMoving = TO_LEFT_LANE_RIGHT_TURN;
                        }
                    }
                    else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                        // Remain in this stage until both IRs have the similar distance to obstacle (i.e. turn car)
                        // and the driven parts of the turn are plausible.
                        const double IR_FR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                        const double IR_RR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);

                        if ((fabs(IR_FR - IR_RR) < HEADING_PARALLEL) && ((stageToRightLaneLeftTurn - stageToRightLaneRightTurn) > 0)) {
                            // Straight forward again.
                            stageMoving = CONTINUE_ON_LEFT_LANE;

                            // Reset PID controller.
                            m_eSum = 0;
                            m_eOld = 0;
                        }
                    }
                    else if (stageMeasuring == END_OF_OBJECT) {
                        // Find end of object.
                        distanceToObstacle = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_RIGHT);

                        if (distanceToObstacle < 0) {
                            // Move to right lane again.
                            stageMoving = TO_RIGHT_LANE_RIGHT_TURN;

                            // Disable measuring until requested from moving state machine again.
                            stageMeasuring = DISABLE;
                        }
                    }
                }

                // Create container for finally sending the set values for the control algorithm.
                Container c2(m_vehicleControl);
                // Send container.
                getConference().send(c2);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature

