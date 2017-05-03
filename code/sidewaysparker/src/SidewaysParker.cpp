/**
 * sidewaysparker - Sample application for realizing a sideways parking car.
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

#include <cstdio>
#include <cmath>
#include <iostream>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "SidewaysParker.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;

        SidewaysParker::SidewaysParker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "SidewaysParker") {
        }

        SidewaysParker::~SidewaysParker() {}

        void SidewaysParker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void SidewaysParker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        int stageMoving = 0;
        int stageMeasuring = 0;

        // Create vehicle control data.
        VehicleControl vc;

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SidewaysParker::body() {

            double distanceOld = 0;
            double absPathStart = 0;
            double absPathEnd = 0;
            double absPathPark = 0;



            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();
                double frontRightInfrared = sbd.getValueForKey_MapOfDistances(0);


                // Our code for parking!!
                parallelPark();






                // Measuring state machine.
                switch (stageMeasuring) {
                    case 0:
                        {
                            // Initialize measurement.
                            distanceOld = frontRightInfrared;
                            stageMeasuring++;
                        }
                    break;
                    case 1:
                        {
                            absPathPark = vd.getAbsTraveledPath();
                            // Checking for sequence +, -.
                            if ((distanceOld > 0) && (frontRightInfrared < 0)) {
                                // Found sequence +, -.
                                stageMeasuring = 2;
                                absPathStart = vd.getAbsTraveledPath();
                            }

                            if(absPathPark > 7){
                                stageMoving = 1;
                            }

                            distanceOld = frontRightInfrared;
                        }
                    break;
                    case 2:
                        {
                            // Checking for sequence -, +.
                            if ((distanceOld < 0) && (frontRightInfrared > 0)) {
                                // Found sequence -, +.
                                stageMeasuring = 1;
                                absPathEnd = vd.getAbsTraveledPath();

                                const double GAP_SIZE = (absPathEnd - absPathStart);

                                cerr << "Size = " << GAP_SIZE << endl;

                                if ((stageMoving < 1) && (GAP_SIZE > 7)) {
                                    stageMoving = 1;
                                }
                            }
                            distanceOld = frontRightInfrared;
                        }
                    break;
                }

                // Create container for finally sending the data.
                Container c(vc);
                // Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

        void SidewaysParker::stop(){

            vc.setSpeed(0);
            vc.setSteeringWheelAngle(0);

        }

        void SidewaysParker::reverse(){
            vc.setSpeed(-1);
            vc.setSteeringWheelAngle(0);
        }

        void SidewaysParker::forward(){
            vc.setSpeed(2);
            vc.setSteeringWheelAngle(0);
        }

        void SidewaysParker::slowForward(){
            vc.setSpeed(.4);
            vc.setSteeringWheelAngle(0);
        }

        void SidewaysParker::reverseTurnRight(){
            vc.setSpeed(-2);
            vc.setSteeringWheelAngle(25);
        }

        void SidewaysParker::reverseTurnLeftSlow(){
            vc.setSpeed(-.4);
            vc.setSteeringWheelAngle(-25);
        }

        void SidewaysParker::parallelPark() {

            Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
            SensorBoardData data = containerSensorBoardData.getData<SensorBoardData>();


            //double rearRightInfrared = data.getValueForKey_MapOfDistances(2);
            // double frontRightInfrared = data.getValueForKey_MapOfDistances(0);
            //double frontRightUltrasonic = data.getValueForKey_MapOfDistances(4);
            //double rearRightUltrasonic = data.getValueForKey_MapOfDistances(5);
            //double rearInfrared = data.getValueForKey_MapOfDistances(1);

            // Moving state machine.
            if (stageMoving == 0) {
                // Go forward.
               forward();
            }
            if ((stageMoving > 0) && (stageMoving < 40)) {
                // Move slightly forward.
               slowForward();
                stageMoving++;
            }
            if ((stageMoving >= 40) && (stageMoving < 45)) {
                // Stop.
                stop();
                stageMoving++;
            }
            if ((stageMoving >= 45) && (stageMoving < 85)) {
                // Backwards, steering wheel to the right.
                reverseTurnRight();
                stageMoving++;
            }
            if ((stageMoving >= 85) && (stageMoving < 220)) {
                // Backwards, steering wheel to the left.
                reverseTurnLeftSlow();
                stageMoving++;
            }
            if (stageMoving >= 220) {
                // Stop.
                stop();
            }


        }
        }
} // automotive::miniature

