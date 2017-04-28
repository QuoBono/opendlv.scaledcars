/**
 * boxparker - Sample application for realizing a box parking car.
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

#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "BoxParker.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::base::module;
        using namespace odcore::data;
        using namespace automotive;

        BoxParker::BoxParker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "BoxParker"),
            m_foundGaps() {}

        BoxParker::~BoxParker() {}

        int stageMoving = 0;


        // Create vehicle control data.
        VehicleControl vc;


        void BoxParker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void BoxParker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        vector<double> BoxParker::getFoundGaps() const {
            return m_foundGaps;
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode BoxParker::body() {
            double distanceOld = 0;
            double absPathStart = 0;
            double absPathEnd = 0;


            int stageMeasuring = 0;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();


                // 2. Get most recent sensor board data describing virtual sensor data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();




                // Our code for parking!!
                if( parallelPark()==1){
                    break;
                }




                // Measuring state machine.
                switch (stageMeasuring) {
                    case 0:
                        {
                            // Initialize measurement.
                            distanceOld = sbd.getValueForKey_MapOfDistances(2);
                            stageMeasuring++;
                        }
                    break;
                    case 1:
                        {
                            // Checking for distance sequence +, -.
                            if ((distanceOld > 0) && (sbd.getValueForKey_MapOfDistances(2) < 0)) {
                                // Found distance sequence +, -.
                                stageMeasuring = 2;
                                absPathStart = vd.getAbsTraveledPath();
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(2);
                        }
                    break;
                    case 2:
                        {
                            // Checking for distance sequence -, +.
                            if ((distanceOld < 0) && (sbd.getValueForKey_MapOfDistances(2) > 0)) {
                                // Found distance sequence -, +.
                                stageMeasuring = 1;
                                absPathEnd = vd.getAbsTraveledPath();

                                const double GAP_SIZE = (absPathEnd - absPathStart);
                                cerr << "Size = " << GAP_SIZE << endl;
                                m_foundGaps.push_back(GAP_SIZE);

                                if ((stageMoving < 1) && (GAP_SIZE > 3.0)) {
                                    stageMoving = 1;
                                }
                            }
                            distanceOld = sbd.getValueForKey_MapOfDistances(2);
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


        void BoxParker::stop(){

            vc.setSpeed(0);
            vc.setSteeringWheelAngle(0);

        }

        void BoxParker::reverse(){
            vc.setSpeed(-1);
            vc.setSteeringWheelAngle(0);
        }

        void BoxParker::accelerate(){
            vc.setSpeed(1);
            vc.setSteeringWheelAngle(0);
        }

        void BoxParker::reverseTurnRight(){
            vc.setSpeed(-1);
            vc.setSteeringWheelAngle(25);
        }

        bool BoxParker::carOnRight(){
            Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
            SensorBoardData data = containerSensorBoardData.getData<SensorBoardData> ();
            double rearRightInfrared = data.getValueForKey_MapOfDistances(2);
            double frontRightInfrared = data.getValueForKey_MapOfDistances(0);

            if((rearRightInfrared<0.6) && (frontRightInfrared < 0.6)){
                return true;
            } else{
                return false;
            }

        }


        int BoxParker::parallelPark() {

            Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
            SensorBoardData data = containerSensorBoardData.getData<SensorBoardData> ();


            int result = 0;
            //double rearRightInfrared = data.getValueForKey_MapOfDistances(2);
           // double frontRightInfrared = data.getValueForKey_MapOfDistances(0);
            double frontRightUltrasonic = data.getValueForKey_MapOfDistances(4);
            //double rearRightUltrasonic = data.getValueForKey_MapOfDistances(5);
            //double rearInfrared = data.getValueForKey_MapOfDistances(1);




            // commnt out the first if to remove moving part of the code
            // Moving state machine.
            if (stageMoving == 0) {
                // Go forward.
                accelerate();

            }
                if(stageMoving >= 1 && stageMoving < 25){
                    cerr<< "Moving extra: " << stageMoving << endl;
                    accelerate();
                    stageMoving++;
                }


            if (stageMoving >= 25 && stageMoving < 110) {
                    cerr<< "backin up: "<< endl;
                    reverseTurnRight();
                    stageMoving++;

            }
            if(stageMoving >=110 && stageMoving<120){
                stop();
                
                stageMoving++;
            }
            if (stageMoving >= 120) {

                if (frontRightUltrasonic < 0) {
                    cerr<< "reversing " << endl;
                    reverse();
                }else{
                    stop();
                    cerr<< "stopping " << endl;
                    result = 1;
                }
            }
            return result;
        }

    } // miniature
} // automotive

