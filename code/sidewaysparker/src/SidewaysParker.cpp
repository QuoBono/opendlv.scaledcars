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




#include "SidewaysParker.h"

#include "odvdscaledcarsdatamodel/GeneratedHeaders_ODVDScaledCarsDataModel.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include <../../decitionmaker/include/DecitionMaker.h>

namespace scaledcars {


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

    void SidewaysParker::nextContainer(Container &c) {

        if (c.getDataType() == ParkMSG::ID()) {
            ParkMSG pm = c.getData<ParkMSG> ();
            automotive::VehicleControl vcontrol = pm.getControl();
            automotive::miniature::SensorBoardData sdata = pm.getData();
            automotive::VehicleData vdata = pm.getVehicleData();
            cerr << sdata.toString() << endl;
           findSpot(vcontrol,sdata,vdata);
        }

    }





        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode SidewaysParker::body() {
            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                //cerr << "StageMoving: " << stageMoving << endl;
//                Container containerParkMSG = getKeyValueDataStore().get(scaledcars::ParkMSG::ID());
//                ParkMSG pm = containerParkMSG.getData<ParkMSG> ();
//                vc = pm.getControl();
//                data = pm.getData();
            }
            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }


        void SidewaysParker::stop(){

            vc.setSpeed(0);
            vc.setSteeringWheelAngle(0);

        }


        void SidewaysParker::reverse(){
            vc.setSpeed(2);
            vc.setSteeringWheelAngle(0);
        }

        void SidewaysParker::slowReverse(){
            vc.setSpeed(-0.5);
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
            vc.setSteeringWheelAngle(45);
        }

        void SidewaysParker::reverseTurnLeftSlow(){
            vc.setSpeed(-.5);
            vc.setSteeringWheelAngle(-45);
        }




        void SidewaysParker::findSpot(automotive::VehicleControl vcontrol, automotive::miniature::SensorBoardData sdata, automotive::VehicleData vdata){
            data = sdata;
            vc = vcontrol;
            vd = vdata;



            double frontRightInfrared = data.getValueForKey_MapOfDistances(0);



            SidewaysParker::parallelPark();




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
                        absPathParkStart = vd.getAbsTraveledPath();

                        //cerr << "case1"<<endl;

                        if(absPathParkStart - absPathParkEnd > 6){
                            if(parkAfterCar != 1){
                                stageMoving = 1;
                            }
                            parkAfterCar = 1;

                        }


                        // Checking for sequence +, -.
                        if ((distanceOld > 0) && (frontRightInfrared < 0)) {


                            // Found sequence +, -.
                            stageMeasuring = 2;
                            absPathStart = vd.getAbsTraveledPath();



                        }
                        //cerr << "absParkStart: " << absPathParkStart << endl;
                        //cerr << "absParkEnd: " << absPathParkEnd << endl;


                        distanceOld = frontRightInfrared;
                    }
                        break;
                    case 2:
                        //cerr << "case2"<<endl;

                        {

                        if(vd.getAbsTraveledPath() - absPathStart > 5){
                            if(parkAfterCar != 3){
                                stageMoving = 1;
                            }
                            parkAfterCar = 3;
                        }

                        // Checking for sequence -, +.
                        if ((distanceOld < 0) && (frontRightInfrared > 0)) {
                            // Found sequence -, +.
                            stageMeasuring = 1;
                            absPathEnd = vd.getAbsTraveledPath();
                            absPathParkEnd = vd.getAbsTraveledPath();


                            const double GAP_SIZE = (absPathEnd - absPathStart);

                            cerr << "Size = " << GAP_SIZE << endl;

                            if ((stageMoving < 1) && (GAP_SIZE > 5)) {
                                stageMoving = 1;
                                parkAfterCar = 2;
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


        void SidewaysParker::parallelPark() {

        cerr << stageMoving << endl;

            //cerr << "parkaeftercar: " << parkAfterCar << endl;

            //double rearRightInfrared = data.getValueForKey_MapOfDistances(2);
            //double frontRightInfrared = data.getValueForKey_MapOfDistances(0);
            //double frontRightUltrasonic = data.getValueForKey_MapOfDistances(4);
            //double rearRightUltrasonic = data.getValueForKey_MapOfDistances(5);
            double rearInfrared = data.getValueForKey_MapOfDistances(1);
            double frontUltrasonic = data.getValueForKey_MapOfDistances(3);

            // Moving state machine.
            if (stageMoving == 0) {
                // Go forward.
                forward();
            }

            if (parkAfterCar == 1) {

                if ((stageMoving > 0) && (stageMoving < 15)) {
                    // forward
                    forward();
                    stageMoving++;
                }
                if ((stageMoving >= 15) && (stageMoving < 30)) {
                    // slowforward
                    slowForward();
                    stageMoving++;
                }
                if ((stageMoving >= 30) && (stageMoving < 35)) {
                    // Stop.
                    stop();
                    stageMoving++;
                }
                if ((stageMoving >= 35) && (stageMoving < 71)) {
                    // Backwards, steering wheel to the right.
                    reverseTurnRight();
                    stageMoving++;
                }
                if ((stageMoving >= 71) && (stageMoving < 108)) {
                    // Backwards, steering wheel to the left.
                    reverseTurnLeftSlow();
                    stageMoving++;
                }
                if (stageMoving >= 108) {
                    if (rearInfrared > 2.5) {
                        slowReverse();
                    } else {
                        // Stop.
                        stop();
                        StateMSG stop;
                        stop.setState(0);
                        Container c(stop);
                        getConference().send(c);
                    }
                }
            }

            if (parkAfterCar == 2) {

                if ((stageMoving > 0) && (stageMoving < 25)) {
                    // Move slightly forward.
                    slowForward();
                    stageMoving++;
                }
                if ((stageMoving >= 25) && (stageMoving < 30)) {
                    // Stop.
                    stop();
                    stageMoving++;
                }
                if ((stageMoving >= 30) && (stageMoving < 66)) {
                    // Backwards, steering wheel to the right.
                    reverseTurnRight();
                    stageMoving++;
                }
                if ((stageMoving >= 66) && (stageMoving < 107)) {
                    // Backwards, steering wheel to the left.
                    reverseTurnLeftSlow();
                    stageMoving++;
                }
                if (stageMoving >= 107) {
                    if (frontUltrasonic > 3) {
                        slowForward();
                    } else {
                        stop();
                        StateMSG stop;
                        stop.setState(0);
                        Container c(stop);
                        getConference().send(c);

                    }

                }


            }
            if (parkAfterCar == 3) {


                if ((stageMoving > 0) && (stageMoving < 20)) {
                    // Move forward.
                    forward();
                    stageMoving++;
                }
                if ((stageMoving >= 20) && (stageMoving < 45)) {
                    // Move slightly forward.
                    slowForward();
                    stageMoving++;
                }
                if ((stageMoving >= 45) && (stageMoving < 50)) {
                    // Stop.
                    stop();
                    stageMoving++;
                }
                if ((stageMoving >= 50) && (stageMoving < 86)) {
                    // Backwards, steering wheel to the right.
                    reverseTurnRight();
                    stageMoving++;
                }
                if ((stageMoving >= 86) && (stageMoving < 127)) {
                    // Backwards, steering wheel to the left.
                    reverseTurnLeftSlow();
                    stageMoving++;
                }
                if (stageMoving >= 127) {
                    if (rearInfrared > 2) {
                        slowReverse();
                    } else {
                        stop();
                        StateMSG stop;
                        stop.setState(0);
                        Container c(stop);
                        getConference().send(c);
                    }


                }
            }

    }
} // automotive::miniature