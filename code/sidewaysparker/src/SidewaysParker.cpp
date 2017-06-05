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
            ParkMSG pm = c.getData<ParkMSG>();
            automotive::VehicleControl vcontrol = pm.getControl();
            automotive::miniature::SensorBoardData sdata = pm.getData();
            automotive::VehicleData vdata = pm.getVehicleData();
            //cerr << sdata.toString() << endl;

            if (foundSpot) {
                //cerr << "Found spot" << endl;

                parallelPark(vcontrol, sdata, vdata);
            } else {
                //cerr << "Finding spot" << endl;
                findSpot(vcontrol, sdata, vdata);
            }
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
        ReturnVehicleControl rtmp;
        rtmp.setControl(vc);
        Container send(rtmp);
        getConference().send(send);
        //cerr << "sent vc from sidewaysparker " << endl;

        }


        void SidewaysParker::slowReverse(){
            vc.setSpeed(3);
            vc.setSteeringWheelAngle(0);
            ReturnVehicleControl rtmp;
            rtmp.setControl(vc);
            Container send(rtmp);
            getConference().send(send);
            //cerr << "sent vc from sidewaysparker " << endl;
        }

        void SidewaysParker::forward(){
            vc.setSpeed(1);
            vc.setSteeringWheelAngle(0);
            ReturnVehicleControl rtmp;
            rtmp.setControl(vc);
            Container send(rtmp);
            getConference().send(send);
            //cerr << "sent vc from sidewaysparker " << endl;
        }

        void SidewaysParker::slowForward(){
            vc.setSpeed(1);
            vc.setSteeringWheelAngle(0);
            ReturnVehicleControl rtmp;
            rtmp.setControl(vc);
            Container send(rtmp);
            getConference().send(send);
            //cerr << "sent vc from sidewaysparker " << endl;
        }

        void SidewaysParker::reverseTurnRight(){
            vc.setSpeed(3);
            vc.setSteeringWheelAngle(25);
            ReturnVehicleControl rtmp;
            rtmp.setControl(vc);
            Container send(rtmp);
            getConference().send(send);
            //cerr << "sent vc from sidewaysparker " << endl;
        }

        void SidewaysParker::reverseTurnLeftSlow(){
            vc.setSpeed(3);
            vc.setSteeringWheelAngle(-25);
            ReturnVehicleControl rtmp;
            rtmp.setControl(vc);
            Container send(rtmp);
            getConference().send(send);
            //cerr << "sent vc from sidewaysparker " << endl;
        }

        void SidewaysParker::forwardWithLaneFollower(){
            LaneFollowMSG tmpmsg;
            tmpmsg.setControl(vc);
            tmpmsg.setLanefollow(true);
            Container c(tmpmsg);
            getConference().send(c);
        }

        void SidewaysParker::stopWithLaneFollower(){
            LaneFollowMSG tmpmsg;
            tmpmsg.setControl(vc);
            tmpmsg.setLanefollow(false);
            Container c(tmpmsg);
            getConference().send(c);
        }




        void SidewaysParker::findSpot(automotive::VehicleControl vcontrol, automotive::miniature::SensorBoardData sdata, automotive::VehicleData vdata){
            data = sdata;
            VehicleControl svc = vcontrol;
            vd = vdata;
            double traveledPath = sdata.getValueForKey_MapOfDistances(5);

            //forwardWithLaneFollower();
            forward();
            stopWithLaneFollower();
            double frontRightInfrared = data.getValueForKey_MapOfDistances(0);
            cerr << "traveled path: " << traveledPath << endl;

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
                        //absPathParkStart = vd.getAbsTraveledPath();
                        absPathParkStart = traveledPath;
                        //cerr << "case1"<<endl;

                        if(absPathParkStart - absPathParkEnd > 20){

                                    stageMoving = 1;
                                    foundSpot = true;
                            parkAfterCar = 1;
                            cerr << "set park car to 1" << endl;

                        }


                        // Checking for sequence +, -.
                        if ((distanceOld > 0) && (frontRightInfrared < 0)) {


                            // Found sequence +, -.
                            stageMeasuring = 2;
                            //absPathStart = vd.getAbsTraveledPath();
                            absPathStart = traveledPath;




                        }
                        //cerr << "absParkStart: " << absPathParkStart << endl;
                        //cerr << "absParkEnd: " << absPathParkEnd << endl;


                        distanceOld = frontRightInfrared;
                    }
                        break;
                    case 2:
                        //cerr << "case2"<<endl;

                        {
                            //if(vd.getAbsTraveledPath() - absPathStart > 6){

                            if(traveledPath - absPathStart > 20){

                                stageMoving = 1;
                                foundSpot = true;
                            parkAfterCar = 3;
                            cerr << "set park car to 3" << endl;
                            }



                        // Checking for sequence -, +.
                        if ((distanceOld < 0) && (frontRightInfrared > 0)) {
                            // Found sequence -, +.
                            stageMeasuring = 1;
                            absPathEnd = traveledPath;
                            //absPathEnd = vd.getAbsTraveledPath();
                            absPathParkEnd = traveledPath;
                            //absPathParkEnd = vd.getAbsTraveledPath();


                            const double GAP_SIZE = (absPathEnd - absPathStart);

                            cerr << "Size = " << GAP_SIZE << endl;

                            if ((stageMoving < 1) && (GAP_SIZE > 20)) {
                                    stageMoving = 1;
                                foundSpot = true;
                                    parkAfterCar = 2;
                                    cerr << "set park car to 2" << endl;



                            }
                        }
                        distanceOld = frontRightInfrared;
                    }
                        break;
                }


            }


        void SidewaysParker::parallelPark(automotive::VehicleControl vControl, automotive::miniature::SensorBoardData sData, automotive::VehicleData vData) {
            data = sData;
            VehicleControl svc = vControl;
            vd = vData;
            //cerr << parkAfterCar << endl;

            //cerr << stageMoving << endl;

            //cerr << "parkaeftercar: " << parkAfterCar << endl;

            //double rearRightInfrared = data.getValueForKey_MapOfDistances(2);
            //double frontRightInfrared = data.getValueForKey_MapOfDistances(0);
            //double frontRightUltrasonic = data.getValueForKey_MapOfDistances(4);
            //double rearRightUltrasonic = data.getValueForKey_MapOfDistances(5);
            double rearInfrared = data.getValueForKey_MapOfDistances(1);
            double frontUltrasonic = data.getValueForKey_MapOfDistances(3);
            //double traveledPath = sdata.getValueForKey_MapOfDistances(5);


            // Moving state machine.
            if (stageMoving == 0) {
                // Go forward.


            }

            if (parkAfterCar == 1) {
                stopWithLaneFollower();
                cerr << "Parking with 1, stagemoving: " << stageMoving <<  endl;


                if ((stageMoving >= 0) && (stageMoving < 15)) {
                    // slowforward
                    slowForward();
                    stageMoving++;
                }
                if ((stageMoving >= 15) && (stageMoving <19)) {
                    // slowforward
                    slowReverse();
                    stageMoving++;
                }
                if ((stageMoving >= 19) && (stageMoving < 25)) {
                    // Stop.
                    stop();
                    stageMoving++;
                }
                if ((stageMoving >= 25) && (stageMoving < 35)) {
                    // Backwards, steering wheel to the right.
                    reverseTurnRight();
                    stageMoving++;
                }
                if ((stageMoving >= 35) && (stageMoving < 40)) {
                    // Backwards, steering wheel to the left.
                    reverseTurnLeftSlow();
                    stageMoving++;
                }
                if ((stageMoving >= 40) && (stageMoving < 42)) {
                    // Backwards, steering wheel to the left.
                    slowForward();
                    stageMoving++;
                }
                if (stageMoving >= 42) {
                    if (rearInfrared > 10 && rearInfrared > 0) {
                        //slowReverse();
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
                stopWithLaneFollower();
                cerr << "Parking with 2, stagemoving: " << stageMoving <<  endl;
                if ((stageMoving > 0) && (stageMoving < 10)) {
                    // forward
                    forward();
                    stageMoving++;
                }
                if ((stageMoving >= 10) && (stageMoving < 15)) {
                    // slowforward
                    slowForward();
                    stageMoving++;
                }
                if ((stageMoving >= 15) && (stageMoving < 20)) {
                    // Stop.
                    stop();
                    stageMoving++;
                }
                if ((stageMoving >= 20) && (stageMoving < 40)) {
                    // Backwards, steering wheel to the right.
                    reverseTurnRight();
                    stageMoving++;
                }
                if ((stageMoving >= 40) && (stageMoving < 60)) {
                    // Backwards, steering wheel to the left.
                    reverseTurnLeftSlow();
                    stageMoving++;
                }
                if (stageMoving >= 60) {
                    if (frontUltrasonic > 15) {
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
                stopWithLaneFollower();
                cerr << "Parking with 3, stagemoving: " << stageMoving <<  endl;

                if ((stageMoving >= 0) && (stageMoving < 5)) {
                    // Move slightly forward.
                    slowForward();
                    stageMoving++;
                }
                if ((stageMoving >= 5) && (stageMoving < 10)) {
                    // Stop.
                    stop();
                    stageMoving++;
                }
                if ((stageMoving >= 10) && (stageMoving < 15)) {
                    // Backwards, steering wheel to the right.
                    reverseTurnRight();
                    //cerr << "turning right" << endl;
                    stageMoving++;
                }
                if ((stageMoving >= 15) && (stageMoving < 20)) {
                    // Backwards, steering wheel to the left.

                    if (rearInfrared < 10 && rearInfrared > 0) {
                        stop();
                        stageMoving++;
                    } else {
                        reverseTurnLeftSlow();
                        stageMoving++;

                    }

                }
                if (stageMoving >= 20) {

                        stop();
                        StateMSG stop;
                        stop.setState(0);
                        Container c(stop);
                        getConference().send(c);
                        cerr << "Stopped everything" << endl;


                }
            }
            // Create container for finally sending the data.

    }
} // automotive::miniature