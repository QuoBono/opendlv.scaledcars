/**

 */

#include <cstdio>
#include <cmath>
#include <iostream>


#include "DecitionMaker.h"


#include "odvdscaledcarsdatamodel/GeneratedHeaders_ODVDScaledCarsDataModel.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"
#include <../../sidewaysparker/include/SidewaysParker.h>


namespace scaledcars {


        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;

        DecitionMaker::DecitionMaker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "DecitionMaker") {
        }

        DecitionMaker::~DecitionMaker() {}

        void DecitionMaker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void DecitionMaker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

//    void DecitionMaker::nextContainer(Container &c) {
//
//        if (c.getDataType() == StateMSG::ID()) {
//            StateMSG state = c.getData<StateMSG>();
//            cout << "switching state to:   "     << endl;
//            DecitionMaker::setState(state);
//        }
//        if (c.getDataType() == VehicleData::ID()) {
//            VehicleData tmpvdata = c.getData<VehicleData>();
//            DecitionMaker::setVehicleData(tmpvdata);
//        }
//        if (c.getDataType() == SensorBoardData::ID()) {
//            SensorBoardData tmpsdata = c.getData<SensorBoardData>();
//            DecitionMaker::setSensorData(tmpsdata);
//
//
//        }
//    }




        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode DecitionMaker::body() {


            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {

                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                sData = containerSensorBoardData.getData<SensorBoardData> ();

                cerr << "sensordata: " << sData.toString() << endl;

                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                vData = containerVehicleData.getData<VehicleData> ();


                Container containerStateMSG = getKeyValueDataStore().get(scaledcars::StateMSG::ID());
                StateMSG tmpstate = containerStateMSG.getData<StateMSG> ();
                currentState = tmpstate.getState();


                //double frontRightInfrared = sbd.getValueForKey_MapOfDistances(0);
                //Todo somehow get the state to set what to do
                currentState = park;
                //cerr << "current state is: "<< currentState << endl;

                // Measuring state machine.
                switch (currentState) {
                    case 0:
                    {
                        //Todo do nothing
                        stop();
                        if(laneFollowing){
                            //scaledcars::LaneFollowing::stopLaneFollow();
                            laneFollowing = false;
                        }

                    }
                        break;
                    case 1:
                        {
                            //Todo Lanefollowing
                            forward();
                            if(!laneFollowing){
                                //LaneFollowing::laneFollow();
                                laneFollowing = true;
                            }

                        }
                    break;
                    case 2:
                        {
                            //Todo Parking
                            if(!laneFollowing){
                                //LaneFollowing::laneFollow();
                                forward();
                                laneFollowing = true;
                            }


                                if(!parking){
                                    stop();
                                    ParkMSG tmpmsg;
                                    tmpmsg.setControl(vControl);
                                    tmpmsg.setData(sData);
                                    tmpmsg.setVehicleData(vData);
                                    Container c(tmpmsg);
                                    getConference().send(c);

                                }

                        }
                    break;
                    case 3:
                        {
                            //Todo LanefollowingWithOvertaking
                            if(!laneFollowing){
                                //LaneFollowing::laneFollow();
                                forward();
                            }
                            if(!overtaking){
                                stop();
                                if(!overtaking){
                                    //Overtaking::overtake();
                                }

                            }

                        }
                    break;
                }

                // Create container for finally sending the data.
                Container c(vControl);
                // Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

        void DecitionMaker::stop(){
            vControl.setSpeed(0);
            vControl.setSteeringWheelAngle(0);
        }

        void DecitionMaker::reverse(){
            vControl.setSpeed(2);
            vControl.setSteeringWheelAngle(0);
        }

        void DecitionMaker::slowReverse(){
            vControl.setSpeed(-0.5);
            vControl.setSteeringWheelAngle(0);
        }

        void DecitionMaker::forward(){
            vControl.setSpeed(2);
        }

        void DecitionMaker::slowForward(){
            vControl.setSpeed(.5);

        }

//        void DecitionMaker::setVehicleData(scaledcars::VehicleData vvdata){
//            vData = vvdata;
//        }
//
//        void DecitionMaker::setSensorData(scaledcars::SensorBoardData ssdata){
//            sData = ssdata;
//        }
//
//        void DecitionMaker::setState(scaledcars::StateMSG state){
//            currentState = state.getState();
//        }



} // automotive::miniature

