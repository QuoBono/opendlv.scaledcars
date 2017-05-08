/**

 */

#include <cstdio>
#include <cmath>
#include <iostream>
#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
//#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "DecitionMaker.h"






namespace automotive {
    namespace miniature {

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





        // Create vehicle control data.
        VehicleControl vc;



        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode DecitionMaker::body() {

            Test test;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent vehicle data:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData> ();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();

                //double frontRightInfrared = sbd.getValueForKey_MapOfDistances(0);


                //Todo somehow get the state to set what to do
                currentState = park;


                // Measuring state machine.
                switch (currentState) {
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
                                    string send = "hello";

                                    test(send);
                                    Container c(test);
                                    getConference().send(c);
                                    parking = true;
                                    cerr << "Parking!" << endl;
                                }

//                                if(automotive::miniature::SidewaysParker::isParked()){
//                                     cerr << "Parked!" << endl;
//                                }

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
                Container c(vc);
                // Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

        void DecitionMaker::stop(){
            vc.setSpeed(0);
            vc.setSteeringWheelAngle(0);
        }

        void DecitionMaker::reverse(){
            vc.setSpeed(2);
            vc.setSteeringWheelAngle(0);
        }

        void DecitionMaker::slowReverse(){
            vc.setSpeed(-0.5);
            vc.setSteeringWheelAngle(0);
        }

        void DecitionMaker::forward(){
            vc.setSpeed(2);
        }

        void DecitionMaker::slowForward(){
            vc.setSpeed(.5);

        }


        }
} // automotive::miniature

