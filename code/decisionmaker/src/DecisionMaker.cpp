/**

 */

#include <cstdio>
#include <cmath>
#include <iostream>


#include "DecisionMaker.h"


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

        DecisionMaker::DecisionMaker(const int32_t &argc, char **argv) :
            TimeTriggeredConferenceClientModule(argc, argv, "DecisionMaker") {
        }

        DecisionMaker::~DecisionMaker() {}

        void DecisionMaker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void DecisionMaker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }





        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode DecisionMaker::body() {


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
                        LaneFollowMSG tmpmsg;
                        tmpmsg.setControl(vControl);
                        tmpmsg.setLanefollow(false);
                        Container c(tmpmsg);
                        getConference().send(c);

                    }
                        break;
                    case 1:
                        {
                            LaneFollowMSG tmpmsg;
                            tmpmsg.setControl(vControl);
                            tmpmsg.setLanefollow(true);
                            Container c(tmpmsg);
                            getConference().send(c);

                        }
                    break;
                    case 2:
                        {

                                    ParkMSG tmpmsg;
                                    tmpmsg.setControl(vControl);
                                    tmpmsg.setData(sData);
                                    tmpmsg.setVehicleData(vData);
                                    Container c(tmpmsg);
                                    getConference().send(c);



                        }
                    break;
                    case 3:
                        {







                        }
                    break;
                }
                Container containerReturnMSG = getKeyValueDataStore().get(scaledcars::ReturnVehicleControl::ID());
                ReturnVehicleControl tmpreturn = containerReturnMSG.getData<ReturnVehicleControl> ();
                vControl = tmpreturn.getControl();
                // Create container for finally sending the data.
                Container c(vControl);
                // Send container.
                getConference().send(c);
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }



//        void DecisionMaker::setVehicleData(scaledcars::VehicleData vvdata){
//            vData = vvdata;
//        }
//
//        void DecisionMaker::setSensorData(scaledcars::SensorBoardData ssdata){
//            sData = ssdata;
//        }
//
//        void DecisionMaker::setState(scaledcars::StateMSG state){
//            currentState = state.getState();
//        }



} // automotive::miniature

