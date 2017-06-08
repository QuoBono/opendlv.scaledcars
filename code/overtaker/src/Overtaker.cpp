/**
 * overtaker - Sample application for overtaking obstacles.
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

#include <cstdio>
#include <cmath>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

#include "Overtaker.h"

namespace automotive {
    namespace miniature {

        using namespace std;
        using namespace odcore::base;
        using namespace odcore::data;
        using namespace automotive;
        using namespace automotive::miniature;

        Overtaker::Overtaker(const int32_t &argc, char **argv) :
                TimeTriggeredConferenceClientModule(argc, argv, "overtaker") {
        }

        Overtaker::~Overtaker() {}

        void Overtaker::setUp() {
            // This method will be call automatically _before_ running body().
        }

        void Overtaker::tearDown() {
            // This method will be call automatically _after_ return from body().
        }

        // This method will do the main data processing job.
        odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode Overtaker::body() {

            // Setting up the sensor IDs
            const int32_t ULTRASONIC_FRONT_CENTER = 3;
            //const int32_t ULTRASONIC_FRONT_RIGHT = 4;
            const int32_t INFRARED_FRONT_RIGHT = 0;
            const int32_t INFRARED_REAR_RIGHT = 2;
            const int32_t ODOMETER = 5;
            const double OVERTAKING_DISTANCE = 50;
            const double HEADING_PARALLEL = 5;

            // Vars for storing the IR values
            double FRONT_IR = 0;
            double REAR_RIGHT_IR = 0;

            //double lane_following_angle = 0;

            // Overall state machines for moving and measuring.
            enum StateMachineMoving { FORWARD, TO_LEFT_LANE_LEFT_TURN, TO_LEFT_LANE_RIGHT_TURN, CONTINUE_ON_LEFT_LANE, TO_RIGHT_LANE_RIGHT_TURN, TO_RIGHT_LANE_LEFT_TURN };
            enum StateMachineMeasuring { DISABLE, FIND_OBJECT_INIT, FIND_OBJECT, FIND_OBJECT_PLAUSIBLE, HAVE_BOTH_IR, HAVE_BOTH_IR_SAME_DISTANCE, END_OF_OBJECT };

            StateMachineMoving stageMoving = FORWARD;
            StateMachineMeasuring stageMeasuring = FIND_OBJECT_INIT;

            int beginning_of_overtaking = 0;
            int mid_of_overtaking = 0;
            int end_of_overtaking = 0;
            int total_odometer_distance = 0;
            double relative_odometer_distance = 0;
            double obstacle_distance = 0;
            double obstacle_distance_old = 0;

            //int right_turn_odo = 0;

            while (getModuleStateAndWaitForRemainingTimeInTimeslice() == odcore::data::dmcp::ModuleStateMessage::RUNNING) {
                // 1. Get most recent data from Vehicle-Control:
                Container containerVehicleData = getKeyValueDataStore().get(automotive::VehicleData::ID());
                VehicleData vd = containerVehicleData.getData<VehicleData>();

                //cerr << "this is in overtaking " << vd.getHeading() << endl;

                //lane_following_angle = vd.getSteeringWheelAngle();

                // 2. Get most recent sensor board data:
                Container containerSensorBoardData = getKeyValueDataStore().get(automotive::miniature::SensorBoardData::ID());
                SensorBoardData sbd = containerSensorBoardData.getData<SensorBoardData> ();


                // Create vehicle control data.
                Container containerVehicleControl = getKeyValueDataStore().get(automotive::VehicleControl::ID());
                VehicleControl vc = containerVehicleControl.getData<VehicleControl>();;

                // Moving state machine.
                // Follows the lane while moving forward
                if (stageMoving == FORWARD) {

                    vd.setHeading(1);
                    //cerr << "stage: MOVE FORWARD" << " angle " << vc.getSteeringWheelAngle()<<endl;

                    // Go forward.
                    //vc.setSpeed(1);
                    //vc.setSteeringWheelAngle(vd.getSteeringWheelAngle());

                    //This gets the speed from the lanefollowing
                    //vd.getSpeed();
                    //This gets the steering wheel angle from the
                    //vd.getSteeringWheelAngle();

                    beginning_of_overtaking = sbd.getValueForKey_MapOfDistances(ODOMETER);

                }

                else if (stageMoving == TO_LEFT_LANE_LEFT_TURN) {

                    cerr << "stage: TO_LEFT_LANE_LEFT_TURN" << endl;

                    // Move to the left lane: Turn left part until both IRs see something.
                    mid_of_overtaking = sbd.getValueForKey_MapOfDistances(ODOMETER);

                    vd.setHeading(0);

                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(-1);

                    // State machine measuring: Both IRs need to see something before leaving this moving state.
                    stageMeasuring = HAVE_BOTH_IR;

                }

                    // Right turn on left lane
                else if (stageMoving == TO_LEFT_LANE_RIGHT_TURN) {
                    cerr << "stage: TO_LEFT_LANE_RIGHT_TURN" << endl;

                    // Move to the left lane: Turn right part until both IRs have the same distance to obstacle.
                    end_of_overtaking = sbd.getValueForKey_MapOfDistances(ODOMETER);

                    vd.setHeading(0);

                    //right_turn_odo = end_of_overtaking;

                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(25); //0.5

                    // State machine measuring: Both IRs need to have the same distance before leaving this moving state.
                    stageMeasuring = HAVE_BOTH_IR_SAME_DISTANCE;
                    //stageMoving = CONTINUE_ON_LEFT_LANE;
                }

                else if (stageMoving == CONTINUE_ON_LEFT_LANE) {
                    cerr << "stage: CONTINUE_ON_LEFT_LANE" << endl;

                    // Move to the left lane: Passing stage.
                    //vc.setSpeed(1);
                    //vc.setSteeringWheelAngle(lane_following_angle);

                    vd.setHeading(1);


                    // Find end of object.
                    stageMeasuring = END_OF_OBJECT;
                    //stageToRightLaneRightTurn = true;
                    total_odometer_distance = sbd.getValueForKey_MapOfDistances(ODOMETER);
                }

                else if (stageMoving == TO_RIGHT_LANE_RIGHT_TURN) {
                    cerr << "stage: TO_RIGHT_LANE_RIGHT_TURN" << endl;
                    // Move to the right lane: Turn right part.
                    vd.setHeading(0);
                    vc.setSpeed(1);
                    vc.setSteeringWheelAngle(25); //15

                    relative_odometer_distance = sbd.getValueForKey_MapOfDistances(ODOMETER)-total_odometer_distance;
                    if (relative_odometer_distance >= mid_of_overtaking - beginning_of_overtaking + 90) {
                        stageMoving = TO_RIGHT_LANE_LEFT_TURN;
                        total_odometer_distance = sbd.getValueForKey_MapOfDistances(ODOMETER);
                    }
                }

                else if (stageMoving == TO_RIGHT_LANE_LEFT_TURN) {
                    cerr << "stage: TO_RIGHT_LANE_LEFT_TURN" << endl;
                    //vc.setSpeed(1);
                    vc.setSteeringWheelAngle(-1);

                    //vd.setHeading(1);

                    /* End of turning left on the right lane */

                    /* Car is moving straight now on the right turn and waiting for another obstacle to overtake */
                    relative_odometer_distance = sbd.getValueForKey_MapOfDistances(ODOMETER)-total_odometer_distance;
                    if (relative_odometer_distance >= end_of_overtaking - mid_of_overtaking) {
                        // Start over.
                        stageMoving = FORWARD;
                        stageMeasuring = FIND_OBJECT_INIT;

                        obstacle_distance = 0;
                        obstacle_distance_old = 0;
                    }
                }

                // Measuring state machine.
                if (stageMeasuring == FIND_OBJECT_INIT) {
                    cerr << "stage: FIND_OBJECT_INIT" << endl;
                    obstacle_distance_old = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);
                    stageMeasuring = FIND_OBJECT;
                }

                else if (stageMeasuring == FIND_OBJECT) {
                    cerr << "stage: FIND_OBJECT" << endl;

                    obstacle_distance = sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER);

                    // Approaching an obstacle (stationary or driving slower than us).
                    if ( (obstacle_distance > 0) && (((obstacle_distance_old - obstacle_distance) > 0) || (fabs(obstacle_distance_old - obstacle_distance) < 1e-2)) ) {
                        // Check if overtaking shall be started.
                        vd.setHeading(0);

                        stageMeasuring = FIND_OBJECT_PLAUSIBLE;
                    } else {
                        vd.setHeading(1);
                    }

                    obstacle_distance_old = obstacle_distance;
                }

                else if (stageMeasuring == FIND_OBJECT_PLAUSIBLE) {
                    cerr << "stage: FIND_OBJECT_PLAUSIBLE" << endl;
                    if (sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) < OVERTAKING_DISTANCE && sbd.getValueForKey_MapOfDistances(ULTRASONIC_FRONT_CENTER) > 20) {
                        vd.setHeading(0);
                        vc.setSpeed(0);
                        stageMoving = TO_LEFT_LANE_LEFT_TURN;

                        // Disable measuring until requested from moving state machine again.
                        stageMeasuring = DISABLE;
                    }
                    else {
                        stageMeasuring = FIND_OBJECT;
                    }
                }

                else if (stageMeasuring == HAVE_BOTH_IR) {
                    cerr << "stage: HAVE_BOTH_IR" << endl;
                    // Remain in this stage until both IRs see something.
                    //if ( (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) && (sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) > 0)) {
                    if (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) > 0) {
                        stageMoving = TO_LEFT_LANE_RIGHT_TURN;
                        //stageMoving = CONTINUE_ON_LEFT_LANE;
                    }
                }

                else if (stageMeasuring == HAVE_BOTH_IR_SAME_DISTANCE) {
                    cerr << "stage: HAVE_BOTH_IR_SAME_DISTANCE" << endl;
                    // Remain in this stage until both IRs have the similar distance to obstacle (i.e. turn car)
                    // and the driven parts of the turn are plausible.

                    FRONT_IR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    REAR_RIGHT_IR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);
                    int countdown_distance = mid_of_overtaking + 6;
                    if ((fabs(FRONT_IR - REAR_RIGHT_IR) < HEADING_PARALLEL) && FRONT_IR > 0 && (sbd.getValueForKey_MapOfDistances(ODOMETER) > countdown_distance)) {

                        stageMoving = CONTINUE_ON_LEFT_LANE;
                    }
                }

                else if (stageMeasuring == END_OF_OBJECT) {
                    cerr << "stage: END_OF_OBJECT" << endl;
                    // Find end of object.

                    //FRONT_IR = sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT);
                    //REAR_RIGHT_IR = sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT);

                    if (sbd.getValueForKey_MapOfDistances(INFRARED_FRONT_RIGHT) < 1 && sbd.getValueForKey_MapOfDistances(INFRARED_REAR_RIGHT) < 1) {
                        // Move to right lane again.
                        stageMoving = TO_RIGHT_LANE_RIGHT_TURN;

                        // Disable measuring until requested from moving state machine again.
                        stageMeasuring = DISABLE;
                    }
                }

                // Create container for finally sending the data.
                
                Container c2(vd);
                getConference().send(c2);
                // Send container.
                if(vd.getHeading()>0){
                    cerr << "Lanefollower is ON" << endl;
                }else{
                    cerr << "Lanefollower is OFF" << endl;
                    Container c(vc);
                    getConference().send(c);
                }
            }

            return odcore::data::dmcp::ModuleExitCodeMessage::OKAY;
        }

    }
} // automotive::miniature