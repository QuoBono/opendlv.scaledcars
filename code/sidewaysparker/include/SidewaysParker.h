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

#ifndef SIDEWAYSPARKER_H_
#define SIDEWAYSPARKER_H_

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "odvdscaledcarsdatamodel/GeneratedHeaders_ODVDScaledCarsDataModel.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"



namespace scaledcars {


        using namespace std;
        using namespace scaledcars;
        using namespace automotive::miniature;
        using namespace automotive;

        /**
         * This class is a skeleton to send driving commands to Hesperia-light's vehicle driving dynamics simulation.
         */
        class SidewaysParker : public odcore::base::module::TimeTriggeredConferenceClientModule {
        private:
            /**
             * "Forbidden" copy constructor. Goal: The compiler should warn
             * already at compile time for unwanted bugs caused by any misuse
             * of the copy constructor.
             *
             * @param obj Reference to an object of this class.
             */
            SidewaysParker(const SidewaysParker &/*obj*/);

            /**
             * "Forbidden" assignment operator. Goal: The compiler should warn
             * already at compile time for unwanted bugs caused by any misuse
             * of the assignment operator.
             *
             * @param obj Reference to an object of this class.
             * @return Reference to this instance.
             */
            SidewaysParker& operator=(const SidewaysParker &/*obj*/);

        public:
            /**
             * Constructor.
             *
             * @param argc Number of command line arguments.
             * @param argv Command line arguments.
             */
            SidewaysParker(const int32_t &argc, char **argv);

            virtual ~SidewaysParker();


            odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

        private:


            double distanceOld = 0;
            double absPathStart = 0;
            double absPathEnd = 0;
            double absPathParkStart = 0;
            double absPathParkEnd = 0;
            int stageMoving = 0;
            int stageMeasuring = 0;
            int parkAfterCar = 0;

            automotive::miniature::SensorBoardData data;
            automotive::VehicleData vd;
            automotive::VehicleControl vc;


            bool parked = false;

            virtual void setUp();

            virtual void tearDown();

            virtual void parallelPark();

            virtual void findSpot(automotive::VehicleControl, automotive::miniature::SensorBoardData, automotive::VehicleData);

            virtual void stop();

            virtual void forward();

            virtual void slowForward();

            virtual void reverseTurnRight();

            virtual void reverse();

            virtual void slowReverse();

            virtual void reverseTurnLeftSlow();

            virtual void nextContainer(odcore::data::Container&);
        };


} // automotive::miniature

#endif /*SIDEWAYSPARKER_H_*/