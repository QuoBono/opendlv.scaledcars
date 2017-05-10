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

#ifndef OVERTAKER_H_
#define OVERTAKER_H_

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "odvdscaledcarsdatamodel/GeneratedHeaders_ODVDScaledCarsDataModel.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"

namespace scaledcars {

        using namespace std;
        using namespace scaledcars;
        using namespace automotive;
        using namespace automotive::miniature;

        /**
         * This class is a skeleton to send driving commands to Hesperia-light's vehicle driving dynamics simulation.
         */
        class Overtaker : public odcore::base::module::TimeTriggeredConferenceClientModule {
            private:
                /**
                 * "Forbidden" copy constructor. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the copy constructor.
                 *
                 * @param obj Reference to an object of this class.
                 */
                Overtaker(const Overtaker &/*obj*/);

                /**
                 * "Forbidden" assignment operator. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the assignment operator.
                 *
                 * @param obj Reference to an object of this class.
                 * @return Reference to this instance.
                 */
                Overtaker& operator=(const Overtaker &/*obj*/);

            public:
                /**
                 * Constructor.
                 *
                 * @param argc Number of command line arguments.
                 * @param argv Command line arguments.
                 */
                Overtaker(const int32_t &argc, char **argv);

                virtual ~Overtaker();

                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

            private:
                virtual void setUp();

                virtual void tearDown();

                virtual void nextContainer(odcore::data::Container&);

                //virtual int ls(const char *dir);
        };

    }
 // automotive::miniature

#endif /*OVERTAKER_H_*/
