/**

 */

#ifndef SIDEWAYSPARKER_H_
#define SIDEWAYSPARKER_H_

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"

namespace automotive {
    namespace miniature {

        using namespace std;

        /**
         * This class is a skeleton to send driving commands to Hesperia-light's vehicle driving dynamics simulation.
         */
        class SidewaysParker {

        public:
            /**
             * Constructor.
             *
             * @param argc Number of command line arguments.
             * @param argv Command line arguments.
             */
            SidewaysParker();

            virtual ~SidewaysParker();

            void parallelPark(automotive::miniature::SensorBoardData);

            bool isParked();

            bool findParkingSpot(automotive::miniature::SensorBoardData, automotive::VehicleData);

        private:


            void stop();

            void forward();

            void slowForward();

            void reverseTurnRight();

            void reverse();

            void slowReverse();

            void reverseTurnLeftSlow();
        };

    }
}


#endif /*SIDEWAYSPARKER_H_*/
