/**

 */

#ifndef DECISIONMAKER_H_
#define DECISIONMAKER_H_

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>

#include "opendavinci/odcore/io/conference/ContainerConference.h"
#include "opendavinci/odcore/data/Container.h"

#include "odvdscaledcarsdatamodel/GeneratedHeaders_ODVDScaledCarsDataModel.h"
#include "opendavinci/GeneratedHeaders_OpenDaVINCI.h"
#include "automotivedata/GeneratedHeaders_AutomotiveData.h"




namespace scaledcars {

        using namespace std;
        using namespace scaledcars;


    /**
         * This class is a skeleton to send driving commands to Hesperia-light's vehicle driving dynamics simulation.
         */
        class DecisionMaker : public odcore::base::module::TimeTriggeredConferenceClientModule {
            private:
                /**
                 * "Forbidden" copy constructor. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the copy constructor.
                 *
                 * @param obj Reference to an object of this class.
                 */
                DecisionMaker(const DecisionMaker &/*obj*/);

                /**
                 * "Forbidden" assignment operator. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the assignment operator.
                 *
                 * @param obj Reference to an object of this class.
                 * @return Reference to this instance.
                 */
                DecisionMaker& operator=(const DecisionMaker &/*obj*/);

            public:
                /**
                 * Constructor.
                 *
                 * @param argc Number of command line arguments.
                 * @param argv Command line arguments.
                 */
                DecisionMaker(const int32_t &argc, char **argv);




                virtual ~DecisionMaker();

                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

            private:

            bool laneFollowing = false;
            bool parking = false;
            bool overtaking = false;

            int passive = 0;
            int laneFollow = 1;
            int park = 2;
            int overtake = 3;
            int currentState = 0;


            automotive::miniature::SensorBoardData sData;
            automotive::VehicleData vData;
            automotive::VehicleControl vControl;

            virtual void setUp();

            virtual void tearDown();



        };


} // automotive::miniature

#endif /*DECISIONMAKER_H_*/
