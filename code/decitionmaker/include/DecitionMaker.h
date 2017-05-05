/**

 */

#ifndef DECITIONMAKER_H_
#define DECITIONMAKER_H_

#include "opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h"

namespace automotive {
    namespace miniature {

        using namespace std;

        /**
         * This class is a skeleton to send driving commands to Hesperia-light's vehicle driving dynamics simulation.
         */
        class DecitionMaker : public odcore::base::module::TimeTriggeredConferenceClientModule {
            private:
                /**
                 * "Forbidden" copy constructor. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the copy constructor.
                 *
                 * @param obj Reference to an object of this class.
                 */
                DecitionMaker(const DecitionMaker &/*obj*/);

                /**
                 * "Forbidden" assignment operator. Goal: The compiler should warn
                 * already at compile time for unwanted bugs caused by any misuse
                 * of the assignment operator.
                 *
                 * @param obj Reference to an object of this class.
                 * @return Reference to this instance.
                 */
                DecitionMaker& operator=(const DecitionMaker &/*obj*/);

            public:
                /**
                 * Constructor.
                 *
                 * @param argc Number of command line arguments.
                 * @param argv Command line arguments.
                 */
                DecitionMaker(const int32_t &argc, char **argv);

                virtual ~DecitionMaker();

                odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

            private:
                virtual void setUp();

                virtual void tearDown();

            virtual void stop();

            virtual void forward();

            virtual void slowForward();

            virtual void reverse();

            virtual void slowReverse();

        };

    }
} // automotive::miniature

#endif /*DECITIONMAKER_H_*/
