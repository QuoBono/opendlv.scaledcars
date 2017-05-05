/**
 * Proxy - Proxy code.
 * Copyright (C) 2016 Christian Berger
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

#ifndef CONTROL_PROXY_H
#define CONTROL_PROXY_H

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/data/Container.h>

namespace scaledcars {
namespace control {

using namespace std;

/**
 * Time-triggered proxy.
 */
class Proxy : public odcore::base::module::TimeTriggeredConferenceClientModule {
   private:
    Proxy(const Proxy & /*obj*/) = delete;
    Proxy &operator=(const Proxy & /*obj*/) = delete;

   public:
    /**
     * Constructor.
     *
     * @param argc Number of command line arguments.
     * @param argv Command line arguments.
     */
    Proxy(const int &argc, char **argv);

    virtual ~Proxy();

    virtual void nextContainer(odcore::data::Container &c);

   private:
    void setUp();
    void tearDown();
    odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();
};
}
} // scaledcars::control

#endif /*CONTROL_PROXY_H*/
