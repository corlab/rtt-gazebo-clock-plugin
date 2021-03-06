#ifndef __RTT_GAZEBO_DEPLOYER_RTT_SYSTEM_H
#define __RTT_GAZEBO_DEPLOYER_RTT_SYSTEM_H

#include <cstdlib>

// Boost
#include <boost/bind.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/deployment/ComponentLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>


#include <rtt/os/startstop.h>
#include <rtt/scripting/Scripting.hpp>

#include "rtt_clock.h"

namespace rtt_gazebo_clock {

  class RTTGazeboClockPlugin : public gazebo::SystemPlugin
  {
  public:
    //! Disconnect the world event and cleanup CORBA
    virtual ~RTTGazeboClockPlugin();

    void Load(int argc, char **argv);
    void Init();

    /**
     * \brief Update the RTT clock from the gazebo clock
     *
     * This queries the Gazebo time, and then uses the rtt_rosclock Orocos
     * plugin to both set the RTT::os::TimeService time and trigger any
     * periodic sim clock components using SimClockActivity.
     */
    void updateClock();

    void Update() { }
  private:
    //! Event connection to the world update
    gazebo::event::ConnectionPtr update_connection_;

    boost::thread update_thread_;
    boost::mutex update_mutex_;
    boost::condition_variable update_cond_;

    bool simulate_clock_;
    bool clock_changed_;
    const uint64_t one_E9 = 1000000000ll;

    void updateClockLoop();
  };

}

#endif // ifndef __RTT_GAZEBO_DEPLOYER_RTT_SYSTEM_H
