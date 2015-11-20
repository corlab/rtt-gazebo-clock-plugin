#ifndef __rtt_clock_rtt_clock_H
#define __rtt_clock_rtt_clock_H

#include <rtt/RTT.hpp>
#include <rtt/os/TimeService.hpp>

namespace rtt_clock {

  /** \brief Get the current time according to CLOCK_HOST_REALTIME or the
   * simulation time.
   *
   * This is the time source that should always be used with ROS header
   * timestamps because it is the time that you want to use to broadcast ROS
   * messages to other machines or processes. 
   *
   * When compiled against Xenomai and not running in simulation mode,
   * this function will return the NTP-synchronized clock time via the
   * CLOCK_HOST_REALTIME clock source. Note that this is only supported under
   * Xenomai 2.6 and above.
   *
   * When not compiled against Xenomai and not running in simulation mode, it
   * is a pass-through to ros::Time::now().
   *
   * When running in simulation mode, this will always use the simulation
   * clock, which is based off of the ROS /clock topic. It is a pass-through to
   * rtt_now().
   */
  const uint64_t host_now();

  /** \brief Get the current time according to RTT
   *
   * If the simulation clock is enabled, this will return the simulated time.
   */
  const uint64_t rtt_now();

  //! Set a TaskContext to use a periodic simulation clock activity
  const bool set_sim_clock_activity(RTT::TaskContext *t);

  //! Use a simulated clock source
  const bool enable_sim();
  
  //! Do't use a simulated clock source
  const bool disable_sim();

  //! Update the current simulation time and trigger all simulated TaskContexts
  void update_sim_clock(const uint64_t new_time);
}

#endif // ifndef __rtt_clock_rtt_clock_H
