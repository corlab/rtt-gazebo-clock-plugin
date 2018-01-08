#include <time.h>
#include <climits>
#include <rtt/RTT.hpp>

#include "rtt_clock.h"
#include "rtt_clock_sim_clock_thread.h"

namespace rtt_clock {
	boost::shared_ptr<rtt_clock::SimClockThread> sim_clock_thread;
	// count the zeros...   -987654321--
	const uint64_t one_E9 = 1000000000ULL;
}

const uint64_t rtt_clock::host_now() {

	if (SimClockThread::GetInstance() && SimClockThread::GetInstance()->simTimeEnabled()) {
		return rtt_now();
	}

#ifdef __XENO__
	// Use Xenomai 2.6 feature to get the NTP-synched real-time clock
	timespec ts = {0,0};
	int ret = clock_gettime(CLOCK_HOST_REALTIME, &ts);
	if(ret) {
		RTT::log(RTT::Error) << "Could not query CLOCK_HOST_REALTIME (" << CLOCK_HOST_REALTIME <<"): "<< errno << RTT::endlog();
		return rtt_clock::rtt_now();
	}
	return (ts.tv_sec * one_E9 + ts.tv_nsec);
#else
	timespec ts = { 0, 0 };
	int ret = clock_gettime(CLOCK_REALTIME, &ts);
	if (ret) {
		RTT::log(RTT::Error) << "Could not query CLOCK_REALTIME (" << CLOCK_REALTIME << "): " << errno << RTT::endlog();
		return rtt_clock::rtt_now();
	}

	// TODO ? return ros::Time::now();

// if (g_use_sim_time)
// {
// boost::mutex::scoped_lock lock(g_sim_time_mutex);
// Time t = g_sim_time;
// return t;
// }
// Time t;
// ros_walltime(t.sec, t.nsec);
// return t;
	return (ts.tv_sec * one_E9 + ts.tv_nsec);
#endif
}

const uint64_t rtt_clock::host_wall_now() {
  #ifdef __XENO__
    // Use Xenomai 2.6 feature to get the NTP-synched real-time clock
    timespec ts = {0,0};
    int ret = clock_gettime(CLOCK_HOST_REALTIME, &ts);
    if(ret) {
      RTT::log(RTT::Error) << "Could not query CLOCK_HOST_REALTIME (" << CLOCK_HOST_REALTIME <<"): "<< errno << RTT::endlog();
      return rtt_clock::rtt_wall_now();
    }
    return (ts.tv_sec * one_E9 + ts.tv_nsec);
  #else
	// ros::WallTime now(ros::WallTime::now()); // HAS_CLOCK_GETTIME
	timespec ts = {0,0};
    clock_gettime(CLOCK_REALTIME, &ts);
	if (ts.tv_sec < 0 || ts.tv_sec > UINT_MAX)
      throw std::runtime_error("Timespec is out of dual 32-bit range");
	return (ts.tv_sec * one_E9 + ts.tv_nsec);
  #endif
}

const uint64_t rtt_clock::rtt_now() {
	// count the zeros...   -987654321--
	// const uint64_t one_E9 = 1000000000ULL;
	// NOTE: getNSecs returns wall time, getTicks returns offset time
	uint64_t nsec64 = RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
  	// uint32_t sec32_part = nsec64 / one_E9;
  	// uint32_t nsec32_part = nsec64 % one_E9;
	return nsec64;
}

const uint64_t rtt_clock::rtt_wall_now() {
	// NOTE: getNSecs returns wall time, getTicks returns offset time
	uint64_t nsec64 = RTT::os::TimeService::Instance()->getNSecs();
	// uint64_t sec64_part = nsec64 / one_E9;
	// uint64_t nsec64_part = nsec64 % one_E9;
	return nsec64;
}

const uint64_t rtt_clock::host_offset_from_rtt() {
  return rtt_clock::host_wall_now() - rtt_clock::rtt_wall_now();
}

const bool rtt_clock::enable_sim() {
	return SimClockThread::Instance()->start();
}

const bool rtt_clock::disable_sim() {
	return SimClockThread::Instance()->stop();
}

void rtt_clock::update_sim_clock(const uint64_t new_time) {
	SimClockThread::Instance()->updateClock(new_time);
}
