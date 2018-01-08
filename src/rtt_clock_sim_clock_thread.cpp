/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Johannes Meyer, TU Darmstadt
 *  Copyright (c) 2013, Intermodalics BVBA
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of TU Darmstadt and Intermodalics BVBA
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Copyright (c) 2014, Jonathan Bohren, The Johns Hopkins University
 *  - Generalized for multiple time sources
 *  - Integrated with rtt_clock package
 *********************************************************************/

#include "rtt_clock_sim_clock_thread.h"

#include <rtt/TaskContext.hpp>
#include <rtt/internal/GlobalService.hpp>
#include <rtt/plugin/Plugin.hpp>

#include <rtt/os/StartStopManager.hpp>

#include "rtt_clock.h"

using namespace rtt_clock;

boost::shared_ptr<SimClockThread> SimClockThread::singleton;

boost::shared_ptr<SimClockThread> SimClockThread::GetInstance() {
	return singleton;
}

boost::shared_ptr<SimClockThread> SimClockThread::Instance() {
	// Create a new singleton, if necessary
	boost::shared_ptr<SimClockThread> shared = GetInstance();
	if (!shared) {
		shared.reset(new SimClockThread());
		singleton = shared;
	}

	return shared;
}

void SimClockThread::Release() {
	singleton.reset();
}

namespace {
RTT::os::CleanupFunction cleanup(&SimClockThread::Release);
}

SimClockThread::SimClockThread() :
								RTT::os::Thread(ORO_SCHED_OTHER, RTT::os::LowestPriority, 0.0, 0,
								"rtt_clock_SimClockThread"),
								time_service_(RTT::os::TimeService::Instance()),
								process_callbacks_(false) {
}

SimClockThread::~SimClockThread() {
	this->stop();
}

bool SimClockThread::simTimeEnabled() const {
	return this->isActive();
}

bool SimClockThread::updateClock(const RTT::nsecs new_time) {
	return this->updateClockInternal(new_time);
}

bool SimClockThread::updateClockInternal(const RTT::nsecs new_time) {

	// Make sure the system time isn't being used
	if (time_service_->systemClockEnabled()) {
		time_service_->enableSystemClock(false);
	}

	// Check if time restarted
	if (new_time == 0) {
//		RTT::log(RTT::Warning)
//				<< "Time has reset to 0! Re-setting time service."
//				<< RTT::endlog();

		// Re-set the time service and don't update the activities
		this->resetTimeService();
	} else {
		// Update the RTT time to match the sim time
		using namespace RTT::os;
		//TimeService::ticks rtt_ticks = time_service_->getTicks();
		//TimeService::Seconds rtt_secs = RTT::nsecs_to_Seconds(TimeService::ticks2nsecs(rtt_ticks));

		// Compute the time update
		TimeService::Seconds dt = RTT::nsecs_to_Seconds(new_time - rtt_clock::rtt_now());

		// Check if time went backwards
		if (dt < 0) {
			RTT::log(RTT::Warning) << "Time went backwards by " << dt
					<< " seconds! (" << rtt_clock::rtt_now() << " --> "
					<< new_time << ")" << RTT::endlog();
		}

		// Update the RTT clock
		time_service_->secondsChange(dt);
	}

	return true;
}

void SimClockThread::resetTimeService() {
	// We have to set the Logger reference time to zero in order to get correct logging timestamps.
	// RTT::Logger::Instance()->setReferenceTime(0);
	//
	// Unfortunately this method is not available, therefore shutdown and restart logging.
	// This workaround is not exact.

	// Shutdown the RTT Logger
	RTT::Logger::Instance()->shutdown();

	// Disable the RTT system clock so Gazebo can manipulate time and reset it to 0
	time_service_->enableSystemClock(false);
	assert(time_service_->systemClockEnabled() == false);

	time_service_->ticksChange(-time_service_->ticksSince(0));
	assert(time_service_->getTicks() == 0);

	// Restart the RTT Logger with reference time 0
	RTT::Logger::Instance()->startup();
	assert(RTT::Logger::Instance()->getReferenceTime() == 0);
}

bool SimClockThread::initialize() {

	RTT::log(RTT::Debug)
			<< "[rtt_clock] Attempting to enable global simulation clock source..."
			<< RTT::endlog();

	RTT::log(RTT::Debug)
			<< "[rtt_clock] Switching to simulated time based on a manual clock source..."
			<< RTT::endlog();

	// Reset the timeservice and logger
	this->resetTimeService();

	// We're not processing the callback queue, so we won't loop.
	process_callbacks_ = false;

	return true;
}

void SimClockThread::finalize() {
	RTT::log(RTT::Info) << "Disabling simulated time..." << RTT::endlog();

	// Shutdown the RTT Logger
	RTT::Logger::Instance()->shutdown();

	// Re-enable system clock
	time_service_->enableSystemClock(true);

	// Restart the RTT Logger with reference walltime
	RTT::Logger::Instance()->startup();
}


