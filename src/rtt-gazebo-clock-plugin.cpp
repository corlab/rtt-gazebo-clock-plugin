#include "rtt-gazebo-clock-plugin.h"

#include <cstdlib>

// Boost
#include <boost/bind.hpp>
#include <boost/thread/locks.hpp>

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

// RTT/ROS Simulation Clock Activity
#include "rtt_clock.h"


using namespace rtt_gazebo_clock;

void RTTGazeboClockPlugin::Load(int argc, char **argv) {
	// Initialize RTT
	__os_init(argc, argv);

	RTT::Logger::log().setStdStream(std::cerr);
	RTT::Logger::log().mayLogStdOut(true);
	RTT::Logger::log().setLogLevel(RTT::Logger::Info);
}

void RTTGazeboClockPlugin::Init() {
	// Initialize and enable the simulation clock
	rtt_clock::enable_sim();

	update_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd(
			boost::bind(&RTTGazeboClockPlugin::updateClock, this));

	simulate_clock_ = true;
	clock_changed_ = false;

	// Start update thread
	update_thread_ = boost::thread(
			boost::bind(&RTTGazeboClockPlugin::updateClockLoop, this));
}

RTTGazeboClockPlugin::~RTTGazeboClockPlugin() {
	simulate_clock_ = false;
	if (update_thread_.joinable()) {
		update_thread_.join();
	}
}

void RTTGazeboClockPlugin::updateClock() {
	boost::mutex::scoped_lock lock(update_mutex_);
	clock_changed_ = true;
	update_cond_.notify_one();
}

void RTTGazeboClockPlugin::updateClockLoop() {
	while (simulate_clock_) {
		boost::unique_lock<boost::mutex> lock(update_mutex_);
		while (not clock_changed_) {
			update_cond_.wait(lock);
		}
		// Get the simulation time
		gazebo::common::Time gz_time =
				gazebo::physics::get_world()->GetSimTime();

		//Update the clock from the simulation time and execute the SimClockActivities
		// NOTE: all Orocos TaskContexts which use a SimClockActivity are updated within this call
		rtt_clock::update_sim_clock(gz_time.sec * one_E9 + gz_time.nsec);

	}
}

GZ_REGISTER_SYSTEM_PLUGIN(rtt_gazebo_clock::RTTGazeboClockPlugin)

