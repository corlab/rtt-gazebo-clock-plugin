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
#include <rtt/transports/corba/corba.h>
#include <rtt/transports/corba/TaskContextServer.hpp>

// RTT/ROS Simulation Clock Activity
#include "rtt_clock.h"

#include "rtt_system_plugin.h"

using namespace rtt_gazebo_system;

void RTTSystemPlugin::Load(int argc, char **argv) {
	// Initialize RTT
	__os_init(argc, argv);

	RTT::Logger::log().setStdStream(std::cerr);
	RTT::Logger::log().mayLogStdOut(true);
	RTT::Logger::log().setLogLevel(RTT::Logger::Warning);

	// Setup TaskContext server if necessary
	if (CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
		// Initialize orb
		RTT::corba::TaskContextServer::InitOrb(argc, argv);
		// Propcess orb requests in a thread
		RTT::corba::TaskContextServer::ThreadOrb();
	}

}

void RTTSystemPlugin::Init() {
	// Initialize and enable the simulation clock
	rtt_clock::enable_sim();

	update_connection_ = gazebo::event::Events::ConnectWorldUpdateEnd(
			boost::bind(&RTTSystemPlugin::updateClock, this));

	simulate_clock_ = true;
	clock_changed_ = false;

//	stop_connection_ = gazebo::event::Events::ConnectStop(
//			boost::bind(&RTTSystemPlugin::stop, this));

// Start update thread
	update_thread_ = boost::thread(
			boost::bind(&RTTSystemPlugin::updateClockLoop, this));
}

RTTSystemPlugin::~RTTSystemPlugin() {
	simulate_clock_ = false;
	if (update_thread_.joinable()) {
		update_thread_.join();
	}

	// Stop the Orb thread
	if (!CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
		RTT::corba::TaskContextServer::ShutdownOrb();
		RTT::corba::TaskContextServer::DestroyOrb();
		RTT::corba::TaskContextServer::CleanupServers();
	}
}

void RTTSystemPlugin::updateClock() {
	boost::mutex::scoped_lock lock(update_mutex_);
	clock_changed_ = true;
	update_cond_.notify_one();

	//// Notify the update clock loop
	//boost::lock_guard<boost::mutex> lock(update_mutex_);
	//update_cond_.notify_one();
}

//void RTTSystemPlugin::stop() {
//	simulate_clock_ = false;
//}

void RTTSystemPlugin::updateClockLoop() {
//
//	// Wait for update signal to start the loop
//	{
//		boost::unique_lock<boost::mutex> lock(update_mutex_);
//		update_cond_.wait(lock);
//	}
//
//	while (simulate_clock_) {
//		// Get the simulation time
//		if (!gazebo::physics::worlds_running()) {
//			break;
//		}
//		const gazebo::physics::WorldPtr world = gazebo::physics::get_world();
//		if (!world) {
//			break;
//		}
//		// Get the simulation time
//		gazebo::common::Time gz_time = world->GetSimTime();
//
//		// Update the clock from the simulation time and execute the SimClockActivities
//		// NOTE: all Orocos TaskContexts which use a SimClockActivity are updated within this call
//		rtt_clock::update_sim_clock(gz_time.sec * one_E9 + gz_time.nsec);
//	}

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

GZ_REGISTER_SYSTEM_PLUGIN(rtt_gazebo_system::RTTSystemPlugin)

