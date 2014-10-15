/*
 * hand.cpp
 *
 *  Created on: Oct 15, 2014
 *      Author: robot
 */


#include "ros/ros.h"
// The file below provides access to the barrett::units namespace.
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/math.h>

#include <barrett/standard_main_function.h>

using namespace barrett;
using detail::waitForEnter;

template<size_t DOF>
int wam_main(int argc, char** argv, ProductManager& pm, systems::Wam<DOF>& wam) {

	// Attempt to connect to the hand (BH-280 or newer)
	  Hand* hand = NULL;
	  if (pm.foundHand()) {
	    hand = pm.getHand();
	    std::cout << ">>> Press [Enter] to initialize Hand. (Make sure it has room!)";
	    waitForEnter();
	    hand->initialize();
	  } else {
	    ROS_ERROR("No hand detected! Quitting...");
	    // Wait for the user to press Shift+Idle
	    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	    return 0;
	  }
}
