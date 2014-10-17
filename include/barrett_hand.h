#include "ros/ros.h"
// The file below provides access to the barrett::units namespace.
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>
#include <barrett/systems/wam.h>
#include <barrett/detail/stl_utils.h>
#include <barrett/math.h>

#include <std_srvs/Empty.h>
#include <wam_msgs/BHandFingerPos.h>
#include <wam_msgs/BHandGraspPos.h>
#include <wam_msgs/BHandSpreadPos.h>
#include <wam_msgs/BHandFingerVel.h>
#include <wam_msgs/BHandGraspVel.h>
#include <wam_msgs/BHandSpreadVel.h>

#include <sensor_msgs/JointState.h>


using namespace barrett;

const int RATE = 200; //Execution rate in Hz
const char* bhand_jnts[] = {"inner_f1", "inner_f2", "inner_f3", "spread", "outer_f1", "outer_f2", "outer_f3"};

const std::string HAND_JS_TOPIC = "hand/joint_states";

const std::string HAND_OPN_GRSP_SRV = "hand/open_grasp";
const std::string HAND_CLS_GRSP_SRV = "hand/close_grasp";
const std::string HAND_OPN_SPRD_SRV = "hand/open_spread";
const std::string HAND_CLS_SPRD_SRV = "hand/close_spread";
const std::string HAND_FNGR_POS_SRV = "hand/finger_pos";
const std::string HAND_FNGR_VEL_SRV = "hand/finger_vel";
const std::string HAND_GRSP_POS_SRV = "hand/grasp_pos";
const std::string HAND_GRSP_VEL_SRV = "hand/grasp_vel";
const std::string HAND_SPRD_POS_SRV = "hand/spread_pos";
const std::string HAND_SPRD_VEL_SRV = "hand/spread_vel";

template<size_t DOF>
class BarrettHandInterface{
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	bool isInitialized; //Tracks if the hand is initialized
	std_srvs::Empty::Request def_req;
	std_srvs::Empty::Response def_res;
	sensor_msgs::JointState js;

	systems::Wam<DOF>* hand_wam;
	ProductManager* hand_pm;
	Hand* hand;
	ros::NodeHandle nh;
	ros::Publisher hand_js_pub;
    ros::ServiceServer hand_open_grsp_srv, hand_close_grsp_srv, hand_open_sprd_srv;
    ros::ServiceServer hand_close_sprd_srv, hand_fngr_pos_srv, hand_fngr_vel_srv;
    ros::ServiceServer hand_grsp_pos_srv, hand_grsp_vel_srv, hand_sprd_pos_srv;
    ros::ServiceServer hand_sprd_vel_srv;

    bool handInitialize(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
    bool handOpenGrasp(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
    bool handCloseGrasp(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
    bool handOpenSpread(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
    bool handCloseSpread(std_srvs::Empty::Request &, std_srvs::Empty::Response &);
    bool handFingerPos(wam_msgs::BHandFingerPos::Request &, wam_msgs::BHandFingerPos::Response &);
    bool handFingerVel(wam_msgs::BHandFingerVel::Request &, wam_msgs::BHandFingerVel::Response &);
    bool handGraspPos(wam_msgs::BHandGraspPos::Request &, wam_msgs::BHandGraspPos::Response &);
    bool handGraspVel(wam_msgs::BHandGraspVel::Request &, wam_msgs::BHandGraspVel::Response &);
    bool handSpreadPos(wam_msgs::BHandSpreadPos::Request &, wam_msgs::BHandSpreadPos::Response &);
    bool handSpreadVel(wam_msgs::BHandSpreadVel::Request &, wam_msgs::BHandSpreadVel::Response &);
    bool publishHandInfo();

public:
    BarrettHandInterface(ProductManager &pm, systems::Wam<DOF> &wam): hand_wam(&wam), hand_pm(&pm), hand(NULL), isInitialized(false){};
    void start();
};

/* Initializes the hand if it is not already initialized
 * returns if the hand was initialized
 */
template<size_t DOF>
bool BarrettHandInterface<DOF>::handInitialize(std_srvs::Empty::Request &req = def_req, std_srvs::Empty::Response &res = def_res){

	if(!isInitialized){
		hand->initialize();
		ROS_INFO("The Hand is Initialized");
		isInitialized = true;
	}

	return true;;
}

//Function to open the BarrettHand Grasp
template<size_t DOF>
bool BarrettHandInterface<DOF>::handOpenGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

	if(!isInitialized)
		handInitialize();

    ROS_INFO("Opening the BarrettHand Grasp");
	hand->open(Hand::GRASP, false);
	return true;
}

//Function to close the BarrettHand Grasp
template<size_t DOF>
bool BarrettHandInterface<DOF>::handCloseGrasp(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

	if(!isInitialized)
		handInitialize();

    ROS_INFO("Closing the BarrettHand Grasp");
    hand->close(Hand::GRASP, false);
    return true;
  }

//Function to open the BarrettHand Spread
template<size_t DOF>
bool BarrettHandInterface<DOF>::handOpenSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

	if(!isInitialized)
		handInitialize();

    ROS_INFO("Opening the BarrettHand Spread");
    hand->open(Hand::SPREAD, false);
    return true;
  }

//Function to close the BarrettHand Spread
template<size_t DOF>
bool BarrettHandInterface<DOF>::handCloseSpread(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){

	if(!isInitialized)
		handInitialize();

	ROS_INFO("Closing the BarrettHand Spread");
    hand->close(Hand::SPREAD, false);
    return true;
  }

//Function to control a BarrettHand Finger Position
template<size_t DOF>
bool BarrettHandInterface<DOF>::handFingerPos(wam_msgs::BHandFingerPos::Request &req, wam_msgs::BHandFingerPos::Response &res){

	if(!isInitialized)
		handInitialize();

    ROS_INFO("Moving BarrettHand to Finger Positions: %.3f, %.3f, %.3f radians", req.radians[0], req.radians[1],
             req.radians[2]);
    hand->trapezoidalMove(Hand::jp_type(req.radians[0], req.radians[1], req.radians[2], 0.0), Hand::GRASP, false);
    return true;
  }

//Function to control a BarrettHand Finger Velocity
template<size_t DOF>
bool BarrettHandInterface<DOF>::handFingerVel(wam_msgs::BHandFingerVel::Request &req, wam_msgs::BHandFingerVel::Response &res){

	if(!isInitialized)
		handInitialize();

    ROS_INFO("Moving BarrettHand Finger Velocities: %.3f, %.3f, %.3f m/s", req.velocity[0], req.velocity[1],
             req.velocity[2]);
    hand->velocityMove(Hand::jv_type(req.velocity[0], req.velocity[1], req.velocity[2], 0.0), Hand::GRASP);
    return true;
  }

//Function to control the BarrettHand Grasp Position
template<size_t DOF>
bool BarrettHandInterface<DOF>::handGraspPos(wam_msgs::BHandGraspPos::Request &req, wam_msgs::BHandGraspPos::Response &res){

	if(!isInitialized)
		handInitialize();

    ROS_INFO("Moving BarrettHand Grasp: %.3f radians", req.radians);
    hand->trapezoidalMove(Hand::jp_type(req.radians), Hand::GRASP, false);
    return true;
  }

//Function to control a BarrettHand Grasp Velocity
template<size_t DOF>
bool BarrettHandInterface<DOF>::handGraspVel(wam_msgs::BHandGraspVel::Request &req, wam_msgs::BHandGraspVel::Response &res){

	if(!isInitialized)
		handInitialize();

    ROS_INFO("Moving BarrettHand Grasp: %.3f m/s", req.velocity);
    hand->velocityMove(Hand::jv_type(req.velocity), Hand::GRASP);
    return true;
  }

//Function to control the BarrettHand Spread Position
template<size_t DOF>
bool BarrettHandInterface<DOF>::handSpreadPos(wam_msgs::BHandSpreadPos::Request &req, wam_msgs::BHandSpreadPos::Response &res){

	if(!isInitialized)
		handInitialize();

    ROS_INFO("Moving BarrettHand Spread: %.3f radians", req.radians);
    hand->trapezoidalMove(Hand::jp_type(req.radians), Hand::SPREAD, false);
    return true;
  }

//Function to control a BarrettHand Spread Velocity
template<size_t DOF>
bool BarrettHandInterface<DOF>::handSpreadVel(wam_msgs::BHandSpreadVel::Request &req, wam_msgs::BHandSpreadVel::Response &res){

	if(!isInitialized)
		handInitialize();

    ROS_INFO("Moving BarrettHand Spread: %.3f m/s", req.velocity);
    usleep(5000);
    hand->velocityMove(Hand::jv_type(req.velocity), Hand::SPREAD);
    return true;
  }

//Function to publish the joint states of the hand at desired rate
template<size_t DOF>
bool BarrettHandInterface<DOF>::publishHandInfo(){

	//Set the loop rate at 200 Hz
	ros::Rate loop_rate(RATE);

	while (ros::ok())
	    {
	      hand->update(); // Update the hand sensors
	      Hand::jp_type hi = hand->getInnerLinkPosition(); // get finger positions information
	      Hand::jp_type ho = hand->getOuterLinkPosition();
	      for (size_t i = 0; i < 4; i++) // Save finger positions
	        js.position[i] = hi[i];
	      for (size_t j = 0; j < 3; j++)
	        js.position[j + 4] = ho[j];
	      js.header.stamp = ros::Time::now(); // Set the timestamp
	      hand_js_pub.publish(js); // Publish the BarrettHand joint states
	      loop_rate.sleep();
	    }
}
template<size_t DOF>
void BarrettHandInterface<DOF>::start(){

//Check if the hand is present
	if(hand_pm->getHand()){

		//Initalize the publishers
		hand_js_pub = nh.advertise<sensor_msgs::JointState>(HAND_JS_TOPIC, 1);

	    //Advertise the following services only if there is a BarrettHand present
		hand_open_grsp_srv = nh.advertiseService(HAND_OPN_GRSP_SRV, &BarrettHandInterface<DOF>::handOpenGrasp, this);
		hand_close_grsp_srv = nh.advertiseService(HAND_CLS_GRSP_SRV, &BarrettHandInterface<DOF>::handCloseGrasp, this);
		hand_open_sprd_srv = nh.advertiseService(HAND_OPN_SPRD_SRV, &BarrettHandInterface<DOF>::handOpenSpread, this);
		hand_close_sprd_srv = nh.advertiseService(HAND_CLS_SPRD_SRV, &BarrettHandInterface<DOF>::handCloseSpread, this);
		hand_fngr_pos_srv = nh.advertiseService(HAND_FNGR_POS_SRV, &BarrettHandInterface<DOF>::handFingerPos, this);
		hand_sprd_vel_srv = nh.advertiseService(HAND_FNGR_VEL_SRV, &BarrettHandInterface<DOF>::handSpreadVel, this);
		hand_grsp_pos_srv = nh.advertiseService(HAND_GRSP_POS_SRV, &BarrettHandInterface<DOF>::handGraspPos, this);
		hand_grsp_vel_srv = nh.advertiseService(HAND_GRSP_VEL_SRV, &BarrettHandInterface<DOF>::handGraspVel, this);
		hand_sprd_pos_srv = nh.advertiseService(HAND_SPRD_POS_SRV, &BarrettHandInterface<DOF>::handSpreadPos, this);
		hand_fngr_vel_srv = nh.advertiseService(HAND_SPRD_VEL_SRV, &BarrettHandInterface<DOF>::handFingerVel, this);

		/* TODO Set the safety limits
		 * Move j3 to give room for hand initialization
		 */
	    //Set up the BarrettHand joint state publisher
	    std::vector <std::string> bhand_joints(bhand_jnts, bhand_jnts + 7);
	    js.name.resize(7);
	    js.name = bhand_joints;
	    js.position.resize(7);

	    publishHandInfo();


	}
}