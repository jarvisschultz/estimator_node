// nu_objecttracker.cpp
// Jake Ware and Jarvis Schultz
// Winter 2011

//---------------------------------------------------------------------------
// Notes
//---------------------------------------------------------------------------
/*

 */


//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <ros/ros.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/State.h>
#include <puppeteer_msgs/position_request.h>

#include <visualization_msgs/Marker.h>

#include <math.h>

//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class StateEstimator {

private:
    ros::NodeHandle n_;
    ros::Subscriber tracker_sub;
    ros::Publisher state_pub;
    ros::Publisher marker_pub;
    ros::Time t_now, t_last;
    ros::ServiceClient position_request_client;

    puppeteer_msgs::State state;  // current and previous system states
    puppeteer_msgs::position_request position_request_srv;

    bool error_m;
    bool error_c;
    float xm, ym, zm, xm_last, ym_last, zm_last;  // current and previous mass positions
    float xm_dot, ym_dot, zm_dot, xm_dot_last, ym_dot_last, zm_dot_last;  // current and previous mass velocities
    float xc, zc, xc_last, zc_last;  // current and previous cart positions
    float xc_dot, zc_dot, xc_dot_last, zc_dot_last;  // current and previous cart velocities
    float r, r_last;  // current and previous string lengths
    float r_dot, r_dot_last;  // current and previous rate of change of string length
    float th, th_last;
    float th_dot, th_dot_last;
    float dt;

public:
    StateEstimator() {
	tracker_sub = n_.subscribe("/object1_position", 1, &StateEstimator::trackercb, this);
	state_pub = n_.advertise<puppeteer_msgs::State> ("system_state", 100);
	marker_pub = n_.advertise<visualization_msgs::Marker>("visualization_markers", 1);
	position_request_client = n_.serviceClient<puppeteer_msgs::position_request>("position_request");
	t_now = ros::Time::now();

	// initialize communication error parameters to true for safety
	error_m = true;
	error_c = true;

	// initialize all state variables to zero
	xm = 0.0; ym = 0; zm = 0; xm_last = 0; ym_last = 0; zm_last = 0; 
	xm_dot = 0; ym_dot = 0; zm_dot = 0; xm_dot_last = 0; ym_dot_last = 0; zm_dot_last = 0;
	xc = 0.0; zc = 0.0; xc_last = 0.0; zc_last = 0.0;
	xc_dot = 0.0; zc_dot = 0.0; xc_dot_last = 0.0; zc_dot_last = 0.0;
	r = 0.0; r_last = 0.0; 
	r_dot = 0.0; r_dot_last = 0.0;
	th = 0.0; th_last = 0.0;
	th_dot = 0.0; th_dot_last = 0.0;

	dt = 0.0;
    }

    void trackercb(const puppeteer_msgs::PointPlus &point) {
	ROS_DEBUG("Entered tracker callback");
    
	// check if the operating_condition parameter exists and set its value
	int operating_condition = 0;
	static int first_flag = 0;
	if(ros::param::has("operating_condition")) {
	    if(first_flag == 0)
	    {
		first_flag++;
		ros::param::set("/operating_condition", 0);
	    }
	    else
		ros::param::get("/operating_condition", operating_condition);
	}
	else {
	    ROS_WARN("Cannot Find Parameter: operating_condition");
	    ROS_INFO("Setting operating_condition to IDLE");
	    ros::param::set("/operating_condition", 0);
      
	    return;
	}

	// check to see if we are in calibrate or run state
	if(operating_condition == 1 || operating_condition == 2) {
	    
	    // get error flag from object tracker data
	    error_m = point.error;

	
	    // request robot position
	    position_request_srv.request.robot_index = 2;  // hardcoding this for now
	    position_request_srv.request.type = 'w';
	    position_request_srv.request.Vleft = 0.0;
	    position_request_srv.request.Vright = 0.0;
	    position_request_srv.request.Vtop = 0.0;
	    position_request_srv.request.div = 0;

	    // call service and store error flag
	    if(position_request_client.call(position_request_srv)) {
		error_c = position_request_srv.response.error;
	    }
	    // print error if the service call failed (not the same as a successful service call with a bad reply)
	    else {
		ROS_ERROR("Failed to call service: position_request");
	    }

	    
	    // get current time and dt to last call
	    t_last = t_now;
	    t_now = ros::Time::now();
	    dt = (t_now.toSec()-t_last.toSec());
	    ROS_DEBUG("dt: %f", dt);

	    // store last positions and velocities
	    xm_last = xm;
	    ym_last = ym;
	    zm_last = zm;
	    xm_dot_last = xm_dot;
	    ym_dot_last = ym_dot;
	    zm_dot_last = zm_dot;
	    xc_last = xc;
	    zc_last = zc;
	    xc_dot_last = xc_dot;
	    zc_dot_last = zc_dot;
	    r_last = r;
	    r_dot_last = r_dot;
	    th_last = th;
	    th_dot_last = th_dot;

	    // did we get good mass position data from the object tracker and robot?
	    if(error_m == false && error_c == false) {
		ROS_DEBUG("Successful update to mass and cart");

		// get new mass position data
		xm = point.x;
		ym = point.y;
		zm = point.z;

		// get new robot position data
		xc = position_request_srv.response.xc;
		zc = position_request_srv.response.zc;
		th = position_request_srv.response.th;

		// calculate string length
		// ignore z dimension for now and assume planar motion in x and y
		r = sqrt(powf((xc-xm),2)+powf((ym),2));

		// calculate new velocities
		xm_dot = (xm-xm_last)/dt;
		ym_dot = (ym-ym_last)/dt;
		zm_dot = (zm-zm_last)/dt;
		xc_dot = (xc-xc_last)/dt;
		zc_dot = (zc-zc_last)/dt;
		r_dot = (r-r_last)/dt;
      
		// assign various components of system state
		state.xm = xm;
		state.ym = ym;
		state.xc = xc;
		state.r = r;
		state.xm_dot = xm_dot;
		state.ym_dot = ym_dot;
		state.xc_dot = xc_dot;
		state.r_dot = r_dot;

		// publish system state
		state_pub.publish(state);
	    }

	    // we missed the mass update but got the robot update
	    else if(error_m == true && error_c == false) {
		ROS_WARN("Missed mass update");

		// use old mass velocity values
		xm_dot = xm_dot_last;
		ym_dot = ym_dot_last;
		zm_dot = zm_dot_last;

		// calculate new mass position given these velocities
		xm += xm_dot*dt;
		ym += ym_dot*dt;
		zm += zm_dot*dt;

		// get new robot position data
		xc = position_request_srv.response.xc;
		zc = position_request_srv.response.zc;
		th = position_request_srv.response.th;

		// calculate string length
		// ignore z dimension for now and assume planar motion in x and y
		r = sqrt(powf((xc-xm),2)+powf((ym),2));

		// calculate new velocities
		xc_dot = (xc-xc_last)/dt;
		zc_dot = (zc-zc_last)/dt;
		r_dot = (r-r_last)/dt;
            
		// assign various components of system state
		state.xm = xm;
		state.ym = ym;
		state.xc = xc;
		state.r = r;
		state.xm_dot = xm_dot;
		state.ym_dot = ym_dot;
		state.xc_dot = xc_dot;
		state.r_dot = r_dot;

		// publish system state
		state_pub.publish(state);
	    }

	    // we missed the robot update but got the mass update
	    else if(error_m == false && error_c == true) {
		ROS_WARN("Missed cart update");
      
		// get new mass position data
		xm = point.x;
		ym = point.y;
		zm = point.z;

		// use old robot velocity values
		xc_dot = xc_dot_last;
		zc_dot = zc_dot_last;
      
		// calculate new robot position given these velocities
		xc += xc_dot*dt;
		zc += zc_dot*dt;
      
		// calculate string length
		// ignore z dimension for now and assume planar motion in x and y
		r = sqrt(powf((xc-xm),2)+powf((ym),2));

		// calculate new velocities
		xm_dot = (xm-xm_last)/dt;
		ym_dot = (ym-ym_last)/dt;
		zm_dot = (zm-zm_last)/dt;

		// assign various components of system state
		state.xm = xm;
		state.ym = ym;
		state.xc = xc;
		state.r = r;
		state.xm_dot = xm_dot;
		state.ym_dot = ym_dot;
		state.xc_dot = xc_dot;
		state.r_dot = r_dot;

		// publish system state
		state_pub.publish(state);
	    }
	    // otherwise, we missed both robot and mass state updates
	    else {
		ROS_WARN("Missed both mass and cart updates");

		// use old mass velocity values
		xm_dot = xm_dot_last;
		ym_dot = ym_dot_last;
		zm_dot = zm_dot_last;

		// calculate new mass position given these velocities
		xm += xm_dot*dt;
		ym += ym_dot*dt;
		zm += zm_dot*dt;

		// use old robot velocity values
		xc_dot = xc_dot_last;
		zc_dot = zc_dot_last;

		// calculate string length
		// ignore z dimension for now and assume planar motion in x and y
		r = sqrt(powf((xc-xm),2)+powf((ym),2));

		// calculate new velocities
		xm_dot = (xm-xm_last)/dt;
		ym_dot = (ym-ym_last)/dt;
		zm_dot = (zm-zm_last)/dt;

		// assign various components of system state
		state.xm = xm;
		state.ym = ym;
		state.xc = xc;
		state.r = r;
		state.xm_dot = xm_dot;
		state.ym_dot = ym_dot;
		state.xc_dot = xc_dot;
		state.r_dot = r_dot;

		// publish system state
		state_pub.publish(state);
	    }
	}
    
	// are we in idle or stop condition?
	else if(operating_condition == 0 || operating_condition == 3) {
	    ROS_DEBUG("Estimator node is idle due to operating condition");
	} 
    
	// are we in emergency stop condition?
	else if(operating_condition == 4) {
	    // did we get an emergency stop request?
	    static bool emergency_flag = false;
	    if(operating_condition == 4 && emergency_flag == false) {
		ROS_WARN("Emergency Stop Requested");
		emergency_flag = true;
	    }
	}
    
	// otherwise something terrible has happened
	else {
	    ROS_ERROR("Invalid value for operating_condition");
	}

	ROS_DEBUG("Leaving tracker callback");
    }

// void SetState(int estimate_flag, float x, float y, float th, float dt)
// {
//   /* So this function takes in a new value for the pose of the */
//   /* robot, and a flag that indicates what parameters need to be */
//   /* estimated.  If the robot successfully returned new pose */
//   /* values, we only need to estimate the velocities of the robot; */
//   /* if not we need to estimate the whole state. */

//   // Was it a successful read?
//   if(estimate_flag == 1)
//     {
// 	// Just estimate velocities
// 	x_pos_last = x_pos;
// 	y_pos_last = y_pos;
// 	theta_last = theta;

// 	x_pos = x;
// 	y_pos = y;
// 	theta = th;

//      	x_dot = (x_pos-x_pos_last)/dt;
// 	y_dot = (y_pos-y_pos_last)/dt;
// 	// Need to be careful with angular velocity:
// 	omega = GetAngularVelocity(theta_last, theta, dt);
//     }
//   else
//     {
// 	// Need to estimate the whole state:
// 	x_pos_last = x_pos;
// 	y_pos_last = y_pos;
// 	theta_last = theta;

// 	x_pos = x_pos_last+x_dot*dt;
// 	y_pos = y_pos_last+y_dot*dt;
// 	theta = theta_last+omega*dt;
//     }
    
//   return;
// }


// float GetAngularVelocity(float theta_old, float theta_new, float dt)
// {
//   /* This function returns the estimated angluar velocity of the
//    * robot given two different angles and a timestep, it performs
//    * the angular rollover logic*/

//   if (theta_new >= theta_old && theta_new-theta_old <= M_PI)
//     {
// 	return (theta_new-theta_old)/dt;
//     }
//   else if (theta_new < theta_old && theta_old-theta_new <= M_PI)
//     {
// 	return (theta_new-theta_old)/dt;
//     }
//   else
//     {
// 	if(theta_new >= theta_old)
// 	  {
// 	    return -(theta_old+2.0*M_PI-theta_new)/dt;
// 	  }
// 	else
// 	  {
// 	    return (2.0*M_PI-theta_old+theta_new)/dt;
// 	  }
//     }
//   return 0;
// }
};


//--------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator_node");
  ros::NodeHandle n;

  ROS_INFO("Starting Estimator...\n");
  StateEstimator estimator;

  ros::spin();

  return 0;
}
