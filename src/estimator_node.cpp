// estimator_node.cpp
// Jake Ware and Jarvis Schultz
// Spring 2011

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

#include <math.h>


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class StateEstimator {

private:
    ros::NodeHandle n_;
    ros::Subscriber tracker_sub;
    ros::Publisher state_pub;
    ros::Publisher marker_pub;
    ros::Timer timer;
    ros::Time t_now_tracker, t_last_tracker, t_now_timer, t_last_timer;
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
    float dt_tracker, dt_timer;

public:
    StateEstimator() {
	tracker_sub = n_.subscribe("/object1_position", 1, &StateEstimator::trackercb, this);
	state_pub = n_.advertise<puppeteer_msgs::State> ("system_state", 100);
	position_request_client = n_.serviceClient<puppeteer_msgs::position_request>("position_request");
	timer = n_.createTimer(ros::Duration(0.033), &StateEstimator::timerCallback, this);
	t_now_timer = ros::Time::now();
	t_now_tracker = ros::Time::now();

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

	dt_timer = 0.0;
	dt_tracker = 0.0;

	ROS_INFO("Starting Estimator...\n");
    }

    void trackercb(const puppeteer_msgs::PointPlus &point)
	{
	    ROS_DEBUG("Entered tracker callback");

	    // get time for this update and store it
	    t_last_tracker = t_now_tracker;
	    t_now_tracker = ros::Time::now();
	    dt_tracker = (t_now_tracker.toSec()-t_last_tracker.toSec());
	    ROS_DEBUG("dt_tracker: %f", dt_tracker);

	    // store last mass position values
	    xm_last = xm;
	    ym_last = ym;
	    zm_last = zm;
      
	    // get error flag from object tracker data
	    error_m = point.error;
	    
	    if(error_m == false)
	    {
		// get new mass position data
		xm = point.x;
		ym = point.y;
		zm = point.z;

		// store old velocities:
		xm_dot_last = xm_dot;
		ym_dot_last = ym_dot;
		zm_dot_last = zm_dot;
		
		// set the velocities:
		xm_dot = (xm-xm_last)/dt_tracker;
		ym_dot = (ym-ym_last)/dt_tracker;
		zm_dot = (zm-zm_last)/dt_tracker;
	    }
	    else
	    {
		// since we did not get a good ball position, let's
		// estimate the current position:
		xm += xm_dot*dt_tracker;
		ym += ym_dot*dt_tracker;
		zm += zm_dot*dt_tracker;
		
		// leave the velocities alone
	    }
	    ROS_INFO("Object CB: xm = %f\t xm_dot = %f\t dt = %f\t",xm,xm_dot,dt_tracker);
	}

    void timerCallback(const ros::TimerEvent& e)
	{
	    ROS_DEBUG("timerCallback triggered");

	    // check if the operating_condition parameter exists and set its value
	    int operating_condition = 0;
	    static int first_flag = 0;
	    if(ros::param::has("operating_condition"))
	    {
		if(first_flag == 0)
		{
		    first_flag++;
		    ros::param::set("/operating_condition", 0);
		}
		else
		    ros::param::get("/operating_condition", operating_condition);
	    }
	    else
	    {
		ROS_WARN("Cannot Find Parameter: operating_condition");
		ROS_INFO("Setting operating_condition to IDLE");
		ros::param::set("/operating_condition", 0);
      
		return;
	    }

	    // check to see if we are in calibrate or run state
	    if(operating_condition == 1 || operating_condition == 2)
	    {
	    
		// check to see if robot index parameter exists.
		static int robot_index = 0;
		if(ros::param::has("robot_index"))
		{
		    ros::param::get("/robot_index", robot_index);
		}
		else
		{
		    ROS_WARN("Cannot Find Parameter: robot_index");
		    ROS_INFO("Setting robot_index to 0");
		    ros::param::set("/robot_index", 0);
		}
	
		// request robot position
		position_request_srv.request.robot_index = robot_index;
		position_request_srv.request.type = (uint8_t) 'w';
		position_request_srv.request.Vleft = 0.0;
		position_request_srv.request.Vright = 0.0;
		position_request_srv.request.Vtop = 0.0;
		position_request_srv.request.div = 0;

		// call service and store error flag
		if(position_request_client.call(position_request_srv))
		{
		    error_c = position_request_srv.response.error;
		}
		// print error if the service call failed (not the
		// same as a successful service call with a bad reply)
		else
		{
		    ROS_ERROR("Failed to call service: position_request");
		}
	    
		// get current time and dt to last call
		t_last_timer = t_now_timer;
		t_now_timer = ros::Time::now();
		dt_timer = (t_now_timer.toSec()-t_last_timer.toSec());
		ROS_DEBUG("dt_timer: %f", dt_timer);

		// store last positions and velocities
		xc_last = xc;
		zc_last = zc;
		xc_dot_last = xc_dot;
		zc_dot_last = zc_dot;
		r_last = r;
		r_dot_last = r_dot;
		th_last = th;
		th_dot_last = th_dot;

		// did the robot correctly return its state?
		if(error_c == false)
		{
		    ROS_DEBUG("Successful robot update");

		    // get new robot position data
		    xc = position_request_srv.response.xc;
		    zc = position_request_srv.response.zc;
		    th = position_request_srv.response.th;

		    // calculate robot velocities
		    xc_dot = (xc-xc_last)/dt_timer;
		    zc_dot = (zc-zc_last)/dt_timer;
		    th_dot = (th-th_last)/dt_timer;

		    // calculate string length
		    // ignore z dimension for now and assume planar motion in x and y
		    r = sqrt(powf((xc-xm),2)+powf((ym),2));
		    r_dot = (r-r_last)/dt_timer;
		}

		// we need to estimate the robot position:
		else
		{
		    ROS_WARN("Missed robot update");

		    // Keep old velocities:
		    xc_dot = xc_dot_last;
		    zc_dot = zc_dot_last;
		    th_dot = th_dot_last;

		    // estimate new position:
		    xc += xc_dot*dt_timer;
		    zc += zc_dot*dt_timer;
		    th += th_dot*dt_timer;

		    // calculate string length
		    // ignore z dimension for now and assume planar motion in x and y
		    r = sqrt(powf((xc-xm),2)+powf((ym),2));
		}

		// assign various components of system state
		state.xm = xm;
		state.ym = ym;
		state.xc = xc;
		state.r = r;
		state.xm_dot = xm_dot;
		state.ym_dot = ym_dot;
		state.xc_dot = xc_dot;
		state.r_dot = r_dot;

		// set time stamp
		state.header.stamp = t_now_timer;

		// publish system state
		state_pub.publish(state);
	    }
	    
	    // are we in idle or stop condition?
	    else if(operating_condition == 0 || operating_condition == 3)
	    {
		ROS_DEBUG("Estimator node is idle due to operating condition");
	    } 
    
	    // are we in emergency stop condition?
	    else if(operating_condition == 4)
	    {
		// did we get an emergency stop request?
		static bool emergency_flag = false;
		if(operating_condition == 4 && emergency_flag == false)
		{
		    ROS_WARN("Emergency Stop Requested");
		    emergency_flag = true;
		}
	    }
    
	    // otherwise something terrible has happened
	    else
	    {
		ROS_ERROR("Invalid value for operating_condition");
	    }

	    ROS_DEBUG("Leaving tracker callback");
	}
};


//--------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator_node");
  ros::NodeHandle n;

  StateEstimator estimator;

  ros::spin();

  return 0;
}
