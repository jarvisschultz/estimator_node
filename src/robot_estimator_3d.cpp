// robot_estimator_3d.cpp
// Jarvis Schultz
// Spring 2011

//---------------------------------------------------------------------------
// Notes
//---------------------------------------------------------------------------
/*
  This code runs a main loop on a timer.  When the timer interrupts,
  this code calls the service that requests the robot's pose.  It then
  collects the robot's pose, and estimates it if it has to, and then
  publishes the pose so that the closed-loop trajectory following code
  can determine the next set of commands to send to the robot to
  continue following the trajectory.
 */


//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------

#include <ros/ros.h>
#include <puppeteer_msgs/RobotPose.h>
#include <puppeteer_msgs/position_request.h>

#include <math.h>


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class PoseEstimator {

private:
    ros::NodeHandle node_;
    ros::Publisher pose_pub;
    ros::Timer timer;
    ros::Time t_now_timer, t_last_timer;
    ros::ServiceClient position_request_client;

    puppeteer_msgs::RobotPose pose;
    puppeteer_msgs::position_request position_request_srv;

    bool error_c;
    float xc, zc, xc_last, zc_last;  // current and previous robot positions
    float th, th_last;
    float xc_dot, zc_dot, th_dot;
    float dt_timer;
    
public:
    PoseEstimator() {
	pose_pub = node_.advertise<puppeteer_msgs::RobotPose> ("robot_pose", 100);
	position_request_client = node_.serviceClient<puppeteer_msgs::position_request>("position_request");
	timer = node_.createTimer(ros::Duration(0.033), &PoseEstimator::timerCallback, this);
	t_now_timer = ros::Time::now();

	// initialize communication error parameters to true for safety
	error_c = true;

	// initialize all state variables to zero
	xc = 0.0; zc = 0.0; xc_last = 0.0; zc_last = 0.0;
	th = 0.0; th_last = 0.0;
	xc_dot = 0.0; zc_dot = 0.0; th_dot = 0.0;
	
	dt_timer = 0.0;

	ROS_INFO("Starting Robot Pose Estimator...\n");
    }

    void timerCallback(const ros::TimerEvent& e)
	{
	    ROS_DEBUG("timerCallback triggered");

	    // check if the operating_condition parameter exists and set its value
	    int operating_condition = 0;
	    static bool first_flag = true;
	    static bool emergency_flag = false;
	    if(ros::param::has("operating_condition"))
	    {
		if(first_flag == true)
		{
		    first_flag = false;
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
		first_flag = false;      
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
		th_last = th;

		// did the robot correctly return its state?
		if(error_c == false)
		{
		    ROS_DEBUG("Successful robot update");

		    // get new robot position data
		    xc = position_request_srv.response.xc;
		    zc = position_request_srv.response.zc;
		    th = position_request_srv.response.th;

		    // Set velocities:
		    xc_dot = (xc-xc_last)/dt_timer;
		    zc_dot = (zc-zc_last)/dt_timer;
		    th_dot = (th-th_last)/dt_timer;
		}
		else
		{
		    ROS_WARN("Missed robot update");

		    // estimate new position:
		    xc += xc_dot*dt_timer;
		    zc += zc_dot*dt_timer;
		    th += th_dot*dt_timer;
		}

		// fill out pose message:
		pose.x_robot = xc;
		pose.y_robot = zc;
		pose.theta = th;
		pose.error = error_c;
		
		// set time stamp
		pose.header.stamp = t_now_timer;

		// publish system state
		pose_pub.publish(pose);
	    }
	    
	    // are we in idle or stop condition?
	    else if(operating_condition == 0 || operating_condition == 3)
	    {
		emergency_flag = false;
		ROS_DEBUG("Estimator node is idle due to operating condition");
	    } 
    
	    // are we in emergency stop condition?
	    else if(operating_condition == 4)
	    {
		// did we get an emergency stop request?
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
  ros::init(argc, argv, "robot_estimator_3d");
  ros::NodeHandle node;

  PoseEstimator pose_estimator;

  ros::spin();

  return 0;
}
