// robot_estimator_3d.cpp
// Jarvis Schultz
// Spring 2011

//---------------------------------------------------------------------------
// Notes
// ---------------------------------------------------------------------------
// This is a version of an estimator node that uses both Kinect data,
// and feedback from the robot to provide an optimal estimate of
// system state in the optimization coordinate system.

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
    ros::Subscriber robot_kinect_sub;
    Eigen::Vector3f robot_start_pos;
    Eigen::Vector3f robot_cal_pos;
    
    puppeteer_msgs::RobotPose pose;
    puppeteer_msgs::position_request position_request_srv;

    bool error_c, calibrated_flag;
    float xc, zc, xc_last, zc_last;  // current and previous robot positions
    float th, th_last;
    float xc_dot, zc_dot, th_dot;
    float dt_timer;
    unsigned int calibrate_count = 0;
        
public:
    PoseEstimator() {
	robot_kinect_sub = n_.subscribe("robot_kinect_position", 1, &PoseEstimator::trackercb, this);
	pose_pub = node_.advertise<puppeteer_msgs::RobotPose> ("robot_pose", 100);
	position_request_client = node_.serviceClient<puppeteer_msgs::position_request>("position_request");
	timer = node_.createTimer(ros::Duration(0.033), &PoseEstimator::timercb, this);
	t_now_timer = ros::Time::now();

	// initialize communication error parameters to true for safety
	error_c = true;
	calibrated_flag = false;  // We need to calibrate the sytem
	calibrate_count = 0;

	// initialize all state variables to zero
	xc = 0.0; zc = 0.0; xc_last = 0.0; zc_last = 0.0;
	th = 0.0; th_last = 0.0;
	xc_dot = 0.0; zc_dot = 0.0; th_dot = 0.0;
	
	dt_timer = 0.0;

	ROS_INFO("Starting Robot Pose Estimator...\n");
    }

    void timercb(const ros::TimerEvent& e)
	{
	    ROS_DEBUG("timercb triggered");

	    // check if the operating_condition parameter exists and set its value
	    int operating_condition = 0;
	    static bool first_flag = true;
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
      		return;
	    }

	    // check to see if we are in run state
	    if(operating_condition == 1 || operating_condition == 2)
	    {
		// Let's then get the robot's estimate of its state:
		get_robot_estimate();

		// Now, let's combine the robot's estimate, and the
		// Kinect's estimate to make the optimal estimate

		// Fill out and publish the optimal estimate:

		// // fill out pose message:
		// pose.x_robot = xc;
		// pose.y_robot = zc;
		// pose.theta = th;
		// pose.error = error_c;
		// // set time stamp
		// pose.header.stamp = t_now_timer;
		// // publish system state
		// pose_pub.publish(pose);
		return;
	    }
	    // are we in idle or stop condition?
	    else if(operating_condition == 0 || operating_condition == 3)
		ROS_DEBUG("Estimator node is idle due to operating condition");

	    // are we in emergency stop condition?
	    else if(operating_condition == 4)
		ROS_WARN_ONCE("Emergency Stop Requested");

	    // otherwise something terrible has happened
	    else
		ROS_ERROR("Invalid value for operating_condition");

	    calibrated_flag = false;
	    calibrate_count = 0;
	}

    void trackercb(const puppeteer_msgs::PointPlus &point)
	{
	    // If need to calibrate, let's clear all of the values for the
	    // necessary transformations
	    if(calibrated_flag == false)
	    {
		if(calibrate_count == 0)
		{
		    if(ros::param::has("/robot_x0"))
		    {
			// Get robot's starting position in
			// optimization coordinate system
			float temp;
			robot_start_pos <<
			    ros::param::get("/robot_x0", temp),
			    ros::param::get("/robot_y0", temp),
			    ros::param::get("/robot_z0", temp);
		    }
		    else
		    {
			ROS_WARN("No parameter setting robot's start position");
			std::cout << "\t Using a default value" << std::endl;
			robot_start_pos << 0,0,0;
		    }
		    calibrate_count++;
		    robot_cal_pos << 0,0,0;
		    return;
		}
		// For the next few calls, let's average the robot
		// position to establish the transformation from the
		// oriented_optimization_frame to the actual optimization_frame
		else if (calibrate_count <= NUM_CALIBRATES)
		{
		    if (!point.error)
		    {
			Eigen::Vector3f robot_current_pos;
			robot_current_pos(0) = point.x;
			robot_current_pos(1) = point.y;
			robot_current_pos(2) = point.z;

			calibrate_count++;
			return;
		}
		else
		{
		    calibrate_count = 0;
		    calibrated_flag = true;
		    return;
		}		    
	    }

	    
	    // Each time let's publish the optimization frame:

	    // Transform the received point into the actual
	    // optimization frame, store the old values, and store the
	    // new transformed value

	}

    void get_robot_estimate()
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
