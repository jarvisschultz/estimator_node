// kalman_estimator.cpp
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
#include <iostream>

#include <ros/ros.h>
#include <puppeteer_msgs/RobotPose.h>
#include <puppeteer_msgs/PointPlus.h>
#include <puppeteer_msgs/position_request.h>
#include <puppeteer_msgs/speed_command.h>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>

#include <math.h>

//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define NUM_CALIBRATES (30)
#define STDEV_RATE (10)
#define MAX_BAD_COUNTER (10)
#define NUM_EKF_INITS (3)
FILE *fp;

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class PoseEstimator {

private:
    ros::NodeHandle node_;
    ros::Publisher pose_pub, vo_pub, odom_pub;
    ros::Timer timer;
    ros::Time t_now_timer, t_last_timer;
    ros::ServiceClient position_request_client, correction_client;
    ros::Subscriber robot_kinect_sub;
    Eigen::Vector3d robot_start_pos;
    Eigen::Vector3d robot_cal_pos;
    Eigen::Vector3d robot_estimate;
    Eigen::Vector3d kinect_estimate;
    
    puppeteer_msgs::RobotPose pose;
    puppeteer_msgs::position_request position_request_srv;
    puppeteer_msgs::speed_command srv;

    geometry_msgs::PointStamped transformed_robot, transformed_robot_last;

    bool error_c, calibrated_flag, error_kinect;
    float xc, zc, xc_last, zc_last;  // current and previous robot positions
    float th, th_last;
    float xc_dot, zc_dot, th_dot;
    float dt_timer;
    unsigned int calibrate_count, error_counter;
    tf::TransformListener tf;
    tf::TransformBroadcaster br;
    unsigned int call_count;
    ros::Time tstamp;
    nav_msgs::Odometry kin_pose, rob_pose;
    
            
public:
    PoseEstimator() {
	robot_kinect_sub = node_.subscribe("/robot_kinect_position", 1,
					   &PoseEstimator::trackercb, this);
	pose_pub = node_.advertise<puppeteer_msgs::RobotPose> ("robot_pose", 100);
	vo_pub = node_.advertise<nav_msgs::Odometry> ("vo", 100);
	odom_pub = node_.advertise<nav_msgs::Odometry> ("odom", 100);
	position_request_client = node_.
	    serviceClient<puppeteer_msgs::position_request>("position_request");
	correction_client = node_.
	    serviceClient<puppeteer_msgs::speed_command>("speed_command");
	timer = node_.
	    createTimer(ros::Duration(0.033), &PoseEstimator::timercb, this);
	t_now_timer = ros::Time::now();

	// initialize communication error parameters to true for safety
	error_c = true;
	calibrated_flag = false;  // We need to calibrate the sytem
	calibrate_count = 0;
	call_count = 0;

	// initialize all state variables to zero
	xc = 0.0; zc = 0.0; xc_last = 0.0; zc_last = 0.0;
	th = 0.0; th_last = 0.0;
	xc_dot = 0.0; zc_dot = 0.0; th_dot = 0.0;
	error_counter = 0;
	
	dt_timer = 0.0;
	tstamp = ros::Time::now();

	// set covariance for the pose messages
	double kin_cov_dist = 0.01;	// in meters^2
	double kin_cov_ori = pow(M_PI,2.0);	// radians^2
	double rob_cov_dist = 0.001;	// in meters^2
	double rob_cov_ori = 0.02;	// radians^2

	// double kin_cov_dist = 0.0025;	// in meters^2
	// double kin_cov_ori = 99999;
	// double rob_cov_dist = 0.00001;	// in meters^2
	// double rob_cov_ori = 0.00001;	// radians^2
	boost::array<double,36ul> kincov = {{kin_cov_dist, 0, 0, 0, 0, 0,
					     0, kin_cov_dist, 0, 0, 0, 0,
					     0, 0,        99999, 0, 0, 0,
					     0, 0, 0,        99999, 0, 0,
					     0, 0, 0, 0,        99999, 0,
					     0, 0, 0, 0, 0,  kin_cov_ori}};
	boost::array<double,36ul> robcov = {{rob_cov_dist, 0, 0, 0, 0, 0,
					     0, rob_cov_dist, 0, 0, 0, 0,
					     0, 0,        99999, 0, 0, 0,
					     0, 0, 0,        99999, 0, 0,
					     0, 0, 0, 0,        99999, 0,
					     0, 0, 0, 0, 0,  rob_cov_ori}};
	kin_pose.pose.covariance = kincov;
	rob_pose.pose.covariance = robcov;	
	
	ROS_INFO("Starting Robot Pose Estimator...\n");
    }
    
    void trackercb(const puppeteer_msgs::PointPlus &point)
	{
	    ROS_DEBUG("trackercb triggered");
	    static bool first_flag = true;
	    error_kinect = point.error;
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
			double temp;
			ros::param::get("/robot_x0", temp);
			robot_start_pos(0) = temp;
			ros::param::get("/robot_y0", temp);
			robot_start_pos(1) = temp;
			ros::param::get("/robot_z0", temp);
			robot_start_pos(2) = temp;
		    }
		    else
		    {
			ROS_WARN_ONCE
			    ("No parameter setting robot's start position");
			ROS_WARN_ONCE("Using a default value");
			robot_start_pos << 0, 0, 0;
		    }
		    calibrate_count++;
		    robot_cal_pos << 0, 0, 0;
		    return;
		}
		// For the next few calls, let's average the robot
		// position to establish the transformation from the
		// oriented_optimization_frame to the actual optimization_frame
		else if (calibrate_count <= NUM_CALIBRATES)
		{
		    if (!point.error)
		    {
		    	robot_cal_pos(0) += point.x;
		    	robot_cal_pos(1) += point.y;
		    	robot_cal_pos(2) += point.z;

		    	calibrate_count++;
		    	return;
		    }
		}
		else
		{
		    // Find average robot_cal_pos
		    robot_cal_pos = (robot_cal_pos/((float) NUM_CALIBRATES))-
			robot_start_pos;
		    calibrate_count = 0;
		    calibrated_flag = true;
		    call_count = 0;
		    first_flag = true;
		}		    
	    }

	    // Publish optimization frame
	    tf::Transform transform;
	    tstamp = ros::Time::now();
	    transform.setOrigin(tf::Vector3(robot_cal_pos(0),
					    robot_cal_pos(1), robot_cal_pos(2)));
	    transform.setRotation(tf::Quaternion(0,0,0,1));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "oriented_optimization_frame",
						  "optimization_frame"));

	    // Now publish a child to optimization frame called map,
	    // and a child to map called odom.  All three frames are
	    // in the same location but a different orientation (map
	    // and odom have z up), this is just to be aligned with
	    // the ROS standards for using the navigation stack.
	    transform.setOrigin(tf::Vector3(0.0,-robot_cal_pos(1),0.0));
	    transform.setRotation(tf::Quaternion(.707107,0.0,0.0,-0.707107));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "optimization_frame",
						  "map"));
	    transform.setOrigin(tf::Vector3(0,0,0));
	    transform.setRotation(tf::Quaternion(0,0,0,1));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "map",
						  "odom"));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "odom",
						  "odom_combined"));	    

	    // Reset transform values for transforming data from
	    // Kinect frame into optimization frame
	    transform.setOrigin(tf::Vector3(robot_cal_pos(0),
					    robot_cal_pos(1), robot_cal_pos(2)));
	    transform.setRotation(tf::Quaternion(0,0,0,1));
	    // Transform the received point into the actual
	    // optimization frame, store the old values, and store the
	    // new transformed value
	    if (!point.error)
	    {
		if (first_flag == true)
		{
		    Eigen::Affine3d gwo;
		    Eigen::Vector3d tmp_point; 
		    tf::TransformTFToEigen(transform, gwo);
		    gwo = gwo.inverse();
		    tmp_point << point.x, point.y, point.z;
		    tmp_point = gwo*tmp_point;

		    transformed_robot.header.frame_id = "optimization_frame";
		    transformed_robot.header.stamp = tstamp;
		    transformed_robot.point.x = tmp_point(0);
		    transformed_robot.point.y = tmp_point(1);
		    transformed_robot.point.z = tmp_point(2);
		    
		    transformed_robot_last = transformed_robot;
		    first_flag = false; 
		}
		else
		{
		    transformed_robot_last = transformed_robot;
		    Eigen::Affine3d gwo;
		    Eigen::Vector3d tmp_point; 
		    tf::TransformTFToEigen(transform, gwo);
		    gwo = gwo.inverse();
		    tmp_point << point.x, point.y, point.z;
		    tmp_point = gwo*tmp_point;

		    transformed_robot.header.frame_id = "optimization_frame";
		    transformed_robot.header.stamp = tstamp;
		    transformed_robot.point.x = tmp_point(0);
		    transformed_robot.point.y = tmp_point(1);
		    transformed_robot.point.z = tmp_point(2);
		}		
	    }
	    return;
	}

    void timercb(const ros::TimerEvent& e)
	{
	    ROS_DEBUG("timercb triggered");

	    // check if the operating_condition parameter exists and set its value
	    int operating_condition = 0;
	    static bool first_flag = true;
	    static unsigned int init_ekf_count = 0;
	    
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
		if(calibrated_flag == true)
		{
		    if(init_ekf_count <= NUM_EKF_INITS)
		    {
			get_kinect_estimate(1);
			get_robot_estimate();
			transform_robot_estimate(1);
		    }
		    else
		    {
			get_kinect_estimate(operating_condition);
			get_robot_estimate();
			transform_robot_estimate(operating_condition);
		    }
		    init_ekf_count++;
		}
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
	    init_ekf_count = 0;
	}

    void get_robot_estimate(void)
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
		ROS_INFO("Setting robot_index to 1");
		ros::param::set("/robot_index", 1);
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
		error_counter = 0;
	    }
	    else
	    {
		ROS_WARN("Missed robot update");

		// estimate new position:
		xc += xc_dot*dt_timer;
		zc += zc_dot*dt_timer;
		th += th_dot*dt_timer;

		error_counter++;
	    }
	    if (error_counter >= MAX_BAD_COUNTER)
	    {
		ROS_WARN_THROTTLE(5,"Serial communication problems detected!");
		ros::param::set("/operating_condition", 4);
	    }
		    
		
	    robot_estimate << xc, zc, th;
	}

    void get_kinect_estimate(int op)
	{
	    // Let's first get the transform from /optimization_frame
	    // to /map
	    static tf::StampedTransform transform;
	    geometry_msgs::PointStamped tmp;
		    
	    try
	    {
		tf.lookupTransform(
		    "map", "optimization_frame",
		    tstamp, transform);
		tf.transformPoint("map", transformed_robot, tmp);

	    }
	    catch(tf::TransformException& ex)
	    {
		ROS_ERROR(
		    "Error trying to lookupTransform from /map "
		    "to /optimization_frame: %s", ex.what());
		return;
	    }
	    
	    // Now, let's transform the measured pose into the /map
	    // frame
	    kin_pose.header.stamp = tstamp;
	    kin_pose.header.frame_id = "odom_combined";
	    kin_pose.child_frame_id = "base_footprint";
	    tmp.point.z = 0.0;
	    kin_pose.pose.pose.position = tmp.point;
	    double theta = 0.0;
	    if (op == 2)
	    {
		theta = atan2(transformed_robot.point.x-
				     transformed_robot_last.point.x,
				     transformed_robot.point.z-
				     transformed_robot_last.point.z);
		theta = clamp_angle(theta-M_PI/2.0);
	    }
	    else
	    {
		theta = robot_estimate(2);
		theta = clamp_angle(-theta);
	    }
	    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
	    kin_pose.pose.pose.orientation = quat;				 
	    
	    // Now let's publish the estimated pose as a
	    // nav_msgs/Odometry message on a topic called /vo
	    vo_pub.publish(kin_pose);
	    
	    return;
	}

    void transform_robot_estimate(int op)
	{
	    // First, we need to get the robot's returned estimate
	    // into a PointPlus message and publish it
	    pose.x_robot = robot_estimate(0);
	    pose.y_robot = robot_estimate(1);
	    pose.theta = robot_estimate(2);
	    pose.error = error_c;
	    pose.header.stamp = t_now_timer;
	    pose.header.frame_id = "optimization_frame";
	    if(op == 2)
		pose_pub.publish(pose);

	    // Now let's fill in the fields of the robot's odometry
	    // pose and publish the results
	    rob_pose.header.stamp = pose.header.stamp;
	    rob_pose.header.frame_id = "odom_combined";
	    rob_pose.child_frame_id = "base_footprint";
	    rob_pose.pose.pose.position.x = pose.x_robot;
	    rob_pose.pose.pose.position.y = -pose.y_robot;
	    rob_pose.pose.pose.position.z = 0.0;

	    double theta = pose.theta;
	    theta = clamp_angle(-theta);
	    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
	    rob_pose.pose.pose.orientation = quat;

	    odom_pub.publish(rob_pose);
	    return;
	}

    double clamp_angle(double theta)
	{
	    double th = theta;
	    while(th > M_PI)
		th -= 2.0*M_PI;
	    while(th <= M_PI)
		th += 2.0*M_PI;
	    return th;
	}
};


//--------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ROSCONSOLE_AUTOINIT;
    
    ros::init(argc, argv, "robot_estimator_3d");
    // log4cxx::LoggerPtr my_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Debug]);
    ros::NodeHandle node;

    PoseEstimator pose_estimator;

    ros::spin();

    return 0;
}
