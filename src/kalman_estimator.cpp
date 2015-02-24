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
#include <angles/angles.h>

#include <math.h>

//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define NUM_CALIBRATES (30)
#define NUM_EKF_INITS (3)
#define ROBOT_CIRCUMFERENCE (57.5) // centimeters
#define DEFAULT_RADIUS (ROBOT_CIRCUMFERENCE/M_PI/2.0/100.) // meters

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class PoseEstimator {

private:
    ros::NodeHandle node_;
    ros::Publisher vo_pub;
    ros::Timer timer;
    ros::Time t_now_timer, t_last_timer;
    ros::Subscriber robot_kinect_sub, mass_kinect_sub;
    Eigen::Vector3d robot_start_pos, mass_start_pos;
    Eigen::Vector3d robot_cal_pos, mass_cal_pos;
    Eigen::Vector3d kinect_estimate;
    double robot_start_ori, robot_radius;
    
    puppeteer_msgs::position_request position_request_srv;
    puppeteer_msgs::speed_command srv;

    geometry_msgs::PointStamped transformed_robot, transformed_robot_last;
    geometry_msgs::PointStamped mass_pos;

    bool calibrated_flag;
    unsigned int calibrate_count_robot, calibrate_count_mass;
    tf::TransformListener tf;
    tf::TransformBroadcaster br;
    ros::Time tstamp;
    nav_msgs::Odometry kin_pose;
    
            
public:
    PoseEstimator() {
	robot_kinect_sub = node_.subscribe("/robot_kinect_position", 1,
					   &PoseEstimator::trackercb, this);
	mass_kinect_sub = node_.subscribe("/object1_position", 1,
					   &PoseEstimator::masscb, this);
	vo_pub = node_.advertise<nav_msgs::Odometry> ("vo", 100);
	timer = node_.
	    createTimer(ros::Duration(0.033), &PoseEstimator::timercb, this);
	t_now_timer = ros::Time::now();

	// initialize communication error parameters to true for safety
	calibrated_flag = false;  // We need to calibrate the sytem
	calibrate_count_mass = 0; calibrate_count_robot = 0;
	tstamp = ros::Time::now();

	// set covariance for the pose messages
	double kin_cov_dist = 0.01;	// in meters^2
	double kin_cov_ori = pow(M_PI,2.0);	// radians^2
	boost::array<double,36ul> kincov = {{kin_cov_dist, 0, 0, 0, 0, 0,
					     0, kin_cov_dist, 0, 0, 0, 0,
					     0, 0,        99999, 0, 0, 0,
					     0, 0, 0,        99999, 0, 0,
					     0, 0, 0, 0,        99999, 0,
					     0, 0, 0, 0, 0,  kin_cov_ori}};
	kin_pose.pose.covariance = kincov;

	// get the size of the robot:
	if(ros::param::has("/robot_radius"))
	    ros::param::get("/robot_radius", robot_radius);
	else
	{
	    robot_radius = DEFAULT_RADIUS;
	    ros::param::set("/robot_radius", robot_radius);
	}
	
	ROS_INFO("Starting Robot Pose Estimator...\n");
    }

    void masscb(const puppeteer_msgs::PointPlus &p)
    	{
	    if (!p.error)
	    {
		mass_pos.point.x = p.x;
		mass_pos.point.y = p.y;
		mass_pos.point.z = p.z;
	    }
    	}
    
    void trackercb(const puppeteer_msgs::PointPlus &p) 
	{
	    ROS_DEBUG("trackercb triggered");
	    static bool first_flag = true;
	    static bool consider_mass = false;
	    puppeteer_msgs::PointPlus point;
	    ROS_DEBUG("Received Point Position = %f,%f,%f",p.x,p.y,p.z);

	    point = p;	    
	    // before we do anything, let's correct the values of
	    // point to make up for the size of the robot
	    point = correct_vals(point);
	    ROS_DEBUG("Corrected Point Position = %f, %f, %f",point.x,point.y,point.z);

	    // If need to calibrate, let's clear all of the values for the
	    // necessary transformations
	    if(calibrated_flag == false)
	    {
		if(calibrate_count_robot == 0)
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
			ros::param::get("/robot_th0", robot_start_ori);
		    }
		    else
		    {
			ROS_WARN_ONCE
			    ("No parameter setting robot's start position");
			ROS_WARN_ONCE("Using a default value");
			robot_start_pos << 0, 0, 0;
		    }

		    if (ros::param::has("/tracking_mass"))
		    {
			double temp;
			consider_mass = true;
			ros::param::get("/mass_x0", temp);
			mass_start_pos(0) = temp;
			ros::param::get("/mass_y0", temp);
			mass_start_pos(1) = temp;
			ros::param::get("/mass_z0", temp);
			mass_start_pos(2) = temp;
		    }			
		    
		    calibrate_count_robot++;
		    calibrate_count_mass = 0;
		    robot_cal_pos << 0, 0, 0;
		    mass_cal_pos << 0, 0, 0;
		    return;
		}
		// For the next few calls, let's average the robot
		// position to establish the transformation from the
		// oriented_optimization_frame to the actual optimization_frame
		else if (calibrate_count_robot <= NUM_CALIBRATES)
		{
		    if (!point.error)
		    {
		    	robot_cal_pos(0) += point.x;
		    	robot_cal_pos(1) += point.y;
		    	robot_cal_pos(2) += point.z;

			if (consider_mass)
			{
			    calibrate_count_mass++;
			    mass_cal_pos(0) += mass_pos.point.x;
			    mass_cal_pos(1) += mass_pos.point.y;
			    mass_cal_pos(2) += mass_pos.point.z;
			}

		    	calibrate_count_robot++;
		    	return;
		    }
		}
		else
		{
		    // Find average robot_cal_pos
		    robot_cal_pos = (robot_cal_pos/((float) NUM_CALIBRATES))-
			robot_start_pos;
		    // find average mass_cal_pos
		    if (consider_mass)
		    {
			mass_cal_pos = (mass_cal_pos/
					((float) calibrate_count_mass))
			    -mass_start_pos;
			// cal_pos = (mass_cal_pos+robot_cal_pos)/2;
			// cal_pos = robot_cal_pos;
			// cal_pos(1) = mass_cal_pos(1);
		    }
		    
		    calibrate_count_robot = 0;
		    calibrate_count_mass = 0;
		    calibrated_flag = true;
		    first_flag = true;
		}		    
	    }

	    tf::Transform transform;
	    tstamp = ros::Time::now();
	    // If we are tracking mass publish optimization frame
	    if (consider_mass)	    
	    {
		transform.setOrigin(tf::Vector3(mass_cal_pos(0),
						mass_cal_pos(1),
						mass_cal_pos(2)));
		transform.setRotation(tf::Quaternion(0,0,0,1));
		br.sendTransform(tf::StampedTransform(transform, tstamp,
						      "oriented_optimization_frame",
						      "optimization_frame"));
	    }
	    else
	    {
		transform.setOrigin(tf::Vector3(robot_cal_pos(0),
						robot_cal_pos(1),
						robot_cal_pos(2)));
		transform.setRotation(tf::Quaternion(0,0,0,1));
		br.sendTransform(tf::StampedTransform(transform, tstamp,
						      "oriented_optimization_frame",
						      "optimization_frame"));
	    }

	    
	    // Publish /map frame based on robot calibration
	    transform.setOrigin(tf::Vector3(robot_cal_pos(0),
					    0,
					    robot_cal_pos(2)));
	    transform.setRotation(tf::Quaternion(.707107,0.0,0.0,-0.707107));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "oriented_optimization_frame",
						  "map"));
	    
	    // publish one more frame that is the frame the robot
	    // calculates its odometry in.
	    transform.setOrigin(tf::Vector3(0,0,0));
	    transform.setRotation(tf::Quaternion(1,0,0,0));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "map",
						  "robot_odom_pov"));

	    // Reset transform values for transforming data from
	    // Kinect frame into optimization frame
	    if (consider_mass)
	    {
		transform.setOrigin(tf::Vector3(mass_cal_pos(0),
						mass_cal_pos(1), mass_cal_pos(2)));
		transform.setRotation(tf::Quaternion(0,0,0,1));
	    }
	    else
	    {
		transform.setOrigin(tf::Vector3(robot_cal_pos(0),
						robot_cal_pos(1), robot_cal_pos(2)));
		transform.setRotation(tf::Quaternion(0,0,0,1));
	    }		
	    
	    // Transform the received point into the actual
	    // optimization frame, store the old values, and store the
	    // new transformed value
	    if (!point.error)
	    {
		Eigen::Affine3d gwo;
		Eigen::Vector3d tmp_point; 
		tf::transformTFToEigen(transform, gwo);
		gwo = gwo.inverse();
		tmp_point << point.x, point.y, point.z;
		tmp_point = gwo*tmp_point;

		transformed_robot_last = transformed_robot;
		transformed_robot.header.frame_id = "optimization_frame";
		transformed_robot.header.stamp = tstamp;
		transformed_robot.point.x = tmp_point(0);
		transformed_robot.point.y = tmp_point(1);
		transformed_robot.point.z = tmp_point(2);


		if (first_flag == true)
		{
		    transformed_robot_last = transformed_robot;
		    first_flag = false;
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
			get_kinect_estimate(1);
		    else
			get_kinect_estimate(operating_condition);
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
	    calibrate_count_mass = 0;
	    calibrate_count_robot = 0;
	    init_ekf_count = 0;
	}

 
    void get_kinect_estimate(int op)
	{
	    // Let's first get the transform from /optimization_frame
	    // to /map
	    static tf::StampedTransform transform;
	    geometry_msgs::PointStamped tmp;
		    
	    try{
		tf.lookupTransform(
		    "map", "optimization_frame",
		    tstamp, transform);
		tf.transformPoint("map", transformed_robot, tmp);

	    }
	    catch(tf::TransformException& ex){
		ROS_ERROR(
		    "Error trying to lookupTransform from /map "
		    "to /optimization_frame: %s", ex.what());
		return;
	    }
	    
	    // Now we can publish the Kinect's estimate of the robot's
	    // pose
	    kin_pose.header.stamp = tstamp;
	    kin_pose.header.frame_id = "map";
	    kin_pose.child_frame_id = "base_footprint_kinect";
	    tmp.point.z = 0.0;
	    kin_pose.pose.pose.position = tmp.point;
	    double theta = 0.0;
	    if (op == 2)
	    {
		theta = atan2(transformed_robot.point.x-
				     transformed_robot_last.point.x,
				     transformed_robot.point.z-
				     transformed_robot_last.point.z);
		theta = angles::normalize_angle(theta-M_PI/2.0);
	    }
	    else
	    {
		theta = robot_start_ori;
		theta = angles::normalize_angle(-theta); 
	    }
	    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
	    kin_pose.pose.pose.orientation = quat;				 
	    
	    // Now let's publish the estimated pose as a
	    // nav_msgs/Odometry message on a topic called /vo
	    vo_pub.publish(kin_pose);

	    // now, let's publish the transform that goes along with it
	    geometry_msgs::TransformStamped kin_trans;
	    tf::Quaternion q1, q2;
	    q1 = tf::createQuaternionFromYaw(theta);
	    q2 = tf::Quaternion(1.0,0,0,0);
	    q1 = q1*q2;
	    tf::quaternionTFToMsg(q1, quat);

	    kin_trans.header.stamp = tstamp;
	    kin_trans.header.frame_id = kin_pose.header.frame_id;
	    kin_trans.child_frame_id = kin_pose.child_frame_id;
	    kin_trans.transform.translation.x = kin_pose.pose.pose.position.x;
	    kin_trans.transform.translation.y = kin_pose.pose.pose.position.y;
	    kin_trans.transform.translation.z = kin_pose.pose.pose.position.z;
	    kin_trans.transform.rotation = quat;

	    ROS_DEBUG("Sending transform for output of estimator node");
	    br.sendTransform(kin_trans);
	    
	    return;
	}

    // this function accounts for the size of the robot:
    puppeteer_msgs::PointPlus correct_vals(puppeteer_msgs::PointPlus &p)
	{
	    ROS_DEBUG("correct_vals called");
	    puppeteer_msgs::PointPlus point;
	    point = p;
	    	    
	    // let's create a unit vector from the kinect frame to the
	    // robot's location
	    Eigen::Vector3d ur;
	    ur << p.x, p.y, p.z;
	    // now turn it into a unit vector:
	    ur = ur/ur.norm();
	    // now we can correct the values of point
	    ur = ur*robot_radius;
	    
	    point.x = point.x+ur(0);
	    point.y = point.y+ur(1);
	    point.z = point.z+ur(2);
	    
	    return(point);	    	    
	}

}; //  End Of PoseEstimator Class


//--------------------------------------------------------------------------
// Main
//--------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ROSCONSOLE_AUTOINIT;
    
    ros::init(argc, argv, "robot_estimator_3d");
    // log4cxx::LoggerPtr my_logger =
    // 	log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
    // my_logger->setLevel(
    // 	ros::console::g_level_lookup[ros::console::levels::Debug]);
    ros::NodeHandle node;

    PoseEstimator pose_estimator;

    ros::spin();

    return 0;
}
