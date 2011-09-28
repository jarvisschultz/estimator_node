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
#include <Eigen/Array>
#include <geometry_msgs/Point.h>

#include <math.h>

//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------
#define NUM_CALIBRATES (30)
#define STDEV_RATE (10)
FILE *fp;

//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class PoseEstimator {

private:
    ros::NodeHandle node_;
    ros::Publisher pose_pub;
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
    unsigned int calibrate_count;
    tf::TransformListener tf;
    tf::TransformBroadcaster br;
    float Kx, Ky, Kth;
    unsigned int call_count;

            
public:
    PoseEstimator() {
	robot_kinect_sub = node_.subscribe("/robot_kinect_position", 1, &PoseEstimator::trackercb, this);
	pose_pub = node_.advertise<puppeteer_msgs::RobotPose> ("robot_pose", 100);
	position_request_client = node_.serviceClient<puppeteer_msgs::position_request>("position_request");
	correction_client = node_.serviceClient<puppeteer_msgs::speed_command>("speed_command");
	timer = node_.createTimer(ros::Duration(0.033), &PoseEstimator::timercb, this);
	t_now_timer = ros::Time::now();

	// initialize communication error parameters to true for safety
	error_c = true;
	calibrated_flag = false;  // We need to calibrate the sytem
	calibrate_count = 0;
	call_count = 0;

	// Set gains:
	Kx = 0.75; Ky = Kx;
	Kth = 0.0;
	
	// initialize all state variables to zero
	xc = 0.0; zc = 0.0; xc_last = 0.0; zc_last = 0.0;
	th = 0.0; th_last = 0.0;
	xc_dot = 0.0; zc_dot = 0.0; th_dot = 0.0;
	
	dt_timer = 0.0;

	ROS_INFO("Starting Robot Pose Estimator...\n");

	fp = fopen("/home/jarvis/Desktop/data.txt","w");
	fprintf(fp,"Kinect Robot Optimal");
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
			ROS_WARN_ONCE("No parameter setting robot's start position");
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
		    robot_cal_pos = (robot_cal_pos/((float) NUM_CALIBRATES))-robot_start_pos;
		    calibrate_count = 0;
		    calibrated_flag = true;
		    call_count = 0;
		    Kth = 0.0;
		    first_flag = true;
		    return;
		}		    
	    }

	    // Publish optimization frame
	    tf::Transform transform;
	    ros::Time tstamp = ros::Time::now();
	    transform.setOrigin(tf::Vector3(robot_cal_pos(0), robot_cal_pos(1), robot_cal_pos(2)));
	    transform.setRotation(tf::Quaternion(0,0,0,1));
	    br.sendTransform(tf::StampedTransform(transform, tstamp,
						  "/oriented_optimization_frame",
						  "/optimization_frame"));
	    
	    // Transform the received point into the actual
	    // optimization frame, store the old values, and store the
	    // new transformed value
	    if (!point.error)
	    {
		if (first_flag == true)
		{
		    Eigen::eigen2_Transform3d gwo;
		    Eigen::Vector3d tmp_point; 
		    tf::TransformTFToEigen(transform, gwo);
		    gwo = gwo.inverse();
		    tmp_point << point.x, point.y, point.z;
		    tmp_point = gwo*tmp_point;

		    transformed_robot.header.frame_id = "/optimization_frame";
		    transformed_robot.header.stamp = ros::Time::now();
		    transformed_robot.point.x = tmp_point(0);
		    transformed_robot.point.y = tmp_point(1);
		    transformed_robot.point.z = tmp_point(2);
		    
		    transformed_robot_last = transformed_robot;
		    first_flag = false; 
		}
		else
		{
		    transformed_robot_last = transformed_robot;
		    Eigen::eigen2_Transform3d gwo;
		    Eigen::Vector3d tmp_point; 
		    tf::TransformTFToEigen(transform, gwo);
		    gwo = gwo.inverse();
		    tmp_point << point.x, point.y, point.z;
		    tmp_point = gwo*tmp_point;

		    transformed_robot.header.frame_id = "/optimization_frame";
		    transformed_robot.header.stamp = ros::Time::now();
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
		get_kinect_estimate();
		if(calibrated_flag == true)
		{
		    get_robot_estimate();
		    get_optimal_estimate();
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
	    }
	    else
	    {
		ROS_WARN("Missed robot update");

		// estimate new position:
		xc += xc_dot*dt_timer;
		zc += zc_dot*dt_timer;
		th += th_dot*dt_timer;
	    }
	    robot_estimate << xc, zc, th;
	}

    void get_kinect_estimate(void)
	{
	    static float th_array[STDEV_RATE];
	    static float tmp[STDEV_RATE] ;
	    static float stdev;
	    static float theta_last = 0.0;
	    static float alpha = 0.1;
	    float theta = 0.0, x = 0.0, z = 0.0;
	    unsigned int i = 0;
	    x = transformed_robot.point.x;
	    z = transformed_robot.point.z;
	    
	    kinect_estimate << x, z, 0.0;
		
	    theta = atan2(z-transformed_robot_last.point.z,
			  x-transformed_robot_last.point.x);
	    
	    // Let's low pass filter this:
	    theta = alpha*theta+(1-alpha)*theta_last;
	    theta_last = theta;
	    
	    while(theta <= 0)
		theta += 2.0*M_PI;
	    while(th > 2.0*M_PI)
		theta -= 2.0*M_PI;

	    if (call_count < STDEV_RATE)
	    {
		th_array[call_count] = theta;
		call_count++;
		kinect_estimate(2) = theta;
		return;
	    }
	    kinect_estimate(2) = theta;


	    // Copy old array, and then set new array:
	    memcpy(tmp, th_array, sizeof(th_array));
	    th_array[0] = theta;	    
	    for(i=1; i<STDEV_RATE; i++)
		th_array[i] = tmp[i-1];

	    stdev = calculate_stdev(th_array, STDEV_RATE);

	    Kth = 0.01*sqrtf(1+powf(stdev,2.0))/stdev;
	    if(Kth >= 1.0) Kth = 1.0;
	    ROS_DEBUG("Changing the gain on theta to %f", Kth);
	    return;
	}

    float calculate_stdev(const float ar[], const unsigned int size)
	{
	    // First calculate the mean:
	    unsigned int i = 0;
	    float mean = 0.0;
	    float stdev = 0.0;
	    for (i=0; i<size; i++)
		mean += ar[i];
	    mean /= ((float) size);

	    // Now, the standard deviation:
	    for (i=0; i<size; i++)
		stdev += powf((ar[i]-mean),2.0);
	    stdev /= ((float) size);
	    stdev = sqrtf(stdev);
	    return stdev;
	}

    void get_optimal_estimate(void)
	{
	    Eigen::Vector3d optimal_estimate, gains;
	    gains << Kx, Ky, Kth;
	    // gains << 0.0, 0.0, 0.0;
	    ROS_INFO("Kth = %f",Kth);
	    printf("Kinect = %f\t Robot = %f\n",kinect_estimate(2),
		   robot_estimate(2));
	    
	    fprintf(fp, "%f\t%f\t",kinect_estimate(2), robot_estimate(2));
	    
	    for(unsigned int i = 0; i < 3; i++)
		optimal_estimate(i) = robot_estimate(i)+
		    gains(i)*(kinect_estimate(i)-robot_estimate(i));
	    printf("Optimal = %f\n",optimal_estimate(2));
	    fprintf(fp, "%f\n",optimal_estimate(2));
	    
	    // pose.x_robot = optimal_estimate(0);
	    // pose.y_robot = optimal_estimate(1);
	    // pose.theta = optimal_estimate(2);
	    pose.x_robot = robot_estimate(0);
	    pose.y_robot = robot_estimate(1);
	    pose.theta = robot_estimate(2);
	    pose.error = error_c;
	    pose.header.stamp = ros::Time::now();
	    pose.header.frame_id = "/optimization_frame";

	    pose_pub.publish(pose);

	    return;
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
