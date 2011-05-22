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

//---------------------------------------------------------------------------
// Global Variables
//---------------------------------------------------------------------------


//---------------------------------------------------------------------------
// Objects and Functions
//---------------------------------------------------------------------------

class StateEstimator {

private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher state_pub;
  ros::Time t_now, t_last, dt;
  ros::ServiceClient client;

  puppeteer_msgs::State state, state_last;  // current and previous system states
  puppeteer_msgs::position_request srv;

  bool error_m;
  bool error_c;
  float xm, ym, zm, xm_last, ym_last, zm_last;  // current and previous mass positions
  float xm_dot, ym_dot, zm_dot, xm_dot_last, ym_dot_last, zm_dot_last;  // current and previous mass velocities
  float xc, zc, xc_last, zc_last;  // current and previous cart positions
  float xc_dot, zc_dot, xc_dot_last, zc_dot_last;  // current and previous cart velocities
  float r, r_last;  // current and previous string lengths
  float r_dot, r_dot_last;  // current and previous rate of change of string length

public:
  StateEstimator() {
    sub_ = n_.subscribe("/object1_position", 1, &StateEstimator::trackercb, this);
    state_pub = n_.advertise<puppeteer_msgs::State> ("system_state", 100);
    client = n_.serviceClient<puppeteer_msgs::position_request>("position_request");
    t_now = ros::Time::now();

    error_m = false;
    error_c = false;

    xm = 0.0; ym = 0; zm = 0; xm_last = 0; ym_last = 0; zm_last = 0; 
    xm_dot = 0; ym_dot = 0; zm_dot = 0; xm_dot_last = 0; ym_dot_last = 0; zm_dot_last = 0;
    xc = 0.0; zc = 0.0; xc_last = 0.0; zc_last = 0.0;
    xc_dot = 0.0; zc_dot = 0.0; xc_dot_last = 0.0; zc_dot_last = 0.0;
    r = 0.0; r_last = 0.0; 
    r_dot = 0.0; r_dot_last = 0.0;
    
  }

  void trackercb(const puppeteer_msgs::PointPlus &point) {
    // request robot position
    srv.request.robot_index = 2;  // hardcoding this for now
    srv.request.type = 'w';
    srv.request.Vleft = 0.0;
    srv.request.Vright = 0.0;
    srv.request.Vtop = 0.0;
    srv.request.div = 0;

    if(client.call(srv)) {
      if(srv.response.error == 1) {
	ROS_DEBUG("Send Successful: speed_command\n");
      }
      else if(srv.response.error == 0) {
	ROS_DEBUG("Send Request Denied: speed_command\n");
	static bool request_denied_notify = true;
	if(request_denied_notify) {
	  ROS_ERROR("Send Requests Denied: speed_command\n");
	  request_denied_notify = false;
	}
      }
    }
    else {
      ROS_ERROR("Failed to call service: speed_command\n");
    }


    t_last = t_now;
    t_now = ros::Time::now();
    dt = t_now-t_last;

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

    // save current state as last
    state_last = state;

    // get error flag from object tracker data
    error_m = point.error;

    // did we get good mass position data from the object tracker and robot?
    if(error_m == false) {
      // get new mass position data
      xm = point.x;
      ym = point.y;
      zm = point.z;

      // get mew robot position data

      // calculate string length

      // calculate new velocities
      xm_dot = (xm - xm_last)/dt;
      ym_dot = (ym - ym_last)/dt;
      zm_dot = (zm - zm_last)/dt;
      xc_dot = (xc - xc_last)/dt;
      zc_dot = (zc - zc_last)/dt;
      r_dot = (r - r_last)/dt;
      
      // assign various components of system state
      state.xm = xm;
      state.ym = ym;
      state.xc = xc;
      state.r = r;
      state.xm_dot = xm_dot;
      state.ym_dot = xc_dot;
      state.xc_dot = xc_dot;
      state.r_dot = r_dot;

      // publish system state
      state_pub.publish(state);
    }
    // estimate positions and velocities if we missed a data point
    else {
      ROS_WARN("Missing Data Point");

      // use old velocity values
      xm_dot = xm_dot_last;
      ym_dot = ym_dot_last;
      zm_dot = zm_dot_last;
      xc_dot = xc_dot_last;
      zc_dot = zc_dot_last;
      r_dot = r_dot_last;

      // calculate new positions given these velocities
      xm += xm_dot*dt;
      ym += ym_dot*dt;
      zm += zm_dot*dt;
      xc += xc_dot*dt;
      zc += zc_dot*dt;
      r += r_dot*dt;

      // assign various components of system state
      state.xm = xm;
      state.ym = ym;
      state.xc = xc;
      state.r = r;
      state.xm_dot = xm_dot;
      state.ym_dot = xc_dot;
      state.xc_dot = xc_dot;
      state.r_dot = r_dot;

      // publish system state
      state_pub.publish(state);
    }
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


//---------------------------------------------------------------------------
// Main
//---------------------------------------------------------------------------

int main(int argc, char **argv)
{
  ros::init(argc, argv, "estimator_node");
  ros::NodeHandle n;

  ROS_INFO("Starting Estimator...\n");
  StateEstimator estimator;

  ros::spin();

  return 0;
}
