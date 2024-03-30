#include "ros/ros.h"
#include "std_msgs/Int16.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

// Create odometry data publishers
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;

//ros::Publisher duty_right_pub;
//ros::Publisher duty_left_pub;
//std_msgs::Int16 duty_right_val ;
//std_msgs::Int16 duty_left_val ;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

// Robot physical constants
const double TICKS_PER_REVOLUTION_RIGHT = 410; // For reference purposes. 1447
const double TICKS_PER_REVOLUTION_LEFT = 410; // For reference purposes.

const double WHEEL_RADIUS = 0.0315; // Wheel radius in meters
const double WHEEL_BASE = 0.162; // Center of left tire to center of right tire
const double TICKS_PER_METER_RIGHT = TICKS_PER_REVOLUTION_RIGHT /(PI*2*WHEEL_RADIUS); // Original was 2800
const double TICKS_PER_METER_LEFT = TICKS_PER_REVOLUTION_LEFT /(PI*2*WHEEL_RADIUS); // Original was 2800

// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;

//for velocity
double linear = 0;
double angular = 0;
double pre_linear = 0;
double pre_angular = 0;
//double wheel_right_vel_est = 0;
//double wheel_left_vel_est = 0;
double wheel_right_enc_vel_est = 0;
double wheel_left_enc_vel_est = 0;
double wheel_right_enc_vel_real = 0;
double wheel_left_enc_vel_real = 0;
int rightTicks = 0;
int leftTicks = 0;

//for PID
double Kp_r = 0.02; // 0.02
double Kp_l = 0.02;

double Ki_l = 0.000; // 0.0001
double Ki_r = 0.000;
double Kd = 0.0000;
double right_diff = 0;
double pre_right_diff = 0;
double left_diff = 0;
double pre_left_diff = 0;
double right_time = 0;
double pre_right_time = 0;
double left_time = 0;
double pre_left_time = 0;
int lastCountL = 0;
int lastCountR = 0;

bool mFirst = true;

using namespace std;



// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::Int16& leftCount) {

    if(leftCount.data != 0 ) {

     leftTicks = (leftCount.data - lastCountL);
    if (leftTicks > 10000) {
      leftTicks = 0 - (65535 - leftTicks);
    }
    else if (leftTicks < -10000) {
      leftTicks = 65535-leftTicks;
    }
    else{}
    distanceLeft = leftTicks/TICKS_PER_METER_LEFT;
  }
  lastCountL = leftCount.data;
  }


// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::Int16& rightCount) {


  if(rightCount.data != 0) {

    rightTicks = rightCount.data - lastCountR;
    if (rightTicks > 10000) {
      rightTicks = 0 - (65535 - rightTicks);
    }
    else if (rightTicks < -10000) {
      rightTicks = 65535 - rightTicks;
    }
    else{}
    distanceRight = rightTicks/TICKS_PER_METER_RIGHT;
  }
  lastCountR = rightCount.data;
  }

// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat() {

  tf2::Quaternion q;

  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

  for(int i = 0; i<36; i++) {
    if(i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
     }
     else if (i == 21 || i == 28 || i== 35) {
       quatOdom.pose.covariance[i] += 0.1;
     }
     else {
       quatOdom.pose.covariance[i] = 0;
     }
  }

  odom_data_pub_quat.publish(quatOdom);
}

// Update odometry information
void update_odom() {

  // Calculate the average distance
  double cycleDistance = (distanceRight + distanceLeft) / 2;
  // Calculate the number of radians the robot has turned since the last cycle
  double cycleAngle = asin((distanceRight-distanceLeft)/(WHEEL_BASE));
  distanceRight = 0;
  distanceLeft = 0;
  // Average angle during the last cycle
  double avgAngle = cycleAngle/2 + odomOld.pose.pose.orientation.z;

  if (avgAngle > PI) {
    avgAngle -= 2*PI;
  }
  else if (avgAngle < -PI) {
    avgAngle += 2*PI;
  }
  else{}

  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle)*cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle)*cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y)
     || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }

  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
  else{}

  // Compute the velocity
  odomNew.header.stamp = ros::Time::now();
  odomNew.twist.twist.linear.x = cycleDistance/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  odomNew.twist.twist.angular.z = cycleAngle/(odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());

  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;

  // Publish the odometry message
  odom_data_pub.publish(odomNew);
}

int main(int argc, char **argv) {

  // Set the data fields of the odometry message
  odomNew.header.frame_id = "odom";
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;

  // Launch ROS and create a node
  ros::init(argc, argv, "robot_odom");
  ros::NodeHandle node;

  // Subscribe to ROS topics
  ros::Subscriber subForRightCounts = node.subscribe("right_ticks", 100, Calc_Right, ros::TransportHints().tcpNoDelay());
  ros::Subscriber subForLeftCounts = node.subscribe("left_ticks", 100, Calc_Left, ros::TransportHints().tcpNoDelay());
  //ros::Subscriber subForCmdVel = node.subscribe("cmd_vel", 100, Calc_Vel, ros::TransportHints().tcpNoDelay());
  // Publisher of simple odom message where orientation.z is an euler angle
  odom_data_pub = node.advertise<nav_msgs::Odometry>("odom_data_euler", 100);
  // Publisher of full odom message where orientation is quaternion
  odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("odom_data_quat", 100);

  //duty_right_pub = node.advertise<std_msgs::Int16>("duty_right", 100);
  //duty_left_pub = node.advertise<std_msgs::Int16>("duty_left", 100);

  ros::Rate loop_rate(30);
  right_time = ros::Time::now().toSec();
  left_time = ros::Time::now().toSec();
  //duty_right_val.data = 0;
  //duty_left_val.data = 0;

  while(ros::ok()) {

      update_odom();
      publish_quat();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
