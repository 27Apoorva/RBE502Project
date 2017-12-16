#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/GridCells.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <autorally_msgs/chassisState.h>
#include <autorally_msgs/wheelSpeeds.h>
#include <math.h>
#include <tf2_msgs/TFMessage.h>

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}
// void stateEstimatorCallback(const nav_msgs::OdometryConstPtr& state_estimator_msg)
// {
//   double x_;      ///< X position of vehicle relative to initial pose
//   double y_;      ///< Y position of vehicle relative to initial pose
//
//   x_ = state_estimator_msg->pose.pose.position.x;
//   y_ = state_estimator_msg->pose.pose.position.y;
//
//     ROS_INFO("My position(x,y): [%f,%f]", x_,y_);
// }
void stateEstimatorCallback(const tf2_msgs::TFMessage::ConstPtr& state_estimator_msg)
{
 float x_;      ///< X position of vehicle relative to initial pose
   float y_;      ///< Y position of vehicle relative to initial pose
   int size;
   size = state_estimator_msg->transforms.size();
   ROS_INFO("SIZE %d",size);
  x_ = state_estimator_msg->transforms[1].transform.translation.x;
  y_ = state_estimator_msg->transforms[1].transform.translation.y;

  ROS_INFO("My position(x,y): [%f,%f]",x_,y_);

}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "tf");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  tf::TransformListener listener;
  //ROS_INFO("%f",listener.x);
  ros::NodeHandle n;
//  ros::Rate rate(10.0);
  // try
  // {
  //   tf::StampedTransform transform;
  //   listener.lookupTransform("/chassis","/base_link",
  //                           ros::Time(0), transform);
  //   ROS_INFO("got tranform %f", transform.getOrigin().x());
  // }
  // catch (tf::TransformException &ex)
  // {
  //   ROS_ERROR("%s",ex.what());
  //   ros::Duration(1.0).sleep();
  // }


  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber state_estimator_sub;
  state_estimator_sub = n.subscribe("tf", 1,stateEstimatorCallback);

//  ros::Subscriber sub = n.subscribe("tf", 1000, chatterCallback);
  geometry_msgs::Point p1;
  p1.x = 10.9;
  p1.y = 100;
  p1.z = 0;
  ROS_INFO("%f", p1.x);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
//from nav_msgs.msg import GridCells
//from std_msgs.msg import String
//from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
//from nav_msgs.msg import Odometry, OccupancyGrid, Path
// from kobuki_msgs.msg import BumperEvent
// from copy import deepcopy
// from kobuki_msgs.msg import BumperEvent
// from tf.transformations import euler_from_quaternion
// import rospy, tf, numpy, math, random, time, os
