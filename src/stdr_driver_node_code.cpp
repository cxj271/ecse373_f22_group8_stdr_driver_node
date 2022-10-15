#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <sstream>



ros::Publisher *p_cmd_vel_pub;

// Need these to be global messages
geometry_msgs::Twist desired;  // the desired velocity taken from rqt_gui
geometry_msgs::Twist cmd_vel;  // command velocity to the robot. may differ from the desired
sensor_msgs::LaserScan laser;  // readings from the lidar
bool found_laser = false;      // only runs the part of the while loop that warns and stops if the lidar was set up



void desiredVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
   desired.linear = msg->linear;
   desired.angular = msg->angular;
   //ROS_INFO("Desired forward velocity is %2.2f; Desired angular velocity %2.2f]", msg->linear.x, msg->angular.z);
   //p_cmd_vel_pub->publish(*msg);
}

void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // need this to avoid "Segmentation fault (core dumped)" error
   found_laser = true;
   ROS_DEBUG_ONCE("Thanks for making the robot!");
   laser.ranges=msg->ranges;
   //ROS_INFO("LaserScan sequence number is: %i", msg->header.seq);
}



/**
* This tutorial demonstrates simple sending of messages over the ROS system.
*/
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
 ros::init(argc, argv, "talker");

 /**
  * NodeHandle is the main access point to communications with the ROS system.
  * The first NodeHandle constructed will fully initialize this node, and the last
  * NodeHandle destructed will close down the node.
  */
 ros::NodeHandle n;

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

 ros::Subscriber sub = n.subscribe("des_vel", 1000, desiredVelCallback);
 ros::Subscriber sub1 = n.subscribe("laser_1", 1000, LaserScanCallback);

 /**
  * The advertise() function is how you tell ROS that you want to
  * publish on a given topic name. This invokes a call to the ROS
  * master node, which keeps a registry of who is publishing and who
  * is subscribing. After this advertise() call is made, the master
  * node will notify anyone who is trying to subscribe to this topic name,
  * and they will in turn negotiate a peer-to-peer connection with this
  * node.  advertise() returns a Publisher object which allows you to
  * publish messages on that topic through a call to publish().  Once
  * all copies of the returned Publisher object are destroyed, the topic
  * will be automatically unadvertised.
  *
  * The second parameter to advertise() is the size of the message queue
  * used for publishing messages.  If messages are published more quickly
  * than we can send them, the number here specifies how many messages to
  * buffer up before throwing some away.
  */
 ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
 p_cmd_vel_pub = &cmd_vel_pub;

 ros::Rate loop_rate(10);

 /**
  * A count of how many messages we have sent. This is used to create
  * a unique string for each message.
  */

// initializing variables
 int count = 0;
 bool close = false;
 bool stop = false;
 while (ros::ok())
 {
   /**
    * This is a message object. You stuff it with data, and then publish it.
    */

    //set the command to the desired
    cmd_vel = desired;

    //reset to false
    close = false;
    stop = false;

   if (found_laser){
      // looks for objects plus/minus ~20 degrees from straight ahead
    for (int i=115; i<156; i++){

      	if (laser.ranges[i] < 1.5){
      	  close = true;

      	  if(laser.ranges[i] < 0.85){
      	    stop = true;
      	  }
      	}
     }

      	if (stop && desired.linear.x > 0){
      	ROS_ERROR("Too close to wall, stop");

        cmd_vel.linear.x = 0;
        }

        else if (close){
        ROS_WARN("slow down!! wall ahead");
        }

        else {
          ROS_INFO("You're Good...");
        }

        cmd_vel_pub.publish(cmd_vel);

   }

   else if (!found_laser){
     ROS_FATAL("Add a robot with lidar!!");
   }



      ros::spinOnce();

    loop_rate.sleep();
    ++count;

}
   return 0;
}
