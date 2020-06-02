#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Path.h>

#include <sstream>




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
    ros::init(argc, argv, "test_pub");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n;

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
    ros::Publisher obj_pub = n.advertise<nav_msgs::Path>("/real_objs", 1);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped transformStamped2;
    geometry_msgs::TransformStamped transformStamped3;

    ros::Rate loop_rate(10);

    /**
     * A count of how many messages we have sent. This is used to create
     * a unique string for each message.
     */

    int count = 0;
    while (ros::ok())
    {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        nav_msgs::Path Path;

        // push_back obj_msg to Path msg

        //push_back obj0
        geometry_msgs::PoseStamped obj_msg0;
        obj_msg0.header.stamp = ros::Time::now();
        obj_msg0.header.frame_id = "map";
        obj_msg0.pose.position.x = -0.1008;
        obj_msg0.pose.position.y = -2.2281;
        obj_msg0.pose.position.z = 1.5609;
        obj_msg0.pose.orientation.w = 0.0239;
        obj_msg0.pose.orientation.x = 0.0240;
        obj_msg0.pose.orientation.y = 0.7082;
        obj_msg0.pose.orientation.z = -0.7052;
        // std::cout << "[Debug] obj_msg0: \n" << obj_msg0 << std::endl;
        Path.poses.push_back(obj_msg0);

        // push_back obj1
        geometry_msgs::PoseStamped obj_msg1;
        obj_msg1.header.stamp = ros::Time::now();
        obj_msg1.header.frame_id = "map";
        obj_msg1.pose.position.x = -0.7345;
        obj_msg1.pose.position.y = -2.1867;
        obj_msg1.pose.position.z = 1.5334;
        obj_msg1.pose.orientation.w = 0.0239;
        obj_msg1.pose.orientation.x = 0.0240;
        obj_msg1.pose.orientation.y = 0.7082;
        obj_msg1.pose.orientation.z = -0.7052;
        // std::cout << "[Debug] obj_msg1: \n" << obj_msg1 << std::endl;
        Path.poses.push_back(obj_msg1);

        //push_back obj2
        geometry_msgs::PoseStamped obj_msg2;
        obj_msg2.header.stamp = ros::Time::now();
        obj_msg2.header.frame_id = "map";
        obj_msg2.pose.position.x = -1.2585;
        obj_msg2.pose.position.y = -2.1484;
        obj_msg2.pose.position.z = 1.6762;
        obj_msg2.pose.orientation.w = 0.0239;
        obj_msg2.pose.orientation.x = 0.0240;
        obj_msg2.pose.orientation.y = 0.7082;
        obj_msg2.pose.orientation.z = -0.7052;
        // std::cout << "[Debug] obj_msg2: \n" << obj_msg2 << std::endl;
        Path.poses.push_back(obj_msg2);


        //TF obj0
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "map";
        transformStamped.child_frame_id = "real_obj0";
        transformStamped.transform.translation.x = -0.1008;
        transformStamped.transform.translation.y = -2.2281;
        transformStamped.transform.translation.z = 1.5609;

        transformStamped.transform.rotation.w = 0.0239;
        transformStamped.transform.rotation.x = 0.0240;
        transformStamped.transform.rotation.y = 0.7082;
        transformStamped.transform.rotation.z = -0.7052;

        //TF obj1
        transformStamped2.header.stamp = ros::Time::now();
        transformStamped2.header.frame_id = "map";
        transformStamped2.child_frame_id = "real_obj1";
        transformStamped2.transform.translation.x = -0.7345;
        transformStamped2.transform.translation.y = -2.1867;
        transformStamped2.transform.translation.z = 1.5334;

        transformStamped2.transform.rotation.w = 0.0239;
        transformStamped2.transform.rotation.x = 0.0240;
        transformStamped2.transform.rotation.y = 0.7082;
        transformStamped2.transform.rotation.z = -0.7052;

        //TF obj2
        transformStamped3.header.stamp = ros::Time::now();
        transformStamped3.header.frame_id = "map";
        transformStamped3.child_frame_id = "real_obj2";
        transformStamped3.transform.translation.x = -1.2585;
        transformStamped3.transform.translation.y = -2.1484;
        transformStamped3.transform.translation.z = 1.6762;

        transformStamped3.transform.rotation.w = 0.0239;
        transformStamped3.transform.rotation.x = 0.0240;
        transformStamped3.transform.rotation.y = 0.7082;
        transformStamped3.transform.rotation.z = -0.7052;

        br.sendTransform(transformStamped);
        br.sendTransform(transformStamped2);
        br.sendTransform(transformStamped3);
        

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        obj_pub.publish(Path);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }


  return 0;
}