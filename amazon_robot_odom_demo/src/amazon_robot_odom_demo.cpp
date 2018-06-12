/*
 * amazon_robot_odom_demo.cpp
 *
 *  Created on:
 *  Author: Samour Hanon
 * Taken from: https://answers.ros.org/question/205134/odometry-based-out-and-back-script-py-to-c/
 * Modified by: Arsalan Akhter 
 */
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <angles/angles.h>
#include "std_msgs/Float32.h"
double x_current = 0;
double y_current = 0;
double theta_current = 0;
const double PI = 3.14159265358979323846;

tf::TransformListener *odom_listener;

void get_odom()
{
    tf::StampedTransform transform;
    try
    {
        odom_listener->lookupTransform("/odom", "/amazon_warehouse_robot/base", ros::Time(0), transform);

        x_current = transform.getOrigin().x();
        y_current = transform.getOrigin().y();
        theta_current = angles::normalize_angle_positive(tf::getYaw(transform.getRotation()));
        ROS_INFO("odom (x, y, theta) : (%f, %f, %f)", x_current, y_current, theta_current);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
    }
}

void move(ros::Publisher cmd_VelPublisher, double move_distance, double linearSpeed, int direction)
{
    // Current position
    double x = x_current;
    double y = y_current;

    // Initialize the movement command message
    geometry_msgs::Twist cmd_velMsg;
    // Set the movement command linear speed for forward motion
    cmd_velMsg.linear.x = direction * linearSpeed;

    // How fast to update the robot's movement?
    // Set the equivalent ROS rate variable
    ros::Rate rate(10.0);

    double d = 0;
    // Enter the loop to move the robot
    while (d < move_distance && ros::ok())
    {
        //Publish the Twist message and sleep 1 cycle
        cmd_VelPublisher.publish(cmd_velMsg);

        ros::spinOnce();
        rate.sleep();

        // Get the current odometry
        get_odom();

        // Compute the Euclidean distance from the start position
        d = sqrt(pow((x_current - x), 2) + pow((y_current - y), 2));
        ROS_INFO("d: %f", d);
    }

    // Stop the robot
    cmd_velMsg.linear.x = 0;
    cmd_VelPublisher.publish(cmd_velMsg);

    return;
}

void turn(ros::Publisher cmd_VelPublisher, double turn_angle, double angularSpeed, int direction)
{
    // Initialize the movement command message
    geometry_msgs::Twist cmd_velMsg;
    //Set the movement command rotation speed
    cmd_velMsg.angular.z = direction * angularSpeed;

    // How fast to update the robot's movement?
    // Set the equivalent ROS rate variable
    ros::Rate rate(10.0);

    // Current angle
    get_odom();
    double last_angle = theta_current;
    double angle = 0;

    if (direction == 1){	
     while ((angle < turn_angle) && ros::ok())
    {
        //Publish the Twist message and sleep 1 cycle
        cmd_VelPublisher.publish(cmd_velMsg);

        //ROS_INFO("current_theta: %f, angle + turn_angle: %f", angle, (angle + turn_angle));

        ros::spinOnce();
        rate.sleep();

        // Get the current odometry
        get_odom();

        // Compute the amount of rotation since the last loop
        angle += angles::normalize_angle(theta_current - last_angle);
        last_angle = theta_current;

        ROS_INFO("angle: %f", angle);
    }
   }
   else if (direction == -1){
     while ((angle > turn_angle) && ros::ok())
    {
        //Publish the Twist message and sleep 1 cycle
        cmd_VelPublisher.publish(cmd_velMsg);

        //ROS_INFO("current_theta: %f, angle + turn_angle: %f", angle, (angle + turn_angle));

        ros::spinOnce();
        rate.sleep();

        // Get the current odometry
        get_odom();

        // Compute the amount of rotation since the last loop
        angle -= angles::normalize_angle(theta_current - last_angle);
        last_angle = theta_current;

        ROS_INFO("angle: %f", angle);
    }
   }
   else 
        ROS_ERROR("Direction can only be +1 or -1");

    // Stop turning the robot
    cmd_velMsg.angular.z = 0;
    cmd_VelPublisher.publish(cmd_velMsg);
}

int main(int argc, char **argv)
{
    // Initiate new ROS node named "out_and_back"
    ros::init(argc, argv, "amazon_robot_odom_demo");
    ros::NodeHandle node;
   ros::Rate rate(0.2);

    // Publisher to the robot's velocity command topic
    ros::Publisher cmd_VelPublisher;
    // Advertise a new publisher for the simulated robot's velocity command topic
    cmd_VelPublisher = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    // Publisher to the robot's joint_cmd topic
    ros::Publisher joint_cmd;
    // Advertise a new publisher for the simulated robot's joint_cmd topic
    joint_cmd = node.advertise<std_msgs::Float32>("/amazon_warehouse_robot/joint_cmd", 10);


    odom_listener = new tf::TransformListener();
    // Find out if the robot uses /base_link or /base_footprint
    try
    {
        odom_listener->waitForTransform("/odom", "amazon_warehouse_robot/base", ros::Time(0), ros::Duration(1.0));

    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("Cannot find transform between /amazon_warehouse_robot/base and /odom :", ex.what());
        return 0;
    }

    // Robot specific variables
    // -----------------------
    // Set the forward linear speed to 1.0 meters per second
    double linear_speed = 0.5;

    // Set the travel distance in meters
    double travel_distance = 3.0;

    // Set the rotation speed in radians per second
    double angular_speed = 0.1;

    // Set the rotation angle to PI radians (180 degrees)
    double turn_angle1 = M_PI_2;
    double turn_angle2 = M_PI;
	
    // Set the joint force to be  applied on joint_cmd
    std_msgs::Float32 force;
    force.data = 20;
    // Lets write a sequence that moves the robot under a pallet	
    move(cmd_VelPublisher, 0.25, linear_speed, 1);
    //rate.sleep();
    turn(cmd_VelPublisher, turn_angle1, angular_speed, 1);
    //rate.sleep();
    move(cmd_VelPublisher, 3.75, linear_speed, 1);
    //rate.sleep();
    turn(cmd_VelPublisher, turn_angle1-0.1, angular_speed, 1);
    rate.sleep();
    move(cmd_VelPublisher, 1.0, linear_speed, -1);
    rate.sleep();
    //turn(cmd_VelPublisher, turn_angle1, angular_speed, 1);
    //rate.sleep();
    //move(cmd_VelPublisher, 1, linear_speed, 1);
    joint_cmd.publish(force);
    rate.sleep();
    //rate.sleep();
    move(cmd_VelPublisher, 1, linear_speed, 1);
    rate.sleep();
    turn(cmd_VelPublisher, turn_angle1-0.2, angular_speed, 1);
    rate.sleep();
    move(cmd_VelPublisher, 6, linear_speed, 1);
    rate.sleep();
    turn(cmd_VelPublisher, turn_angle1+0.1, angular_speed, 1);
    move(cmd_VelPublisher, 2, linear_speed, 1);
    rate.sleep();       
    force.data = 0;
    joint_cmd.publish(force);
    rate.sleep();
    move(cmd_VelPublisher, 2, linear_speed, 1);
    

    delete odom_listener;

    return 0;
}
