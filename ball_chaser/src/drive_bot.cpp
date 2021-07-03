#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h" //TODO: Include the ball_chaser "DriveToTarget" header file 

// ROS::Publisher motor commands;

ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
{
	// Save req in a variable
	auto linear_x = (float)req.linear_x;
	auto angular_z = (float)req.angular_z;

	ROS_INFO("DriveToPositionRequest received - linear_x:%1.2f, angular_z:%1.2f", linear_x, angular_z);

	// Publish velocities to robot wheel joint
	//geometry_msgs::Twist linear, angular; // create twist object
	//linear.x = linear_x;
	//angular.z = angular_z;

	//linear_publisher.publish(linear);
	//angular_publisher.publish(angular);

	// A neater way of publishing to robot wheel joint
	// Create a motor_command object of type geometry_msgs::Twist
	geometry_msgs::Twist motor_command;
	// Set wheel velocities and yaw orientation
	motor_command.linear.x = linear_x;
	motor_command.angular.z = angular_z;
	// Publish angles to drive the robot
	motor_command_publisher.publish(motor_command);

	
	// Wait 3 sec for robot to move
	// ros::Duration(3).sleep();

	// Return response msg
	res.msg_feedback = "Move command is set - linear_x: " + std::to_string(linear_x)
                                     + " , angular_z: " + std::to_string(angular_z);
  	ROS_INFO_STREAM(res.msg_feedback);

	return true;
}

int main(int argc, char** argv) 
{
	// Initialize a ROS node
	ros::init(argc, argv, "drive_bot"); 

	// Create a ROS NodeHandle object
	ros::NodeHandle n;

	// Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
	motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

	// TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
	ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);
	ROS_INFO("Send commands to move the robot!");

	// TODO: Delete the loop, move the code to the inside of the callback function and make the necessary changes to publish th e requested velocities instead of constant values

	//while (ros::ok()) {
		//// Create a motor_command object of type geometry_msgs::Twist 
		//geometry_msgs::Twist motor_command;
		//// Set wheel velocities, forward [0.5, 0.0] 
		//motor_command.linear.x = 0.5; 
		//motor_command.angular.z = 0.0;
		//// Publish angles to drive the robot 
		//motor_command_publisher.publish(motor_command);

	//}

	// TODO: Handle ROS communication events 
	ros::spin();

	return 0; 
}
