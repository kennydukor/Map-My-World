#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h" 
#include <sensor_msgs/Image.h>

// Define a global client that can request services

ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction

void drive_robot(float lin_x, float ang_z) {

	// TODO: Request a service and pass the velocities to it to drive the robot
	ROS_INFO_STREAM("Moving robot towards ball");

	// Request 
	ball_chaser::DriveToTarget srv;
	srv.request.linear_x = lin_x;
	srv.request.angular_z = ang_z;

	// Call the command_robot service with request values given above
	if (!client.call(srv))
		ROS_ERROR("Failed to call service /ball_chaser/command_robot");
}

// This callback function continuously executes and reads the image data

void process_image_callback(const sensor_msgs::Image& img)

{
    	int white_pixel = 255;
    
    	// TODO: Loop through each pixel in the image and check if there's a bright white one
    	// Then, identify if this pixel falls in the left, mid, or right side of the image
    	// Depending on the white ball position, call the drive_bot function and pass velocities to it
    	// Request a stop when there's no white ball seen by the camera

	// Documentation for sensor_msgs/Image Message https://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/Image.html
	
	auto img_sec_width = img.step / 3; // Divide img width (.step) to three part (left, right, middle)
	int img_unrolled = img.step * img.height; // unrolled image pixel (img.step returns unrolled RGB)
	int left_trace_cnt = 0; // number of white traces found in the left
	int right_trace_cnt = 0; // number of white traces found in the right
	int middle_trace_cnt = 0; // number of white traces found in the middle

	
	for (int i=0; i < img_unrolled; i += 3) // first three pixel is RGB, next three is next RGB 
	{
		// Get RGB colors
		int red_channel = img.data[i];
		int green_channel = img.data[i+1];
		int blue_channel = img.data[i+2];

		if (red_channel == white_pixel && green_channel == white_pixel && blue_channel == white_pixel) {
			// determine location of white image in sections (left, right, middle)
			int section = i % img.step; // modulus return same value across the row wrt the width
			if (section < img_sec_width) {left_trace_cnt += 1; }
			else if (section > img_sec_width && section < img_sec_width * 2) {middle_trace_cnt += 1; }
			else {right_trace_cnt += 1; }
		}
	}

	// Drive robot towards the ball
	if (left_trace_cnt > right_trace_cnt && left_trace_cnt > middle_trace_cnt) {
		// Drive to left
		drive_robot(0.4, 0.15);
	} 
	else if (right_trace_cnt > left_trace_cnt && right_trace_cnt > middle_trace_cnt) {
		// Drive to right		
		drive_robot(0.4, -0.15);
	} 
	else if (middle_trace_cnt > right_trace_cnt && middle_trace_cnt > left_trace_cnt) {
		// Drive to straight		
		drive_robot(0.4, 0.0);
	} 
	else {
		// Drive to stop
		drive_robot(0.0, 0.0);
	}
	

}

int main(int argc, char** argv)

{
    	// Initialize the process_image node and create a handle to it
    	ros::init(argc, argv, "process_image");
    	ros::NodeHandle n;

    	// Define a client service capable of requesting services from command_robot
    	client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

	// Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
	ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

	// Handle ROS communication events
	ros::spin();
}
