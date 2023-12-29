#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

enum IMG_ZONE {
    none = 0,
    zone1,
    zone2,
    zone3
};

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z) {
    ball_chaser::DriveToTarget srv;

    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service command_robot");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img) {
    int white_pixel = 255;

    // Divide step into three zones representing left, center, and right zones.
    // Truncate result since we don't need to be precise.
    int zone_width = img.step / 3;
    int zone1_right_boundary = zone_width;
    int zone2_right_boundary = zone_width * 2;
    int zone3_right_boundary = zone_width * 3;

    IMG_ZONE zone = IMG_ZONE::none;
    // Loop through each pixel in the image looking for white ball.
    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel &&
            img.data[i + 1] == white_pixel &&
            img.data[i + 2] == white_pixel) {

            // Found the ball, now locate which zone of the image it's in.
            unsigned int loc = i % img.step;

            if (loc <= zone1_right_boundary) {
                zone = IMG_ZONE::zone1;
            } else if (loc > zone1_right_boundary && loc <= zone2_right_boundary) {
                zone = IMG_ZONE::zone2;
            } else if (loc > zone2_right_boundary && loc <= zone3_right_boundary) {
                zone = IMG_ZONE::zone3;
            }

            break;
        }
    }

    if (zone == IMG_ZONE::zone1) {
        // Drive left
        drive_robot(0.0, 0.5);
    } else if (zone == IMG_ZONE::zone2) {
        // Drive forward
        drive_robot(0.5, 0.0);
    } else if (zone == IMG_ZONE::zone3) {
        // Drive right
        drive_robot(0.0, -0.5);
    } else {
        // Don't search for the ball, just stop the robot.
        drive_robot(0.0, 0.0);
    }
}

int main(int argc, char **argv) {
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}