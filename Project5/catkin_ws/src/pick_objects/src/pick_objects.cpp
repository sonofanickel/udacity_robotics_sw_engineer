#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/String.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;

  ros::Publisher arrived_pub = n.advertise<std_msgs::String>("turtlebot_arrived", 10);
  std_msgs::String arrived_at_pickup_msg;
  arrived_at_pickup_msg.data = "pickup,1.5,1.0";
  std_msgs::String arrived_at_dropoff_msg;
  arrived_at_dropoff_msg.data = "dropoff,-1,0";

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = 1.5;
  goal.target_pose.pose.position.y = 1.0;
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending pickup zone");
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the robot has reached the pickup zone!");
    arrived_pub.publish(arrived_at_pickup_msg);

    ROS_INFO("Picking up package, please wait.");
    ros::Duration(5).sleep();
    
    // Define the drop off zone
    move_base_msgs::MoveBaseGoal goal2;
    goal2.target_pose.header.frame_id = "map";
    goal2.target_pose.header.stamp = ros::Time::now();
  
    goal2.target_pose.pose.position.x = -1.0;
    goal2.target_pose.pose.position.y = 0.0;
    goal2.target_pose.pose.orientation.w = 1.0;

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Sending drop off zone");
    ac.sendGoal(goal2);
  
    // Wait an infinite time for the results
    ac.waitForResult();
  
    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Hooray, the robot has reached the drop off zone!");
      arrived_pub.publish(arrived_at_dropoff_msg);
    } else {
      ROS_INFO("The robot failed to reach the drop off zone for some reason");
    }
  } else {
    ROS_INFO("The robot failed to reach the pickup zone for some reason");
  }

  return 0;
}
