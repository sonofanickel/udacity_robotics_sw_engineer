#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

void turtlebotArrivedCallback(const std_msgs::String::ConstPtr& msg)
{
   // Decode message. Format is "<location name>,X,Y", e.g. "pickup,1,1", "dropoff,-1,-1"
   std::stringstream ss(msg->data);
   std::vector<std::string> tokens;

   while( ss.good() )
   {
      std::string token;
      getline( ss, token, ',' );
      tokens.push_back(token);
   }

   if (tokens.size() != 3)
   {
      ROS_WARN("Received malformed turtlebot_arrived message.");
   }
   else if (tokens[0] == "pickup")
   {
      // Hide marker
      marker.action = visualization_msgs::Marker::DELETE;
      marker_pub.publish(marker);
   }
   else if (tokens[0] == "dropoff")
   {
      // Show marker
      marker.pose.position.x = std::stod(tokens[1].c_str());
      marker.pose.position.y = std::stod(tokens[2].c_str());
      marker.action = visualization_msgs::Marker::ADD;
      marker_pub.publish(marker);
   }
   else
   {
      ROS_WARN("Received unknown data in turtlebot_arrived message.");
   }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber sub = n.subscribe("turtlebot_arrived", 10, turtlebotArrivedCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "add_markers";
  marker.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  marker.type = shape;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = 1.5;
  marker.pose.position.y = 1.0;
  marker.pose.position.z = 0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  marker.scale.x = 0.2;
  marker.scale.y = 0.2;
  marker.scale.z = 0.2;

  // Set the color -- be sure to set alpha to something non-zero!
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration(0);

  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    if (!ros::ok())
    {
      return 0;
    }
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }
  marker_pub.publish(marker);
  ROS_INFO("published the marker");

  ros::spin();
}
