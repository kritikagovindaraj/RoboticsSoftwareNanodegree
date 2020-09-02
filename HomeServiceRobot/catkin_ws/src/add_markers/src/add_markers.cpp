#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>


double distance_threshold = 0.01;
double PICKUP_X = 2.0, PICKUP_Y = 2.0;
double DROPOFF_X = -2.0, DROPOFF_Y = 3.0;
double robot_x, robot_y;

int WAIT_TIME = 5;
int current_wait_ = 0;

void robotPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &amcl_msg)
{
  // Update robot position
  robot_x = amcl_msg->pose.pose.position.x;
  robot_y = amcl_msg->pose.pose.position.y;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  int state = 0;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  visualization_msgs::Marker mark;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
  mark.header.frame_id = "map";
  mark.header.stamp = ros::Time::now();

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  mark.ns = "basic_shapes";
  mark.id = 0;

  // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
  mark.type = shape;

  // Set marker orientation
  mark.pose.orientation.x = 0.0;
  mark.pose.orientation.y = 0.0;
  mark.pose.orientation.z = 0.0;
  mark.pose.orientation.w = 1.0;

  // Set the scale of the marker -- 1x1x1 here means 1m on a side
  mark.scale.x = 0.3;
  mark.scale.y = 0.3;
  mark.scale.z = 0.3;

  // Set the color -- be sure to set alpha to something non-zero!
  mark.color.r = 0.0f;
  mark.color.g = 0.0f;
  mark.color.b = 1.0f;
  mark.color.a = 1.0;

  mark.pose.position.z = 0;

  mark.lifetime = ros::Duration();

  // Subscribe to /amcl_pose
  ros::Subscriber sub1 = n.subscribe("/amcl_pose", 1000, robotPoseCallback);

  while (ros::ok())
  {

    // State transitions

    if (state == 0)
    {
      // Calculate manhattan distance
      double pickup_distance = abs(robot_x - PICKUP_X) + abs(robot_y - PICKUP_Y);

      if (pickup_distance > distance_threshold)
      {
        mark.action = visualization_msgs::Marker::ADD;
        mark.pose.position.x = PICKUP_X;
        mark.pose.position.y = PICKUP_Y;
      }
      else
      {
        state = 1;
        mark.action = visualization_msgs::Marker::DELETE;
        ROS_INFO("object was picked up");
        ros::Duration(2.0).sleep();
      }
    }

    else if (state == 1)
    {
      if (current_wait_ < WAIT_TIME)
      {
        current_wait_ += 1;
      }
      else
      {
        state = 2;
      }
    }

    else if (state == 2)
    {
      // Calculate manhattan distance
      double dropoff_distance = abs(robot_x - DROPOFF_X) + abs(robot_y - DROPOFF_Y);

      if (dropoff_distance > distance_threshold)
      {
        mark.action = visualization_msgs::Marker::DELETE;
      }
      else
      {
        mark.action = visualization_msgs::Marker::ADD;
        mark.pose.position.x = DROPOFF_X;
        mark.pose.position.y = DROPOFF_Y;
        ROS_INFO("object was dropped off");
        ros::Duration(2.0).sleep();
      }
    }

    // Publish the Marker
    marker_pub.publish(mark);

    // Sleep for 1 seconds
    sleep(1);

    // Handle ROS communication events
    ros::spinOnce();
  }
}
