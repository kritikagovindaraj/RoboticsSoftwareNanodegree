#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

ros::ServiceClient client;

void drive_robot(float lin_x, float ang_z)
{
  ROS_INFO_STREAM("Driving the robot");

  ball_chaser::DriveToTarget srv;
  srv.request.linear_x = lin_x;
  srv.request.angular_z = ang_z;

  if(!client.call(srv))
    {
      ROS_ERROR("Failed to call service");
    }
}
void process_image_callback(const sensor_msgs::Image img)
{
  int i;

  int ball_position;
  int ball_position_centre;
  int ball_position_sum = 0;
  int white_pixel_num = 0;

  for(i=0; i+2 < img.data.size(); i=i+3)
    {
      if((img.data[i]==255) && (img.data[i+1] == 255) && (img.data[i+2] ==255))
	{
	  ball_position = (i%(img.width * 3))/3;
	  ball_position_sum += ball_position;
	  white_pixel_num ++;
	}
    }
  if(white_pixel_num == 0)
    {
      drive_robot(0,0);
    }
  else
    {
      ball_position_centre = ball_position_sum/white_pixel_num;
      if(ball_position_centre < img.width/3)
	{
	  drive_robot(0.1, 0.5);
	}
      else if(ball_position_centre > img.width * 2/3)
	{
	  drive_robot(0.1, -0.5);
	}
      else
	{
	  drive_robot(0.1,0);
	}
    }
    
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "process_image");
  ros::NodeHandle n;

  client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
  ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw",10,process_image_callback);
  ros::spin();

  return 0;
}
