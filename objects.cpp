#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <task_part_sim/simCom.h>
#include <task_part_sim/simDet.h>
using namespace std;

struct objectList{
int id;
float x;
float y;
}tempObj;

vector<objectList> objLoc;

void simulationDetails(const task_part_sim::simDet::ConstPtr& msg)
{
    int i;
    for(i=0;i<msg->numObjects;i++)
    {
        tempObj.id=i;
        tempObj.x=INT_MAX;
        tempObj.y=INT_MAX;
        objLoc.push_back(tempObj);
    }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "task_part_sim");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  ros::Rate r(30);

  float f = 0.0;
  while (ros::ok())
  {
    visualization_msgs::Marker points;
    points.header.frame_id = "/simulation";
    points.header.stamp = ros::Time::now();
    points.ns = "task_part_sim";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

 geometry_msgs::Point p;
     p.x = -5;
      p.y = 1;
      p.z = 0;
 points.points.push_back(p);
      p.x = 5;
      p.y = 1;
      p.z = 0;
 points.points.push_back(p);
      p.x = 0;
      p.y = 0;
      p.z = 0;
 points.points.push_back(p);


    marker_pub.publish(points);
    r.sleep();
  }
}
