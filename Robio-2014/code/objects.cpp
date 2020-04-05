/******************************************************
 *    objects.cpp                                      *
 *    Purpose: Object simulation code for ROS and Rviz *
 *                                                     *
 *    @author Nishant Sharma                           *
 *    @version 1.0 02/06/14                            *
 *    @note objects have hardcoded values              *
 ******************************************************/

// Including RosC++ Header Files
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// Including C++ Header Files
#include <iostream>
#include <math.h>
#include <stdlib.h>

// Including Custom Message Header Files
#include <ants2014/objLocate.h>
#include <ants2014/robObj.h>
#include <ants2014/simCom.h>
#include <ants2014/simDet.h>

using namespace std;

int foodValue1 = 1, foodValue2 = 10;

visualization_msgs::Marker nest, points;
geometry_msgs::Point p;
ros::Publisher objLocate;
ants2014::objLocate objectLocation;

int startFlag = 0; // For the master to set start flag to 1
int X, Y, i;       // temp X,Y used
int numStaticObject;

// Structure for storing object locations
struct objectList {
  int id;
  float x;
  float y;
  int value;
} tempObject;

// Vector for storing list of object locations
vector<objectList> objectLocationList;
vector<objectList> staticObjectLocationList1;
vector<objectList> staticObjectLocationList2;
// initialization function
void node_init() {
  // Nest parameter settings
  nest.header.frame_id = "/simulation";
  nest.header.stamp = ros::Time::now();
  nest.ns = "ObjectAndNest";
  nest.action = visualization_msgs::Marker::ADD;
  nest.pose.orientation.w = 1.0;
  nest.id = 0;
  nest.type = visualization_msgs::Marker::CUBE;
  nest.scale.z = 0.05;
  nest.color.g = 1.0f;
  nest.color.r = 1.0f;
  nest.color.b = 1.0f;
  nest.color.a = 1.0;
  nest.lifetime = ros::Duration();
  nest.pose.position.z = 0;

  // Points parameter settings
  points.header.frame_id = "/simulation";
  points.header.stamp = ros::Time::now();
  points.ns = "ObjectAndNest";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 1;
  points.type = visualization_msgs::Marker::POINTS;

  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.5;
  points.scale.y = 0.5;

  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;
}

// Function to set different parameters as recieved
void simulationDetails(const ants2014::simDet::ConstPtr &msg) {

  numStaticObject = msg->numObjects;

  X = msg->goalX;
  X += (numStaticObject / 2) * (-4);
  Y = msg->goalY + msg->source;
  i = 0;
  // Initializing Static object List not to be changed during simulation
  // for(i=0;i<numStaticObject;i++)
  {
    tempObject.id = i++;
    tempObject.x = -15;
    tempObject.y = 15;
    tempObject.value = foodValue1;
    staticObjectLocationList1.push_back(tempObject);
    // X+=4;
    tempObject.id = i++;
    tempObject.x = -19;
    tempObject.y = 15;
    tempObject.value = foodValue1;
    staticObjectLocationList1.push_back(tempObject);

    tempObject.id = i++;
    tempObject.x = -11;
    tempObject.y = 15;
    tempObject.value = foodValue1;
    staticObjectLocationList1.push_back(tempObject);

    tempObject.id = i++;
    tempObject.x = -15;
    tempObject.y = 11;
    tempObject.value = foodValue1;
    staticObjectLocationList1.push_back(tempObject);

    tempObject.id = i++;
    tempObject.x = -15;
    tempObject.y = 19;
    tempObject.value = foodValue1;
    staticObjectLocationList1.push_back(tempObject);
  }
  i = 0;
  // food patch 2
  {
    tempObject.id = i++;
    tempObject.x = 15;
    tempObject.y = 15;
    tempObject.value = foodValue2;
    staticObjectLocationList2.push_back(tempObject);

    // X+=4;
    tempObject.id = i++;
    tempObject.x = 19;
    tempObject.y = 15;
    tempObject.value = foodValue2;
    staticObjectLocationList2.push_back(tempObject);

    tempObject.id = i++;
    tempObject.x = 11;
    tempObject.y = 15;
    tempObject.value = foodValue2;
    staticObjectLocationList2.push_back(tempObject);

    tempObject.id = i++;
    tempObject.x = 15;
    tempObject.y = 11;
    tempObject.value = foodValue2;
    staticObjectLocationList2.push_back(tempObject);

    tempObject.id = i++;
    tempObject.x = 15;
    tempObject.y = 19;
    tempObject.value = foodValue2;
    staticObjectLocationList2.push_back(tempObject);
  }

  // Updating Nest location on RVIZ values
  nest.scale.x = msg->goalW;
  nest.scale.y = msg->goalL;
  nest.pose.position.x = msg->goalX;
  nest.pose.position.y = msg->goalY;

  cout << "\nDetails Received\n";
}

// Function to start or stop any simulation
void simulationCommand(const ants2014::simCom::ConstPtr &msg) {
  if (msg->command == 0) {
    exit(1);
  } else {
    startFlag = 1;
    cout << "\nNode Started\n";
  }
}

// Function to change Object Location (Add/Remove) during Simulation
void robObject(const ants2014::robObj::ConstPtr &msg) {
  // If the object is Picked
  if (msg->flag == -1) {
    for (i = 0; i < objectLocationList.size(); i++) {
      if (objectLocationList[i].x == msg->X &&
          objectLocationList[i].y == msg->Y) {
        objectLocationList.erase(objectLocationList.begin() + i);
        cout << "Object Deleted : " << i << " \n";
        cout << "TOTAL : " << objectLocationList.size() << " \n\n";
        break;
      }
    }
  }

  // If the Object is Droped
  if (msg->flag == 1) {
    tempObject.x = msg->X;
    tempObject.y = msg->Y;
    tempObject.value = msg->value;
    objectLocationList.push_back(tempObject);
    cout << "Object Added : " << objectLocation.id << " \n";
    cout << "TOTAL : " << objectLocationList.size() << " \n\n";
  }
}

// Main Function
int main(int argc, char **argv) {
  // Initializing ROS NODE values
  ros::init(argc, argv, "objects");
  ros::NodeHandle node;

  // Defination for ROS publisher on custom messages
  ros::Publisher marker_pub =
      node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  objLocate = node.advertise<ants2014::objLocate>("objectLocation", 10);

  node_init();

  int i = 0;

  // Defination for ROS subscriber on custom messages
  ros::Subscriber simDet =
      node.subscribe("simulationDetails", 5000, simulationDetails);
  ros::Subscriber simCom =
      node.subscribe("simulationCommand", 10, simulationCommand);
  ros::Subscriber robObj = node.subscribe("robObject", 1000, robObject);

  // Waiting for start command
  while (startFlag == 0)
    ros::spinOnce();

  while (ros::ok()) {
    ros::spinOnce();
    points.points.clear();

    // Publishing static Objects
    for (i = 0; i < staticObjectLocationList1.size(); i++) {
      objectLocation.id = staticObjectLocationList1[i].id = i;
      p.x = objectLocation.X = staticObjectLocationList1[i].x;
      p.y = objectLocation.Y = staticObjectLocationList1[i].y;
      p.z = 0;
      objectLocation.value = foodValue1;
      objectLocation.flag = 0;
      objectLocation.patch = 0;
      objLocate.publish(objectLocation);
      points.points.push_back(p);
    }

    for (i = 0; i < staticObjectLocationList2.size(); i++) {
      objectLocation.id = staticObjectLocationList2[i].id = i;
      p.x = objectLocation.X = staticObjectLocationList2[i].x;
      p.y = objectLocation.Y = staticObjectLocationList2[i].y;
      objectLocation.value = foodValue2;
      p.z = 0;
      objectLocation.flag = 0;
      objectLocation.patch = 1;
      objLocate.publish(objectLocation);
      points.points.push_back(p);
    }

    // Publishing dynamic Objects
    for (i = 0; i < objectLocationList.size(); i++) {
      objectLocation.id = objectLocationList[i].id = i + numStaticObject;
      p.x = objectLocation.X = objectLocationList[i].x;
      p.y = objectLocation.Y = objectLocationList[i].y;
      objectLocation.value = objectLocationList[i].value;
      p.z = 0;
      objectLocation.flag = 0;
      objLocate.publish(objectLocation);
      points.points.push_back(p);
    }

    // publishing Points and Nest
    marker_pub.publish(points);
    marker_pub.publish(nest);

    ros::Duration(0.5).sleep();
  }
}
