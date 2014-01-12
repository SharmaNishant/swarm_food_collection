#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <task_part_sim/simCom.h>
#include <task_part_sim/simDet.h>
#include <task_part_sim/robLocate.h>
#include <task_part_sim/objLocate.h>
#include <task_part_sim/robObj.h>
#include <task_part_sim/logInfo.h>
using namespace std;

#define neighSearch 0
#define randWalk 1
#define toNest 2
#define toSource 3

int robotID=0;

int state=randWalk,startFlag=0;
float robotSpeed=0.001,direction,objMin,robMin;

task_part_sim::robLocate myLoc;
visualization_msgs::Marker marker;
std_msgs::ColorRGBA  blue,yellow,red;

int bound_flag=0;

struct boundaryVal{
float x1;
float y1;
float x2;
float y2;
float x3;
float y3;
}boundary;

struct nestVal{
float x1;
float y1;
float x2;
float y2;
float x3;
float y3;
float x;
float y;
}nest;

struct objectList{
int id;
float x;
float y;
float dist;
}tempObj,curObj;

struct robotList{
int id;
float x;
float y;
float dist;
}tempRob,curRob;

vector<robotList> roboLoc;
vector<objectList> objLoc;

void node_init()
{
    //defining color values
    {
        yellow.r = 255;
        yellow.g = 255;
        yellow.b = 0;

        red.g = blue.g = 0;
        red.r = 255;
        red.b = 0;

        blue.r = 0;
        blue.b = 255;

        yellow.a = red.a = blue.a = 1;
    }

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;
    marker.pose.position.z = 0;
	//set the orientation
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;


	marker.lifetime = ros::Duration();

    // Set the marker type
    marker.type = visualization_msgs::Marker::SPHERE;
}

void simulationCommand(const task_part_sim::simCom::ConstPtr& msg)
{

    if(msg->command==0)
    {
        exit(1);
    }
    else
    {
        startFlag=1;
    }
}

void simulationDetails(const task_part_sim::simDet::ConstPtr& msg)
{
    int i;
    for(i=0;i<msg->numRobots;i++)
    {
        tempRob.id=i;
        tempRob.x=INT_MAX;
        tempRob.y=INT_MAX;
        tempRob.dist=INT_MAX;
        roboLoc.push_back(tempRob);
    }

    for(i=0;i<msg->numObjects;i++)
    {
        tempObj.id=i;
        tempObj.x=INT_MAX;
        tempObj.y=INT_MAX;
        tempObj.dist=INT_MAX;
        objLoc.push_back(tempObj);
    }

    //setting boundary value
    {
        boundary.x1=msg->bounX;
        boundary.y1=msg->bounY;
        boundary.x2=msg->bounX;
        boundary.y2=msg->bounY+msg->bounL;
        boundary.x3=msg->bounX+msg->bounW;
        boundary.y3=msg->bounY;
    }

    //setting goal value
    {
        nest.x=msg->goalX;
        nest.y=msg->goalY;

        nest.x2=msg->goalX+(msg->goalW/2);
        nest.y2=msg->goalY-(msg->goalL/2);

        nest.x3=msg->goalX-(msg->goalW/2);
        nest.y3=msg->goalY+(msg->goalL/2);

        nest.x1=msg->goalX+(msg->goalW/2);
        nest.y1=msg->goalY+(msg->goalL/2);
    }
}

void robotLocator(const task_part_sim::robLocate::ConstPtr& msg)
{
    roboLoc[msg->id].x = msg->X;
    roboLoc[msg->id].y = msg->Y;
}

void objectLocator(const task_part_sim::objLocate::ConstPtr& msg)
{
    objLoc[msg->id].x = msg->X;
    objLoc[msg->id].y = msg->Y;
}

int main( int argc, char** argv )
{
node_init();
ros::init(argc, argv, "myRobots");
ros::NodeHandle n;
ros::Publisher rviz = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
ros::Publisher myLocate = n.advertise<task_part_sim::robLocate>("robotLocation", 1);

marker.header.frame_id = "/simulation";
marker.header.stamp = ros::Time::now();
marker.ns = "task_part_sim";
myLoc.id = marker.id = robotID;


ros::Subscriber simDet = n.subscribe("simulationDetails", 500, simulationDetails);
ros::Subscriber simCom = n.subscribe("simulationCommand", 1000, simulationCommand);
ros::Subscriber robotLocate = n.subscribe("robotLocation", 1000, robotLocator);
ros::Subscriber objectLocate = n.subscribe("objectLocation", 1000, objectLocator);
    int i;
    while(startFlag==0) ros::spinOnce();
  while (ros::ok())
  {
    ros::spinOnce();
    // Set the marker action.  Options are ADD and DELETE
    marker.action = visualization_msgs::Marker::ADD;
        objMin=1.0;
        robMin=1.0;
        curObj.id=-1;
        curRob.id=-1;
        for(i=0;i<objLoc.size();i++)
        {
         objLoc[i].dist = sqrt(pow(marker.pose.position.x-objLoc[i].x,2) + pow(marker.pose.position.y-objLoc[i].y,2));
         if(objLoc[i].dist<objMin)
         {
                curObj=objLoc[i];
                objMin=objLoc[i].dist;
         }
        }
        for(i=0;i<roboLoc.size();i++)
        {
         roboLoc[i].dist = sqrt(pow(marker.pose.position.x-roboLoc[i].x,2) + pow(marker.pose.position.y-roboLoc[i].y,2));
        }
    if(state==randWalk)
    {
        float randAngle = (rand() % 500);
        float angleChange = (randAngle/100) - 2.0;
        cout<<angleChange<<"\n";
        direction += angleChange;
    }

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    myLoc.X = marker.pose.position.x += robotSpeed+cos(direction);
    myLoc.Y = marker.pose.position.y += robotSpeed+sin(direction);

    // Set the color
	marker.color=red;

    // Publish the marker
    rviz.publish(marker);
    myLocate.publish(myLoc);
    ros::Duration(0.2).sleep();
  }
}


