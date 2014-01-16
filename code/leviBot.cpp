/******************************************************
*    leviBot.cpp                                      *
*    Purpose: Robot simulation code for ROS and Rviz  *
*             with levi's Walk instead of random walk *
*                                                     *
*    @author Nishant Sharma                           *
*    @version 0.1 15/01/14                            *
******************************************************/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <math.h>
#include <stdlib.h>
#include <iostream>

//including the custom msg files
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
#define toObject 4

float leviLength[20]={1,1.5,2,2.5,1,1.5,2,3,3.5,1,1.5,5,1,1.7,6,7,2,2.50,3,8};
float leviFlag=0, leviTrav;

int totObjectsDeposited=0, totRobots=0, bound_flag=0, robotID=0, state=randWalk, startFlag=0, obstFlag=0, toFlag=0;
float X, Y, lastX, lastY, robotSpeed=0.001, direction, objMin, robMin, colDir, disToSource, nSLimit=2.0, robMinDist=2.0, robRepForce=3.0 ;

task_part_sim::robObj robObjct;
task_part_sim::robLocate myLoc;
visualization_msgs::Marker marker;
std_msgs::ColorRGBA blue,yellow,red;

//for toSource
float lastSourceX,lastSourceY,sourceDirection,sdFlag=0,sourceDistance;

//for randWalk
float rwSTime,rwETime,rwTTime,rwAvgTime,randWalkCount=0;

//for neighSearch
float centerX, centerY, nsFlag=0, nsTime=0;

//for toNest
float selLen=100,STime,ETime,TTime,disTrav;

//a structure to store the cost values respective of each length
struct costFn{
int id;
float length;
float cost;
}tempCost;

//structure to store the boundary details
struct boundaryVal{
float x1;
float y1;
float x2;
float y2;
float x3;
float y3;
float dv13;
float dv12;
float dv1m2;
float dv1m3;
}boundary;

//structure to store the nest details
struct nestVal{
float x1;
float y1;
float x2;
float y2;
float x3;
float y3;
float dv13;
float dv12;
float dv1m2;
float dv1m3;
float x;
float y;
}nest;

//structure to store the objects information
struct objectList{
int id;
float x;
float y;
float dist;
}tempObj,curObj;

//structure to store the other bots information
struct robotList{
int id;
float x;
float y;
float dist;
}tempRob,curRob;

//vector definition to make lists of objects and other robots
vector<robotList> roboLoc;
vector<objectList> objLoc;

//initialization of all the static part of the code
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
    marker.pose.position.y = 0;
    marker.pose.position.x = 0;
	//set the orientation
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    //setting the marker duration
	marker.lifetime = ros::Duration();

    // Set the marker type
    marker.type = visualization_msgs::Marker::SPHERE;
}

//function to start or stop the code based on the message received from the master node
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

//function to set the simulation details based on the message received from the master node
void simulationDetails(const task_part_sim::simDet::ConstPtr& msg)
{
    disToSource=msg->source;
    totRobots = msg->numRobots;


//initializing the other robots
    int i;
    for(i=0;i<msg->numRobots;i++)
    {
        tempRob.id=i;
        tempRob.x=INT_MAX;
        tempRob.y=INT_MAX;
        tempRob.dist=INT_MAX;
        roboLoc.push_back(tempRob);
    }

//initializing the objects information
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
        boundary.x1=msg->bounX-msg->bounW/2;
        boundary.y1=msg->bounY-msg->bounL/2;

        boundary.x2=msg->bounX+msg->bounW/2;
        boundary.y2=msg->bounY-msg->bounL/2;

        boundary.x3=msg->bounX-msg->bounW/2;
        boundary.y3=msg->bounY+msg->bounL/2;

        boundary.dv13 = ((boundary.x3-boundary.x1)*(boundary.x3-boundary.x1))+((boundary.y3-boundary.y1)*(boundary.y3-boundary.y1));
        boundary.dv12 = ((boundary.x2-boundary.x1)*(boundary.x2-boundary.x1))+((boundary.y2-boundary.y1)*(boundary.y2-boundary.y1));

      //  cout<<boundary.x1<<" "<<boundary.y1<<" "<<boundary.x2<<" "<<boundary.y2<<" "<<boundary.x3<<" "<<boundary.y3<<" \n";
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

        nest.dv13 = ((-nest.x1+nest.x3)*(-nest.x1+nest.x3))+((-nest.y1+nest.y3)*(-nest.y1+nest.y3));
        nest.dv12 = ((-nest.x1+nest.x2)*(-nest.x1+nest.x2))+((-nest.y1+nest.y2)*(-nest.y1+nest.y2));
       // cout<<nest.x<<" "<<nest.y<<nest.x1<<" "<<nest.y1<<nest.x2<<" "<<nest.y2<<nest.x3<<" "<<nest.y3<<"\n";
    }
}

//function updating other robot locations based on the message received from other robots
void robotLocator(const task_part_sim::robLocate::ConstPtr& msg)
{
    roboLoc[msg->id].x = msg->X;
    roboLoc[msg->id].y = msg->Y;
}

//function updating other object information based on the message received from object node
void objectLocator(const task_part_sim::objLocate::ConstPtr& msg)
{
    objLoc[msg->id].x = msg->X;
    objLoc[msg->id].y = msg->Y;
}

//main function
int main( int argc, char** argv )
{
    srand(time(0));
    node_init();
    ros::init(argc, argv, "myRobots");
    ros::NodeHandle n;

    //setting the publisher for the nodes
    ros::Publisher rviz = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher myLocate = n.advertise<task_part_sim::robLocate>("robotLocation", 10);
    ros::Publisher robObject = n.advertise<task_part_sim::robObj>("robObject", 10);

    marker.header.frame_id = "/simulation";
    marker.header.stamp = ros::Time::now();
    marker.ns = "robots";
    myLoc.id = marker.id = robotID;

    //setting the subscriber for the nodes
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

    obstFlag=0;
    objMin=1.0;
    robMin=1.0;
    curObj.id=-1;
    curRob.id=-1;

    //distance from object
    for(i=0;i<objLoc.size();i++)
    {
        objLoc[i].dist = sqrt(pow(marker.pose.position.x-objLoc[i].x,2) + pow(marker.pose.position.y-objLoc[i].y,2));
        if(objLoc[i].dist<objMin)
        {
            curObj=objLoc[i];
            objMin=objLoc[i].dist;
        }
    }

    //distance from other robots
    for(i=0;i<roboLoc.size();i++)
    {
        if(i==robotID)continue;
        roboLoc[i].dist = sqrt(pow(marker.pose.position.x-roboLoc[i].x,2) + pow(marker.pose.position.y-roboLoc[i].y,2));
    }

    //position update for the random walk state
    if(state==randWalk)
    {
        float randAngle = (rand() % 500);
        float angleChange = (randAngle/100) - 2.0;
        //cout<<angleChange<<" rw\n";
        cout<<"Random Walk\n\n";
        direction += angleChange;
        marker.color=red;
        if(curObj.id!=-1)
        {
            state=toObject;
            continue;
        }
    }

    //position update when the robot is moving towards the object
    if(state==toObject)
    {
        cout<<"Going towards the Object\n\n";
        if(curObj.id!=-1)
        {
            direction=atan2((curObj.y-marker.pose.position.y),(curObj.x-marker.pose.position.x));
            if(curObj.dist<=0.5)  //objMinDist
            {
                lastSourceX = curObj.x;
                lastSourceY = curObj.y;
                if(curObj.id<totRobots)
                {
                robObjct.id = curObj.id;
                robObjct.X = INT_MAX;
                robObjct.Y = INT_MAX;
                robObject.publish(robObjct);
                }
                state=toNest;
                toFlag=0;
                disTrav=0;
                ros::Duration(0.5).sleep();
                continue;
            }
        }
        else
        {
            state=randWalk;
            toFlag=0;
            continue;
        }
        marker.color=blue;
    }

    //position update when the robot is moving towards the nest
    if(state==toNest)
    {
        cout<<"Going towards the Nest\n\n";
        direction = atan2((nest.y-marker.pose.position.y),(nest.x-marker.pose.position.x));
       /* if(disTrav>=selLen)
        {
            robObjct.id = curObj.id;
            robObjct.X = marker.pose.position.x;
            robObjct.Y = marker.pose.position.y;
            robObject.publish(robObjct);
            state=toSource;
            disTrav=0;
            continue;
        }*/

        //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
        nest.dv1m2 = ((-nest.x1+marker.pose.position.x)*(-nest.x1+nest.x2))+((-nest.y1+marker.pose.position.y)*(-nest.y1+nest.y2));
        nest.dv1m3 = ((-nest.x1+marker.pose.position.x)*(-nest.x1+nest.x3))+((-nest.y1+marker.pose.position.y)*(-nest.y1+nest.y3));

        //test if the robot has entered the nest
        if((0 < nest.dv1m2) && (nest.dv1m2 < nest.dv12) && (0 < nest.dv1m3) && (nest.dv1m3 < nest.dv13))
        {
            robObjct.id = curObj.id;
            robObjct.X = INT_MAX;
            robObjct.Y = INT_MAX;
            robObject.publish(robObjct);
            state=toSource;
            disTrav=0;
            sdFlag=0;
        }
        marker.color=blue;
        disTrav+=robotSpeed;
        //cout<<"to nest data \n"<<nest.dv1m2<<"  "<<nest.dv12<<"  "<<nest.dv1m3<<"  "<<nest.dv13<<"\n\n  ";
    }

    //position update when the robot is moving back towards the source
    if(state==toSource)
    {
        cout<<"Going back towards the last Source\n\n";
        if(sdFlag==0)
        {
            sdFlag=1;
            sourceDirection = atan2((lastSourceY-marker.pose.position.y),(lastSourceX-marker.pose.position.x));
            sourceDistance = sqrt(pow(lastSourceX-marker.pose.position.x,2) + pow(lastSourceY-marker.pose.position.y,2));
            //cout<<sourceDistance<<"is the source distance";
            ros::Duration(0.5).sleep();
            lastSourceX= marker.pose.position.x;
            lastSourceY= marker.pose.position.y;
        }
        sourceDirection+=0.0005;
        //sourceDistance-=robotSpeed;
        //cout<<"dis travel"<<disTrav<<"\n";
        if(disTrav>sourceDistance)
        {
            nsFlag=0;
            disTrav=0;
            state=neighSearch;
            continue;
        }
        marker.color=yellow;
        disTrav=sqrt(pow(lastSourceX-marker.pose.position.x,2) + pow(lastSourceY-marker.pose.position.y,2));;
    }

    //position update while Searching the neighbourhood of the predict Source Location
    if(state==neighSearch)
    {
        cout<<"Searching the neighbourhood of the predicted Source Location\n";
        if(nsFlag==0)
        {
            centerX=marker.pose.position.x;
            centerY=marker.pose.position.y;
            nsFlag=1;
            nsTime=0;
        }
        float randAngle = (rand() % 500);
        float angleChange = (randAngle/100) - 2.0;
        //cout<<angleChange<<" ns \n";
        if(nsTime>15)  //nsTime max value
        {
            nsFlag=0;
            state=randWalk;
        }
        if(curObj.id!=-1)
        {
            nsFlag=0;
            state=toObject;
        }
        nsTime++;
        direction += angleChange;
        marker.color=yellow;
    }

    obstFlag=0;
    colDir=0;
    X=Y=0;
    lastX = marker.pose.position.x;
    lastY = marker.pose.position.y;

    //now actual step change as calculated in the state above
    {
        for(i=0;i<roboLoc.size();i++)
        {
            if(roboLoc[i].dist< robMinDist)
            {
                    obstFlag=1;
                    colDir= atan2((marker.pose.position.y-roboLoc[i].y),(marker.pose.position.x-roboLoc[i].x));
                    X+= (robRepForce-roboLoc[i].dist)+cos(colDir);
                    Y+= (robRepForce-roboLoc[i].dist)+sin(colDir);
            }
        }

        if(state==randWalk)
        {

            myLoc.X = marker.pose.position.x += robotSpeed+cos(direction) + X;
            myLoc.Y = marker.pose.position.y += robotSpeed+sin(direction) + Y;


        }
        if(state==toSource)
        {
            myLoc.X = marker.pose.position.x += robotSpeed+cos(sourceDirection) + X;
            myLoc.Y = marker.pose.position.y += robotSpeed+sin(sourceDirection) + Y;

        }
        if(state==toObject)
        {
            myLoc.X = marker.pose.position.x += robotSpeed+cos(direction) + X;
            myLoc.Y = marker.pose.position.y += robotSpeed+sin(direction) + Y;

        }
        if(state==neighSearch)
        {
            myLoc.X = marker.pose.position.x += robotSpeed+cos(direction) + X;
            myLoc.Y = marker.pose.position.y += robotSpeed+sin(direction) + Y;
            if((sqrt(pow(centerX-marker.pose.position.x,2) + pow(centerY-marker.pose.position.y,2))) > nSLimit )
            {
                marker.pose.position.x = lastX;
                marker.pose.position.y = lastY;
                continue;
            }

        }
        if(state==toNest)
        {
            myLoc.X = marker.pose.position.x += robotSpeed+cos(direction) + X;
            myLoc.Y = marker.pose.position.y += robotSpeed+sin(direction) + Y;
        }

        boundary.dv1m3 = ((marker.pose.position.x-boundary.x1)*(boundary.x3-boundary.x1))+((marker.pose.position.y-boundary.y1)*(boundary.y3-boundary.y1));
        boundary.dv1m2 = ((marker.pose.position.x-boundary.x1)*(boundary.x2-boundary.x1))+((marker.pose.position.y-boundary.y1)*(boundary.y2-boundary.y1));

       // cout<<"boudn values check\n"<<boundary.dv12<<"  "<<boundary.dv1m2<<"  "<<boundary.dv13<<"  "<<boundary.dv1m3<<"\n";
        if(!((0< boundary.dv1m2) && (boundary.dv1m2 < boundary.dv12)) || !((0<boundary.dv1m3) && (boundary.dv1m3 < boundary.dv13)))
        {
            direction = atan2((nest.y-lastY),(nest.x-lastX));
            marker.pose.position.x = lastX+(robotSpeed+cos(direction));
            marker.pose.position.y = lastY+(robotSpeed+sin(direction));
            ros::Duration(0.5).sleep();
            cout<<"out of boundary";
            //continue;
        }
    }
    cout<<myLoc.X<<"  " << myLoc.Y<<"\n"<<"\n";
    // Publish the marker
    rviz.publish(marker);
    myLocate.publish(myLoc);
    ros::Duration(0.2).sleep();
    }
}
