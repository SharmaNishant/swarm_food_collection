/*********************************************************
*    master.cpp                                          *
*    Purpose: Master node for simulation on ROS and Rviz *
*                                                        *
*    @author Nishant Sharma                              *
*    @version 0.3 14/01/14                               *
*********************************************************/

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <stdlib.h>
#include <iostream>
#include <task_part_sim/simCom.h>
#include <task_part_sim/simDet.h>
#include <task_part_sim/logInfo.h>

using namespace std;

//publishing the log as it is received from the bots
void taskPartLog(const task_part_sim::logInfo::ConstPtr& msg)
{
    cout<<msg->id<<"   "<<msg->randTime<<"   "<<msg->flag<<"\n";
}

int main( int argc, char** argv )
{
    float time=0;
ros::init(argc, argv, "master");
ros::NodeHandle n;

ros::Publisher simDet = n.advertise<task_part_sim::simDet>("simulationDetails", 10);
ros::Publisher simCom = n.advertise<task_part_sim::simCom>("simulationCommand", 10);
ros::Subscriber taskPartLoger = n.subscribe("taskPartLog", 1000, taskPartLog);

task_part_sim::simDet details;
task_part_sim::simCom command;
cout<<"Enter Boundary X, Y, Length, Width\n";
cin>>details.bounX;
cin>>details.bounY;
cin>>details.bounL;
cin>>details.bounW;

cout<<"Enter Nest X, Y, Length, Width\n";
cin>>details.goalX;
cin>>details.goalY;
cin>>details.goalL;
cin>>details.goalW;

cout<<"Enter No.of bots\n";
cin>>details.numRobots;

cout<<"Enter No.of objects, distance from Nest\n";
cin>>details.numObjects;
details.numObjects+=details.numRobots;
cin>>details.source;
simDet.publish(details);
ros::spinOnce();
char ch;
cout<<"press enter to start";
cin>>ch;
command.command=1;
simCom.publish(command);
 while (ros::ok())
 {
     ros::spinOnce();
     time+=1;
     if(time>=600)
     {
        command.command=0;
        simCom.publish(command);
        exit(1);
     }
    ros::Duration(1).sleep();
 }
}
