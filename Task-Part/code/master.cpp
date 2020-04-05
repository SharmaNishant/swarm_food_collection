/*********************************************************
 *    master.cpp                                          *
 *    Purpose: Master node for simulation on ROS and Rviz *
 *                                                        *
 *    @author Nishant Sharma                              *
 *    @version 1.1 09/02/14                               *
 *********************************************************/

// Including RosC++ Header Files
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// Including C++ Header Files
#include <iostream>
#include <stdlib.h>

// Including Custom Message Header Files
#include <task_part_sim/simCom.h>
#include <task_part_sim/simDet.h>

using namespace std;

int main(int argc, char **argv) {

  // To Track Time of the Simulation
  long time = 0;

  // Initializing ROS NODE values
  ros::init(argc, argv, "master");
  ros::NodeHandle node;

  // Defination for ROS publisher on custom message
  ros::Publisher simDet =
      node.advertise<task_part_sim::simDet>("simulationDetails", 10);
  ros::Publisher simCom =
      node.advertise<task_part_sim::simCom>("simulationCommand", 10);

  // Defining Simulation Detail and Command msg type variables
  task_part_sim::simDet details;
  task_part_sim::simCom command;

  // User Input of desiered values
  cout << "\nEnter Outer Boundary Length and Width\n";
  details.bounX = 0;
  details.bounY = 0;
  cin >> details.bounL;
  cin >> details.bounW;

  cout << "\nNest Position ->\n"
       << "Enter center X, center Y, Length, Width:\n";
  cin >> details.goalX;
  cin >> details.goalY;
  cin >> details.goalL;
  cin >> details.goalW;

  cout << "\nEnter Total Number of Robots Present:\n";
  cin >> details.numRobots;

  cout << "\nEnter Total Number of Objects and Their Distance From The Nest:\n";
  cin >> details.numObjects;
  cin >> details.source;

  // Publishing Simulation Details
  simDet.publish(details);

  // sending value out
  ros::spinOnce();

  // For Start/Stop Command
  char simulationCommand;
  cout << "\nPress [ Y ] to Start:\n";
  cin >> simulationCommand;

  // Publishing Command
  if (simulationCommand == 'Y' || simulationCommand == 'y') {
    command.command = 1;
    simCom.publish(command);
    ros::spinOnce();
  } else {
    command.command = 0;
    simCom.publish(command);
    ros::spinOnce();
    exit(1);
  }

  while (ros::ok()) {
    ros::spinOnce();

    time += 1;

    // When Time is up
    if (time >= 600) {
      command.command = 0;
      simCom.publish(command);
      cout << "\nTerminating all Nodes\n\n";
      ros::spinOnce();
      exit(1);
    }

    if ((time % 60) == 0)
      cout << (time / 60) << " minutes passed\n\n";

    ros::Duration(1).sleep();
  }
}
