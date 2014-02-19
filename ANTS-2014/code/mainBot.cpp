/*****************************************************
*    mainBot.cpp                                     *
*    Purpose: Robot simulation code for ROS and Rviz *
*                                                    *
*    @author Nishant Sharma                          *
*    @version 1.0 18/02/14                           *
*****************************************************/

//Including RosC++ Header Files
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

//Including C++ Header Files
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>

//including the custom msg files
#include <ants2014/simCom.h>
#include <ants2014/simDet.h>
#include <ants2014/robLocate.h>
#include <ants2014/objLocate.h>
#include <ants2014/robObj.h>
#include <ants2014/logInfo.h>

using namespace std;

//Defining Total States for a Robot
#define neighSearch 0
#define randWalk 1
#define toNest 2
#define toSource 3
#define toObject 4

//unique for each robot
string rosName = "robot";
int robotID=0;

char filename[] = "robot.txt";

//for
fstream file;

float stepSize = 10.0;

int state = randWalk;

double alpha = 0.9, beta = 1, tradeOffValue;

int totalObjectsDeposited = 0,towardsObjectFlag = 0, startSimulation = 0, ostacleFlag = 0;

float deviateX, deviateY, tempX, tempY, lastX, lastY, robotSpeed = 0.75, lifePenalty = 0.02, robotLifeThreshold = 500, robotLifeCost = 0, robotDirection;

float colisionAvoidDirection, distanceToSource, neighbourSearchLimit = 2.0, robotMinimumDistance = 1.0, robotRepulsionForce = 1.0;

int selectedPatch = -1, tradeOffFlag=0;

long secs, startTime, tempTime;

double robotX,robotY;

//double estimatePatch1X = 100 , estimatePatch1Y = 100, estimatePatch2X = 100, estimatePatch2Y = 100;

//double survivalPatch1 = 0.2, survivalPatch2 = 0.8;

ants2014::robObj       robotObject;
ants2014::robLocate    thisRobotLocation;
visualization_msgs::Marker  robotRvizMarker;
std_msgs::ColorRGBA         blue,yellow,red;

//for toSource
float lastSourceX,lastSourceY,sourceDirection,sourceDetailSaveFlag = 0,sourceDistance;

//for randWalk
float randomWalkTime,randWalkCount = 0;

//for neighSearch
float neighbourSearchCenterX, neighbourSearchCenterY, neighbourSearchFlag = 0, neighbourSearchTime = 0;

//for toNest
float selectedLength = 100, totalTime, distanceTravelled = 0, selectedCostID;

//for Cost
//float lastStepTime, lastRandomWalkTime;

//a structure to store the cost values respective of each length
/*struct costFunction{
int id;
float length;
float cost;
}tempCost;*/

struct patchFunction{
int id;
int object;
double estimateX, estimateY;
double survival;
double cost;
int visit;
}tempPatch;

//structure to store the boundary details
struct boundaryDetails{
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
struct nestValues{
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
int value;
int patch;
float dist;
}tempObj, curObj;

//structure to store the other bots information
struct robotList{
int id;
float x;
float y;
float dist;
}tempRob, curRob;

//vector definition to make lists of objects and other robots
vector<robotList> robotLocationList;
//vector<costFunction> costFunction;
vector<patchFunction> patchFunctionList;

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

    // Set the scale of the robotRvizMarker -- 1x1x1 here means 1m on a side
    robotRvizMarker.scale.x = 0.5;
    robotRvizMarker.scale.y = 0.5;
    robotRvizMarker.scale.z = 0.5;
    robotRvizMarker.pose.position.z = 0;
    robotY = 0;
    robotX = 0;
	//set the orientation
    robotRvizMarker.pose.orientation.x = 0.0;
    robotRvizMarker.pose.orientation.y = 0.0;
    robotRvizMarker.pose.orientation.z = 0.0;
    robotRvizMarker.pose.orientation.w = 1.0;

    //setting the robotRvizMarker duration
	robotRvizMarker.lifetime = ros::Duration();

    // Set the robotRvizMarker type
    robotRvizMarker.type = visualization_msgs::Marker::SPHERE;



    //state = toSource;
    //lastSourceX = patchFunctionList[1].estimateX;
    //lastSourceY = patchFunctionList[1].estimateY;
}

//function to start or stop the code based on the message received from the master node
void simulationCommand(const ants2014::simCom::ConstPtr& msg)
{
    if(msg->command == 0)
    {
        exit(1);
    }
    else
    {
        startSimulation = 1;
    }
}

//function to set the simulation details based on the message received from the master node
void simulationDetails(const ants2014::simDet::ConstPtr& msg)
{
    distanceToSource = msg->source;

    //initializing the other robots
    int i;
    for(i = 0;i < msg->numRobots; i++)
    {
        tempRob.id = i;
        tempRob.x = INT_MAX;
        tempRob.y = INT_MAX;
        tempRob.dist = INT_MAX;
        robotLocationList.push_back(tempRob);
    }

    //initializing the cost Function
   /* int totalSteps = (distanceToSource/stepSize);

    for(i = 0;i <= totalSteps; i++)
    {
        tempCost.id = i;
        tempCost.length = (i+1) * stepSize;
        tempCost.cost = 0;
        costFunction.push_back(tempCost);
    }*/

    tempPatch.id = 0;
    tempPatch.object = 1;
    tempPatch.cost = INT_MAX;
    tempPatch.estimateX = -15;
    tempPatch.estimateY = 15;
    tempPatch.survival = 0.8;
    tempPatch.visit = 0;
    patchFunctionList.push_back(tempPatch);

    tempPatch.id = 1;
    tempPatch.object = 10;
    tempPatch.cost = INT_MAX;
    tempPatch.estimateX = 15;
    tempPatch.estimateY = 15;
    tempPatch.survival = 0.2;
    tempPatch.visit = 0;
    patchFunctionList.push_back(tempPatch);

    //setting boundary value
    {
        boundary.x1 = msg->bounX - msg->bounW/2;
        boundary.y1 = msg->bounY - msg->bounL/2;

        boundary.x2 = msg->bounX + msg->bounW/2;
        boundary.y2 = msg->bounY - msg->bounL/2;

        boundary.x3 = msg->bounX - msg->bounW/2;
        boundary.y3 = msg->bounY + msg->bounL/2;

        boundary.dv13 = ((boundary.x3 - boundary.x1) * (boundary.x3 - boundary.x1)) + ((boundary.y3 - boundary.y1) * (boundary.y3 - boundary.y1));
        boundary.dv12 = ((boundary.x2 - boundary.x1) * (boundary.x2 - boundary.x1)) + ((boundary.y2 - boundary.y1) * (boundary.y2 - boundary.y1));

      //  cout<<boundary.x1<<" "<<boundary.y1<<" "<<boundary.x2<<" "<<boundary.y2<<" "<<boundary.x3<<" "<<boundary.y3<<" \n";
    }

    //setting goal value
    {
        robotX = nest.x = msg->goalX;
        robotY = nest.y = msg->goalY;

        nest.x2 = msg->goalX + (msg->goalW/2);
        nest.y2 = msg->goalY - (msg->goalL/2);

        nest.x3 = msg->goalX - (msg->goalW/2);
        nest.y3 = msg->goalY + (msg->goalL/2);

        nest.x1 = msg->goalX + (msg->goalW/2);
        nest.y1 = msg->goalY + (msg->goalL/2);

        nest.dv13 = (( -nest.x1 + nest.x3) * ( -nest.x1 + nest.x3)) + (( -nest.y1 + nest.y3) * ( -nest.y1 + nest.y3));
        nest.dv12 = (( -nest.x1 + nest.x2) * ( -nest.x1 + nest.x2)) + (( -nest.y1 + nest.y2) * ( -nest.y1 + nest.y2));
       // cout<<nest.x<<" "<<nest.y<<nest.x1<<" "<<nest.y1<<nest.x2<<" "<<nest.y2<<nest.x3<<" "<<nest.y3<<"\n";
    }
}

//function updating other robot locations based on the message received from other robots
void robotLocator(const ants2014::robLocate::ConstPtr& msg)
{
    robotLocationList[msg->id].x = msg->X;
    robotLocationList[msg->id].y = msg->Y;
}

//function updating other object information based on the message received from object node
void objectLocator(const ants2014::objLocate::ConstPtr& msg)
{
    if(curObj.id == -1)
    {
        tempObj.id = msg->id;
        tempObj.x = msg->X;
        tempObj.y = msg->Y;
        tempObj.value = msg->value;
        tempObj.patch = msg->patch;
        tempObj.dist = sqrt(pow(robotX - tempObj.x,2) + pow(robotY - tempObj.y,2));
        if(tempObj.dist < 2.0)
        {
            curObj = tempObj;
            //cout<<"\nObject located with distance = "<<curObj.dist<<"\n";
            //state = toObject;
        }
    }
//    else
//    {
//        tempObj.id = msg->id;
//        tempObj.x = msg->X;
//        tempObj.y = msg->Y;
//        tempObj.dist = sqrt(pow(robotX - tempObj.x,2) + pow(robotY - tempObj.y,2));
//        if(tempObj.dist < curObj.dist)
//            curObj = tempObj;
//    }
}

//main function
int main( int argc, char** argv )
{

    //for randwalk in the beginning
    curObj.id = -1;
    //log file
    file.open (filename, std::fstream::out);// | std::fstream::app);

    //for random values
    srand(time(0));

    //initalizing node
    node_init();
    ros::init(argc, argv, rosName);
    ros::NodeHandle n;

    startTime = ros::Time::now().toSec();

    //setting the publisher for the nodes
    ros::Publisher rviz = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher myLocate = n.advertise<ants2014::robLocate>("robotLocation", 10);
    ros::Publisher robObject = n.advertise<ants2014::robObj>("robObject", 10);

    robotRvizMarker.header.frame_id = "/simulation";
    robotRvizMarker.header.stamp = ros::Time::now();
    robotRvizMarker.ns = "robots";
    thisRobotLocation.id = robotRvizMarker.id = robotID;

    //setting the subscriber for the nodes
    ros::Subscriber simDet = n.subscribe("simulationDetails", 500, simulationDetails);
    ros::Subscriber simCom = n.subscribe("simulationCommand", 1000, simulationCommand);
    ros::Subscriber robotLocate = n.subscribe("robotLocation", 1000, robotLocator);
    ros::Subscriber objectLocate = n.subscribe("objectLocation", 1000, objectLocator);

    int i;

    //waiting for master node's signal to start
    while(startSimulation == 0) ros::spinOnce();

    int startPatch = rand() % 2;
    state = toSource;
    lastSourceX = patchFunctionList[startPatch].estimateX;
    lastSourceY = patchFunctionList[startPatch].estimateY;
    tempX = robotX;  tempY = robotY;
    sourceDetailSaveFlag=0;

    while (ros::ok())
    {
        ros::spinOnce();

        // Set the robotRvizMarker action.  Options are ADD and DELETE
        robotRvizMarker.action = visualization_msgs::Marker::ADD;

        ostacleFlag = 0;
        //curRob.id=-1;

        //distance from other robots
        for(i = 0;i < robotLocationList.size(); i++)
        {
            if(i == robotID)continue;
            robotLocationList[i].dist = sqrt(pow(robotX - robotLocationList[i].x,2) + pow(robotY - robotLocationList[i].y,2));
        }

        //position update for the random walk state
        if(state == randWalk)
        {
            float randAngle = (rand() % 300);    //500
            float angleChange = (randAngle/100) - 1.0;   //2.0
            int randValCheck = rand() % 2;

            //cout<<"Random Walk\n\n";

            //50% probability of direction change
            if(randValCheck == 1)   robotDirection += angleChange;

            robotRvizMarker.color = red;

            if(curObj.id != -1)
            {
                towardsObjectFlag = 0;
                state = toObject;
                cout<<"State Changed : towards Object\n\n";
                continue;
            }
        }

        //position update when the robot is moving towards the object
        if(state == toObject)
        {
            //if object not found in 15 iterations
            if(towardsObjectFlag > 15)
            {
                state = randWalk;
                curObj.id = -1;
                cout<<"State Changed : Rand Walk\n\n";
                continue;
            }
            towardsObjectFlag++;

           // cout<<"Going towards the Object\n\n";

            //some object is in sight
            if(curObj.id != -1)
            {
                robotDirection = atan2((curObj.y-robotY),(curObj.x-robotX));
                curObj.dist = sqrt(pow(robotX - curObj.x,2) + pow(robotY - curObj.y,2));
               // cout<<"\nObject located with distance = "<<curObj.dist<<"\n";
                //if object distance is less than this value
                if(curObj.dist <= 2.0)  //objMinDist
                {
                    //updating cost function
                  /*  costFunction[selectedCostID].cost = (0.75 * costFunction[selectedCostID].cost) + (0.25 * ((distanceToSource/costFunction[selectedCostID].length) * (totalTime) + randomWalkTime));
                    lastStepTime = totalTime;
                    lastRandomWalkTime = randomWalkTime;
                    randomWalkTime = 0;
                    totalTime = 0; */
                    float cost = INT_MAX;

                    tempX = lastSourceX = curObj.x;
                    tempY = lastSourceY = curObj.y;

                    //sending object removing commands
                    {
                    robotObject.id = curObj.id;
                    robotObject.X = curObj.x;
                    robotObject.Y = curObj.y;
                    robotObject.flag = -1;
                    robObject.publish(robotObject);
                    }

                    state = toNest;
                    cout<<"State Changed : towards Nest\n\n";

                    double life = rand()%100;
                    life = life / 100;

                    cout<<"\n\nLIFE value is : "<<life<<"\n\n";

                    if(life > patchFunctionList[curObj.patch].survival)
                    {
                        robotSpeed -= lifePenalty*robotSpeed;
                        cout<<"bot speed is : "<<robotSpeed<<"\n\n";
                        if(robotSpeed <= 0)
                        {
                            cout<<"\n\n\nBOT GOT KILLED!!!! :( :'(\n\n";
                            file<<"\n\n\nBOT GOT KILLED!!!! :( :'(\n\n";
                            exit(1);
                        }
                    }

/*
                    //selecting new cost
                    cost = INT_MAX;
                    for(i = 0;i < costFunction.size(); i++)
                    {
                        if(cost > costFunction[i].cost)
                        {
                            selectedLength = costFunction[i].length;
                            selectedCostID = i;
                            cost = costFunction[i].cost;
                        }
                    }

                    //one time
                    if(startSimulation == 1)
                    {
                        startSimulation = 0;
                        selectedCostID = costFunction.size()-1;
                        selectedLength = 0;
                    }
                    */
                    //cout<<"Selected Cost Length is " << selectedLength<<" \n";

                    distanceTravelled = 0;

                    ros::Duration(0.5).sleep();
                    totalTime += 0.5;

                    continue;
                }
            }
            else
            {
                state = randWalk;
                cout<<"State Changed : Rand Walk\n\n";
                continue;
            }
            robotRvizMarker.color = blue;
        }

        //position update when the robot is moving towards the nest
        if(state == toNest)
        {

            //cout<<"Going towards the Nest\n\n";

            robotDirection = atan2((nest.y - robotY),(nest.x - robotX));

            //if selected cost length is travelled
           /* if(distanceTravelled >= selectedLength)
            {
                //sending msg to add object
                robotObject.id = robotID;
                tempX = robotObject.X = robotX;
                tempY = robotObject.Y = robotY;
                robotObject.flag = 1;
                robObject.publish(robotObject);

                //updating state
                state = toSource;
                curObj.id = -1;
                cout<<"State Changed : towards Source\n\n";

                distanceTravelled = 0;
                sourceDetailSaveFlag = 0;
                continue;
            }*/

            //checking if robot is in the nest or not
            //(0<AM⋅AB<AB⋅AB)∧(0<AM⋅AD<AD⋅AD)
            nest.dv1m2 = (( -nest.x1 + robotX) * ( -nest.x1 + nest.x2)) + (( -nest.y1 + robotY) * ( -nest.y1 + nest.y2));
            nest.dv1m3 = (( -nest.x1 + robotX) * ( -nest.x1 + nest.x3)) + (( -nest.y1 + robotY) * ( -nest.y1 + nest.y3));

            //test if the robot has entered the nest
            if((0 < nest.dv1m2) && (nest.dv1m2 < nest.dv12) && (0 < nest.dv1m3) && (nest.dv1m3 < nest.dv13))
            {
                //saving depositing point value
                tempX = robotX;
                tempY = robotY;
               /* if(curObj.patch == 0)
                    patchFunctionList[0].cost = (0.9 * patchFunctionList[0].cost) + (0.1 * (( (totalTime) / (curObj.value * patchFunctionList[0].survival)) + randomWalkTime));
                else
                    patchFunctionList[1].cost = (0.9 * patchFunctionList[1].cost) + (0.1 * (( (totalTime) / (curObj.value * patchFunctionList[1].survival)) + randomWalkTime));

                if(patchFunctionList[0].object == 0 && curObj.patch == 0)
                {
                    patchFunctionList[0].object == curObj.value;
                }
                if(patchFunctionList[1].object == 0 && curObj.patch == 1)
                {
                    patchFunctionList[1].object == curObj.value;
                }*/

                patchFunctionList[curObj.patch].visit++;
                //cout<<"\nPatch 1 Cost : "<<patchFunctionList[0].cost<<", Patch 2 Cost : "<<patchFunctionList[1].cost<<"\n";

                robotLifeCost = ( (1 - alpha) * robotLifeCost) + (alpha * (( (totalTime) / (curObj.value)) + (beta * randomWalkTime)));

                cout<<"\nLife Cost = " << robotLifeCost <<"\n\n";

                totalObjectsDeposited += curObj.value;

                tradeOffValue = (robotLifeCost / totalObjectsDeposited) * (ros::Time::now().toSec() - startTime);

                cout<<"\nTrade off Value = " << tradeOffValue <<"\n\n";


                patchFunctionList[curObj.patch].cost = (100 * curObj.value * patchFunctionList[curObj.patch].survival) / ( sqrt(pow(curObj.x - robotX,2) + pow(curObj.y - robotY,2)));
                patchFunctionList[curObj.patch].estimateX = curObj.x;
                patchFunctionList[curObj.patch].estimateY = curObj.y;

                cout<<"net Cost p1 "<< patchFunctionList[0].cost<<" | net Cost p2 "<<patchFunctionList[1].cost<<"\n\n";

                if(tradeOffValue <= robotLifeThreshold)
                {
                    tradeOffFlag = 0;
                    cout<<"Towards safe place\n";
                    if(patchFunctionList[0].survival > patchFunctionList[1].survival)
                    {
                        state = toSource;
                        lastSourceX = patchFunctionList[0].estimateX;
                        lastSourceY = patchFunctionList[0].estimateY;
                    }
                    else
                    {
                        state = toSource;
                        lastSourceX = patchFunctionList[1].estimateX;
                        lastSourceY = patchFunctionList[1].estimateY;
                    }
                }
                else
                {
                    tradeOffFlag = 1;
                    cout<<"Towards more Food value\n";
                    if(patchFunctionList[0].cost > patchFunctionList[1].cost)
                    {
                        state = toSource;
                        lastSourceX = patchFunctionList[0].estimateX;
                        lastSourceY = patchFunctionList[0].estimateY;
                    }
                    else
                    {
                        state = toSource;
                        lastSourceX = patchFunctionList[1].estimateX;
                        lastSourceY = patchFunctionList[1].estimateY;
                    }
                }


                file<<"Cost : "<< robotLifeCost<<" | lastStep Ts&Tp : "<< totalTime <<" | randWalkTime : "<<randomWalkTime<<" | foodValue : "<<curObj.value<<" | OverallTime : "<<((ros::Time::now().toSec() - startTime))<<" | totalObjectsDeposited : "<<totalObjectsDeposited<<" | netFoodValue 1 : "<<patchFunctionList[0].cost<<" | netFoodValue 2 : "<<patchFunctionList[1].cost<<" | tradeOffValue :"<<tradeOffValue<<" | tradeOFF : "<<tradeOffFlag<<" | lastSelectedPatch : "<<curObj.patch<<" | robotSpeed : "<<robotSpeed<<" |\n";

                totalTime = 0;
                randomWalkTime = 0;

                /*{
                    if(patchFunctionList[0].cost == 0 || patchFunctionList[1].cost == 0)
                    {
                        state = toSource;
                    }
                    else if((patchFunctionList[0].cost / ((patchFunctionList[0].object * patchFunctionList[0].survival))) > (patchFunctionList[1].cost/((patchFunctionList[1].object * patchFunctionList[1].survival))))
                    {
                        state = toSource;
                        lastSourceX = patchFunctionList[1].estimateX;
                        lastSourceY = patchFunctionList[1].estimateY;
                    }
                    else if((patchFunctionList[0].cost / ((patchFunctionList[0].object * patchFunctionList[0].survival))) < (patchFunctionList[1].cost/((patchFunctionList[1].object * patchFunctionList[1].survival))))
                    {
                        state = toSource;
                        lastSourceX = patchFunctionList[0].estimateX;
                        lastSourceY = patchFunctionList[0].estimateY;
                    }
                    else
                    {
                        state = toSource;
                       // cout<<"State Changed : towards Source\n\n";
                    }

                }*/


                //updatng state
               // state = toSource;
                curObj.id = -1;
                cout<<"State Changed : towards Source\n\n";

                distanceTravelled = 0;
                sourceDetailSaveFlag = 0;
               // file<<"Object Deposited\n";
            }

            robotRvizMarker.color = blue;

            //updating travelled Distance to then compute with selected length
            distanceTravelled = sqrt(pow(lastSourceX - robotX,2) + pow(lastSourceY - robotY,2));
        }

        //position update when the robot is moving back towards the source
        if(state == toSource)
        {
            //cout<<"Going back towards the last Source\n\n";

            //setting estimated distance and direction
            if(sourceDetailSaveFlag == 0)
            {
                sourceDetailSaveFlag = 1;
                sourceDirection = atan2((lastSourceY - robotY),(lastSourceX - robotX));
                sourceDistance = sqrt(pow(lastSourceX - robotX,2) + pow(lastSourceY - robotY,2));
                //cout<<sourceDistance<<"is the source distance";
                ros::Duration(0.5).sleep();
            }
            sourceDirection += 0.0001;

            //cout<<"dis travel"<<distanceTravelled<<"\n";

            //after reaching the estimated destination
            if(distanceTravelled > sourceDistance)
            {
                neighbourSearchFlag = 0;
                distanceTravelled = 0;
                state = neighSearch;
                cout<<"State Changed : Neighbour Search\n\n";
                continue;
            }

            robotRvizMarker.color = yellow;

            distanceTravelled = sqrt(pow(tempX - robotX,2) + pow(tempY - robotY,2));
        }

        //position update while Searching the neighbourhood of the predict Source Location
        if(state == neighSearch)
        {
            //cout<<"Searching the neighbourhood of the predicted Source Location\n";

            //setting center for neighboursearch
            if(neighbourSearchFlag == 0)
            {
                curObj.id = -1;
                neighbourSearchCenterX = robotX;
                neighbourSearchCenterY = robotY;
                neighbourSearchFlag = 1;
                neighbourSearchTime = 0;
            }
            float randAngle = (rand() % 500);
            float angleChange = (randAngle/100) - 2.0;
            //cout<<angleChange<<" ns \n";

            //if object not found
            if(neighbourSearchTime > 15)  //neighbourSearchTime max value
            {
                neighbourSearchFlag = 0;
                state = randWalk;
                cout<<"State Changed : Rand Walk\n\n";
                continue;
            }

            //when object found
            if(curObj.id != -1)
            {
                neighbourSearchFlag = 0;
                towardsObjectFlag = 0;
                state = toObject;
                cout<<"State Changed : towards Object\n\n";
                continue;
            }

            neighbourSearchTime++;
            robotDirection += angleChange;
            robotRvizMarker.color = yellow;
        }

    //state machine ends here

        ostacleFlag = 0;
        colisionAvoidDirection = 0;
        deviateX = deviateY = 0;
        lastX = robotX;
        lastY = robotY;

        //now actual step change as calculated in the state above
        {
            //checking all robots in the range
            for(i = 0;i < robotLocationList.size(); i++)
            {
                if(robotLocationList[i].dist < robotMinimumDistance)
                {
                        ostacleFlag = 1;
                        colisionAvoidDirection = atan2((robotY - robotLocationList[i].y),(robotX - robotLocationList[i].x));
                        deviateX += ((robotRepulsionForce - robotLocationList[i].dist) * cos(colisionAvoidDirection));
                        deviateY += ((robotRepulsionForce - robotLocationList[i].dist) * sin(colisionAvoidDirection));
                        ros::Duration(0.2).sleep();
                }
            }

            if(state == randWalk)
            {
                thisRobotLocation.X = robotX =(robotX + (robotSpeed * cos(robotDirection)) + deviateX);
                thisRobotLocation.Y = robotY =(robotY + (robotSpeed * sin(robotDirection)) + deviateY);
                randomWalkTime += 0.2;
            }

            if(state == toSource)
            {
                thisRobotLocation.X = robotX = (robotX + (robotSpeed * cos(sourceDirection)) + deviateX);
                thisRobotLocation.Y = robotY = (robotY + (robotSpeed * sin(sourceDirection)) + deviateY);
                totalTime += 0.2;
            }

            if(state == toObject)
            {
                thisRobotLocation.X = robotX = (robotX + (robotSpeed * cos(robotDirection)) + deviateX);
                thisRobotLocation.Y = robotY = (robotY + (robotSpeed * sin(robotDirection)) + deviateY);
                totalTime += 0.2;
            }

            if(state == neighSearch)
            {
                thisRobotLocation.X = robotX = (robotX + (robotSpeed * cos(robotDirection)) + deviateX);
                thisRobotLocation.Y = robotY = (robotY + (robotSpeed * sin(robotDirection)) + deviateY);
                //if going out of the neighbourhood
                if((sqrt(pow(neighbourSearchCenterX - robotX,2) + pow(neighbourSearchCenterY - robotY,2))) > neighbourSearchLimit )
                {
                    robotX = lastX;
                    robotY = lastY;
                    continue;
                }
                totalTime+=0.2;
            }

            if(state == toNest)
            {
                thisRobotLocation.X = robotX = (robotX + (robotSpeed * cos(robotDirection)) + deviateX);
                thisRobotLocation.Y = robotY = (robotY + (robotSpeed * sin(robotDirection)) + deviateY);
                totalTime += 0.2;
            }

            //checking if the robot is in the boundary or not
            boundary.dv1m3 = ((robotX - boundary.x1) * (boundary.x3 - boundary.x1)) + ((robotY - boundary.y1) * (boundary.y3 - boundary.y1));
            boundary.dv1m2 = ((robotX - boundary.x1) * (boundary.x2 - boundary.x1)) + ((robotY - boundary.y1) * (boundary.y2 - boundary.y1));

           // cout<<"boudn values check\n"<<boundary.dv12<<"  "<<boundary.dv1m2<<"  "<<boundary.dv13<<"  "<<boundary.dv1m3<<"\n";
            if(!((0 < boundary.dv1m2) && (boundary.dv1m2 < boundary.dv12)) || !((0 < boundary.dv1m3) && (boundary.dv1m3 < boundary.dv13)))
            {
                robotDirection = atan2((nest.y - lastY),(nest.x - lastX));
                robotX = lastX + (robotSpeed * cos(robotDirection));
                robotY = lastY + (robotSpeed * sin(robotDirection));
                ros::Duration(0.5).sleep();
                randomWalkTime += 0.5;
                cout<<"Out of boundary\n";
                //continue;
            }
        }

        //cout<<thisRobotLocation.X<<"  " << thisRobotLocation.Y<<"\n"<<"\n";
        // Publish the robotRvizMarker

        robotRvizMarker.pose.position.x=robotX;
        robotRvizMarker.pose.position.y=robotY;
        rviz.publish(robotRvizMarker);

        myLocate.publish(thisRobotLocation);

        ros::Duration(0.2).sleep();
        //logging record
        secs = ros::Time::now().toSec();
       // file<<"Time :"<<secs<<" | POS x : "<<thisRobotLocation.X<<" | POS y : "<<thisRobotLocation.Y<<" | State : "<<state<<" | Cost 1 : "<<patchFunctionList[0].cost<<" | Cost 2 : "<<patchFunctionList[1].cost<<" | Patch 1 Visit : " << patchFunctionList[0].visit <<" | Patch 2 Visit : "<< patchFunctionList[1].visit <<" | totalFood : "<< totalObjectsDeposited<<"|\n";;
        //file<<"|" << patchFunctionList[0].visit <<"|"<< patchFunctionList[1].visit <<"|\n";
        file.flush();
        }
}

/*The End*/
