#include "ros/ros.h"
#include <string>
#include <ctime>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#define NODE_NAME "reloj_nodo"
#define RELOJ_MSG_NAME "still_alive"
#define RESET_TOPIC_NAME "reset_topic"
#define START_TOPIC_NAME "start_topic"
#define COUNTDOWN_TIME 

using namespace std;
using namespace ros;

clock_t startTime;
bool clock_start = false;

void printClock(){
	ROS_INFO("SECONDS FROM START/RESET: %lf", (double)(clock() - startTime)/(double)CLOCKS_PER_SEC);

}

void funcionCallbackReset(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("RESET MESSAGE: %s", msg->data.c_str());
	startTime = clock();
	clock_start = true;
}

void funcionCallbackStart(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("START MESSAGE: %s", msg->data.c_str());
	startTime = clock();
	clock_start = true;

}

int main(int argc, char **argv){
	
	init(argc,argv, NODE_NAME);
	NodeHandle node;

	ROS_INFO("Node created and registered");

	Subscriber subscriptor1 = node.subscribe(START_TOPIC_NAME, 0, funcionCallbackStart);
	Subscriber subscriptor2 = node.subscribe(RESET_TOPIC_NAME, 0, funcionCallbackReset);
	
	Publisher publisher = node.advertise<std_msgs::Bool>(RELOJ_MSG_NAME, 0);

	Duration seconds_sleep(0.3);

	while (node.ok()){

		if (clock_start){
			printClock();
		}

		spinOnce();
		ROS_DEBUG ("Se duerme el nodo emisor durante un segundo");

		seconds_sleep.sleep();

		
	}	

	return 0;
}		
