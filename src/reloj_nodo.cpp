#include "ros/ros.h"
#include <string>
#include <ctime>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "boost/date_time/posix_time/posix_time.hpp"

#define NODE_NAME "reloj_nodo"
#define RELOJ_MSG_NAME "still_alive"
#define RESET_TOPIC_NAME "reset_topic"
#define START_TOPIC_NAME "start_topic"
#define COUNTDOWN_TIME 

using namespace std;
using namespace ros;
using namespace boost::posix_time;

Time startTime;
bool clock_start = false;
int totalSeconds = 0;

Publisher publisher;

void timerCallback(const ros::TimerEvent&){
	std_msgs::Bool still_alive;
	still_alive.data = true;
	publisher.publish(still_alive);
}

void printClock(){

	totalSeconds = (Time::now() - startTime).toSec();

	ptime t_local(second_clock::local_time());
	ptime t_utc(second_clock::universal_time());

	ROS_INFO("LOCAL HOUR: %s", to_simple_string(t_local).c_str());
	ROS_INFO("UTC HOUR: %s", to_simple_string(t_utc).c_str());

	ROS_INFO("SECONDS FROM START/RESET: %lf", (double)(Time::now() - startTime).toSec());

}

void funcionCallbackReset(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("RESET MESSAGE: %s", msg->data.c_str());
	startTime = Time::now();
	clock_start = true;
}

void funcionCallbackStart(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("START MESSAGE: %s", msg->data.c_str());
	startTime = Time::now();
	clock_start = true;

}


int main(int argc, char **argv){
	
	init(argc,argv, NODE_NAME);
	NodeHandle node;


	ROS_INFO("Node created and registered");

	Subscriber subscriptor1 = node.subscribe(START_TOPIC_NAME, 0, funcionCallbackStart);
	Subscriber subscriptor2 = node.subscribe(RESET_TOPIC_NAME, 0, funcionCallbackReset);
	Timer timer = node.createTimer(Duration(60), timerCallback);

	
	publisher = node.advertise<std_msgs::Bool>(RELOJ_MSG_NAME, 0);

	//Duration seconds_sleep(0.3);
	Rate rate(3);

	while (node.ok()){

		if (clock_start){
			printClock();
		}

		spinOnce();
		ROS_DEBUG ("Se duerme el nodo emisor durante un segundo");

		rate.sleep();

		
	}	

	return 0;
}		
