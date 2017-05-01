#include "ros/ros.h"
#include "std_msgs/String.h"

#define NODE_NAME "emoción_usuario_nodo"
#define INFO_MSG_NAME "emocion_topic"

using namespace std;
using namespace ros;


int main(int argc, char **argv){
	
	init(argc,argv, NODE_NAME);
	NodeHandle node;

	ROS_INFO("Node created and registered");
	Publisher publisher = node.advertise<std_msgs::String>(INFO_MSG_NAME, 0);


	Duration seconds_sleep(1);

	while (node.ok()){

		std_msgs::String emocion;


		ROS_INFO("Introduzca la emocion del usuario:"):
		cin >> emocion.data;

		publisher.publish(emocion);

		spinOnce();
		ROS_DEBUG ("Se duerme el nodo emisor durante un segundo");

		seconds_sleep.sleep();

		
	}	

	return 0;
}		
