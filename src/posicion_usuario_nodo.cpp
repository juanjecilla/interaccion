#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "interaccion/pos_usuario.h"

#define NODE_NAME "posicion_usuario_nodo"
#define INFO_MSG_NAME "pos_usuario_topic"

using namespace std;
using namespace ros;


int main(int argc, char **argv){
	
	init(argc,argv, NODE_NAME);
	NodeHandle node;

	ROS_INFO("Node created and registered");
	Publisher publisher = node.advertise<interaccion::pos_usuario>(INFO_MSG_NAME, 0);

	Duration seconds_sleep(1);

	while (node.ok()){

		int x; 
		int y;
		int z;

		interaccion::pos_usuario message;

		ROS_INFO("Introduzca la coordenada X del usuario:");
		cin >> x;

		ROS_INFO("Introduzca la coordenada Y del usuario:");
		cin >> y;

		ROS_INFO("Introduzca la coordenada Z del usuario:");
		cin >> z;

		message.x = x;
		message.y = y;
		message.z = z;

		publisher.publish(message);

		spinOnce();
		ROS_DEBUG ("Se duerme el nodo emisor durante un segundo");

		seconds_sleep.sleep();

	}	

	return 0;
}