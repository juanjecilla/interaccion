#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "interaccion/inf_personal_usuario.h"

#define NODE_NAME "informacion_personal_nodo"
#define INFO_MSG_NAME "inf_pers_topic"

using namespace std;
using namespace ros;


int main(int argc, char **argv){
	
	init(argc,argv, NODE_NAME);
	NodeHandle node;

	ROS_INFO("Node created and registered");
	Publisher publisher = node.advertise<interaccion::inf_personal_usuario>(INFO_MSG_NAME, 0);

	Duration seconds_sleep(1);

	while (node.ok()){

		string nombre;
		string aux;
		int edad;
		int numIdiomas;
		vector<string> idiomas;

		interaccion::inf_personal_usuario message;

		ROS_INFO("Introduzca el nombre del usuario");
		cin >> nombre;
		ROS_INFO("Introduzca la edad del usuario");
		cin >> edad;
		ROS_INFO("¿Cuántos idiomas hablas?");
		cin >> numIdiomas;

		int i;
		for (i = 0; i<numIdiomas; i++){
			ROS_INFO("Introduce el idioma %d", i+1);
			cin >> aux;
			idiomas.push_back(aux);
		}

		message.nombre = nombre;
		message.edad = edad;
		message.idiomas = idiomas;

		publisher.publish(message);

		spinOnce();
		ROS_DEBUG ("Se duerme el nodo emisor durante un segundo");

		seconds_sleep.sleep();

		
	}	



	return 0;
}