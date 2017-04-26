#include "ros/ros.h"
#include "interaccion/multiplicador.h"

#define NODE_NAME "matematico_nodo"
#define INFO_MSG_NAME "user_topic"
#define SRV_NAME "multiplication_service"

using namespace std;
using namespace ros;

bool multiplicationService(interaccion::multiplicador::Request &req, interaccion::multiplicador::Response &res){

	res.resultado = req.entrada * 2;

	ROS_INFO("Petición: x = %d", (int)req.entrada);

	ROS_INFO("Respuesta: %d", (int)res.resultado);

	return true;

}

int main(int argc, char **argv){

	init(argc, argv, NODE_NAME);

	NodeHandle node;

	ServiceServer service = node.advertiseService(SRV_NAME,multiplicationService);

	ROS_INFO("Servicio matemático registrado.");	

	spin();

	return 0;

}