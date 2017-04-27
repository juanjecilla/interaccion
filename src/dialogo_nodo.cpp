#include "ros/ros.h"
#include <string>
#include "interaccion/usuario.h"
#include "interaccion/multiplicador.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <cstdlib>

#define NODE_NAME "dialogo_nodo"
#define INFO_TOPIC_NAME "user_topic"
#define RELOJ_TOPIC_NAME "still_alive"
#define RESET_MSG_NAME "reset_topic"
#define START_MSG_NAME "start_topic"
#define SRV_NAME "multiplication_service"

using namespace std;
using namespace ros;

ServiceClient client;
interaccion::multiplicador srv;

bool empaquetador_ready = false;
bool matematico_ready = false;
bool first_time = true;


void funcionCallbackShowMessage(const interaccion::usuario::ConstPtr& msg){

	ROS_INFO("Información personal:");
	ROS_INFO("\tNombre: %s", msg->infPersonal.nombre.c_str());
	ROS_INFO("\tEdad: %d", msg->infPersonal.edad);
	ROS_INFO("\tIdiomas:");

	for (int i = 0; i < msg->infPersonal.idiomas.size(); ++i){
		ROS_INFO("\t\t%s", msg->infPersonal.idiomas[i].c_str());
	}

	ROS_INFO("Emocion: %s", msg->emocion.c_str());

	ROS_INFO("Posición usuario:");
	ROS_INFO("\tCoordenada X: %d", msg->posicion.x);
	ROS_INFO("\tCoordenada Y: %d", msg->posicion.y);
	ROS_INFO("\tCoordenada Z: %d", msg->posicion.z);

	stringstream textToSpeech;
	string text;

	textToSpeech << "Hola " 
				 << msg->infPersonal.nombre.c_str() 
				 << ", tienes " 
				 << (int)msg->infPersonal.edad 
				 << " años y hablas ";

	for (int i = 0; i < msg->infPersonal.idiomas.size(); ++i){
		textToSpeech << msg->infPersonal.idiomas[i].c_str() << " ";
	}
				 
	textToSpeech << ". Estás muy " 
				 << msg->emocion.c_str()
				 << " en tus coordenasdas X:"
				 << msg->posicion.x
				 << ", Y:"
				 << msg->posicion.y
			     << ", Z:"
				 << msg->posicion.z
				 << ". Buenos días majete.";

	text = textToSpeech.str();
	ROS_INFO("TEXTO: %s", text.c_str());
	string command = "espeak -v es \"" + text + "\"";
	printf("COMMAND: %s", command.c_str());
	system (command.c_str()); 

	srv.request.entrada = msg->infPersonal.edad;
	empaquetador_ready = true;

	if (client.call(srv)){
		ROS_INFO("Respuesta del servicio: %d", (int)srv.response.resultado);
		matematico_ready = true;

	} else {
		ROS_ERROR("Fallo al llamar al servicio: %s", SRV_NAME);
		matematico_ready = false;
		return;
	}
}

void functionCallbackReloj(const std_msgs::Bool::ConstPtr& msg){

	ROS_INFO("CLOCK TIMER STILL ALIVE: %s", msg->data ? "True" : "False");

}

int main(int argc, char **argv){
	
	init(argc,argv, NODE_NAME);
	NodeHandle node;

	ROS_INFO("Node created and registered");

	Subscriber subscriptor1 = node.subscribe(INFO_TOPIC_NAME, 0, funcionCallbackShowMessage);
	Subscriber subscriptor2 = node.subscribe(RELOJ_TOPIC_NAME, 0, functionCallbackReloj);
	
	Publisher publisher1 = node.advertise<std_msgs::String>(START_MSG_NAME, 0);
	Publisher publisher2 = node.advertise<std_msgs::String>(RESET_MSG_NAME, 0);

	client = node.serviceClient<interaccion::multiplicador>(SRV_NAME);

	Duration seconds_sleep(1);

	while (node.ok()){

		std_msgs::String message;

		if (matematico_ready && empaquetador_ready){
			matematico_ready = false;
			empaquetador_ready = false;

			if (first_time){
				first_time = false;
				message.data = "start_timer";
				publisher1.publish(message);
			} else {
				message.data = "reset_timer";
				publisher2.publish(message);
			}

		}

		spinOnce();
		seconds_sleep.sleep();

	}	
	
	return 0;
}		
