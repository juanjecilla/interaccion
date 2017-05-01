/**
 ** empaquetador_nodo es un nodo del paquete interaccion que reune la informacion introducida en los
 ** nodos anteriores y se encarga de mandarla a dialogo_nodo.
**/

// Librerias y paquetes usados por el nodo
#include "ros/ros.h"
#include <string>
#include "interaccion/inf_personal_usuario.h"
#include "interaccion/pos_usuario.h"
#include "interaccion/usuario.h"
#include "std_msgs/String.h"

// Macros auxiliares para definir los nombres del nodo y los mensajes y topics utilizados
#define NODE_NAME "empaquetador_nodo"
#define INFO_MSG_NAME "user_topic"
#define INFO_TOPIC_NAME_1 "emocion_topic"
#define INFO_TOPIC_NAME_2 "inf_pers_topic"
#define INFO_TOPIC_NAME_3 "pos_usuario_topic"

// Declaracion de los espacios de nombres para facilitar la programacion
using namespace std;
using namespace ros;

// Mensaje de tipo usuario donde se reune la informacion recibida.
interaccion::usuario user;

// Flags auxiliares para saber la informacion que se ha recibido 
bool emotion_ready = false;
bool position_ready = false;
bool information_ready = false;

/**
 ** funcionCallbackEmotionTopic es la funcion que se ejecuta cuando se recibe el mensaje procedente
 ** de emocion_nodo con la informacion proporcionada por el usuario
 ** @params: msg mensaje de tipo String con la emocion proporcionada
 ** @return: void
**/
void funcionCallbackEmotionTopic(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("Emocion: %s", msg->data.c_str());
	user.emocion = msg->data.c_str(); // Actualizamos el contenido del mensaje
	emotion_ready = true; // Actualizamos el flag
}

/**
 ** funcionCallbackInfPersonalTopic es la funcion que se ejecuta cuando se recibe el mensaje procedente
 ** de informacion_personal_nodo con la informacion proporcionada por el usuario
 ** @params: msg mensaje de tipo inf_personal_usuario con la informacion proporcionada
 ** @return: void
**/
void funcionCallbackInfPersonalTopic(const interaccion::inf_personal_usuario::ConstPtr& msg){

	ROS_INFO("Nombre: %s", msg->nombre.c_str());
	ROS_INFO("Edad: %d", msg->edad);
	for (int i = 0; i < msg->idiomas.size(); ++i){
		ROS_INFO("Idiomas: %s", msg->idiomas[i].c_str());
	}

	user.infPersonal = *msg; // Actualizamos el contenido del mensaje
	information_ready = true; // Actualizamos el flag
}

/**
 ** main es la funcion que se ejecuta cada vez que se inicia el nodo de ros.
 ** @params: argc numero de argumentos proporcionados
 ** @params: argv puntero con el valor de los argumentos proporcionados
 ** @return: int valor con el resultado de la ejecucion
**/
void funcionCallbackPositionTopic(const interaccion::pos_usuario::ConstPtr& msg){

	ROS_INFO("Coordenada X: %d", msg->x);
	ROS_INFO("Coordenada Y: %d", msg->y);
	ROS_INFO("Coordenada Y: %d", msg->z);

	user.posicion = *msg; // Actualizamos el contenido del mensaje
	position_ready = true; // Actualizamos el flag

}

/**
 ** fuctionCallbackShowMessage es la funcion que se ejecuta cuando se recibe el mensaje procedente
 ** de empaquetador_nodo con la informacion proporcionada por el usuario
 ** @params: msg mensaje de tipo usuario con la informacion proporcionada
 ** @return: void
**/
int main(int argc, char **argv){
	
	// Inicializacion del nodo Ros con los argumentos y el nombre proporcionados
	init(argc,argv, NODE_NAME);
	NodeHandle node; // Manejador del nodo ros

	ROS_INFO("Node created and registered");

	// Suscriptores a los posibles mensajes de entrada procedentes de emocion_usuario_nodo
	// posicion_usuario_nodo e informacion_personal_nodo

	Subscriber subscriptor1 = node.subscribe(INFO_TOPIC_NAME_1, 0, funcionCallbackEmotionTopic);
	Subscriber subscriptor2 = node.subscribe(INFO_TOPIC_NAME_2, 0, funcionCallbackInfPersonalTopic);
	Subscriber subscriptor3 = node.subscribe(INFO_TOPIC_NAME_3, 0, funcionCallbackPositionTopic);
	
	// Publicador de mensajes para mandar la informacion a dialogo_nodo
	Publisher publisher = node.advertise<interaccion::usuario>(INFO_MSG_NAME, 0);


	// Variable para dormir el nodo un segundo en cada iteracion
	Duration seconds_sleep(1);

	// Bucle principal
	while (node.ok()){

		// Comprobacion de flags para ver si toda la informacion está lista
		if (emotion_ready && information_ready && position_ready){
			// Reseteo de los flags
			emotion_ready = false;
			position_ready = false;
			information_ready = false;
			publisher.publish(user); // Publicacion del mensaje
		}

		// Funcion necesaria para ejecutar funciones a la vez que el nodo se queda 
		// esperando a posibles mensajes o servicios
		spinOnce();

		// El nodo se duerme durante un segundo antes de empezar la siguiente iteracion
		seconds_sleep.sleep();
		
	}	

	return 0;
}		
