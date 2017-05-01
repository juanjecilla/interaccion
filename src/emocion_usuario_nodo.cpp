/**
 ** emocion_usuario_nodo es un nodo del paquete interaccion que permite la introduccion de la emocion
 ** de un usuario mediante el teclado.
**/

// Librerias y paquetes usados por el nodo
#include "ros/ros.h"
#include "std_msgs/String.h"

// Macros auxiliares para definir los nombres del nodo y los mensajes y topics utilizados
#define NODE_NAME "emoción_usuario_nodo"
#define INFO_MSG_NAME "emocion_topic"

// Declaracion de los espacios de nombres para facilitar la programacion
using namespace std;
using namespace ros;

/**
 ** main es la funcion que se ejecuta cada vez que se inicia el nodo de ros.
 ** @params: argc numero de argumentos proporcionados
 ** @params: argv puntero con el valor de los argumentos proporcionados
 ** @return: int valor con el resultado de la ejecucion
**/
int main(int argc, char **argv){
	
	// Inicializacion del nodo Ros con los argumentos y el nombre proporcionados
	init(argc,argv, NODE_NAME);
	NodeHandle node; // Manejador del nodo ros

	ROS_INFO("Node created and registered");

	// PUblicador de mensajes para mandar la informacion introducida al empaquetador_nodo
	Publisher publisher = node.advertise<std_msgs::String>(INFO_MSG_NAME, 0);

	// Variable para dormir el nodo un segundo en cada iteracion
	Duration seconds_sleep(1);

	// Bucle principal
	while (node.ok()){

		// Mensaje estandar de tipo string que almacena la emocion introducida para enviarla a empaquetador_nodo
		std_msgs::String emocion;

		// Introduccion por teclado de la emocion del usuario
		ROS_INFO("Introduzca la emocion del usuario:");
		cin >> emocion.data;

		// El publicador manda el mensaje para que el empaquetador_nodo lo trate
		publisher.publish(emocion);

		// Funcion necesaria para ejecutar funciones a la vez que el nodo se queda 
		// esperando a posibles mensajes o servicios
		spinOnce();
		ROS_DEBUG ("Se duerme el nodo emisor durante un segundo");

		// El nodo se duerme durante un segundo antes de empezar la siguiente iteracion
		seconds_sleep.sleep();

	}	

	return 0;
}		
