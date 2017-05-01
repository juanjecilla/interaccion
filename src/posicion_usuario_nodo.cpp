/**
 ** posicion_usuario es un nodo del paquete interaccion que permite la introduccion de la posicion
 ** de un usuario mediante sus coordenadas X, Y y Z.
**/

// Librerias y paquetes usados por el nodo
#include "ros/ros.h"
#include "interaccion/pos_usuario.h"

// Macros auxiliares para definir los nombres del nodo y los mensajes y topics utilizados
#define NODE_NAME "posicion_usuario_nodo"
#define INFO_MSG_NAME "pos_usuario_topic"

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
	Publisher publisher = node.advertise<interaccion::pos_usuario>(INFO_MSG_NAME, 0);

	// Variable para dormir el nodo un segundo en cada iteracion
	Duration seconds_sleep(1);

	// Bucle principal
	while (node.ok()){

		// Variables auxiliares para la introduccion de coordenadas
		int x; 
		int y;
		int z;

		// Mensaje de tipo pos_usuario que almacena las coordenadas introducidas por el usuario
		interaccion::pos_usuario message;

		// Introducción por teclado de la coordenada X
		ROS_INFO("Introduzca la coordenada X del usuario:");
		cin >> x;

		// Introducción por teclado de la coordenada Y
		ROS_INFO("Introduzca la coordenada Y del usuario:");
		cin >> y;

		// Introducción por teclado de la coordenada Z
		ROS_INFO("Introduzca la coordenada Z del usuario:");
		cin >> z;

		// Se establecen los valores del mensaje con la informacion introducida
		message.x = x;
		message.y = y;
		message.z = z;

		// El publicador manda el mensaje para que el empaquetador_nodo lo trate
		publisher.publish(message);

		// Funcion necesaria para ejecutar funciones a la vez que el nodo se queda 
		// esperando a posibles mensajes o servicios
		spinOnce();
		ROS_DEBUG ("Se duerme el nodo emisor durante un segundo");

		// El nodo se duerme durante un segundo antes de empezar la siguiente iteracion
		seconds_sleep.sleep();

	}	

	return 0;
}