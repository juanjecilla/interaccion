/**
 ** informacion_personal_nodo es un nodo del paquete interaccion que permite la introduccion de
 ** los datos de un usuario mediante teclado.
**/

// Librerias y paquetes usados por el nodo
#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "interaccion/inf_personal_usuario.h"

// Macros auxiliares para definir los nombres del nodo y los mensajes y topics utilizados
#define NODE_NAME "informacion_personal_nodo"
#define INFO_MSG_NAME "inf_pers_topic"

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
	Publisher publisher = node.advertise<interaccion::inf_personal_usuario>(INFO_MSG_NAME, 0);

	// Variable para dormir el nodo un segundo en cada iteracion
	Duration seconds_sleep(1);

	// Bucle principal
	while (node.ok()){

		// Variables auxiliares para la introduccion de los datos
		string nombre;
		string aux;
		int edad;
		int numIdiomas;
		vector<string> idiomas;

		// Mensaje de tipo inf_personal_usuario que almacena los datos para envialos al empaquetador_nodo
		interaccion::inf_personal_usuario message;

		// Introduccion del nombre del usuario por teclado
		ROS_INFO("Introduzca el nombre del usuario");
		cin >> nombre;

		// Introduccion de la edad del usuario por teclado
		ROS_INFO("Introduzca la edad del usuario");
		cin >> edad;

		// Introduccion del numero de idiomas que habla el usuario por teclado
		ROS_INFO("¿Cuántos idiomas hablas?");
		cin >> numIdiomas;

		// Bucle para rellenar el vector de idiomas que habla el usuario
		int i;
		for (i = 0; i<numIdiomas; i++){
			ROS_INFO("Introduce el idioma %d", i+1);
			cin >> aux;
			idiomas.push_back(aux);
		}

		// Se establecen los valores del mensaje con la informacion introducida
		message.nombre = nombre;
		message.edad = edad;
		message.idiomas = idiomas;
		
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