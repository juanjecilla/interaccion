/**
 ** matematico_nodo es un nodo del paquete interaccion que implementa un servicio para multiplicar
 ** la edad introducida por 2.
**/

// Librerias y paquetes usados por el nodo
#include "ros/ros.h"
#include "interaccion/multiplicador.h"

// Macros auxiliares para definir los nombres del nodo y los mensajes y topics utilizados
#define NODE_NAME "matematico_nodo"
#define INFO_MSG_NAME "user_topic"
#define SRV_NAME "multiplication_service"

// Declaracion de los espacios de nombres para facilitar la programacion
using namespace std;
using namespace ros;

/**
 ** multiplicationService es la funcion que se invoca al servicio desde dialogo nodo.
 ** @params: req valor de entrada del servicio
 ** @params: res valor de salida del servicio
 ** @return: bool valor con el resultado de la ejecucion
**/
bool multiplicationService(interaccion::multiplicador::Request &req, interaccion::multiplicador::Response &res){

	res.resultado = req.entrada * 2;

	ROS_INFO("Petición: x = %d", (int)req.entrada);

	ROS_INFO("Respuesta: %d", (int)res.resultado);

	return true;

}

/**
 ** main es la funcion que se ejecuta cada vez que se inicia el nodo de ros.
 ** @params: argc numero de argumentos proporcionados
 ** @params: argv puntero con el valor de los argumentos proporcionados
 ** @return: int valor con el resultado de la ejecucion
**/
int main(int argc, char **argv){

	// Inicializacion del nodo Ros con los argumentos y el nombre proporcionados
	init(argc, argv, NODE_NAME);
	NodeHandle node; // Manejador del nodo ros

	// Servidor que proporciona el servicio de multiplicacion que se queda escuchando peticiones
	ServiceServer service = node.advertiseService(SRV_NAME,multiplicationService);

	ROS_INFO("Servicio matemático registrado.");	

	// Funcion que bloquea el nodo esperando a que se realice una llamada al servicio
	spin();

	return 0;

}