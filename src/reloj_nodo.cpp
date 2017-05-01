/**
 ** Reloj_nodo.cpp es un nodo del paquete interaccion que se encarga de imprimir la 
 ** hora local y la hora UTC a una frecuencia de 3 veces por segundo junto con los 
 ** segundos transcurridos desde que se ha recibido el mensaje de start o reset. Adicionalmente,
 ** usa un ros::Timer para notificar al dialogo_nodo que el nodo sigue activo cada 60
 ** segundos.
**/

// Librerias y paquetes usados por el nodo
#include "ros/ros.h"
#include <string>
#include <ctime>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "boost/date_time/posix_time/posix_time.hpp"

// Macros auxiliares para definir los nombres del nodo y los mensajes y topics utilizados
#define NODE_NAME "reloj_nodo"
#define RELOJ_MSG_NAME "still_alive"
#define RESET_TOPIC_NAME "reset_topic"
#define START_TOPIC_NAME "start_topic"
#define COUNTDOWN_TIME 60

// Declaracion de los espacios de nombres para facilitar la programacion
using namespace std;
using namespace ros;
using namespace boost::posix_time;

Time startTime; // Variable Time que almacena el valor del tiempo en el momento que llega un mensaje
bool clock_start = false; // Flag necesario para empezar a mostrar la hora solo cuando se haya recibido el primer mensaje
int totalSeconds = 0; // Segundos transcurridos desde que se ha recibido el primer mensaje

Publisher publisher; // Publicador de mensajes que manda el mensaje still_alive

/**
 ** timerCallback es la funcion que se ejecuta cada vez que se vence el tiempo indicado por el timer.
 ** Una vez vencido el timer, se envia el mensaje de still_alive al que esta suscrito el dialogo.
 ** @params: TimerEvent parametro con el evento generado por el timer.
 ** @return: void
**/
void timerCallback(const ros::TimerEvent&){
	std_msgs::Bool still_alive;
	still_alive.data = true;
	publisher.publish(still_alive);
}

/**
 ** printClock es la funcion auxiliar que se encarga de imprimir la hora local y la hora UTC.
 ** Adicionalmente, imprime los segundos transcurridos desde que se recibio el mensaje de start/reset.
 ** @params: void
 ** @return: void
**/
void printClock(){

	totalSeconds = (Time::now() - startTime).toSec(); // Segundos desde el inicio hasta el momento actual

	ptime t_local(second_clock::local_time()); // Declaracion de una variable de tiempo local
	ptime t_utc(second_clock::universal_time()); // Declaracion de una variable de tiempo UTC

	ROS_INFO("LOCAL HOUR: %s", to_simple_string(t_local).c_str());
	ROS_INFO("UTC HOUR: %s", to_simple_string(t_utc).c_str());

	ROS_INFO("SECONDS FROM START/RESET: %lf", (double)(Time::now() - startTime).toSec());

}

/**
 ** fuctionCallbackReset es la funcion que se ejecuta cada vez que se recibe un mensaje de reset 
 ** emitido desde el dialogo_nodo. Resetea la variable usada para contar los segundos que han 
 ** transcurrido y activa el flag de impresión de hora.
 ** @params: msg std_msgs::String con el valor reset_timer
 ** @return: void
**/
void funcionCallbackReset(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("RESET MESSAGE: %s", msg->data.c_str());
	startTime = Time::now();
	clock_start = true;
}

/**
 ** fuctionCallbackStart es la funcion que se ejecuta cada vez que se recibe un mensaje de start 
 ** emitido desde el dialogo_nodo. Resetea la variable usada para contar los segundos que han 
 ** transcurrido y activa el flag de impresión de hora.
 ** @params: msg std_msgs::String con el valor start_timer
 ** @return: void
**/
void funcionCallbackStart(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("START MESSAGE: %s", msg->data.c_str());
	startTime = Time::now();
	clock_start = true;
}

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

	// Suscriptores a los posibles mensajes de entrada procedentes de dialogo_nodo
	Subscriber subscriptor1 = node.subscribe(START_TOPIC_NAME, 0, funcionCallbackStart);
	Subscriber subscriptor2 = node.subscribe(RESET_TOPIC_NAME, 0, funcionCallbackReset);
	
	// Timer que se vence cada 60 segundos para indicar que el nodo sigue activo
	Timer timer = node.createTimer(Duration(COUNTDOWN_TIME), timerCallback);

	// Publicador de mensajes para indicar que el nodo sigue activo
	publisher = node.advertise<std_msgs::Bool>(RELOJ_MSG_NAME, 0);

	// Variable Rate para indicar la frecuencia con la que se tiene que ejecutar el bucle principal
	Rate rate(3);

	// Bucle principal
	while (node.ok()){

		// Impresion de hora y segundos si el flag esta activo
		if (clock_start){
			printClock();
		}

		// Funcion necesaria para ejecutar funciones a la vez que el nodo se queda 
		// esperando a posibles mensajes o servicios
		spinOnce();
		ROS_DEBUG ("Se duerme el nodo emisor durante un segundo");

		// Duerme el nodo durante el tiempo necesario para cumplir la frecuencia indicada
		rate.sleep();
		
	}	

	return 0;
}		
