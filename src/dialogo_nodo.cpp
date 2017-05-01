/**
 ** dialogo_nodo es un nodo del paquete interaccion que muestra por pantalla la informacion recibida
 ** por el empaquetador y hace una llamada al sistema para sintetizar voz con la informacion recibida.
 ** Es el encargado de llamar a reloj_nodo para iniciar la presentacion de la hora y la llamada al servicio
 ** de multiplicacion de la edad introducida.
**/

// Librerias y paquetes usados por el nodo
#include "ros/ros.h"
#include <string>
#include "interaccion/usuario.h"
#include "interaccion/multiplicador.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <cstdlib>

// Macros auxiliares para definir los nombres del nodo y los mensajes y topics utilizados
#define NODE_NAME "dialogo_nodo"
#define INFO_TOPIC_NAME "user_topic"
#define RELOJ_TOPIC_NAME "still_alive"
#define RESET_MSG_NAME "reset_topic"
#define START_MSG_NAME "start_topic"
#define SRV_NAME "multiplication_service"

// Declaracion de los espacios de nombres para facilitar la programacion
using namespace std;
using namespace ros;

// Cliente del servicio multiplicador de edad
ServiceClient client;

// Tipo del servicio de multiplicacion de edad
interaccion::multiplicador srv;

// Flags auxiliares para saber la informacion que se ha recibido 
bool empaquetador_ready = false;
bool matematico_ready = false;
bool first_time = true;

/**
 ** fuctionCallbackShowMessage es la funcion que se ejecuta cuando se recibe el mensaje procedente
 ** de empaquetador_nodo con la informacion proporcionada por el usuario
 ** @params: msg mensaje de tipo usuario con la informacion proporcionada
 ** @return: void
**/
void funcionCallbackShowMessage(const interaccion::usuario::ConstPtr& msg){

	// Se muestra por pantalla toda la informacíon que ha sido proporcionada
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

	// Variable auxiliar para concatenar strings con valores de variables 
	stringstream textToSpeech;
	string text;

	// Construccion del string final
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

	// Conversion de stringstream a string
	text = textToSpeech.str();
	ROS_INFO("TEXTO: %s", text.c_str());

	// Construccion del comando para la sintesis de voz
	string command = "espeak -v es \"" + text + "\"";
	printf("COMMAND: %s", command.c_str());

	// Llamada al sistema con el comando de sintesis de voz para su ejecucion
	system (command.c_str()); 

	// Se establece el parametro de entrada del servicio de multiplicacion
	srv.request.entrada = msg->infPersonal.edad;

	// Se activa el flag de que la informacion del empaquetador ha sido recibida
	empaquetador_ready = true;

	// Se realiza la llamada al servicio de multiplicacion para obtener la edad multiplicada por 2
	if (client.call(srv)){
		ROS_INFO("Respuesta del servicio: %d", (int)srv.response.resultado);
		// Se activa el flag de la multiplicacion lista
		matematico_ready = true;

	} else {
		ROS_ERROR("Fallo al llamar al servicio: %s", SRV_NAME);
		// Se desactiva el flag de la multiplicacion lista
		matematico_ready = false;
		return;
	}
}

/**
 ** functionCallbackReloj es la funcion que se ejecuta cada vez que se recibe un mensaje de 
 ** still_alive emitido desde el reloj_nodo. Muestra un mensaje para informar de que el reloj
 ** sigue activo.
 ** @params: msg std_msgs::Bool con el valor still_alive
 ** @return: void
**/
void functionCallbackReloj(const std_msgs::Bool::ConstPtr& msg){

	ROS_INFO("CLOCK TIMER STILL ALIVE: %s", msg->data ? "True" : "False");

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

	// Suscriptores a los posibles mensajes de entrada procedentes de empaquetador_nodo y reloj_nodo
	Subscriber subscriptor1 = node.subscribe(INFO_TOPIC_NAME, 0, funcionCallbackShowMessage);
	Subscriber subscriptor2 = node.subscribe(RELOJ_TOPIC_NAME, 0, functionCallbackReloj);
	
	// Publicador de mensajes para iniciar o reiniciar el reloj
	Publisher publisher1 = node.advertise<std_msgs::String>(START_MSG_NAME, 0);
	Publisher publisher2 = node.advertise<std_msgs::String>(RESET_MSG_NAME, 0);

	// Inicializacion de la llamada al servicio de multiplicacion
	client = node.serviceClient<interaccion::multiplicador>(SRV_NAME);

	// Variable para dormir el nodo un segundo en cada iteracion
	Duration seconds_sleep(1);

	// Bucle principal
	while (node.ok()){

		// Mensaje String para iniciar o resetear el reloj de reloj_nodo
		std_msgs::String message;

		// Comprobacion de flags para ver si toda la informacion está lista
		if (matematico_ready && empaquetador_ready){
			// Reseteamos los flags
			matematico_ready = false;
			empaquetador_ready = false;

			// La primera vez se manda el mensaje start y a continuacion resets
			if (first_time){
				first_time = false;
				message.data = "start_timer";
				publisher1.publish(message); // Publicacion del mensaje
			} else {
				message.data = "reset_timer";
				publisher2.publish(message); // Publicacion del mensaje
			}

		}

		// Funcion necesaria para ejecutar funciones a la vez que el nodo se queda 
		// esperando a posibles mensajes o servicios
		spinOnce();

		// El nodo se duerme durante un segundo antes de empezar la siguiente iteracion
		seconds_sleep.sleep();

	}	
	
	return 0;
}		
