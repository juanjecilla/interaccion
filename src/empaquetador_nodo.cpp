#include "ros/ros.h"
#include <string>
#include "interaccion/inf_personal_usuario.h"
#include "interaccion/pos_usuario.h"
#include "interaccion/usuario.h"
#include "std_msgs/String.h"

#define NODE_NAME "empaquetador_nodo"
#define INFO_MSG_NAME "user_topic"
#define INFO_TOPIC_NAME_1 "emocion_topic"
#define INFO_TOPIC_NAME_2 "inf_pers_topic"
#define INFO_TOPIC_NAME_3 "pos_usuario_topic"

using namespace std;
using namespace ros;

interaccion::usuario user;

bool emotion_ready = false;
bool position_ready = false;
bool information_ready = false;

void funcionCallbackEmotionTopic(const std_msgs::String::ConstPtr& msg){

	ROS_INFO("Emocion: %s", msg->data.c_str());
	user.emocion = msg->data.c_str();
	emotion_ready = true;


}

void funcionCallbackInfPersonalTopic(const interaccion::inf_personal_usuario::ConstPtr& msg){

	ROS_INFO("Nombre: %s", msg->nombre.c_str());
	ROS_INFO("Edad: %d", msg->edad);
	ROS_INFO("Idiomas: %s", msg->idiomas[0].c_str());

	user.infPersonal = *msg;
	information_ready = true;
}

void funcionCallbackPositionTopic(const interaccion::pos_usuario::ConstPtr& msg){

	ROS_INFO("Coordenada X: %d", msg->x);
	ROS_INFO("Coordenada Y: %d", msg->y);
	ROS_INFO("Coordenada Y: %d", msg->z);

	user.posicion = *msg;
	position_ready = true;

}


int main(int argc, char **argv){
	
	init(argc,argv, NODE_NAME);
	NodeHandle node;

	ROS_INFO("Node created and registered");

	Subscriber subscriptor1 = node.subscribe(INFO_TOPIC_NAME_1, 0, funcionCallbackEmotionTopic);
	Subscriber subscriptor2 = node.subscribe(INFO_TOPIC_NAME_2, 0, funcionCallbackInfPersonalTopic);
	Subscriber subscriptor3 = node.subscribe(INFO_TOPIC_NAME_3, 0, funcionCallbackPositionTopic);
	
	Publisher publisher = node.advertise<interaccion::usuario>(INFO_MSG_NAME, 0);


	Duration seconds_sleep(1);

	while (node.ok()){

		if (emotion_ready && information_ready && position_ready){
			emotion_ready = false;
			position_ready = false;
			information_ready = false;
			publisher.publish(user);
		}



		spinOnce();
		ROS_DEBUG ("Se duerme el nodo emisor durante un segundo");

		seconds_sleep.sleep();

		
	}	

	return 0;
}		
