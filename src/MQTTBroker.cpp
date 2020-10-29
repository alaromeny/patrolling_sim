#include "ros/ros.h"
#include <string>
#include <sstream>
#include <std_msgs/Int16MultiArray.h>
#include <queue>

#include "message_types.h"


#define NUM_MAX_ROBOTS 32
// #define MSG_DELAY_TIME 0.1


uint teamsize;
uint packet_id_counter = 0;
int MQTT_ON;
int QoS_LEVEL = 0;
int MSG_DELAY_TIME;
std::vector<ros::Subscriber> robot_subscibers_list;
std::vector<ros::Publisher>  robot_publishers_list;

ros::Subscriber results_sub_monitor;
ros::Publisher results_pub_monitor;


template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}

struct MQTT_msg {
  std_msgs::Int16MultiArray::ConstPtr msg;
  ros::Time msg_time;
  uint packet_id;
  bool dup_flag;
  bool retain_flag;
  int QoS_level;
  uint sender_id;
  uint msg_type;
} ;

std::queue<MQTT_msg> inbound_msg_queue;
std::queue<MQTT_msg> outbound_msg_queue;


void printMQTTQoS(){

  switch(MQTT_ON) {
    QoS_LEVEL = MQTT_ON - 1;
    case 0:
        printf("MQTT_ON setting is 0; this node shouldn't be running!!!\n");
      break;
    case 1:
        printf("MQTT_ON setting is %d; QoS is lvl %d\n" , MQTT_ON, QoS_LEVEL);
      break;
    case 2:
        printf("MQTT_ON setting is %d; QoS is lvl %d\n" , MQTT_ON, QoS_LEVEL);
      break;
    case 3:
        printf("MQTT_ON setting is %d; QoS is lvl %d\n" , MQTT_ON, QoS_LEVEL);
      break;
    default:
        printf("MQTT_ON error, value is %d\n", MQTT_ON);
  }
}

std::vector<int> getMessageData(const std_msgs::Int16MultiArray::ConstPtr& msg){

    std::vector<signed short>::const_iterator it = msg->data.begin();    
    std::vector<int> vresults;

    vresults.clear();
    
    for (size_t k=0; k<msg->data.size(); k++) {
        vresults.push_back(*it); it++;
    } 

    return vresults;
}


int getMessageType(const std_msgs::Int16MultiArray::ConstPtr& msg){
    std::vector<int> vresults = getMessageData(msg);
    return vresults[1];
}

int getMessageSender(const std_msgs::Int16MultiArray::ConstPtr& msg){
    std::vector<int> vresults = getMessageData(msg);
    return vresults[0];
}

void getMessageHeader(const std_msgs::Int16MultiArray::ConstPtr& msg, int *msg_header){
    std::vector<int> vresults = getMessageData(msg);
    msg_header[0] = vresults[0];
    msg_header[1] = vresults[1];
}


void broadcastMessage(const std_msgs::Int16MultiArray::ConstPtr& msg, double delay_time){

    int msg_header[2];
    getMessageHeader(msg, msg_header);
    int id_sender = msg_header[0];
    int msg_type = msg_header[1];


    for(int loop=0;loop<teamsize;loop++){
      //if(loop != id_sender){
        // printf("FORWARDING MESSAGE FROM %d TO %d ...\n",id_sender, loop);
        robot_publishers_list[loop].publish(msg);
      //}
    }
    results_pub_monitor.publish(msg);
}


void pushMessageToQueue(const std_msgs::Int16MultiArray::ConstPtr& msg){

    MQTT_msg new_msg;
    new_msg.msg = msg;
    new_msg.msg_time = ros::Time::now();
    new_msg.packet_id = packet_id_counter;
    new_msg.QoS_level = QoS_LEVEL;

    packet_id_counter++;
    int msg_header[2];
    getMessageHeader(msg, msg_header);

    new_msg.sender_id = msg_header[0];
    new_msg.msg_type = msg_header[1];

    if(new_msg.msg_type < 51){
      inbound_msg_queue.push(new_msg);
      printf("New message recieved of type %d. Adding to inbound queue\n", new_msg.msg_type);
    } else {
      //do MQTT protocol here
    }
}

void forwardDelayedMessages(const ros::TimerEvent&){


  int queue_size = inbound_msg_queue.size();
  int i = 0;

  for (i=0;i<queue_size;i++){
    MQTT_msg curr_delayed_msg = inbound_msg_queue.front();
    ros::Time msg_timestamp = curr_delayed_msg.msg_time;
    ros::Time current_time = ros::Time::now();

    double delay_time = current_time.toSec() - msg_timestamp.toSec();
    if (delay_time >= MSG_DELAY_TIME){

      inbound_msg_queue.pop();

      broadcastMessage(curr_delayed_msg.msg, delay_time);

    } else {
      return;
    }
  }
}

void robotCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    pushMessageToQueue(msg);
}

void monitorCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
    pushMessageToQueue(msg);
}

int main(int argc, char **argv) {


  //   uint teamsize;
  char teamsize_str[3];
  teamsize = atoi(argv[1]);
  
  if ( teamsize >= NUM_MAX_ROBOTS || teamsize <1 ){
    ROS_INFO("The Teamsize must be an integer number between 1 and %d", NUM_MAX_ROBOTS);
    return 0;
  }else{
    strcpy (teamsize_str, argv[1]); 
        // printf("teamsize: %s\n", teamsize_str);
        printf("teamsize: %u\n", teamsize);
  }


  ros::init(argc, argv, "MQTTBroker");

  ros::NodeHandle nh;


  if(ros::param::has("/MQTT_on")){
    ros::param::get("/MQTT_on", MQTT_ON);
    printMQTTQoS();
  } else{
    MQTT_ON = 0;
    ROS_WARN("Cannot read parameter /MQTT_on. Using default value!");
  }

  if(ros::param::has("/Msg_Delay_Time")){
    ros::param::get("/Msg_Delay_Time", MSG_DELAY_TIME);
  } else {
    MSG_DELAY_TIME = 0.0;
    ROS_WARN("Cannot read parameter /Msg_Delay_Time. Using default value!");

  }

  int i;
  for(i=0; i<teamsize; i++){

    //creating subscriber for results of robot i

    std::string topic_sub = "/results/robot";
    topic_sub += ToString(i);
    topic_sub += "_out";

    ros::Subscriber results_sub = nh.subscribe<std_msgs::Int16MultiArray>(topic_sub, 100, robotCallback);
    robot_subscibers_list.push_back(results_sub);
    printf("Creating subscriber for topic named: %s\n", topic_sub.c_str());

    //creating matching publishers

    std::string topic_pub = "/results/robot";
    topic_pub += ToString(i);
    topic_pub += "_in";

    ros::Publisher results_pub = nh.advertise<std_msgs::Int16MultiArray>(topic_pub, 100);

    robot_publishers_list.push_back(results_pub);
    printf("Creating publisher for topic named: %s\n", topic_pub.c_str());

  }

  results_sub_monitor = nh.subscribe<std_msgs::Int16MultiArray>("/results/monitor_out", 100, monitorCallback);
  results_pub_monitor = nh.advertise<std_msgs::Int16MultiArray>("/results/monitor_in", 100);

  printf("Created %d topics.\n", i +1);



  // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  // ros::Subscriber results_sub = nh.subscribe<std_msgs::Int16MultiArray>("results", 100, chatterCallback ); //Subscrever "results" vindo dos robots


  ros::Timer timerDelayedMessages =
      nh.createTimer(ros::Duration(1.0 / 40.0), forwardDelayedMessages);

  ros::spin();

  return 0;
}

