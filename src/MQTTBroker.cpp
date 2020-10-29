#include "ros/ros.h"
#include <string>
#include <sstream>
#include <std_msgs/Int16MultiArray.h>
#include <queue>


#define NUM_MAX_ROBOTS 32
#define MSG_DELAY_TIME 0.2


uint teamsize;
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


struct delayed_msg {
  std_msgs::Int16MultiArray::ConstPtr msg;
  ros::Time msg_time;
} ;

std::queue<delayed_msg> delayed_msg_queue;

void broadcastMessage(const std_msgs::Int16MultiArray::ConstPtr& msg){

    std::vector<signed short>::const_iterator it = msg->data.begin();    
    std::vector<int> vresults;

    vresults.clear();
    
    for (size_t k=0; k<msg->data.size(); k++) {
        vresults.push_back(*it); it++;
    } 
// 
    int id_sender = vresults[0];
    int msg_type = vresults[1];


    for(int loop=0;loop<teamsize;loop++){
      //if(loop != id_sender){
        // printf("FORWARDING MESSAGE FROM %d TO %d ...\n",id_sender, loop);
        robot_publishers_list[loop].publish(msg);
      //}
    }
    results_pub_monitor.publish(msg);
}


void forwardDelayedMessages(){


  int queue_size = delayed_msg_queue.size();
  int i = 0;

  for (i=0;i<queue_size;i++){
    delayed_msg curr_delayed_msg = delayed_msg_queue.front();
    ros::Time msg_timestamp = curr_delayed_msg.msg_time;
    ros::Time current_time = ros::Time::now();
    if (current_time.toSec() - msg_timestamp.toSec() > MSG_DELAY_TIME){
      delayed_msg_queue.pop();

      // robot_publishers_list[id_sender].publish(msg);
    }

  }

  // robot_publishers_list[id_sender].publish(msg);

}

void robotCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
  for(int loop=0;loop<teamsize;loop++){
      //if(loop != id_sender){
      // printf("FORWARDING MESSAGE FROM %d TO %d ...\n",id_sender, loop);
        robot_publishers_list[loop].publish(msg);
      //}
    }
  
    results_pub_monitor.publish(msg);
    
    // broadcastMessage(msg);

    // delayed_msg new_msg;
    // new_msg.msg = msg;
    // new_msg.msg_time = ros::Time::now();

    // delayed_msg_queue.push(new_msg);

}

void monitorCallback(const std_msgs::Int16MultiArray::ConstPtr& msg) {
   
    // if(id_sender == -1){
      // printf("MESSAGE FROM MONITOR ID: %d \n",id_sender);  
      for(int loop=0;loop<teamsize;loop++){
        robot_publishers_list[loop].publish(msg);
      }
    // }

    // broadcastMessage(msg);


    // delayed_msg new_msg;
    // new_msg.msg = msg;
    // new_msg.msg_time = ros::Time::now();

    // delayed_msg_queue.push(new_msg);

}

int main(int argc, char **argv)
{


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



  ros::spin();

  return 0;
}

