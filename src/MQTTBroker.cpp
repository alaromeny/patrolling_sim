#include "ros/ros.h"
#include <string>
#include <sstream>
#include <std_msgs/Int16MultiArray.h>


#define NUM_MAX_ROBOTS 32



uint teamsize;
std::vector<ros::Subscriber> robot_subscibers_list;

template <typename T>
std::string ToString(T val)
{
    std::stringstream stream;
    stream << val;
    return stream.str();
}


void chatterCallback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
    std::vector<signed short>::const_iterator it = msg->data.begin();    
    std::vector<int> vresults;

    vresults.clear();
    
    for (size_t k=0; k<msg->data.size(); k++) {
        vresults.push_back(*it); it++;
    } 

    int id_sender = vresults[0];
    int msg_type = vresults[1];
    
    printf(" MESSAGE FROM %d TYPE %d ...\n",id_sender, msg_type);
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

    std::string topic = "results_robot";
    topic += ToString(i);

    ros::Subscriber results_sub = nh.subscribe<std_msgs::Int16MultiArray>(topic, 100, chatterCallback ); //Subscrever "results" vindo dos robots
    robot_subscibers_list.push_back(results_sub);
    printf("Creating subscriber for topic named: %s\n", topic.c_str());

  }

  printf("Created -%u- topics\n", i);





  // ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
  ros::Subscriber results_sub = nh.subscribe<std_msgs::Int16MultiArray>("results", 100, chatterCallback ); //Subscrever "results" vindo dos robots



  ros::spin();

  return 0;
}

