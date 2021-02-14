#include "ros/ros.h"
#include <string>
#include <sstream>
#include <queue>

//message types
#include <std_msgs/Int16MultiArray.h>
#include <patrolling_sim/MQTT_Message.h>
#include <patrolling_sim/DTAG_Message.h>
#include <patrolling_sim/DTAP_Message.h>
#include <patrolling_sim/GBS_Message.h>
#include <patrolling_sim/SEBS_Message.h>
//MQTT definitions
#include "mqtt_types.h"


#define NUM_MAX_ROBOTS 32
#define MSG_DELAY_TIME 0.2


uint teamsize;
uint packet_ID_counter;
uint QoS_level;

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

// void setQoSLvl() {
//     ros::param::get("/MQTT_service_level", QoS_level);
//     ROS_INFO_STREAM("MQTT_QoS is set to: " << MQTT_MODE);
// }

// bool getQoSLvl() {
//     return QoS_level;
// }


void handleMessage(const patrolling_sim::MQTT_Message::ConstPtr& msg) {

    uint message_Type = msg->message_Type;
    uint protocol_Type = msg->protocol_Type;
    uint packet_ID = msg->packet_ID;
    uint QoS_level = msg->service_Level;
    uint sender_ID = msg->sender_ID;


    //this is an unhandled packet
    if(packet_ID==-1){
        patrolling_sim::MQTT_Message packetCreated;
        //add packet_ID
        packetCreated.packet_ID = packet_ID_counter;
        packet_ID_counter++;
    }

}

void robotCallback(const patrolling_sim::MQTT_Message::ConstPtr& msg) {
    // for(int loop=0;loop<teamsize;loop++){
    //     robot_publishers_list[loop].publish(msg);
    // }
    // results_pub_monitor.publish(msg);

    handleMessage(msg);
}


void printTopicList(){
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    std::string topic_MQTT = "/MQTT";
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        std::string topic_name = info.name;
        if (topic_name.find(topic_MQTT) != std::string::npos) {
            std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
        }
    }
}

int  createTopicList(ros::NodeHandle* nh){
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    int topicNum = 0;

    std::string topic_MQTT = "/MQTT";
    std::string topic_IN = "_IN";
    std::string topic_OUT = "_OUT";
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        std::string topic_name = info.name;
        if (topic_name.find(topic_MQTT) != std::string::npos) {
            std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
            if (topic_name.find(topic_OUT) != std::string::npos){
                //create and add subsciber to list
                std::cout << "Creating sub for" << topic_name << std::endl;
                
                ros::Subscriber results_sub = nh->subscribe<patrolling_sim::MQTT_Message>(topic_name, 100, robotCallback);
                robot_subscibers_list.push_back(results_sub);
                

                //need to create matching topic
                //no one is yet publishing so it wont be registered
                //but robots are subscibing to it
                std::string::size_type i = topic_name.find(topic_OUT);
                topic_name.erase(i, topic_OUT.length());
                topic_name += topic_IN;
                std::cout << "Creating pub for" << topic_name << std::endl;
                //create and add publisher to list
                ros::Publisher results_pub = nh->advertise<patrolling_sim::MQTT_Message>(topic_name, 100);
                robot_publishers_list.push_back(results_pub);
                topicNum++;
            }
        }
    }
    return topicNum;
}

int main(int argc, char **argv) {

    char teamsize_str[3];
    teamsize = atoi(argv[1]);

    std::string algo = argv[2];

    //set counter to start at zero
    packet_ID_counter = 0;


    printf("*algo = %s\n", algo.c_str());
    // printf("Algorithm shorthand is %s", argv[2].c_str());

    if ( teamsize >= NUM_MAX_ROBOTS || teamsize <1 ) {
        ROS_INFO("The Teamsize must be an integer number between 1 and %d", NUM_MAX_ROBOTS);
        return 0;
    } else{
        strcpy (teamsize_str, argv[1]); 
        // printf("teamsize: %s\n", teamsize_str);
        printf("teamsize: %u\n", teamsize);
    }


    ros::init(argc, argv, "MQTTBroker");

    ros::NodeHandle nh;






    // int i;
    // for(i=0; i<teamsize; i++){

    //     //creating subscriber for results of robot i

    //     // topic names are /MQTT/robot<id>/<ALGO>_results
    //     //so for robot 0 and algorithm SEBS we would have
    //     // /MQTT/robot0/SEBS_results

    //     std::string topic_sub = "/MQTT/robot";
    //     topic_sub += ToString(i);
    //     topic_sub += "/SEBS_results";

    //     ros::Subscriber results_sub = nh.subscribe<patrolling_sim::MQTT_Message>(topic_sub, 100, robotCallback);
    //     robot_subscibers_list.push_back(results_sub);
    //     // printf("Creating subscriber for topic named: %s\n", topic_sub.c_str());

    //     //creating matching publishers

    //     std::string topic_pub = "/MQTT/robot";
    //     topic_pub += ToString(i);
    //     topic_pub += "/SEBS_results";

    //     ros::Publisher results_pub = nh.advertise<patrolling_sim::MQTT_Message>(topic_pub, 100);
    //     robot_publishers_list.push_back(results_pub);
    //     // printf("Creating publisher for topic named: %s\n", topic_pub.c_str());

    // }


    

    // give the robots time to launch
    // ros::Duration(1).sleep();

    int topicNum = createTopicList(&nh);
    printf("Created %d topics.\n", topicNum);
    printTopicList();

    ros::spin();

    return 0;
}

