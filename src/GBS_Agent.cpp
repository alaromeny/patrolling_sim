/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the ISR University of Coimbra nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: David Portugal (2011-2014), and Luca Iocchi (2014-2016)
*********************************************************************/

#include <sstream>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <patrolling_sim/GBS_Message.h>
#include <patrolling_sim/MQTT_Message.h>

#include "PatrolAgent.h"
#include "getgraph.h"
#include "algorithms.h"
#include "mqtt_types.h"


using namespace std;

class GBS_Agent: public PatrolAgent {

private:

    double G1, G2;
    double edge_min;  
    int NUMBER_OF_ROBOTS;
    bool arrived;
    bool MQTT_mode;
    uint vertex_arrived;
    int robot_arrived;  
    ros::Subscriber gbs_results_sub;
    ros::Publisher  gbs_results_pub;  

public:
    virtual void init(int argc, char** argv);
    virtual int compute_next_vertex();
    //original function
    virtual void send_results();
    //needed for new message type
    virtual void do_send_ROS_message();
    //needed for MQTT mode
    virtual void do_send_MQTT_message();

    //original function
    virtual void receive_results();
    //needed for new message type
    virtual void receive_results(const patrolling_sim::GBS_Message::ConstPtr& msg);
    //needed for MQTT mode
    virtual void receive_results(patrolling_sim::GBS_Message* msg);
    virtual void processEvents();
    virtual void ROS_resultsCB(const patrolling_sim::GBS_Message::ConstPtr& msg);
    virtual void MQTT_resultsCB(const patrolling_sim::MQTT_Message::ConstPtr& msg);
    virtual void MQTT_handler(const patrolling_sim::MQTT_Message::ConstPtr& msg);


};



void GBS_Agent::init(int argc, char** argv) {
  
    PatrolAgent::init(argc,argv);
    ros::NodeHandle nh;

    MQTT_mode = getMQTTMode();

    NUMBER_OF_ROBOTS = atoi(argv[3]);
    arrived = false; 

    /** Define G1 and G2 **/
    G1 = 0.1;

    //default:
    G2 = 100.0;
    edge_min = 1.0;
  
    if (graph_file=="maps/grid/grid.graph") {  
        if (NUMBER_OF_ROBOTS == 1){G2 = 20.54;}
        if (NUMBER_OF_ROBOTS == 2){G2 = 17.70;}
        if (NUMBER_OF_ROBOTS == 4){G2 = 11.15;}
        if (NUMBER_OF_ROBOTS == 6){G2 = 10.71;}
        if (NUMBER_OF_ROBOTS == 8){G2 = 10.29;}
        if (NUMBER_OF_ROBOTS == 12){G2 = 9.13;}
    
    }else if (graph_file=="maps/example/example.graph") {
        if (NUMBER_OF_ROBOTS == 1){G2 = 220.0;}
        if (NUMBER_OF_ROBOTS == 2){G2 = 180.5;}
        if (NUMBER_OF_ROBOTS == 4){G2 = 159.3;}
        if (NUMBER_OF_ROBOTS == 6){G2 = 137.15;}
        if (NUMBER_OF_ROBOTS == 8 || NUMBER_OF_ROBOTS == 12){G2 = 126.1;}
        edge_min = 20.0;
    
    }else if (graph_file=="maps/cumberland/cumberland.graph") {
        if (NUMBER_OF_ROBOTS == 1){G2 = 152.0;}
        if (NUMBER_OF_ROBOTS == 2){G2 = 100.4;}
        if (NUMBER_OF_ROBOTS == 4){G2 = 80.74;}
        if (NUMBER_OF_ROBOTS == 6){G2 = 77.0;}
        if (NUMBER_OF_ROBOTS == 8 || NUMBER_OF_ROBOTS == 12){G2 = 63.5;}    
        edge_min = 50.0; 
    }
  
    printf("G1 = %f, G2 = %f\n", G1, G2); 

    if(MQTT_mode){
        gbs_results_pub = nh.advertise<patrolling_sim::MQTT_Message>("GBS_results_OUT", 100);
        gbs_results_sub = nh.subscribe<patrolling_sim::MQTT_Message>("GBS_results_IN", 10,  boost::bind(&GBS_Agent::MQTT_resultsCB, this, _1));  
    } else {
        gbs_results_pub = nh.advertise<patrolling_sim::GBS_Message>("GBS_results", 100);
        gbs_results_sub = nh.subscribe<patrolling_sim::GBS_Message>("GBS_results", 10,  boost::bind(&GBS_Agent::ROS_resultsCB, this, _1));  
    }

    //overwrite the patrolAgent pub and sub with custom messages
    

}



// Executed at any cycle when goal is not reached
void GBS_Agent::processEvents() {
      
    if (arrived && NUMBER_OF_ROBOTS>1){ //a different robot arrived at a vertex: update idleness table and keep track of last vertices positions of other robots.

        //Update Idleness Table:
        double now = ros::Time::now().toSec();
                
        for(int i=0; i<dimension; i++){
            if (i == vertex_arrived){
                //actualizar last_visit[dimension]
                last_visit[vertex_arrived] = now; 
		//ROS_INFO("Just updated idleness of vertex %d", i);		
            }         
            //actualizar instantaneous_idleness[dimension]
            instantaneous_idleness[i] = now - last_visit[i];
        }
        
        arrived = false;
    }
    
    ros::spinOnce();
}

int GBS_Agent::compute_next_vertex() {
    return greedy_bayesian_strategy(current_vertex, vertex_web, instantaneous_idleness, G1, G2, edge_min);
}


void GBS_Agent::send_results() {   
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    // [ID,msg_type,vertex]
    std_msgs::Int16MultiArray msg;   
    msg.data.clear();
    msg.data.push_back(value);
    msg.data.push_back(GBS_MSG_TYPE);
    msg.data.push_back(current_vertex);
    do_send_message(msg);
}

void GBS_Agent::receive_results() {
  
    std::vector<int>::const_iterator it = vresults.begin();
    int id_sender = *it; it++;
    int msg_type = *it; it++;
    
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    
  	if ((id_sender==value) || (msg_type!=GBS_MSG_TYPE)) 
    	return;
        
    robot_arrived = vresults[0];
    vertex_arrived = vresults[2];
    arrived = true;
}

void GBS_Agent::ROS_resultsCB(const patrolling_sim::GBS_Message::ConstPtr& msg) { 
     
    receive_results(msg);
    ros::spinOnce();
  
}

void GBS_Agent::MQTT_resultsCB(const patrolling_sim::MQTT_Message::ConstPtr& msg) { 
     
    MQTT_handler(msg);
    ros::spinOnce();  
}


void GBS_Agent::receive_results(const patrolling_sim::GBS_Message::ConstPtr& msg) {
    // int16 sender_ID
    // int16 vertex

    int id_sender = msg->sender_ID;
    int value = ID_ROBOT;
    if (value==-1){
        value=0;
    }
    if (id_sender==value){
        return;
    }
    robot_arrived = msg->sender_ID;
    vertex_arrived = msg->vertex;
    arrived = true;

    printf("Robot %d processed message from robot %d\n", ID_ROBOT, id_sender); 
}

void GBS_Agent::receive_results(patrolling_sim::GBS_Message* msg) {
    // int16 sender_ID
    // int16 vertex

    int id_sender = msg->sender_ID;
    int value = ID_ROBOT;
    if (value==-1){
        value=0;
    }
    if (id_sender==value){
        return;
    }
    robot_arrived = msg->sender_ID;
    vertex_arrived = msg->vertex;
    arrived = true;

    printf("MQTT: Robot %d processed message from robot %d\n", ID_ROBOT, id_sender); 
}

void GBS_Agent::MQTT_handler(const patrolling_sim::MQTT_Message::ConstPtr& msg) {

    patrolling_sim::GBS_Message data = msg->GBS_Message;
    receive_results(&data);
}



void GBS_Agent::do_send_MQTT_message(){
    // int16 sender_ID
    // int16 message_Type
    // int16 protocol_Type
    // int16 service_Level
    // int16 packet_ID
    // GBS_Message  GBS_Message

    //check we are using MQTT
    //otherwise ROStopics won't be mapped to correct topic name
    if (MQTT_mode){
        int value = ID_ROBOT;
        if (value==-1){value=0;}
        // [ID,msg_type,vertex]
        patrolling_sim::GBS_Message msg;
        msg.sender_ID = value;
        msg.vertex    = current_vertex;

        //add GBS message to MQTT packet
        //and set values
        patrolling_sim::MQTT_Message mqtt_msg;
        //who is the original sender
        //so broker know who is subscibing to this robot
        //and knows how to communicate for QoS 1 and 2
        mqtt_msg.sender_ID = value;
        //sets the MQTT service level for the message
        mqtt_msg.service_Level = 0;
        //lets other robots know what message type is contained
        mqtt_msg.message_Type = GBS_MESSAGE;
        //lets MQTT broker know this is an unhandled packet
        mqtt_msg.packet_ID = -1;
        mqtt_msg.protocol_Type = -1;

        gbs_results_pub.publish(mqtt_msg);
        ros::spinOnce();
    } else {
        ROS_WARN("Tried to send MQTT message but MQTT param set to false");
        return;
    }

}

void GBS_Agent::do_send_ROS_message() {
    int value = ID_ROBOT;
    if (value==-1){value=0;}
    // [ID,msg_type,vertex]
    patrolling_sim::GBS_Message msg;
    msg.sender_ID = value;
    msg.vertex    = current_vertex;

    gbs_results_pub.publish(msg);
    ros::spinOnce();
}

int main(int argc, char** argv) {

    GBS_Agent agent;
    agent.init(argc,argv);    
    agent.run();

    return 0; 
}
