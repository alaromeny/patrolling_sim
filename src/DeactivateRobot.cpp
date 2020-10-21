#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>


 typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

 using namespace std;
  
 ros::NodeHandle* n_ptr;
 int teamsize;
 double last_cmd_vel_time;
 
 
 void cmd_velCB(const geometry_msgs::Twist::ConstPtr& msg){
     //ROS_INFO("receiving cmd_vels");
     last_cmd_vel_time = ros::Time::now().toSec();
 }
 

 bool DeactivateRobotCallback(patrolling_sim::DeactivateRobotSrv::Request& Req, patrolling_sim::DeactivateRobotSrv::Response& Rep){
        
    if (Req.deactivation_time.data < 1) {
        ROS_INFO("Service was called with an invalid sleep time: (%d). Leaving.", Req.teamsize.data);
        return false;
    }

    ROS_INFO("Deactivate Service Called.");



    //array of pointers:
    // MoveBaseClient *ac_ptr[teamsize];
    // ros::Rate loop_rate(1); //1 sec
    
    // for (j=teamsize-1; j>=0; j--){
        
    //     char move_string[20];
    //     sprintf(move_string,"robot_%d/move_base",j);
    //     //ROS_INFO("%s",move_string);

        
    //     MoveBaseClient ac(move_string, true);
    //     ac_ptr[j] = &ac;
        
    //     //wait for the action server to come up
    //     while(!ac.waitForServer(ros::Duration(5.0))){
    //         ROS_INFO("Waiting for the move_base action server to come up");
    //     } 
    //     ROS_INFO("Connected with move_base action server");     
        
    //     move_base_msgs::MoveBaseGoal goal;

    //     //wait a bit and send next goal to other robots.
    //     // i=0;        
    //     // while( i<Req.sleep_between_goals.data ){
    //     //     i++;
    //     //     ros::spinOnce();    //trigger cmd_vel callbacks
    //     //     loop_rate.sleep(); 
    //     // }
    // }
    
    return true;
 }
 
 
int main(int argc, char **argv){

  ros::init(argc, argv, "DeactivateRobot");
  ros::NodeHandle n;
  n_ptr = &n;
  
  ros::Duration(2.0).sleep();   //waiting for all navigation topics to show up.
   
  //get cmd_vel topics:
  // ros::master::V_TopicInfo master_topics;
  // ros::master::getTopics(master_topics);
  
  // vector<string> cmd_vel_topic_array(32, ""); 
  // teamsize=0;

  // for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++) {
  //       const ros::master::TopicInfo& info = *it;
  //       if(info.datatype=="geometry_msgs/Twist"){   //cmd_vel topics
  //           cmd_vel_topic_array[teamsize] = info.name;
  //           teamsize++;
  //       }
  // }
  
  //for(int o=0; o<teamsize; o++){ ROS_ERROR("%s",cmd_vel_topic_array[o].c_str()); }
  
  // if (teamsize==0){
  //   ROS_ERROR("No navigation information retrieved. Is \"move_base\" running?");
  //   return -1;   
    
  // }else{
  //     ROS_INFO("Detected %d robots.", teamsize);
  // }
  
  // vector<ros::Subscriber> cmd_vel_sub(teamsize);
  
  // for(int o=0; o<teamsize; o++){ //create subscribers with a common callback:
  //     cmd_vel_sub[o] = n.subscribe(cmd_vel_topic_array[o], 1, cmd_velCB);
  // }  

  // ros::ServiceServer service = n.advertiseService("DeactivateRobotSrv", DeactivateRobotSrvCallback);
  // ROS_WARN("Ready to Deactivate robots during experiment.");
  
  ros::spin();

  return 0;
}