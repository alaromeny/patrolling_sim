#!/usr/bin/env python2
# license removed for brevity
import rospy
import sys
import random
from std_msgs.msg import String
from patrolling_sim.msg import MQTT_Message







class MQTT_Broker:
    
    def __init__(self, message_delay, loss_rate):
        self.pubList = []
        self.subList = []
        self.packet_ID_counter = 0
        self.algorithm_name = ''
        self.N_robots = 0
        self.dt = rospy.Duration(secs=message_delay)
        self.loss_rate = loss_rate #this is in percent
        self.newPacketsQueue = []

    def callBack(self, data):
        print("CALLBACK")
        print(data.sender_ID)
        self.MQTT_handler(data)

    def MQTT_handler(self, data):
        # print(data)
        if data.packet_ID == -1:
            self.packet_ID_counter
            print("Unhandled Packet")
            data.packet_ID = self.packet_ID_counter
            now = rospy.get_rostime()
            self.packet_ID_counter = self.packet_ID_counter +1
            queueItem = (now, data)
            self.newPacketsQueue.append(queueItem)
            print(len(self.newPacketsQueue))
            QoS = data.service_Level
            if QoS == 1:
                print("Need to send a PUBACK")

    def publishPackets(self,packet):
        sender_ID = packet.sender_ID
        print(packet)
        loss = random.randrange(100)
        if loss <= self.loss_rate:
            print('Kill Packet')
            return
        else:
            for i in range(0,self.N_robots):
                if i != sender_ID:
                    print("Sending Message to " + str(i))
                    self.pubList[i].publish(packet)


    def forwardPackets(self):
        print("Ready to forward messages")
        listSize = len(self.newPacketsQueue)
        print(listSize)

        while listSize > 0:
            print("Messages are Queued")
            print(listSize)
            item = self.newPacketsQueue[0]
            now = rospy.get_rostime()
            timeDiff =  now - item[0]
            if timeDiff > self.dt:
                packet = self.newPacketsQueue.pop(0)
                print(packet)
                self.publishPackets(packet[1])
                listSize = len(self.newPacketsQueue)
            else:
                break
            # print(self.newPacketsQueue.pop(0))
            # print(len(self.newPacketsQueue))
    #     # for i in range(0, len(self.newPacketsQueue)):
    #     #     packet = self.newPacketsQueue[i]
    #     #     print('Publishing Packet: ' + str(i) + ' of ' + str(len(self.newPacketsQueue)))
    #     #     print(packet)
    #     #     avoidRobot = packet.sender_ID
    #     #     for j in range(0, self.N_robots):
    #     #         if j != avoidRobot:
    #     #             self.pubList[j].publish(packet)



    def createTopics(self, N_robots, ALG_SHORT):
        # create subs and pubs for each robot
        print("Parameters are: " + str(N_robots) + " " + ALG_SHORT)
        for i in range(0,N_robots):
            # /MQTT/robot'+str(i)+'/'+ALG_SHORT+'_results_IN '
            topicName = '/MQTT/robot'+str(i)+'/'+ALG_SHORT+'_results'
            # names are relative to robot, which is why it's backwards for Broker
            pub_name = topicName + '_IN'
            sub_name = topicName + '_OUT'
            self.pubList.append(rospy.Publisher( pub_name, MQTT_Message, queue_size=10))
            self.subList.append(rospy.Subscriber(sub_name, MQTT_Message, self.callBack))
            print("Created topics named: " + pub_name)
            print("Created topics named: " + sub_name)
            # def MQTT_handler(data):
            #     #  uint message_Type = msg->message_Type;
            #     # uint protocol_Type = msg->protocol_Type;
            #     # uint packet_ID = msg->packet_ID;
            #     # uint QoS_level = msg->service_Level;
            #     # uint sender_ID = msg->sender_ID;
            #     # //this is an unhandled packet
            #     # if(packet_ID==-1){
            #     #     patrolling_sim::MQTT_Message packetCreated;
            #     #     //add packet_ID
            #     #     packetCreated.packet_ID = packet_ID_counter;
            #     #     packet_ID_counter++;
            #     # 

def main(args):
    #first one is delay, set to low
    #second is loss rate, set to -1 to turn it off
    MB = MQTT_Broker(0.1, -1)
    rospy.init_node('MQTT_Broker', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    MB.algorithm_name = rospy.get_param("/algorithm")
    MB.N_robots = rospy.get_param("/number_of_robots")
    print("Number of robots is " + str(MB.N_robots))
    MB.createTopics(MB.N_robots, MB.algorithm_name)
    try:
        while not rospy.is_shutdown():
            MB.forwardPackets()
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)