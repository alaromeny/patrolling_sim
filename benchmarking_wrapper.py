import roslaunch
import rospy

rospy.init_node('benchmarking_wrapper', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)
launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/siobhan/PhDSims/patrolling_benchmarking/src/patrolling_sim/launch/map.launch"])



launch.start()
rospy.loginfo("started")

rospy.sleep(3)
# 3 seconds later
launch.shutdown()
rate = rospy.Rate(1) # 10hz
while not rospy.is_shutdown():
    rospy.loginfo('stage launch')
    rate.sleep()