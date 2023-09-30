#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import String

def talker():
    pub = rospy.Publisher('topic_name', String, queue_size=10)
    rospy.init_node('publisher_name', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        string_message = "Local time: %s" % rospy.get_time()
        rospy.loginfo(string_message)
        pub.publish(string_message)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

