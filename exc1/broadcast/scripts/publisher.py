#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float32

def talker(n, k):
    pub = rospy.Publisher('hoeppe', Float32, queue_size=10)
    rospy.init_node('nodeA', anonymous=True)
    rate = rospy.Rate(20) 
    
    while not rospy.is_shutdown():
        rospy.loginfo(k)
        pub.publish(k)
        k += n
        rate.sleep()

if __name__ == '__main__':
    try:
        talker(n=4, k=1)
    except rospy.ROSInterruptException:
        pass