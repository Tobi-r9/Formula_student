#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def divider(data):
    k = data.data
    q = 0.15
    c = k/q
    pub = rospy.Publisher('/kthfs/result', Float32, queue_size=10)
    rospy.loginfo("final publish %f", c)
    pub.publish(c)
     
def listener():

    rospy.init_node('nodeB', anonymous=True)
    rospy.Subscriber("hoeppe", Float32, divider(0.15))
    rospy.spin()

if __name__ == '__main__':
    listener()