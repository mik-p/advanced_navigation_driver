#!/usr/bin/env python

import rospy, random
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('speed', Float32, queue_size=10)
    rospy.init_node('dummy_talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    speed = 4.2
    while not rospy.is_shutdown():
        speed += random.uniform(-0.01, 0.01)
        pub.publish(speed)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
