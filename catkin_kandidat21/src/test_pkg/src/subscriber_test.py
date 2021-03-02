#! /usr/bin/env python
import rospy
from std_msgs.msg import Int32

global counter
counter = 0


def callback(msg):
    if (msg.data % 5 == 0):
        global counter
        print('counter is', msg.data, 'this subscriber has done', counter,'operations!!')
        counter += 1



def main():
    rospy.init_node('topic_subscriber')
    sub = rospy.Subscriber("/counter",Int32, callback)
    rate = rospy.Rate(2)
    count = Int32();
    count.data = 0

    rospy.spin()


if __name__ == "__main__":
    main()