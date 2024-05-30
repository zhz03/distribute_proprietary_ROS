#!/usr/bin/env python

import rospy
from std_msgs.msg import String


def topic_callback(msg):
    print(msg)


if __name__ == '__main__':
    # initialize node
    rospy.init_node('subscriber_node')

    # subscriber subscribe to topic
    topic_name = '/hello/uclamobility'
    # data_class: String
    # callback function to get data
    subscriber = rospy.Subscriber(topic_name, String, topic_callback)

    rospy.spin()
