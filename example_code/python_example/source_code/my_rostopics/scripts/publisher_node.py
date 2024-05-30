#!/usr/bin/env python
# coding: utf-8

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    # create a node 
    rospy.init_node('publisher_node')

    # topic name, needs to be unique
    # name rules: '/a/b/c/d'
    topic_name = '/hello/uclamobility'
    # data_class: data type
    # using String
    publisher = rospy.Publisher(topic_name, String, queue_size=100)

    count = 0
    print("/hello/uclamobility is publishing, check it in another terminal using 'rostopic list'")
    rate = rospy.Rate(4)
    while not rospy.is_shutdown():
        # send out the data
        msg = String()
        msg.data = 'hello topic {}'.format(count)
        publisher.publish(msg)

        rate.sleep()
        count += 1
