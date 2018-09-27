#!/usr/bin/env python2
"""
Listens for UWB position on UAVCAN and forwards it on ROS
"""

import sys

import uavcan
import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped

def uwb_position_cb(event):
    br = tf2_ros.TransformBroadcaster()
    tf = TransformStamped()

    x = event.message.x
    y = event.message.y

    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = 'world'
    tf.child_frame_id = 'uwb_link'
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = 0.0

    rospy.loginfo('x: {} y: {}'.format(x, y))
    br.sendTransform(tf)

def main():
    if len(sys.argv) < 3:
        print("usage: position_publisher.py uavcan_dsdl_path can_interface")
        return

    dsdl_path, can_interface = sys.argv[1], sys.argv[2]

    rospy.init_node('position_publisher')

    node = uavcan.make_node(can_interface)
    uavcan.load_dsdl(dsdl_path)

    handle = node.add_handler(uavcan.thirdparty.cvra.uwb_beacon.TagPosition,
                              uwb_position_cb)

    node.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
