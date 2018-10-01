#!/usr/bin/env python2
"""
Listens for UWB position on UAVCAN and forwards it on ROS
"""

import sys

import uavcan
import rospy

import tf2_ros
from geometry_msgs.msg import Point, TransformStamped

def tf_from_position(x, y, z):
    tf = TransformStamped()

    tf.header.stamp = rospy.Time.now()
    tf.header.frame_id = 'world'
    tf.child_frame_id = 'uwb_link'
    tf.transform.translation.x = x
    tf.transform.translation.y = y
    tf.transform.translation.z = z

    return tf

def main():
    if len(sys.argv) < 3:
        print("usage: position_publisher.py uavcan_dsdl_path can_interface")
        return

    dsdl_path, can_interface = sys.argv[1], sys.argv[2]

    rospy.init_node('position_publisher', disable_signals=True)
    uwb_position_pub = rospy.Publisher('uwb_position', Point, queue_size=1)

    node = uavcan.make_node(can_interface)
    uavcan.load_dsdl(dsdl_path)

    def uwb_position_cb(event):
        x, y = event.message.x, event.message.y

        br = tf2_ros.TransformBroadcaster()
        br.sendTransform(tf_from_position(x, y, 0))

        uwb_position_pub.publish(Point(x, y, 0))

    handle = node.add_handler(uavcan.thirdparty.cvra.uwb_beacon.TagPosition,
                              uwb_position_cb)

    try:
        node.spin()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
