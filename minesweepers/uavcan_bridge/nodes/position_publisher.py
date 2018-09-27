#!/usr/bin/env python3
"""
Listens for UWB position on UAVCAN and forwards it on ROS
"""

import uavcan
import rospy

import tf2_ros
from geometry_msgs.msg import TransformStamped

DSDL_DIR = os.path.join(os.path.dirname(__file__), '../../../uavcan_data_types/cvra')

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
    rospy.init_node('position_publisher')

    node = uavcan.make_node('can0') # TODO: use rosparam
    uavcan.load_dsdl(DSDL_DIR)

    handle = node.add_handler(uavcan.thirdparty.cvra.uwb_beacon.TagPosition,
                              uwb_position_cb)

    while not rospy.is_shutdown():
        node.spin(1.0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
