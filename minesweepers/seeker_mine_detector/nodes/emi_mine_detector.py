#!/usr/bin/env python2
"""
Listens for EMI raw signals on UAVCAN, then processes it to determine if we see
a mine or not and decide to send a message it over ROS
"""

import sys

import rospy
import uavcan

from seeker_msgs.msg import MineInfo


def main():
    if len(sys.argv) < 3:
        print("usage: emi_mine_detector.py uavcan_dsdl_path can_interface")
        return

    dsdl_path, can_interface = sys.argv[1], sys.argv[2]

    rospy.init_node('emi_mine_detector')
    mine_detection_pub = rospy.Publisher('metal_mine', MineInfo, queue_size=1)

    node = uavcan.make_node(can_interface)
    uavcan.load_dsdl(dsdl_path)

    def on_emi_signal_cb(event):
        rospy.loginfo('{}'.format(event.message))

    handle = node.add_handler(uavcan.thirdparty.cvra.metal_detector.EMIRawSignal,
                              on_emi_signal_cb)

    node.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
