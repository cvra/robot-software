#!/usr/bin/env python2
"""
Watches the mine topic and displays a visible warning if something triggers.
"""
import rospy
import uavcan
import threading

import argparse
from seeker_msgs.msg import MineInfo
from subprocess import Popen

LED_DURATION = 4
ALARM_FILE = "/home/cvra/alarm.ogg"


class LedSignalApplication:
    def __init__(self, node, ros_topic_name):
        self.node = node
        self.led_board_id = None
        self.led_voltage = 0
        self.mplayer_process = None

        # UAVCAN callbacks
        self.node.periodic(0.1, self._publish_led_command)
        self.node.add_handler(uavcan.protocol.NodeStatus, self._node_status_callback)

        # ROS callbacks
        rospy.Subscriber(ros_topic_name, MineInfo, self._mine_detected_callback)

    def _publish_led_command(self):
        # Haven't discovered the board yet
        if self.led_board_id is None:
            return
        msg = uavcan.thirdparty.cvra.motor.control.Voltage(node_id=self.led_board_id,
                                                           voltage=self.led_voltage)
        self.node.broadcast(msg)


    def _node_status_callback(self, event):
        """
        Callback called when periodic node info is received, for node discovery.
        """,
        src_id = event.transfer.source_node_id
        rospy.logdebug('Got a heartbeat from node %s', src_id)

        if self.led_board_id is None:
            self.node.request(uavcan.protocol.GetNodeInfo.Request(), src_id,
                              self._board_info_callback)

    def _board_info_callback(self, event):
        board = event.transfer.source_node_id
        name = str(event.response.name)

        if name == 'signal_led':
            self.led_board_id = board
            rospy.loginfo("Discovered LED board {}".format(self.led_board_id))

    def _mine_detected_callback(self, data):
        rospy.loginfo('Got a mine: %s', data)

        # Start a led
        self.led_voltage = 12

        def shutdown():
            self.led_voltage = 0

        threading.Timer(LED_DURATION, shutdown).start()

        # If the sound was never played or the sound has finished playing
        if self.mplayer_process is None or \
            self.mplayer_process.poll() is not None:
            self.mplayer_process = Popen(['mplayer', ALARM_FILE])


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("topic", help="Mine information topic")
    parser.add_argument("dsdl_path", help="Path to UAVCAN DSDL messages.")
    parser.add_argument("iface", help="CAN interface.")
    parser.add_argument("--uavcan-id", type=int, default=123, help="UAVCAN node id for this app")

    return parser.parse_known_args()

def main():
    args, _ = parse_args()

    node = uavcan.make_node(args.iface, node_id=args.uavcan_id)
    uavcan.load_dsdl(args.dsdl_path)

    rospy.init_node('mine_alarm_signal')

    app = LedSignalApplication(node, args.topic)

    # Finally, start uavcan and ROS
    threading.Thread(target=node.spin).start()
    rospy.spin()


if __name__ == '__main__':
    main()
