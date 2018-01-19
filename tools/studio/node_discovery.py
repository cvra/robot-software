import threading
import time
import uavcan

class UavcanNode:
    def __init__(self, interface, node_id):
        self.node = uavcan.make_node(interface, node_id=node_id)

    def spin(self):
        threading.Thread(target=self._uavcan_thread).start()

    def _uavcan_thread(self):
        while True:
            try:
                self.node.spin(0.1)
                time.sleep(0.01)
            except uavcan.UAVCANException as ex:
                print('Node error:', ex)
                return

def main():
    node = UavcanNode(interface="slcan0", node_id=127)
    node.spin()

if __name__ == '__main__':
    main()
