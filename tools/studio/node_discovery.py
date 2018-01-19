import threading
import time
import uavcan

class UavcanNode:
    def __init__(self, interface, node_id):
        self.node = uavcan.make_node(interface, node_id=node_id)
        self.handlers = []

    def add_handler(self, topic, callback):
        self.handlers.append(self.node.add_handler(topic, callback))

    def spin(self):
        threading.Thread(target=self._uavcan_thread).start()

    def _uavcan_thread(self):
        while True:
            try:
                self.node.spin(0.1)
                time.sleep(0.01)
            except uavcan.UAVCANException as ex:
                print('Node error:', ex)
                self._uavcan_exit()
                return

    def _uavcan_exit(self):
        for handler in self.handlers:
            handler.remove()

def main():
    node = UavcanNode(interface="slcan0", node_id=127)

    def node_status_callback(event):
        print('NodeStatus message from node', event.transfer.source_node_id)
        print('Node uptime:', event.message.uptime_sec, 'seconds')
        print(uavcan.to_yaml(event))
        print('')

    node.add_handler(uavcan.protocol.NodeStatus, node_status_callback)
    node.spin()

if __name__ == '__main__':
    main()
