import threading
import time
import uavcan

class UavcanNode:
    def __init__(self, interface, node_id):
        self.handlers = []
        self.node_lock = threading.RLock()
        self.node = uavcan.make_node(interface, node_id=node_id)

    def add_handler(self, topic, callback):
        self.handlers.append(self.node.add_handler(topic, callback))

    def request(self, request, node_id, callback):
        with self.node_lock:
            self.node.request(request, node_id, callback)

    def publish(self, msg, priority):
        self.node.broadcast(msg, priority=priority)

    def publish_periodically(self, period, publish_cmd):
        self.node.periodic(period, publish_cmd)

    def spin(self):
        threading.Thread(target=self._uavcan_thread).start()

    def _uavcan_thread(self):
        while True:
            try:
                with self.node_lock:
                    self.node.spin(0.1)
                time.sleep(0.01)
            except uavcan.UAVCANException as ex:
                print('Node error:', ex)
                # self._uavcan_exit()
                # return

    def _uavcan_exit(self):
        for handler in self.handlers:
            handler.remove()

