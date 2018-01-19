import threading
import time
import uavcan

def uavcan_thread(node):
    while True:
        try:
            node.spin(0.1)
            time.sleep(0.01)
        except uavcan.UAVCANException as ex:
            print('Node error:', ex)
            return

def main():
    node = uavcan.make_node("slcan0", node_id=127)
    threading.Thread(target=uavcan_thread, args=(node,)).start()

if __name__ == '__main__':
    main()
