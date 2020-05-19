import uavcan

uavcan.load_dsdl('../uavcan_data_types/cvra')

node = uavcan.make_node('/dev/tty.usbmodem3031', node_id=123)

led_status = True

board_id = 1

def led_command_reply_received(event):
    print("reponse data", event.response.data)

while True:
    try:
        node.spin(1)
        node.request(uavcan.thirdparty.cvra.io.LEDCommand.Request(led_status=led_status), board_id, led_command_reply_received)
        led_status = not led_status
    except uavcan.UAVCANException as ex:
        print('Node error:', ex)
