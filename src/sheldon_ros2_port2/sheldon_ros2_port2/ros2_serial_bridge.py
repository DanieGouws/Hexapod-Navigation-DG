import rclpy
from rclpy.node import Node
from rosidl_runtime_py import message_to_ordereddict
import json
import serial
from functools import partial

class JSONSerialBridge(Node):
    def __init__(self, serial_port='/dev/ttyACM0', baudrate=115200):
        super().__init__('json_serial_bridge')

        self.ser = serial.Serial(serial_port, baudrate, timeout=1)      # Comment to disable Serial Port Requirement
        self.topics = {
            '/simple_hexapod/Legs_paths': 'my_hexapod_interfaces/msg/LegPath',                           #subscribed topics whic will be passed on to serial.
            '/simple_hexapod/cmd_vel_path_var': 'my_hexapod_interfaces/msg/PathVarNCmdVel'
        }

        for topic, type_str in self.topics.items():
            msg_module = self.import_msg_class(type_str)
            self.create_subscription(msg_module, topic, partial(self.callback, topic, type_str), 10)

        self.get_logger().info("ROS2 Serial bridge initialized to " + serial_port)

    def import_msg_class(self, full_type_str):
        module_path, msg_name = full_type_str.rsplit('/', 1)
        pkg_name, subfolder = module_path.split('/')
        module = __import__(f'{pkg_name}.{subfolder}', fromlist=[msg_name])
        return getattr(module, msg_name)

    def callback(self, topic, type_str, msg):
        msg_dict = message_to_ordereddict(msg)
        packet = {
            'topic': topic,
            'type': type_str.split('/')[-1],
            'msg': msg_dict
        }
        json_str = json.dumps(packet) + '\n'
        self.ser.write(json_str.encode('utf-8'))             # Comment to disable Serial Port Requirement
        self.get_logger().info("Message printed to serial of topic:" + str(topic))
        self.get_logger().info(json_str)     #Print whole packet to terminal for debugging

def main(args=None):
    rclpy.init(args=args)
    bridge = JSONSerialBridge()
    rclpy.spin(bridge)
    bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
