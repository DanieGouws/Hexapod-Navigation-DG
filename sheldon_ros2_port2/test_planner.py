import rclpy
from rclpy.node import Node
from time import sleep

from sheldon_ros2_port2.PlannerV1 import Planner  # Your converted ROS2 Planner
from sheldon_ros2_port2.hexapodC import HexapodC  # Your converted ROS2 HexapodC
from my_hexapod_interfaces.msg import PathVarNCmdVel

def main():
    rclpy.init()
    node = Node("test_planner_node")

    # Instantiate the hexapod command sender
    command_hexapod = HexapodC(node=node, ns="/simple_hexapod/")
    
    # Instantiate the planner that reacts to messages
    planner = Planner(robot=command_hexapod)

    # Give it a moment for all publishers/subscribers to connect
    sleep(1)

    # Set some command velocity and path parameters
    while(1):
        command_hexapod.set_walk_velocity(x=0.02, y=0.00, t=0.0)
        sleep(5)
        command_hexapod.set_walk_velocity(x=0.00, y=0.02, t=0.0)
        sleep(5)
        command_hexapod.set_walk_velocity(x=-0.02, y=0.00, t=0.0)
        sleep(5)
        command_hexapod.set_walk_velocity(x=0.00, y=-0.02, t=0.0)
        sleep(5)
    # command_hexapod.set_walk_velocity(x=0.0, y=0.00, t=9.0)
    # sleep(20)
    # command_hexapod.set_walk_velocity(x=0.00, y=0.00, t=0.0)
    # # fv

    

    
    

    try:
        # Spin the node to allow callbacks to fire
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down test.")
    finally:
        planner.stop()
        node.destroy_node()
        rclpy.shutdown()

def set_walk_velocity(self, x, y, t):
    HexapodC.x = x
    HexapodC.y = y
    HexapodC.t = t

    msg = PathVarNCmdVel()
    msg.linear.x = x
    msg.linear.y = y
    msg.angular.z = t

    msg.path_var.bh = HexapodC.BH
    msg.path_var.ss = HexapodC.Ss
    msg.path_var.sh = HexapodC.Sh
    msg.path_var.fh = HexapodC.Fh
    msg.path_var.rd = HexapodC.Rd
    msg.path_var.p = HexapodC.p
    msg.path_var.r = HexapodC.r

    self._pub_cmd_vel_path_var.publish(msg)
    self.node.get_logger().info("Published to PathVarNCmdVel")

if __name__ == '__main__':
    main()
