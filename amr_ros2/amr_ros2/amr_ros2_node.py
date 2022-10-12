# ROS Client Library for Python
import rclpy

# Handles the creation of nodes
from rclpy.node import Node

from opcua_interfaces.msg import ServerAction,AmrStatus, TmStatus


class AMR_node_class(Node):
    """
    Create a Opcua_Client class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('amr_ros2')
        self.amr_fdbk = AmrStatus()
        self.amr_publisher_ = self.create_publisher(AmrStatus, 'amr_fdbk', 10)
        self.amr_subscriber = self.create_subscription(
            ServerAction,
            'opcua_action',
            self.amr_action_from_opcua,
            10)
        self.amr_subscriber
        timer_period = 0.5  # seconds

        self.amr_status_timer = self.create_timer(timer_period, self.amr_feedback_to_opcua)

        self.amr_status_msg = AmrStatus()

    def amr_feedback_to_opcua(self):

        self.amr_fdbk.status = True
        self.amr_fdbk.battery_perc = 99.98

        self.amr_fdbk.position[0] = 100.2
        self.amr_fdbk.position[1] = 200.4
        self.amr_fdbk.position[2] = 90.0

        self.amr_fdbk.imu[0] = 1.2
        self.amr_fdbk.imu[1] = 1.2
        self.amr_fdbk.imu[2] = 1.2
        self.amr_fdbk.imu[3] = 1.2

        self.amr_fdbk.imu[4] = 1.2
        self.amr_fdbk.imu[5] = 1.2
        self.amr_fdbk.imu[6] = 1.2

        self.amr_fdbk.imu[7] = 1.2
        self.amr_fdbk.imu[8] = 1.2
        self.amr_fdbk.imu[9] = 1.2

        self.amr_fdbk.odom[1] = 7.8
        self.amr_fdbk.odom[0] = 7.8
        self.amr_fdbk.odom[2] = 7.8

        self.amr_fdbk.odom[3] = 7.8
        self.amr_fdbk.odom[4] = 7.8
        self.amr_fdbk.odom[5] = 7.8


        self.amr_publisher_.publish(self.amr_fdbk)

    def amr_action_from_opcua(self, amr_action):
        print("OPCUA AMR Action Request", amr_action.action_request)
        print("OPCUA AMR E-Stop Request", amr_action.estop)
        print("OPCUA AMR Position Request", amr_action.goto_pos)
        print("OPCUA AMR Pause Request", amr_action.pause_amr)
        print("OPCUA AMR Ready Request", amr_action.ready_amr)
        print("----------------------------------------------")


def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    AMR_node = AMR_node_class()

    # Spin the node so the callback function is called.
    rclpy.spin(AMR_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    AMR_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
