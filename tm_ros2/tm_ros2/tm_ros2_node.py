# ROS Client Library for Python
import rclpy

# Handles the creation of nodes
from rclpy.node import Node

from opcua_interfaces.msg import ServerAction, AmrStatus, TmStatus


class TM_node_class(Node):
    """
    Create a Opcua_Client class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('amr_ros2')
        self.tm_fdbk = TmStatus()
        self.amr_publisher_ = self.create_publisher(TmStatus, 'tm_fdbk', 10)
        self.amr_subscriber = self.create_subscription(
            ServerAction,
            'opcua_action',
            self.amr_action_from_opcua,
            10)
        self.amr_subscriber
        timer_period = 0.5  # seconds

        self.tm_status_timer = self.create_timer(timer_period, self.tm_feedback_to_opcua)

        self.tm_fdbk = TmStatus()
        self.amr_state = None

    def tm_feedback_to_opcua(self):

        if self.amr_state == "Idle":
            self.tm_fdbk.pick = True
            self.tm_fdbk.place = True

        else:
            self.tm_fdbk.pick = False
            self.tm_fdbk.place = False

        self.amr_publisher_.publish(self.tm_fdbk)

    def amr_action_from_opcua(self, amr_action):
        print("OPCUA AMR Action Request", amr_action.action_request)
        self.amr_state = amr_action.action_request

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    TM_node = TM_node_class()

    # Spin the node so the callback function is called.
    rclpy.spin(TM_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    TM_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

if __name__ == '__main__':
    main()
