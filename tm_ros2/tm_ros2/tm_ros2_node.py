# ROS Client Library for Python
import rclpy

# Handles the creation of nodes
from rclpy.node import Node

from opcua_interfaces.msg import ServerAction, AmrStatus, TmStatus


class TM_node_class(Node):
    """
        TM_Sim_Node Class: Subclass of Node
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('tm_ros2')
        self.tm_fdbk_ = TmStatus()
        # self.amr_publisher_ = self.create_publisher(TmStatus, 'tm_fdbk', 10)
        self.tm_subscriber_ = self.create_subscription(
            ServerAction,
            'opcua_action',
            self.amr_action_from_opcua,
            10)
        self.tm_subscriber_
        timer_period = 0.5  # seconds

        # self.tm_status_timer_ = self.create_timer(timer_period, self.tm_feedback_to_opcua)

        self.tm_fdbk_ = TmStatus()
        self.amr_state = None
        self.tm_pick_state = False
        self.tm_place_state = False

    # def tm_feedback_to_opcua(self):
    #     """
    #         Dummy Function to Publish TM Current Status information
    #         Checks the AMR action status to set TM Status
    #     """
    #     if self.amr_state == "Idle":
    #         self.tm_fdbk_.pick = True
    #         self.tm_fdbk_.place = True
    #
    #     else:
    #         self.tm_fdbk_.pick = False
    #         self.tm_fdbk_.place = False
    #
    #     self.amr_publisher_.publish(self.tm_fdbk_)

    def amr_action_from_opcua(self, amr_action):
        """
            Dummy Function to Set AMR Status Action Request
        """
        self.amr_state = amr_action.action_request
        self.tm_pick_state = amr_action.pick_operation
        self.tm_place_state = amr_action.place_operation

        if self.amr_state == "Idle":
            print("Server State: IDLE")
            if self.tm_pick_state:
                print("Pick Operation Requested")
            elif self.tm_place_state:
                print("Place Operation Requested")
            elif self.tm_pick_state and self.tm_place_state:
                print("Request Either Pick Operation or Place Operation")

        else:
            if self.tm_pick_state or self.tm_place_state:
                print("Robot in Operation: Request Pick/Place in Idle State")



def main(args=None):

    # Rclpy library Initialization
    rclpy.init(args=args)

    # Node creation for AMR Status information simulation
    TM_node = TM_node_class()

    # Node Spin to TM Status Update Alive
    rclpy.spin(TM_node)

    # Node Destroy to free Resources used by Node
    TM_node.destroy_node()

    # ROS Client Library Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
