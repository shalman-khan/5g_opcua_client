# ROS Client Library for Python
import rclpy

# Import for Node Creation
from rclpy.node import Node

# Import for Message types used for Action and Feedbacks
from opcua_interfaces.msg import ServerAction,AmrStatus, TmStatus


class AMR_Sim_Node(Node):
    """
        AMR_Sim_Node Class: Subclass of Node
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Node Class's Constructor Initialization
        super().__init__('amr_ros2')

        # Publisher Initialization: For sending AMR Status to OPCUA
        # Ex: AMR current Position, AMR IMU information
        self.amr_publisher_ = self.create_publisher(AmrStatus, 'amr_fdbk', 10)

        # Subscriber Initialization: For Receiving OPCUA Action Requests
        # For Example: Server's request to go to a position or execute E stop
        self.amr_subscriber_ = self.create_subscription(
            ServerAction,
            'opcua_action',
            self.amr_action_from_opcua,
            10)
        self.amr_subscriber_

        # Frequency to Publish amr status
        timer_period = 0.5  # seconds

        # Timer to run the Publisher based on timer_period
        self.amr_status_timer = self.create_timer(timer_period, self.amr_feedback_to_opcua)

        # Variable Initialization for Message type to be Published
        self.amr_fdbk_ = AmrStatus()


    def amr_feedback_to_opcua(self):
        """
            Dummy Function to Publish AMR Current Status information
            Replace the variables values based on the AMR API
        """

        self.amr_fdbk_.status = True
        self.amr_fdbk_.battery_perc = 99.98

        self.amr_fdbk_.position[0] = 100.2
        self.amr_fdbk_.position[1] = 200.4
        self.amr_fdbk_.position[2] = 90.0

        self.amr_fdbk_.imu[0] = 1.2
        self.amr_fdbk_.imu[1] = 1.2
        self.amr_fdbk_.imu[2] = 1.2
        self.amr_fdbk_.imu[3] = 1.2

        self.amr_fdbk_.imu[4] = 1.2
        self.amr_fdbk_.imu[5] = 1.2
        self.amr_fdbk_.imu[6] = 1.2

        self.amr_fdbk_.imu[7] = 1.2
        self.amr_fdbk_.imu[8] = 1.2
        self.amr_fdbk_.imu[9] = 1.2

        self.amr_fdbk_.odom[1] = 7.8
        self.amr_fdbk_.odom[0] = 7.8
        self.amr_fdbk_.odom[2] = 7.8

        self.amr_fdbk_.odom[3] = 7.8
        self.amr_fdbk_.odom[4] = 7.8
        self.amr_fdbk_.odom[5] = 7.8


        self.amr_publisher_.publish(self.amr_fdbk_)

    def amr_action_from_opcua(self, amr_action):
        """
            Dummy Function to Retrieve Server Request
            Transfer Retrieved Variable Values for Further Operations
        """
        print("OPCUA AMR Action Request", amr_action.action_request)
        print("OPCUA AMR E-Stop Request", amr_action.estop)
        print("OPCUA AMR Position Request", amr_action.goto_pos)
        print("OPCUA AMR Pause Request", amr_action.pause_amr)
        print("OPCUA AMR Ready Request", amr_action.ready_amr)
        print("----------------------------------------------")


def main(args=None):

    # Rclpy library Initialization
    rclpy.init(args=args)

    # Node creation for AMR Status information simulation
    AMR_node = AMR_Sim_Node()

    # Node Spin to AMR Status Update Alive
    rclpy.spin(AMR_node)

    # Node Destroy to free Resources used by Node
    AMR_node.destroy_node()

    # ROS Client Library Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
