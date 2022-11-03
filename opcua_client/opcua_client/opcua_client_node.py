# ROS Client Library for Python
import rclpy

# Import for Node Creation
from rclpy.node import Node

# Import for Message types used for Action and Feedbacks
from opcua_interfaces.msg import ServerAction, AmrStatus, TmStatus

import sys
import time
sys.path.insert(0, "..")

# Import from OPCUA Library
from opcua import Client, ua

class Opcua_Client(Node):
    """
        OPCUA_Client Class: Subclass of Node
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Node Class's Constructor Initialization
        super().__init__('opcua_client')

        while True:
            try:
                self.client_ = Client("opc.tcp://admin@localhost:4840/freeopcua/server/")
                self.client_.connect()
                print("Connected to 5G OPCUA Server...... ")
                break
            except:
                time.sleep(1.0)
                print("OPCUA Server Not Available ... ")


        # Publisher Initialization: For OPCUA Server Action Request
        # Ex: AMR action to goto a position, AMR Emergency Stop
        self.action_publisher_ = self.create_publisher(ServerAction, 'opcua_action', 10)

        # Subscriber Initialization: For Receiving AMR Status Information
        self.amr_subscriber_ = self.create_subscription(
            AmrStatus,
            'amr_fdbk',
            self.amr_feedback_to_opcua,
            10)
        self.amr_subscriber_

        # # Subscriber Initialization: For Receiving TM12(Manipulator) Status Information
        # self.tm_subscriber_ = self.create_subscription(
        #     TmStatus,
        #     'tm_fdbk',
        #     self.tm_feedback_to_opcua,
        #     10)
        # self.tm_subscriber_

        # Frequency to Publish action_publisher_
        timer_period = 0.5  # seconds

        # Timer to run the Publisher based on timer_period
        self.get_action_timer_ = self.create_timer(timer_period, self.get_action)

        # Variable Initialization for Message type to be Published
        self.action_msg_ = ServerAction()



    def ua_get_root_node(self):
        """
        Function To Retrieve Root Node information from OPCUA Address Space
        """
        self.ua_node = self.client_.get_root_node()

    def ua_set_value(self, address_variable, variant_type, value):
        """
        Function To Set/Update Root Node information to OPCUA Address Space
        """
        ua_variable = self.ua_node.get_child(["0:Objects", "2:Variables", address_variable])
        ua_variable.set_value(ua.DataValue(ua.Variant(value, variant_type)))

    def ua_get_value(self, address_variable):
        """
        Function To Retrieve Specific Variable Value from OPCUA Address Space
        """
        return self.ua_node.get_child(["0:Objects", "2:Variables", address_variable]).get_value()

    def amr_feedback_to_opcua(self, amr_fdbk_):
        """
        Function To Send AMR Updated Status Feedback Information to OPCUA Address Space
        """
        self.ua_get_root_node()

        # AMR status set
        self.ua_set_value("2:bGet_AMR_status", ua.VariantType.Boolean, amr_fdbk_.status)

        # Set Battery Percentage Data
        self.ua_set_value("2:fGet_AMR_battery_life", ua.VariantType.Float, amr_fdbk_.battery_perc)

        # Set AMR Positions Data
        self.ua_set_value("2:fGet_AMR_pos_x", ua.VariantType.Float, amr_fdbk_.position[0])
        self.ua_set_value("2:fGet_AMR_pos_y", ua.VariantType.Float, amr_fdbk_.position[1])
        self.ua_set_value("2:fGet_AMR_pos_theta", ua.VariantType.Float, amr_fdbk_.position[2])

        # # Set Inertial Measurement Unit Data
        self.ua_set_value("2:fGet_AMR_imu_orient_x", ua.VariantType.Float, amr_fdbk_.imu[0])
        self.ua_set_value("2:fGet_AMR_imu_orient_y", ua.VariantType.Float, amr_fdbk_.imu[1])
        self.ua_set_value("2:fGet_AMR_imu_orient_z", ua.VariantType.Float, amr_fdbk_.imu[2])
        self.ua_set_value("2:fGet_AMR_imu_orient_w", ua.VariantType.Float, amr_fdbk_.imu[3])

        self.ua_set_value("2:fGet_AMR_imu_ang_vel_x", ua.VariantType.Float, amr_fdbk_.imu[4])
        self.ua_set_value("2:fGet_AMR_imu_ang_vel_y", ua.VariantType.Float, amr_fdbk_.imu[5])
        self.ua_set_value("2:fGet_AMR_imu_ang_vel_z", ua.VariantType.Float, amr_fdbk_.imu[6])

        self.ua_set_value("2:fGet_AMR_imu_lin_acc_x", ua.VariantType.Float, amr_fdbk_.imu[7])
        self.ua_set_value("2:fGet_AMR_imu_lin_acc_y", ua.VariantType.Float, amr_fdbk_.imu[8])
        self.ua_set_value("2:fGet_AMR_imu_lin_acc_z", ua.VariantType.Float, amr_fdbk_.imu[9])


        # Set Odometry Data
        self.ua_set_value("2:fGet_AMR_odom_pose_lin_x", ua.VariantType.Float, amr_fdbk_.odom[0])
        self.ua_set_value("2:fGet_AMR_odom_pose_lin_y", ua.VariantType.Float, amr_fdbk_.odom[1])
        self.ua_set_value("2:fGet_AMR_odom_pose_lin_z", ua.VariantType.Float, amr_fdbk_.odom[2])

        self.ua_set_value("2:fGet_AMR_odom_twist_orien_x", ua.VariantType.Float, amr_fdbk_.odom[3])
        self.ua_set_value("2:fGet_AMR_odom_twist_orien_y", ua.VariantType.Float, amr_fdbk_.odom[4])
        self.ua_set_value("2:fGet_AMR_odom_twist_ang_z", ua.VariantType.Float, amr_fdbk_.odom[5])

    # def tm_feedback_to_opcua(self, tm_fdbk_):
    #     """
    #     Function To Send TM12M Updated Status Feedback Information to OPCUA Address Space
    #     """
    #     self.ua_get_root_node()
    #
    #     # Set Pick and Place Status
    #     self.ua_set_value("2:bPick_cmm_st", ua.VariantType.Boolean, tm_fdbk_.pick)
    #     self.ua_set_value("2:bPlace_cmm_st", ua.VariantType.Boolean, tm_fdbk_.place)



    def get_action(self):
        """
        Function To Receive Updated AMR and TM Action Request from OPCUA Address Space
        """
        self.ua_get_root_node()

        # Required AMR action status
        # print(self.ua_get_value("2:sSet_send_action_to_AMR"))
        self.action_msg_.action_request = self.ua_get_value("2:sSet_send_action_to_AMR")

        # Required AMR E-Stop Safety
        self.action_msg_.estop = self.ua_get_value("2:bSet_AMR_e_stop")

        # Required AMR Go To Position
        self.action_msg_.goto_pos[0] = self.ua_get_value("2:fSet_AMR_pos_x")
        self.action_msg_.goto_pos[1] = self.ua_get_value("2:fSet_AMR_pos_y")
        self.action_msg_.goto_pos[2] = self.ua_get_value("2:fSet_AMR_pos_theta")

        # Required AMR Pause/Ready Action
        self.action_msg_.pause_amr = self.ua_get_value("2:bSet_pause_AMR")
        self.action_msg_.ready_amr = self.ua_get_value("2:bSet_ready_AMR")

        # Required TM Pick/Place Action
        self.action_msg_.pick_operation = self.ua_get_value("2:bPick_cmm_st")
        self.action_msg_.place_operation = self.ua_get_value("2:bPlace_cmm_st")

        self.action_publisher_.publish(self.action_msg_)

def main(args=None):

    # Rclpy library Initialization
    rclpy.init(args=args)

    # Node creation for OPCUA client
    opcua_client_node = Opcua_Client()

    # Node Spin to keep client alive and communicating with OPCUA Server
    rclpy.spin(opcua_client_node)

    # Node Destroy to free Resources used by Node
    opcua_client_node.destroy_node()

    # ROS Client Library Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
