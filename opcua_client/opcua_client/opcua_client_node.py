# ROS Client Library for Python
import rclpy

# Handles the creation of nodes
from rclpy.node import Node

# Enables usage of the String message type
from std_msgs.msg import String
from opcua_interfaces.msg import ServerAction, AmrStatus, TmStatus

import sys
sys.path.insert(0, "..")


from opcua import Client, ua

class Opcua_Client(Node):
    """
    Create a Opcua_Client class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('opcua_client')

        # Create the publisher. This publisher will publish a String message
        # to the addison topic. The queue size is 10 messages.
        self.publisher_ = self.create_publisher(String, 'opcua', 10)
        self.action_publisher_ = self.create_publisher(ServerAction, 'opcua_action', 10)


        self.amr_fdbk_subs = self.create_subscription(
            AmrStatus,
            'amr_fdbk',
            self.amr_feedback_to_opcua,
            10)
        self.amr_fdbk_subs

        self.tm_fdbk_subs = self.create_subscription(
            TmStatus,
            'tm_fdbk',
            self.tm_feedback_to_opcua,
            10)
        self.tm_fdbk_subs

        # We will publish a message every 0.5 seconds
        timer_period = 0.5  # seconds

        # Create the timer
        self.get_action_timer = self.create_timer(timer_period, self.get_action)

        # Initialize a counter variable
        self.i = 0
        self.action_msg = ServerAction()

        self.client = Client("opc.tcp://admin@localhost:4840/freeopcua/server/")
        # try:
        self.client.connect()

        print("Connected to 5G OPCUA Server...... ")

        # finally:
        #     client.disconnect()


    def timer_callback(self):
        """
        Callback function.
        This function gets called every 0.5 seconds.
        """
        # Create a String message
        msg = String()

        # Set the String message's data
        msg.data = 'OPCUA client Running: %d' % self.i

        # Publish the message to the topic
        self.publisher_.publish(msg)

        # Display the message on the console
        self.get_logger().info('Publishing: "%s"' % msg.data)

        # Increment the counter by 1
        self.i += 1

    def action_from_opcua(self):
        """
        Callback function.
        This function gets called every 0.5 seconds.
        """
        # Publish the message to the topic
        self.action_publisher_.publish(self.action_msg)


    def ua_get_root_node(self):
        self.ua_node = self.client.get_root_node()

    def ua_set_value(self, address_variable, variant_type, value):
        ua_variable = self.ua_node.get_child(["0:Objects", "2:Variables", address_variable])
        ua_variable.set_value(ua.DataValue(ua.Variant(value, variant_type)))

    def ua_get_value(self, address_variable):
        return self.ua_node.get_child(["0:Objects", "2:Variables", address_variable]).get_value()

    def amr_feedback_to_opcua(self, amr_fdbk):

        self.ua_get_root_node()

        # AMR status set
        self.ua_set_value("2:bGet_AMR_status", ua.VariantType.Boolean, amr_fdbk.status)

        # Set Battery Percentage Data
        self.ua_set_value("2:fGet_AMR_battery_life", ua.VariantType.Float, amr_fdbk.battery_perc)

        # Set AMR Positions Data
        self.ua_set_value("2:fGet_AMR_pos_x", ua.VariantType.Float, amr_fdbk.position[0])
        self.ua_set_value("2:fGet_AMR_pos_y", ua.VariantType.Float, amr_fdbk.position[1])
        self.ua_set_value("2:fGet_AMR_pos_theta", ua.VariantType.Float, amr_fdbk.position[2])

        # # Set Inertial Measurement Unit Data
        self.ua_set_value("2:fGet_AMR_imu_orient_x", ua.VariantType.Float, amr_fdbk.imu[0])
        self.ua_set_value("2:fGet_AMR_imu_orient_y", ua.VariantType.Float, amr_fdbk.imu[1])
        self.ua_set_value("2:fGet_AMR_imu_orient_z", ua.VariantType.Float, amr_fdbk.imu[2])
        self.ua_set_value("2:fGet_AMR_imu_orient_w", ua.VariantType.Float, amr_fdbk.imu[3])

        self.ua_set_value("2:fGet_AMR_imu_ang_vel_x", ua.VariantType.Float, amr_fdbk.imu[4])
        self.ua_set_value("2:fGet_AMR_imu_ang_vel_y", ua.VariantType.Float, amr_fdbk.imu[5])
        self.ua_set_value("2:fGet_AMR_imu_ang_vel_z", ua.VariantType.Float, amr_fdbk.imu[6])

        self.ua_set_value("2:fGet_AMR_imu_lin_acc_x", ua.VariantType.Float, amr_fdbk.imu[7])
        self.ua_set_value("2:fGet_AMR_imu_lin_acc_y", ua.VariantType.Float, amr_fdbk.imu[8])
        self.ua_set_value("2:fGet_AMR_imu_lin_acc_z", ua.VariantType.Float, amr_fdbk.imu[9])


        # Set Odometry Data
        self.ua_set_value("2:fGet_AMR_odom_pose_lin_x", ua.VariantType.Float, amr_fdbk.odom[0])
        self.ua_set_value("2:fGet_AMR_odom_pose_lin_y", ua.VariantType.Float, amr_fdbk.odom[1])
        self.ua_set_value("2:fGet_AMR_odom_pose_lin_z", ua.VariantType.Float, amr_fdbk.odom[2])

        self.ua_set_value("2:fGet_AMR_odom_twist_orien_x", ua.VariantType.Float, amr_fdbk.odom[3])
        self.ua_set_value("2:fGet_AMR_odom_twist_orien_y", ua.VariantType.Float, amr_fdbk.odom[4])
        self.ua_set_value("2:fGet_AMR_odom_twist_ang_z", ua.VariantType.Float, amr_fdbk.odom[5])

    def tm_feedback_to_opcua(self, tm_fdbk):

        self.ua_get_root_node()

        # Set Pick and Place Status
        self.ua_set_value("2:bPick_cmm_st", ua.VariantType.Boolean, tm_fdbk.pick)
        self.ua_set_value("2:bPlace_cmm_st", ua.VariantType.Boolean, tm_fdbk.place)



    def get_action(self):

        self.ua_get_root_node()

        # Required AMR action status
        # print(self.ua_get_value("2:sSet_send_action_to_AMR"))
        self.action_msg.action_request = self.ua_get_value("2:sSet_send_action_to_AMR")

        # Reuired AMR E-Stop Safety
        self.action_msg.estop = self.ua_get_value("2:bSet_AMR_e_stop")

        # Required AMR Position
        self.action_msg.goto_pos[0] = self.ua_get_value("2:fSet_AMR_pos_x")
        self.action_msg.goto_pos[1] = self.ua_get_value("2:fSet_AMR_pos_y")
        self.action_msg.goto_pos[2] = self.ua_get_value("2:fSet_AMR_pos_theta")

        # Required AMR Pause/Ready Action
        self.action_msg.pause_amr = self.ua_get_value("2:bSet_pause_AMR")
        self.action_msg.ready_amr = self.ua_get_value("2:bSet_ready_AMR")

        self.action_from_opcua()

def main(args=None):

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    opcua_client_node = Opcua_Client()

    # Spin the node so the callback function is called.
    rclpy.spin(opcua_client_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    opcua_client_node.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()

    if __name__ == '__main__':
        main()
