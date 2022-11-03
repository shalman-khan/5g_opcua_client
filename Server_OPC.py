#!/usr/bin/env python2
"""
A set of tools (adapted examples) to quickly check the status of clients or servers.

This development was performed on the 23rd of July, 2020.
Maintainer(s) and creator(s):
Walter Frank Pintor Ortiz, walterpintor@gmail.com
Lim Guo Wei, gilm5.gl@gmail.com
https://github.com/FreeOpcUa/python-opcua/blob/master/examples/server-minimal.py
"""

import sys, getopt

sys.path.insert(0, "..")
import time
from opcua import Server, ua

myvar = []
# ip_opc_server_address = "192.168.86.21"
#ip_opc_server_address = "10.100.108.152"
ip_opc_server_address = "127.0.0.1"
opc_port_server_no = 4840

print('Welcome to a customized OPC-UA server for 5G \nFor defined IP addresses, utilize the following arguments')
print('python server.py --opc_ip <opc_ip> --opc_port <opc_port>')
time.sleep(2)


def check_arguments(argv):
    global ip_opc_server_address
    global opc_port_server_no
    opc_ip_read = ip_opc_server_address
    opc_port_read = opc_port_server_no

    try:
        opts, args = getopt.getopt(argv, "hi:o:", ["opc_ip=", "opc_port="])
    except getopt.GetoptError:
        print('python server.py --opc_ip <opc_ip> --opc_port <opc_port>')
        sys.exit(2)
    for opt, arg in opts:
        if opt == '-h':
            print('python server.py --opc_ip <opc_ip> --opc_port <opc_port>')
            sys.exit()
        elif opt in ("-opc_ip", "--opc_ip"):
            opc_ip_read = str(arg)
        elif opt in ("-opc_port", "--opc_port"):
            opc_port_read = arg
    print('OPC-UA IP provided is: ', opc_ip_read)
    print('The OPC-UA port is "', opc_port_read)
    ip_opc_server_address = opc_ip_read
    opc_port_server_no = opc_port_read


if __name__ == "__main__":

    # Check if arguments were given at the start
    check_arguments(sys.argv[1:])

    # setup our server
    server = Server()
    server.set_endpoint("opc.tcp://" + ip_opc_server_address + ":" + str(opc_port_server_no) + "/freeopcua/server/")

    # setup our own namespace, not really necessary but should as spec
    uri = "http://examples.freeopcua.github.io"
    idx = server.register_namespace(uri)

    # get Objects node, this is where we should put our nodes
    objects = server.get_objects_node()

    # Add as needed - address space
    myvar = [None] * 47
    myobj = objects.add_object(idx, "Variables")

    myvar[0] = myobj.add_variable(idx, "sSet_send_action_to_AMR", "Idle")
    myvar[1] = myobj.add_variable(idx, "bGet_AMR_status", False)
    myvar[2] = myobj.add_variable(idx, "fGet_AMR_battery_life", 0.0, ua.VariantType.Float)
    myvar[3] = myobj.add_variable(idx, "fGet_AMR_pos_x", 0.0, ua.VariantType.Float)
    myvar[4] = myobj.add_variable(idx, "fGet_AMR_pos_y", 0.0, ua.VariantType.Float)
    myvar[5] = myobj.add_variable(idx, "fGet_AMR_pos_theta", 0.0, ua.VariantType.Float)
    myvar[6] = myobj.add_variable(idx, "fGet_AMR_imu_orient_x", 0.0, ua.VariantType.Float)
    myvar[7] = myobj.add_variable(idx, "fGet_AMR_imu_orient_y", 0.0, ua.VariantType.Float)
    myvar[8] = myobj.add_variable(idx, "fGet_AMR_imu_orient_z", 0.0, ua.VariantType.Float)
    myvar[9] = myobj.add_variable(idx, "fGet_AMR_imu_orient_w", 1.0, ua.VariantType.Float)
    myvar[10] = myobj.add_variable(idx, "fGet_AMR_imu_ang_vel_x", 0.0, ua.VariantType.Float)
    myvar[11] = myobj.add_variable(idx, "fGet_AMR_imu_ang_vel_y", 0.0, ua.VariantType.Float)
    myvar[12] = myobj.add_variable(idx, "fGet_AMR_imu_ang_vel_z", 0.0, ua.VariantType.Float)
    myvar[13] = myobj.add_variable(idx, "fGet_AMR_imu_lin_acc_x", 0.0, ua.VariantType.Float)
    myvar[14] = myobj.add_variable(idx, "fGet_AMR_imu_lin_acc_y", 0.0, ua.VariantType.Float)
    myvar[15] = myobj.add_variable(idx, "fGet_AMR_imu_lin_acc_z", 0.0, ua.VariantType.Float)
    myvar[16] = myobj.add_variable(idx, "fGet_AMR_odom_pose_lin_x", 0.0, ua.VariantType.Float)
    myvar[17] = myobj.add_variable(idx, "fGet_AMR_odom_pose_lin_y", 0.0, ua.VariantType.Float)
    myvar[18] = myobj.add_variable(idx, "fGet_AMR_odom_pose_lin_z", 0.0, ua.VariantType.Float)
    myvar[19] = myobj.add_variable(idx, "fGet_AMR_odom_twist_orien_x", 0.0, ua.VariantType.Float)
    myvar[20] = myobj.add_variable(idx, "fGet_AMR_odom_twist_orien_y", 0.0, ua.VariantType.Float)
    myvar[21] = myobj.add_variable(idx, "fGet_AMR_odom_twist_ang_z", 0.0, ua.VariantType.Float)

    # AMR connectivity frames and commands
    myvar[22] = myobj.add_variable(idx, "sSet_action_to_AMR", "")
    myvar[23] = myobj.add_variable(idx, "bSet_AMR_e_stop", False)
    myvar[24] = myobj.add_variable(idx, "fSet_AMR_pos_x", 1.0, ua.VariantType.Float)
    myvar[25] = myobj.add_variable(idx, "fSet_AMR_pos_y", 2.0, ua.VariantType.Float)
    myvar[26] = myobj.add_variable(idx, "fSet_AMR_pos_theta", 3.0, ua.VariantType.Float)
    myvar[27] = myobj.add_variable(idx, "bSet_pause_AMR", False)
    myvar[28] = myobj.add_variable(idx, "bSet_ready_AMR", False)

    # CMM station ZED camera object frame
    myvar[29] = myobj.add_variable(idx, "fZEDx_zed", 0.0, ua.VariantType.Float)
    myvar[30] = myobj.add_variable(idx, "fZEDy_zed", 0.0, ua.VariantType.Float)
    myvar[31] = myobj.add_variable(idx, "fZEDz_zed", 0.0, ua.VariantType.Float)
    myvar[32] = myobj.add_variable(idx, "fZEDq1_zed", 0.0, ua.VariantType.Float)
    myvar[33] = myobj.add_variable(idx, "fZEDq2_zed", 0.0, ua.VariantType.Float)
    myvar[34] = myobj.add_variable(idx, "fZEDq3_zed", 0.0, ua.VariantType.Float)
    myvar[35] = myobj.add_variable(idx, "fZEDq4_zed", 2.0, ua.VariantType.Float)

    # IP camera station object frame
    myvar[36] = myobj.add_variable(idx, "fZEDx_ipcam", 0.0, ua.VariantType.Float)
    myvar[37] = myobj.add_variable(idx, "fZEDy_ipcam", 0.0, ua.VariantType.Float)
    myvar[38] = myobj.add_variable(idx, "fZEDz_ipcam", 0.0, ua.VariantType.Float)
    myvar[39] = myobj.add_variable(idx, "fZEDq1_ipcam", 0.0, ua.VariantType.Float)
    myvar[40] = myobj.add_variable(idx, "fZEDq2_ipcam", 0.0, ua.VariantType.Float)
    myvar[41] = myobj.add_variable(idx, "fZEDq3_ipcam", 0.0, ua.VariantType.Float)
    myvar[42] = myobj.add_variable(idx, "fZEDq4_ipcam", 1.0, ua.VariantType.Float)

    # Actions for robots
    myvar[43] = myobj.add_variable(idx, "bPick_ip_st", False, ua.VariantType.Boolean)
    myvar[44] = myobj.add_variable(idx, "bPlace_cmm_st", False, ua.VariantType.Boolean)
    myvar[45] = myobj.add_variable(idx, "bPick_cmm_st", True, ua.VariantType.Boolean)
    myvar[46] = myobj.add_variable(idx, "bPlace_ip_st", False, ua.VariantType.Boolean)

    for variables in range(len(myvar)):
        if myvar[variables] is not None:
            print(variables)
            myvar[variables].set_writable()  # Set MyVariable to be writable by clients
            time.sleep(0.05)

            # starting!
    server.start()

    try:
        count = 0
        while True:
            time.sleep(1)
            for i in range(len(myvar)):
                if myvar[i] is not None:
                    print(myvar[i].get_browse_name(), ", ", myvar[i].get_value())
                    time.sleep(0.2)
    finally:
        # close connection, remove subcsriptions, etc
        print("Closing connection")
        server.stop()
