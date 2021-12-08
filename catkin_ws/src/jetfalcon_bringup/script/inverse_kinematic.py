#!/usr/bin/env python

import yaml, datetime, math
import rospy, rospkg
import numpy as np
from geometry_msgs.msg import Twist
from jetfalcon_msgs.msg import WheelsCmd
from dynamic_reconfigure.server import Server
from jetfalcon_msgs.cfg import InverseKinematicConfig
from std_srvs.srv import Empty, EmptyResponse

class InverseKinematic(object):
    def __init__(self):
        self.package = "jetfalcon_bringup"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]
        rospy.loginfo("[{0}] Initialzing node {0}...".format(self.node_name))

        # setup ROS parameter
        self.ros_parameter = {
            "~calibration_time": "2021-12-08",
            "~length": 125.0,     # mm
            "~radius": 65.0 / 2,  # mm
            "~gain": 1.0,
            "~trim": 0.0,
            "~rpm_5v": 83,        # rpm/s
        }
        self.set_ros_parameter()

        # setup variable
        self.wheels_cmd = WheelsCmd()
        self.mm_to_m = math.pow(10, -3)
        self.msg_status = False

        # setup subscriber
        self.subscriber = rospy.Subscriber("~cmd_vel", Twist, self.callback_cmd_vel)

        # setup publisher 
        self.publisher = rospy.Publisher("~wheels_cmd", WheelsCmd, queue_size=1)

        # setup service-server
        self.service_server = rospy.Service("~save_calibration", Empty, self.callback_service_server)

        # setup timer 
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.1), self.callback_timer)

        # setup rqt_reconfigure
        self.rqt_reconfigure = Server(InverseKinematicConfig, self.callback_rqt_reconfigure)

    def set_ros_parameter(self):
        for key in self.ros_parameter.keys():
            self.ros_parameter[key] = rospy.get_param(key, self.ros_parameter[key])
            rospy.set_param(key, self.ros_parameter[key])
            unit = {
                "~length": "(mm)",
                "~radius": "(mm)",
                "~rpm_5v": "(rpm/min)",
            }.get(key, "")
            rospy.loginfo("[{}] {} = {} {}".format(self.node_name, key, self.ros_parameter[key], unit))

    def callback_cmd_vel(self, msg):
        velocity = msg.linear.x
        omega = msg.angular.z
        self.wheels_cmd.right.data =  ((2 * velocity) + (omega * self.ros_parameter["~length"] * self.mm_to_m)) / (2 * self.ros_parameter["~radius"] * self.mm_to_m)
        self.wheels_cmd.left.data  =  ((2 * velocity) - (omega * self.ros_parameter["~length"] * self.mm_to_m)) / (2 * self.ros_parameter["~radius"] * self.mm_to_m)
        self.wheels_cmd.right.data *= (self.ros_parameter["~gain"] + self.ros_parameter["~trim"] ) * self.ros_parameter["~rpm_5v"] / 60.0 / 5.0
        self.wheels_cmd.left.data *= (self.ros_parameter["~gain"] - self.ros_parameter["~trim"] ) * self.ros_parameter["~rpm_5v"] / 60.0 / 5.0
        self.publisher.publish(self.wheels_cmd)

    def callback_service_server(self, req):
        file_name = "{}/param/inverse_kinematic/{}.yaml".format(rospkg.RosPack().get_path(self.package), self.veh_name)
        time_format = "%Y_%m_%d_%H_%M_%S"
        self.ros_parameter["~calibration_time"] = datetime.datetime.now().strftime(time_format)
        yaml_file = {}
        for key in self.ros_parameter.keys():
            yaml_file[key[1:]] = self.ros_parameter[key]
            rospy.loginfo("[{}] Set parameter {} = {}.".format(self.node_name, key, self.ros_parameter[key]))
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(yaml_file, default_flow_style=False))

        rospy.loginfo("[{}] Save completed! You can check the file \"{}\"".format(self.node_name, file_name))
        return EmptyResponse()


    def callback_timer(self, event):
        pass

    def callback_rqt_reconfigure(self, config, level):
        for key in config["groups"]["parameters"].keys():
            self.ros_parameter["~{}".format(key)] =  config[key]
        return config


    def on_shutdown(self):
        self.wheels_cmd.left.data = 0.0
        self.wheels_cmd.right.data = 0.0
        self.publisher.publish(self.wheels_cmd)
        rospy.loginfo("[{}] Close.".format(self.node_name))
        rospy.loginfo("[{}] shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown = True

if __name__ == "__main__":
    rospy.init_node("inverse_kinematic", anonymous=False)
    inverse_kinematic = InverseKinematic()
    rospy.on_shutdown(inverse_kinematic.on_shutdown)
    rospy.spin()








