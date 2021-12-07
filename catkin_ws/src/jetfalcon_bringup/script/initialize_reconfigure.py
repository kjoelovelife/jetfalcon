#!/usr/bin/env python

import rospy
from dynamic_reconfigure.client import Client


class InitializeReconfigure(object):
    def __init__(self):
        self.package = "jetfalcon_bringup"
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]


        # get ros parameter
        self.target = rospy.get_param("~target", "inverse_kinematic")
        self.ros_parameter = {
            "gain": 1.0,
            "trim": 0.0,
            "rpm_5v": 83, # rpm/s
        }
        self.get_ros_parameter()

        self.client = Client(name="~{}".format(self.target), timeout=10, config_callback=self.callback_client)

    def get_ros_parameter(self):
        for key in self.ros_parameter.keys():
            self.ros_parameter[key] = rospy.get_param("{}/{}".format(self.target, key), self.ros_parameter[key])

    def callback_client(self, config):
        rospy.loginfo("[{}] Updated rqt_reconfig.".format(self.node_name))


if __name__ == "__main__":
    rospy.init_node("initialize_reconfigure", anonymous=False)
    initialize_reconfigure = InitializeReconfigure()
    initialize_reconfigure.client.update_configuration(initialize_reconfigure.ros_parameter)








