#!/usr/bin/env python
from logging import addLevelName
import os, sys, argparse, errno, yaml, time, datetime
import rospy, rospkg
import numpy as np
from Adafruit_MotorHAT import Adafruit_MotorHAT
from jetfalcon_msgs.msg import WheelsCmd
from std_msgs.msg import String
from geometry_msgs.msg import Twist


class Motor(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        # information
        rospy.loginfo("[{0}] Initializing node {0}...".format(self.node_name))

        # setup ROS parameter
        self.ros_parameter = {
            "~alpha": -1.0,
            "~motor_left_ID": 1,
            "~motor_right_ID": 2,
            "~motor_max_pwm": 115.0,
        }
        self.set_ros_parameter()

        # setup motor hat
        self.driver = Adafruit_MotorHAT(i2c_bus=1)
        self.motor = {
            "left": self.driver.getMotor(self.ros_parameter["~motor_left_ID"]),
            "right": self.driver.getMotor(self.ros_parameter["~motor_right_ID"]),
            "max_pwm": self.ros_parameter["~motor_max_pwm"]
        }

        # setup variable
        self.msg_status = False

        # setup subscriber
        self.subscriber = rospy.Subscriber("~wheels_cmd", WheelsCmd, self.callback_wheels_cmd)

        # setup timer
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0), self.callback_timer)


    def set_ros_parameter(self):
        for key in self.ros_parameter.keys():
            self.ros_parameter[key] = rospy.get_param(key, self.ros_parameter[key])
            rospy.set_param(key, self.ros_parameter[key])
            rospy.loginfo("[{}] {} = {}".format(self.node_name, key, self.ros_parameter[key]))

    def callback_timer(self, event):
        for key in self.ros_parameter.keys():
            value = rospy.get_param(key, self.ros_parameter[key])
            if value == self.ros_parameter[key]:
                pass
            else:
                self.ros_parameter[key] = value
                rospy.set_param(key, self.ros_parameter[key])
                rospy.loginfo("[{}] Parameter \"{}\" change! Now is \"{}\"".format(self.node_name, key, self.ros_parameter[key]))

    def callback_wheels_cmd(self, msg):
        left = msg.left.data * self.ros_parameter["~alpha"]
        right = msg.right.data * self.ros_parameter["~alpha"]

        if left > 0:
            self.motor["left"].run(Adafruit_MotorHAT.FORWARD)
        else:
            self.motor["left"].run(Adafruit_MotorHAT.BACKWARD)

        if right > 0:
            self.motor["right"].run(Adafruit_MotorHAT.FORWARD)
        else:
            self.motor["right"].run(Adafruit_MotorHAT.BACKWARD)

        left = int(min(max(abs(left * self.motor["max_pwm"]), 0), self.motor["max_pwm"]))
        right = int(min(max(abs(right * self.motor["max_pwm"]), 0), self.motor["max_pwm"]))
        self.motor["left"].setSpeed(left)
        self.motor["right"].setSpeed(right)


    def on_shutdown(self):
        self.motor["left"].setSpeed(0)
        self.motor["right"].setSpeed(0)
        self.motor["left"].run(Adafruit_MotorHAT.RELEASE)
        self.motor["left"].run(Adafruit_MotorHAT.RELEASE)
        rospy.loginfo("[{}] Close.".format(self.node_name))
        rospy.loginfo("[{}] shutdown.".format(self.node_name))
        rospy.sleep(1)
        rospy.is_shutdown = True

if __name__ == "__main__":
    rospy.init_node("motor", anonymous=False)
    motor = Motor()
    rospy.on_shutdown(motor.on_shutdown)   
    rospy.spin()
