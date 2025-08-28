import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String
from rclpy.qos import QoSProfile
import odrive
from odrive.enums import *
import math
import serial
import os
import numpy as np
import time

class Motor(Node):
    SERIAL_PORT = '/dev/ttyACM0'
    odrv0 = odrive.find_any()
    def __init__(self):
        super().__init__("motor")
        self.object = ''
        self.odrv0 = odrive.find_any()
        self.count = 0.0
        self.wheelDiameter = 0.0702 # m unit
        self.wheelSeperation = 0.465  # m unit
        self.emergency_count = 0

        if not os.path.exists(self.SERIAL_PORT):
            self.get_logger().error("Serial Port not found")
            rclpy.shutdown()

        self.ser = serial.Serial(self.SERIAL_PORT, 115200)
        self.object_sub = self.create_subscription(String, "/detected_object", self.check_object, 10)
        self.twist_subscriber = self.create_subscription(Twist, "/diffbot_base_controller/cmd_vel_unstamped", self.send_cmd_vel, 10)
        self.get_logger().info("motor has started")
        self.position_publisher = self.create_publisher(Float32MultiArray, "/position", 10)
        self.timer_ = self.create_timer(0.02, self.pub_velocity)

        # ===

        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.odrv0.axis0.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
        self.odrv0.axis1.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL

        self.odrv0.axis0.controller.input_vel = 0
        self.odrv0.axis1.controller.input_vel = 0





    def check_object(self, msg):
        self.object = msg.data
   
    def send_cmd_vel(self, msg):
        if self.object == 'per ':
            self.get_logger().info("emergency!! robot stopped.")
            self.odrv0.axis0.controller.input_vel = 0
            self.odrv0.axis1.controller.input_vel = 0
            print(self.emergency_count)
            self.emergency_count += 1
            time.sleep(3)
        else:
            self.get_logger().info("Twist: Linear: %f Angular velocity: %f" % (msg.linear.x, msg.angular.z))
            self.odrv0.axis0.controller.input_vel = (msg.linear.x - msg.angular.z * self.wheelSeperation/2) / (self.wheelDiameter * 3.1415)
            self.odrv0.axis1.controller.input_vel = (msg.linear.x + msg.angular.z * self.wheelSeperation/2) / (self.wheelDiameter * 3.1415)

    def getEncoderData(self, command):
        self.ser.write(command.encode())
        data = self.ser.readline().decode().strip()
        return float(data) / (math.pi * self.wheelDiameter)

    def pub_velocity(self):
        if not self.odrv0:
            try:
                self.odrv0 = odrive.find_any()
            except odrive.utils.TimeoutError:
                self.get_logger().warning("ODrive not found. Retrying...")
                return

        msg = Float32MultiArray()
        self.rightWheelPos = self.getEncoderData("r axis0.encoder.pos_estimate\n") * -0.5
        self.leftWheelPos = self.getEncoderData("r axis1.encoder.pos_estimate\n") * 0.5
        self.rightWheelVel = self.getEncoderData("r axis0.encoder.vel_estimate\n") * -0.05 * 0.03 
        self.leftWheelVel = self.getEncoderData("r axis1.encoder.vel_estimate\n") * 0.05 * 0.03 

        wheel_r_pos = self.rightWheelPos
        wheel_l_pos = self.leftWheelPos
        wheel_r_vel = self.rightWheelVel
        wheel_l_vel = self.leftWheelVel
       
        self.count += 1.0
        msg.data = [wheel_r_pos,wheel_l_pos,wheel_r_vel,wheel_l_vel,self.count]
        self.position_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Motor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

