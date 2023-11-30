#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import FluidPressure
# from scipy import signal
import numpy as np

class PressureNoiseSim(Node):

    def __init__(self):
        super().__init__(node_name="pressure_noise_sim")

        self.init_params()

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        self.pressure_sub = self.create_subscription(msg_type=FluidPressure,
                                                     topic="pressure",
                                                     callback=self.on_pressure,
                                                     qos_profile=qos)

        self.pressure_raw_pub = self.create_publisher(msg_type=FluidPressure,
                                                      topic="pressure_raw",
                                                      qos_profile=qos)
        
        self.rng = np.random.default_rng()

    def init_params(self):
        self.declare_parameters(namespace="",
                                parameters= [
                                    ('noise_amplitude', rclpy.Parameter.Type.DOUBLE)
                                ])
        
    def generate_noise(self) -> float:
        return self.get_parameter("noise_amplitude").get_parameter_value().double_value * self.rng.standard_normal()
    
    def on_pressure(self, pressure_msg: FluidPressure) -> None:
        pressure_raw = pressure_msg.fluid_pressure + self.generate_noise()
        self.publish_pressure_raw(pressure_raw)
        

    def publish_pressure_raw(self, pressure_raw) -> None:
        msg = FluidPressure()
        msg.fluid_pressure = pressure_raw
        self.pressure_raw_pub.publish(msg)

def main():
    rclpy.init()
    node = PressureNoiseSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__=="__main__":
        main()