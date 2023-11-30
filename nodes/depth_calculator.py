#!/usr/bin/env python3
"""
This node takes as input the pressure data and computes a resulting water depth.
"""
import rclpy
from hippo_msgs.msg import DepthStamped
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import FluidPressure
from live_iir_filter import live_iir_filter
from live_fir_filter import live_fir_filter
from scipy import signal
from rcl_interfaces.msg import SetParametersResult

class DepthCalculator(Node):

    def __init__(self):
        super().__init__(node_name='depth_calculator')

        self.init_params()

        qos = QoSProfile(reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST,
                         depth=1)

        self.depth_pub = self.create_publisher(msg_type=DepthStamped,
                                               topic='depth',
                                               qos_profile=1)
        self.pressure_sub = self.create_subscription(msg_type=FluidPressure,
                                                     topic='pressure_raw',
                                                     callback=self.on_pressure,
                                                     qos_profile=qos)
        
        self.pressure_filtered_pub = self.create_publisher(msg_type=FluidPressure,
                                                           topic='pressure_filtered',
                                                           qos_profile=1)
        
        self.create_filters(N=self.get_parameter("filter_order").get_parameter_value().integer_value, Wn=self.get_parameter("critical_freq").get_parameter_value().double_value)

        self.add_on_set_parameters_callback(self.on_params_changed)

    def init_params(self):
        self.declare_parameters(namespace = '', 
                                parameters = [
                                    ('atmospheric_pressure', rclpy.Parameter.Type.INTEGER), # Pa
                                    ('sensor_z_offset', rclpy.Parameter.Type.DOUBLE), # m
                                    ('rho', rclpy.Parameter.Type.INTEGER), # kg/m³
                                    ('g', rclpy.Parameter.Type.DOUBLE), # N/kg
                                    ('enable_filter', rclpy.Parameter.Type.BOOL),
                                    ('filter_order', rclpy.Parameter.Type.INTEGER),
                                    ('critical_freq', rclpy.Parameter.Type.DOUBLE)
                                ])

    def on_pressure(self, pressure_msg: FluidPressure) -> None:
        pressure_raw = pressure_msg.fluid_pressure
        pressure_filtered = self.LiveIIR(pressure_raw)

        if self.get_parameter('enable_filter').get_parameter_value().bool_value:
            depth = self.pressure_to_depth(pressure=pressure_filtered)

             # self.get_logger().info(f'Hello, i received a pressure of {pressure_filtered} Pa.'
             #                   f'This corresponds to a depth of {depth} m.',
             #                   throttle_duration_sec=1)
        else:
            depth = self.pressure_to_depth(pressure=pressure_raw)

            # self.get_logger().info(f'Hello, i received a pressure of {pressure_raw} Pa.'
            #                         f'This corresponds to a depth of {depth} m.',
            #                         throttle_duration_sec=1)
            
        now = self.get_clock().now()

        self.publish_depth_msg(depth=depth, now=now)
        self.publish_pressure_filtered(pressure_filtered)

    def publish_depth_msg(self, depth: float, now: rclpy.time.Time) -> None:
        msg = DepthStamped()
        # Let's add a time stamp
        msg.header.stamp = now.to_msg()
        # and populate the depth field
        msg.depth = depth
        self.depth_pub.publish(msg)

    def pressure_to_depth(self, pressure: float) -> float:
        depth = -(pressure-self.get_parameter('atmospheric_pressure').get_parameter_value().integer_value) / (self.get_parameter('rho').get_parameter_value().integer_value * self.get_parameter('g').get_parameter_value().double_value) + self.get_parameter('sensor_z_offset').get_parameter_value().double_value
        return depth
    
    def publish_pressure_filtered(self, pressure_filtered) -> None:
        msg = FluidPressure()
        msg.fluid_pressure = pressure_filtered
        self.pressure_filtered_pub.publish(msg)

    def create_filters(self, N, Wn) -> None:
        b, a = signal.butter(N=N, Wn=Wn, btype="lowpass", fs=50)
        h = signal.firwin(numtaps=30, cutoff=1, fs=50)
        self.LiveIIR = live_iir_filter(b, a)
        self.LiveFIR = live_fir_filter(h=h)

    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            if param.name == "filter_order":
                self.create_filters(N = param.value, Wn = self.get_parameter("critical_freq").get_parameter_value().double_value)
            elif param.name == "critical_freq":
                self.create_filters(N = self.get_parameter("filter_order").get_parameter_value().integer_value, Wn = param.value)
        return SetParametersResult(successful=True)

def main():
    rclpy.init()
    node = DepthCalculator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
