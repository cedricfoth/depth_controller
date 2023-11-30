#!/usr/bin/env python3

import rclpy
from hippo_msgs.msg import Float64Stamped
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
import math

class DepthSetpointNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_setpoint_publisher')

        self.start_time = self.get_clock().now()

        self.init_params()

        self.depth_setpoint_pub = self.create_publisher(msg_type=Float64Stamped,
                                                        topic='depth_setpoint',
                                                        qos_profile=1)
        
        self.timer = self.create_timer(timer_period_sec=1/50,
                                       callback=self.on_timer)

    def init_params(self):
        self.declare_parameters(namespace='',
                                parameters= [
                                    ('static_setpoint', rclpy.Parameter.Type.BOOL),
                                    ('sin_setpoint', rclpy.Parameter.Type.BOOL),
                                    ('zickzack_setpoint', rclpy.Parameter.Type.BOOL),
                                    ('setpoint', rclpy.Parameter.Type.DOUBLE), # m
                                    ('amplitude', rclpy.Parameter.Type.DOUBLE), # m
                                    ('interval', rclpy.Parameter.Type.DOUBLE) # s
                                ])
        
        self.add_on_set_parameters_callback(self.on_params_changed)

    def on_params_changed(self, params):
        param: rclpy.Parameter
        for param in params:
            if param.name == 'static_setpoint' and param.value:
                self.set_parameters([Parameter('sin_setpoint', rclpy.Parameter.Type.BOOL, False), Parameter('zickzack_setpoint', rclpy.Parameter.Type.BOOL, False)])
            elif param.name == 'sin_setpoint' and param.value:
                self.set_parameters([Parameter('static_setpoint', rclpy.Parameter.Type.BOOL, False), Parameter('zickzack_setpoint', rclpy.Parameter.Type.BOOL, False)])
            elif param.name == 'zickzack_setpoint' and param.value:
                self.set_parameters([Parameter('static_setpoint', rclpy.Parameter.Type.BOOL, False), Parameter('sin_setpoint', rclpy.Parameter.Type.BOOL, False)])
            else:
                continue
        return SetParametersResult(successful=True)
    
    def on_timer(self) -> None:
        now= self.get_clock().now()
        time = self.start_time - now

        if self.get_parameter('static_setpoint').get_parameter_value().bool_value:
            setpoint = self.get_parameter('setpoint').get_parameter_value().double_value
        elif self.get_parameter('sin_setpoint').get_parameter_value().bool_value:
            setpoint = self.get_parameter('amplitude').get_parameter_value().double_value * math.sin(2 * math.pi / self.get_parameter('interval').get_parameter_value().double_value * time.nanoseconds * 1e-9) + self.get_parameter('setpoint').get_parameter_value().double_value
        elif self.get_parameter('zickzack_setpoint').get_parameter_value().bool_value:
            if time.nanoseconds * 1e-9 % (self.get_parameter('interval').get_parameter_value().double_value * 2) <= self.get_parameter('interval').get_parameter_value().double_value:
                setpoint = self.get_parameter('setpoint').get_parameter_value().double_value - self.get_parameter('amplitude').get_parameter_value().double_value + (time.nanoseconds * 1e-9 % self.get_parameter('interval').get_parameter_value().double_value) * 2 * self.get_parameter('amplitude').get_parameter_value().double_value / self.get_parameter('interval').get_parameter_value().double_value
            elif time.nanoseconds * 1e-9 % (self.get_parameter('interval').get_parameter_value().double_value * 2) > self.get_parameter('interval').get_parameter_value().double_value: 
                setpoint = self.get_parameter('setpoint').get_parameter_value().double_value + self.get_parameter('amplitude').get_parameter_value().double_value - (time.nanoseconds * 1e-9 % self.get_parameter('interval').get_parameter_value().double_value) * 2 * self.get_parameter('amplitude').get_parameter_value().double_value / self.get_parameter('interval').get_parameter_value().double_value
        self.publish_setpoint(setpoint)

    def publish_setpoint(self, setpoint: float) -> None:
        msg = Float64Stamped()
        msg.data = setpoint
        msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_setpoint_pub.publish(msg)


def main():
    rclpy.init()
    node = DepthSetpointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
