#!/usr/bin/env python3
"""
This node is your depth controller.
It takes as input a current depth and a given depth setpoint.
Its output is a thrust command to the BlueROV's actuators.
"""
import rclpy
from hippo_msgs.msg import ActuatorSetpoint, DepthStamped, Float64Stamped
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from PID import PID

from std_msgs.msg import Float64, Bool


class DepthControlNode(Node):

    def __init__(self):
        super().__init__(node_name='depth_controller')

        self.current_setpoint = 0.0
        self.current_depth = 0.0

        self.thrust_pub = self.create_publisher(msg_type=ActuatorSetpoint,
                                                topic='thrust_setpoint',
                                                qos_profile=1)

        self.setpoint_sub = self.create_subscription(msg_type=Float64Stamped,
                                                     topic='depth_setpoint',
                                                     callback=self.on_setpoint,
                                                     qos_profile=1)
        self.depth_sub = self.create_subscription(msg_type=DepthStamped,
                                                  topic='depth',
                                                  callback=self.on_depth,
                                                  qos_profile=1)

        self.Kp_pub = self.create_publisher(msg_type=Float64,
                                           topic='Kp',
                                           qos_profile=1)
        
        self.Ki_pub = self.create_publisher(msg_type=Float64,
                                           topic='Ki',
                                           qos_profile=1)
        
        self.Kd_pub = self.create_publisher(msg_type=Float64,
                                           topic='Kd',
                                           qos_profile=1)
        
        self.reset_i_pub = self.create_publisher(msg_type=Bool,
                                                 topic='i_reset',
                                                 qos_profile=1)
        
        self.threshold_i_pub = self.create_publisher(msg_type=Float64,
                                                     topic='threshold_i',
                                                     qos_profile=1)
        
        self.pid_offset_pub = self.create_publisher(msg_type=Float64,
                                                topic='pid_offset',
                                                qos_profile=1)
        
        self.e_pub = self.create_publisher(msg_type=Float64,
                                           topic='e',
                                           qos_profile=1)
        
        self.init_params()

        self.PID = PID(self.get_parameter('gains.p').get_parameter_value().double_value, self.get_parameter('gains.i').get_parameter_value().double_value, self.get_parameter('gains.d').get_parameter_value().double_value, self.get_parameter('pid_offset').get_parameter_value().double_value, self.get_parameter('threshold.i').get_parameter_value().double_value, self.get_parameter('reset.i').get_parameter_value().bool_value)

        self.upper_limit = -0.1
        self.lower_limit = -0.8


    def init_params(self):
        self.declare_parameters(namespace = '', 
                                parameters= [
                                    ('gains.p', rclpy.Parameter.Type.DOUBLE),
                                    ('gains.i', rclpy.Parameter.Type.DOUBLE),
                                    ('gains.d', rclpy.Parameter.Type.DOUBLE),
                                    ('pid_offset', rclpy.Parameter.Type.DOUBLE),
                                    ('threshold.i', rclpy.Parameter.Type.DOUBLE),
                                    ('reset.i', rclpy.Parameter.Type.BOOL)
                                ]
                            )

        self.add_on_set_parameters_callback(self.on_params_changed)

    
    def on_params_changed(self, params):
        self.PID.change_params(self.get_parameter('gains.p').get_parameter_value().double_value, self.get_parameter('gains.i').get_parameter_value().double_value, self.get_parameter('gains.d').get_parameter_value().double_value, self.get_parameter('pid_offset').get_parameter_value().double_value, self.get_parameter('threshold.i').get_parameter_value().double_value, self.get_parameter('reset.i').get_parameter_value().bool_value)
        self.PID.reset_integral()
        return SetParametersResult(successful=True)
    
    def on_setpoint(self, setpoint_msg: Float64Stamped):
        # We received a new setpoint! Let's save it, so that we can use it as
        # soon as we receive new depth data.
        self.current_setpoint = setpoint_msg.data

    def on_depth(self, depth_msg: DepthStamped):
        # We received a new depth message! Now we can get to action!
        self.current_depth = depth_msg.depth
        thrust = self.compute_control_output(depth_msg=depth_msg)
        # either set the timestamp to the current time or set it to the
        # stamp of `depth_msg` because the control output corresponds to this
        # point in time. Both choices are meaningful.
        # option 1:
        # now = self.get_clock().now()
        # option 2:
        timestamp = rclpy.time.Time.from_msg(depth_msg.header.stamp)

        # self.get_logger().info(
        #     f"Hi! I'm your controller running. "
        #     f'I received a depth of {depth_msg.depth} m '
        #     f'and a setpoint of {self.current_setpoint} m. '
        #     f'The calculated thrust is: {thrust}',
        #     throttle_duration_sec=1)

        self.publish_vertical_thrust(thrust=thrust, timestamp=timestamp)
        self.publish_Kp()
        self.publish_Ki()
        self.publish_Kd()
        self.publish_reset_i()
        self.publish_threshold_i()
        self.publish_pid_offset()
        self.publish_e()

    def publish_vertical_thrust(self, thrust: float,
                                timestamp: rclpy.time.Time) -> None:
        msg = ActuatorSetpoint()
        # we want to set the vertical thrust exlusively. mask out xy-components.
        msg.ignore_x = True
        msg.ignore_y = True
        msg.ignore_z = False

        msg.z = thrust

        # Let's add a time stamp
        msg.header.stamp = timestamp.to_msg()

        self.thrust_pub.publish(msg)

    def compute_control_output(self, depth_msg: DepthStamped) -> float:
        if self.lower_limit <= self.current_setpoint <= self.upper_limit:
            # self.get_logger().info(f'Integral: {self.PID.get_integral()} ')
            thrust = self.PID.control(self.current_setpoint - depth_msg.depth)
        else:
            return 0.0
        return thrust
    
    def publish_Kp(self) -> None:
        msg = Float64()
        msg.data = self.get_parameter('gains.p').get_parameter_value().double_value
        self.Kp_pub.publish(msg)

    def publish_Ki(self) -> None:
        msg = Float64()
        msg.data = self.get_parameter('gains.i').get_parameter_value().double_value
        self.Ki_pub.publish(msg)

    def publish_Kd(self) -> None:
        msg = Float64()
        msg.data = self.get_parameter('gains.d').get_parameter_value().double_value
        self.Kd_pub.publish(msg)

    def publish_reset_i(self) -> None:
        msg = Bool()
        msg.data = self.get_parameter('reset.i').get_parameter_value().bool_value
        self.reset_i_pub.publish(msg)

    def publish_threshold_i(self) -> None:
        msg = Float64()
        msg.data = self.get_parameter('threshold.i').get_parameter_value().double_value
        self.threshold_i_pub.publish(msg)

    def publish_pid_offset(self) -> None:
        msg = Float64()
        msg.data = self.get_parameter('pid_offset').get_parameter_value().double_value
        self.pid_offset_pub.publish(msg)

    def publish_e(self) -> None:
        msg = Float64()
        msg.data = self.current_setpoint - self.current_depth
        self.e_pub.publish(msg)



def main():
    rclpy.init()
    node = DepthControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
