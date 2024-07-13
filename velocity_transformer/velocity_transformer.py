#!/usr/bin/env python3
import math
import xml.etree.ElementTree as ET

from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

# Wheel base of hunter robot computed manually from URDF.
DEFAULT_WHEEL_BASE = 0.549


class VelocityTransformer(Node):
    def __init__(self):
        super().__init__('velocity_transformer')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Twist,
            'twist',
            self.cmd_vel,
            10)
        self.wheel_base = self.get_wheel_base_from_urdf(self.get_robot_description())

    def cmd_vel(self, msg):
        '''
        This is a callback function which is called whenever a Twist message is received from the /twist topic.
        It converts the linear and angular velocities of the Twist message into Ackermann convention.
        Ackermann linear velocity is the same as the linear velocity of the Twist message.
        Except when pure angular velocity command is received, then Ackermann linear velocity is the angular velocity.
        Ackermann angular velocity is the steering angle of the Ackermann model.
        '''
        ackermann_twist_msg = Twist()
        # Return angular velocity as linear velocity when pure angular velocity command is received.
        ackermann_linear_velocity = self.handle_pure_angular_velocity(msg.linear.x, msg.angular.z)
        ackermann_twist_msg.linear.x = ackermann_linear_velocity
        # Convert linear and angular velocities to steering angle.
        ackermann_twist_msg.angular.z = self.convert_linear_angular_velocity_to_steering_angle(
            ackermann_linear_velocity, msg.angular.z, self.wheel_base)
        self.publisher.publish(ackermann_twist_msg)
        self.get_logger().info(f'Publishing: "{ackermann_twist_msg}"')

    def get_robot_description(self):
        '''
        This function gets the robot description parameter from the robot state publisher node.
        The robot description is returned in the form of URDF as a string.
        '''
        robot_state_client = self.create_client(GetParameters, 'robot_state_publisher/get_parameters')
        ready = robot_state_client.wait_for_service(timeout_sec=5.0)
        if not ready:
            self.get_logger().error('Wait for service timed out')
            return None
        request = GetParameters.Request()
        request.names = ['robot_description']
        future = robot_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        if response is None:
            e = future.exception()
            self.get_logger().error(f'Service call failed {e}')
            return ""
        return response.values[0].string_value

    @staticmethod
    def handle_pure_angular_velocity(linear_velocity, angular_velocity):
        '''
        This function returns angular velocity as linear velocity when pure angular velocity command is received.
        This is done in order to prevent the robot from stopping when a pure angular velocity command is received.
        '''
        return angular_velocity if (linear_velocity == 0.0 and angular_velocity != 0.0) else linear_velocity


    @staticmethod
    def convert_linear_angular_velocity_to_steering_angle(linear_velocity, angular_velocity, wheel_base):
        '''
        This function converts linear and angular velocities of a ROS Twist message into steering angle.
        '''
        if angular_velocity == 0.0 or linear_velocity == 0.0:
            return 0.0
        radius = linear_velocity / angular_velocity
        return math.atan(wheel_base / radius)

    @staticmethod
    def get_wheel_base_from_urdf(urdf_str):
        '''
        This function extracts the wheel base of a robot from an URDF input as a string.
        If the wheel base cannot be extracted from the URDF, the default wheel base is returned.
        '''
        try:
            root = ET.fromstring(urdf_str)
            front_left_wheel_position = None
            front_right_wheel_position = None
            rear_left_wheel_position = None
            rear_right_wheel_position = None
            for joint in root.findall('joint'):
                if joint.get('name') == 'fr_steer_left_joint':
                    front_left_wheel_position = joint.find('origin').get('xyz')
                if joint.get('name') == 'fr_steer_right_joint':
                    front_right_wheel_position = joint.find('origin').get('xyz')
                if joint.get('name') == 're_left_joint':
                    rear_left_wheel_position = joint.find('origin').get('xyz')
                if joint.get('name') == 're_right_joint':
                    rear_right_wheel_position = joint.find('origin').get('xyz')
        except Exception as e:
            print(f"Failed to parse URDF: {e}")
            print("Using default wheel base defined in the code.")
            return DEFAULT_WHEEL_BASE
        if (
            front_left_wheel_position is None
            or front_right_wheel_position is None
            or rear_left_wheel_position is None
            or rear_right_wheel_position is None
        ):
            print("Failed to extract wheel positions from URDF.")
            print("Using default wheel base defined in the code.")
            return DEFAULT_WHEEL_BASE
        front_x = (float(front_left_wheel_position.split()[0]) + float(front_right_wheel_position.split()[0])) / 2
        rear_x = (float(rear_left_wheel_position.split()[0]) + float(rear_right_wheel_position.split()[0])) / 2
        wheel_base = abs(front_x - rear_x)
        return wheel_base


def main():
    rclpy.init()
    velocity_transformer = VelocityTransformer()
    rclpy.spin(velocity_transformer)
    velocity_transformer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
