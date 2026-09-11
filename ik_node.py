"""ROS 2: /platform_ypr [yaw, pitch, roll] -> /joint_commands and /motor_angles."""
import json

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from ik import DEFAULT_MODEL, IKError, OrientationIK, URDF_INPUT_JOINTS


class IKNode(Node):
    def __init__(self):
        super().__init__('spm_inverse_kinematics')
        model = self.declare_parameter('model_path', str(DEFAULT_MODEL)).value
        self.degrees = self.declare_parameter('degrees', False).value
        self.solver = OrientationIK(model)
        self.joints = self.create_publisher(JointState, '/joint_commands', 10)
        self.motors = self.create_publisher(Float64MultiArray, '/motor_angles', 10)
        self.status = self.create_publisher(String, '/ik_status', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray, '/platform_ypr', self.on_target, 10)
        unit = 'degrees' if self.degrees else 'radians'
        self.get_logger().info(f'Ready: /platform_ypr = [yaw, pitch, roll] in {unit}; outputs in radians.')

    def on_target(self, message):
        try:
            if len(message.data) != 3:
                raise IKError('Expected exactly [yaw, pitch, roll].')
            angles = np.asarray(message.data, dtype=float)
            if self.degrees:
                angles = np.deg2rad(angles)
            result = self.solver.solve(*angles)
        except IKError as exc:
            self.status.publish(String(data=json.dumps({'success': False, 'error': str(exc)})))
            self.get_logger().warning(str(exc))
            return
        command = JointState()
        command.header.stamp = self.get_clock().now().to_msg()
        command.name = list(URDF_INPUT_JOINTS)
        command.position = result.input_angles.tolist()
        self.joints.publish(command)
        self.motors.publish(Float64MultiArray(data=result.input_angles.tolist()))
        self.status.publish(String(data=json.dumps({
            'success': True, 'closure_error_m': result.closure_error_m})))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IKNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
