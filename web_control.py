"""Local joystick UI and IK API. Optional ROS 2 command output."""
import argparse
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
from pathlib import Path

import numpy as np

from ik import IKError, OrientationIK, URDF_INPUT_JOINTS

WEB = Path(__file__).resolve().parent / 'web'


class Controller:
    def __init__(self, ros=False):
        self.solver = OrientationIK()
        self.node = None
        if ros:
            import rclpy
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float64MultiArray
            rclpy.init(args=[])
            self.node = rclpy.create_node('spm_web_control')
            self.joints = self.node.create_publisher(JointState, '/joint_commands', 10)
            self.motors = self.node.create_publisher(Float64MultiArray, '/motor_angles', 10)
        self.state = {'ros': ros, 'ypr_degrees': [0, 0, 0], 'joint_degrees': [0, 0, 0],
                      'joint_radians': [0, 0, 0], 'closure_error_m': 0}

    def command(self, payload):
        if not isinstance(payload, dict):
            raise ValueError('Expected an object with yaw, pitch and roll.')
        values = [payload.get(k) for k in ('yaw', 'pitch', 'roll')]
        if any(type(v) not in (int, float) for v in values):
            raise ValueError('Yaw, pitch and roll must be numbers in degrees.')
        angles = np.asarray(values, dtype=float)
        if not np.isfinite(angles).all() or np.any(np.abs(angles) > [180, 20, 20]):
            raise ValueError('Limits: yaw ±180°, pitch and roll ±20°.')
        result = self.solver.solve(*np.deg2rad(angles))
        if self.node:
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Float64MultiArray
            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name = list(URDF_INPUT_JOINTS)
            msg.position = result.input_angles.tolist()
            self.joints.publish(msg)
            self.motors.publish(Float64MultiArray(data=msg.position))
        self.state = {'ros': self.node is not None, 'ypr_degrees': angles.tolist(),
                      'joint_degrees': np.rad2deg(result.input_angles).tolist(),
                      'joint_radians': result.input_angles.tolist(),
                      'closure_error_m': result.closure_error_m}
        return self.state

    def close(self):
        if self.node:
            import rclpy
            self.node.destroy_node()
            rclpy.shutdown()


def handler_for(controller):
    class Handler(BaseHTTPRequestHandler):
        def reply(self, status, data, content_type='application/json'):
            body = json.dumps(data).encode() if content_type == 'application/json' else data
            self.send_response(status)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == '/api/state':
                return self.reply(200, controller.state)
            files = {'/': ('index.html', 'text/html; charset=utf-8'),
                     '/app.js': ('app.js', 'text/javascript; charset=utf-8'),
                     '/style.css': ('style.css', 'text/css; charset=utf-8')}
            if self.path not in files:
                return self.reply(404, {'error': 'Not found'})
            name, content_type = files[self.path]
            self.reply(200, (WEB / name).read_bytes(), content_type)

        def do_POST(self):
            if self.path != '/api/command':
                return self.reply(404, {'error': 'Not found'})
            # Only same-origin JSON requests can change commands.
            if self.headers.get('Origin') not in (None, 'http://' + self.headers.get('Host', '')):
                return self.reply(403, {'error': 'Origin not allowed'})
            if self.headers.get('Content-Type', '').split(';')[0] != 'application/json':
                return self.reply(415, {'error': 'Use application/json'})
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 1024:
                    raise ValueError('Invalid request length')
                payload = json.loads(self.rfile.read(length))
                self.reply(200, controller.command(payload))
            except (ValueError, IKError) as exc:
                self.reply(422, {'error': str(exc)})

        def log_message(self, fmt, *args):
            if args and str(args[1]) not in ('200',):
                super().log_message(fmt, *args)
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--ros', action='store_true', help='Publish solved joint commands to ROS 2 (radians).')
    args = parser.parse_args()
    controller = Controller(ros=args.ros)
    server = HTTPServer(('127.0.0.1', args.port), handler_for(controller))
    server.timeout = .5
    print(f'Joystick: http://127.0.0.1:{args.port} — ROS output {"on" if args.ros else "off"}', flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        controller.close()


if __name__ == '__main__':
    main()
