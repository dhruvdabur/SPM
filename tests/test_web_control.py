import json
import threading
import unittest
from http.server import HTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from web_control import Controller, handler_for
from simulation import Simulation, WebCommands


class WebTests(unittest.TestCase):
    def setUp(self):
        self.controller = Controller()
        self.server = HTTPServer(('127.0.0.1', 0), handler_for(self.controller))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.url = f'http://127.0.0.1:{self.server.server_port}'

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.thread.join()

    def post(self, payload):
        request = Request(self.url+'/api/command', data=json.dumps(payload).encode(),
                          headers={'Content-Type':'application/json'})
        return json.load(urlopen(request))

    def test_command_and_state(self):
        result = self.post({'yaw':10, 'pitch':5, 'roll':3})
        self.assertEqual(result['ypr_degrees'], [10,5,3])
        self.assertAlmostEqual(result['joint_degrees'][0], -5.536645, places=5)
        self.assertEqual(json.load(urlopen(self.url+'/api/state')), result)
        self.assertIn(b'joystick', urlopen(self.url).read())

    def test_invalid_commands_do_not_change_state(self):
        previous = dict(self.controller.state)
        for payload in [{'yaw':0,'pitch':30,'roll':0}, {'yaw':True,'pitch':0,'roll':0}, {}, []]:
            with self.assertRaises(HTTPError) as exc:
                self.post(payload)
            self.assertEqual(exc.exception.code,422)
            self.assertEqual(self.controller.state,previous)

    def test_browser_command_moves_physics_and_returns_home(self):
        sim = Simulation()
        receiver = WebCommands(self.url)
        for ypr in ([10, 5, 3], [0, 0, 0]):
            self.post(dict(zip(('yaw', 'pitch', 'roll'), ypr)))
            sim.set_target(receiver.poll())
            for _ in range(3000):
                sim.step()
            mujoco.mj_forward(sim.model, sim.data)
            actual = Rotation.from_matrix(sim.data.body('top_plate').xmat.reshape(3, 3))
            expected = Rotation.from_euler('ZYX', ypr, degrees=True)
            self.assertLess((actual * expected.inv()).magnitude(), 1e-4)
            self.assertEqual(sum(w.number for w in sim.data.warning), 0)

    def test_invalid_simulation_target_is_rejected(self):
        sim = Simulation()
        for target in ([0, 0], [0, float('nan'), 0]):
            with self.assertRaises(ValueError):
                sim.set_target(target)
            np.testing.assert_array_equal(sim.target, [0, 0, 0])

    def test_background_receiver_delivers_web_target(self):
        expected = self.post({'yaw': 10, 'pitch': 5, 'roll': 3})
        receiver = WebCommands(self.url)
        receiver.thread.start()
        try:
            np.testing.assert_allclose(receiver.pending.get(timeout=2), expected['joint_radians'])
        finally:
            receiver.close()
        self.assertFalse(receiver.thread.is_alive())


if __name__ == '__main__':
    unittest.main()
