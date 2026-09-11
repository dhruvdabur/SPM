import unittest

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from ik import IKError, OrientationIK


class IKTests(unittest.TestCase):
    def setUp(self):
        self.ik = OrientationIK()

    def check_pose(self, result, ypr):
        data = mujoco.MjData(self.ik.model)
        data.qpos[:] = result.qpos
        mujoco.mj_forward(self.ik.model, data)
        for a, b in self.ik.sites:
            self.assertLess(np.linalg.norm(data.site_xpos[a] - data.site_xpos[b]), 1e-6)
        actual = Rotation.from_matrix(data.body('top_plate').xmat.reshape(3, 3))
        target = Rotation.from_euler('ZYX', ypr) * self.ik.home_rotation
        self.assertLess((actual * target.inv()).magnitude(), 1e-8)

    def test_home_and_pure_yaw(self):
        for yaw in [0, .1, .3, -.2, 0]:
            result = self.ik.solve(yaw, 0, 0)
            np.testing.assert_allclose(result.input_angles, [-yaw]*3, atol=1e-5)
            self.check_pose(result, [yaw, 0, 0])

    def test_mixed_orientations(self):
        rng = np.random.default_rng(5)
        for ypr in rng.uniform(-.2, .2, (25, 3)):
            self.check_pose(self.ik.solve(*ypr), ypr)

    def test_invalid_input_preserves_previous_solution(self):
        self.ik.solve(.1, .05, -.05)
        before = self.ik.previous.copy()
        for invalid in [float('nan'), float('inf')]:
            with self.assertRaises(IKError):
                self.ik.solve(invalid, 0, 0)
            np.testing.assert_array_equal(self.ik.previous, before)

    def test_dynamics_reaches_requested_orientation(self):
        ypr = [.1, .05, -.05]
        result = self.ik.solve(*ypr)
        data = mujoco.MjData(self.ik.model)
        for k in range(3000):
            data.ctrl[:] = result.input_angles * min(k / 500, 1)
            mujoco.mj_step(self.ik.model, data)
        actual = Rotation.from_matrix(data.body('top_plate').xmat.reshape(3, 3))
        target = Rotation.from_euler('ZYX', ypr) * self.ik.home_rotation
        self.assertLess((actual * target.inv()).magnitude(), 1e-4)
        self.assertEqual(sum(w.number for w in data.warning), 0)

    def test_unsolved_target_preserves_branch(self):
        self.ik.solve(.1, 0, 0)
        before = self.ik.previous.copy()
        with self.assertRaises(IKError):
            self.ik.solve(0, np.pi / 2, 0)
        np.testing.assert_array_equal(self.ik.previous, before)
        self.check_pose(self.ik.solve(.12, 0, 0), [.12, 0, 0])


if __name__ == '__main__':
    unittest.main()
