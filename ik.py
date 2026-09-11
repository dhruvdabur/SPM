"""Orientation IK for the closed-loop model, using its actual CAD coordinates."""
import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import sys

# Select the viewer backend before importing MuJoCo, which imports GLFW.
# Match main.py's X11 workaround when launching the CLI viewer on Linux.
if __name__ == '__main__' and '--view' in sys.argv and sys.platform.startswith('linux'):
    os.environ['GLFW_PLATFORM'] = 'x11'
    os.environ['XDG_SESSION_TYPE'] = 'x11'

import mujoco
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

DEFAULT_MODEL = Path(__file__).resolve().parent / 'models' / '3rrr.xml'
INPUT_JOINTS = ('q1', 'q2', 'q3')
PASSIVE_JOINTS = ('blue1_rev', 'blue2_rev', 'blue3_rev')
URDF_INPUT_JOINTS = tuple(f'level-{i}-gear-rev' for i in range(1, 4))


class IKError(ValueError):
    """No acceptable solution was found on the current assembly branch."""


@dataclass
class Solution:
    input_angles: np.ndarray
    passive_angles: np.ndarray
    qpos: np.ndarray
    closure_error_m: float


class OrientationIK:
    """Track the home assembly branch; all angles are radians.

    Orientation is relative to the model's initial top-plate orientation,
    expressed about the world/base axes: Rz(yaw) Ry(pitch) Rx(roll).
    Platform translation is solved, not held fixed at its off-center origin.
    """

    def __init__(self, model_path=DEFAULT_MODEL):
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        self.names = INPUT_JOINTS + PASSIVE_JOINTS
        self.qadr = np.array([self.model.joint(n).qposadr[0] for n in self.names])
        self.dadr = np.array([self.model.joint(n).dofadr[0] for n in self.names])
        joint = self.model.joint('top_plate_free')
        self.free_qadr = int(joint.qposadr[0])
        self.free_dadr = int(joint.dofadr[0])
        self.sites = [(self.model.site(f'blue_{i}_top').id,
                       self.model.site(f'plate_joint_{i}').id) for i in range(1, 4)]
        self.home = self.model.qpos0.copy()
        self.home_rotation = Rotation.from_quat(np.roll(
            self.home[self.free_qadr + 3:self.free_qadr + 7], -1))
        self.previous = self.home.copy()
        self.previous_rotation = Rotation.identity()
        self.lower = np.full(9, -np.inf)
        self.upper = np.full(9, np.inf)
        for i, name in enumerate(self.names):
            j = self.model.joint(name)
            if j.limited[0]:
                self.lower[i], self.upper[i] = j.range

    def _solve_rotation(self, relative_rotation, seed):
        rotation = relative_rotation * self.home_rotation
        quat = np.roll(rotation.as_quat(), 1)
        x0 = np.r_[seed[self.qadr], seed[self.free_qadr:self.free_qadr + 3]]

        def update(x):
            self.data.qpos[:] = self.home
            self.data.qpos[self.qadr] = x[:6]
            self.data.qpos[self.free_qadr:self.free_qadr + 3] = x[6:]
            self.data.qpos[self.free_qadr + 3:self.free_qadr + 7] = quat
            mujoco.mj_kinematics(self.model, self.data)
            mujoco.mj_comPos(self.model, self.data)

        def residual(x):
            update(x)
            return np.concatenate([self.data.site_xpos[a] - self.data.site_xpos[b]
                                   for a, b in self.sites])

        def jacobian(x):
            update(x)
            rows = []
            for a, b in self.sites:
                ja = np.zeros((3, self.model.nv))
                jb = np.zeros_like(ja)
                mujoco.mj_jacSite(self.model, self.data, ja, None, a)
                mujoco.mj_jacSite(self.model, self.data, jb, None, b)
                diff = ja - jb
                rows.append(np.c_[diff[:, self.dadr],
                                  diff[:, self.free_dadr:self.free_dadr + 3]])
            return np.vstack(rows)

        fit = least_squares(residual, x0, jac=jacobian,
                            bounds=(self.lower, self.upper),
                            x_scale=np.r_[np.ones(6), [.1]*3],
                            ftol=1e-12, xtol=1e-12, gtol=1e-12, max_nfev=150)
        error = float(np.max(np.linalg.norm(residual(fit.x).reshape(3, 3), axis=1)))
        if not fit.success or error > 1e-6 or not np.isfinite(fit.x).all():
            raise IKError(f'No solution on current branch (closure error {error:.3g} m).')
        # A small orientation step must not jump to another assembly branch.
        if np.max(np.abs(fit.x[:6] - x0[:6])) > .5:
            raise IKError('Solution would jump assembly branch; approach with smaller commands.')
        return self.data.qpos.copy(), error

    def solve(self, yaw, pitch, roll):
        angles = np.asarray([yaw, pitch, roll], dtype=float)
        if not np.isfinite(angles).all():
            raise IKError('Yaw, pitch and roll must be finite numbers.')
        target = Rotation.from_euler('ZYX', angles)
        delta = (target * self.previous_rotation.inv()).as_rotvec()
        steps = max(1, int(np.ceil(np.linalg.norm(delta) / np.deg2rad(2))))
        seed = self.previous.copy()
        for step in range(1, steps + 1):
            intermediate = Rotation.from_rotvec(delta * step / steps) * self.previous_rotation
            seed, error = self._solve_rotation(intermediate, seed)
        # Commit only after the entire requested move is solvable.
        self.previous = seed.copy()
        self.previous_rotation = target
        return Solution(seed[self.qadr[:3]].copy(), seed[self.qadr[3:]].copy(), seed, error)


def main():
    parser = argparse.ArgumentParser(description='Calculate 3RRR joint angles from platform yaw pitch roll.')
    parser.add_argument('yaw', type=float)
    parser.add_argument('pitch', type=float)
    parser.add_argument('roll', type=float)
    parser.add_argument('--degrees', action='store_true', help='Interpret input angles as degrees (default: radians).')
    parser.add_argument('--model', type=Path, default=DEFAULT_MODEL)
    parser.add_argument('--view', action='store_true', help='Open the solved pose, paused, in MuJoCo.')
    args = parser.parse_args()
    solver = OrientationIK(args.model)
    angles = np.array([args.yaw, args.pitch, args.roll])
    if args.degrees:
        angles = np.deg2rad(angles)
    try:
        result = solver.solve(*angles)
    except IKError as exc:
        parser.exit(2, f'IK failed: {exc}\n')
    print(json.dumps({'joint_names': INPUT_JOINTS,
                      'radians': result.input_angles.tolist(),
                      'degrees': np.rad2deg(result.input_angles).tolist(),
                      'passive_radians': result.passive_angles.tolist(),
                      'closure_error_m': result.closure_error_m}, indent=2))
    if args.view:
        import time
        import mujoco.viewer
        data = mujoco.MjData(solver.model)
        data.qpos[:] = result.qpos
        data.ctrl[:] = result.input_angles
        mujoco.mj_forward(solver.model, data)
        with mujoco.viewer.launch_passive(solver.model, data) as viewer:
            while viewer.is_running():
                viewer.sync()
                time.sleep(1 / 60)


if __name__ == '__main__':
    main()
