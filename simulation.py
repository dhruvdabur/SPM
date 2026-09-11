"""Physics stepping and a background receiver for the joystick server."""
import json
import queue
import threading
from urllib.error import URLError
from urllib.request import urlopen

import mujoco
import numpy as np

from ik import DEFAULT_MODEL


class Simulation:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_path(str(DEFAULT_MODEL))
        self.data = mujoco.MjData(self.model)
        self.motor_ids = [self.model.actuator(f'motor_{i}').id for i in range(1, 4)]
        self.target = np.zeros(3)
        mujoco.mj_forward(self.model, self.data)

    def set_target(self, angles):
        angles = np.asarray(angles, dtype=float)
        if angles.shape != (3,) or not np.isfinite(angles).all():
            raise ValueError('Expected three finite motor angles in radians.')
        self.target = angles.copy()

    def step(self):
        # Ramp all three servos together, limiting the largest change to 0.5 rad/s.
        current = self.data.ctrl[self.motor_ids]
        delta = self.target - current
        distance = float(np.max(np.abs(delta)))
        fraction = min(1.0, 0.5 * self.model.opt.timestep / distance) if distance else 1.0
        self.data.ctrl[self.motor_ids] = current + fraction * delta
        mujoco.mj_step(self.model, self.data)


class WebCommands:
    """Poll outside the physics thread; only the latest valid target is retained."""
    def __init__(self, url):
        self.url = url.rstrip('/') + '/api/state'
        self.pending = queue.Queue(maxsize=1)
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def poll(self):
        with urlopen(self.url, timeout=0.5) as response:
            state = json.load(response)
        angles = np.asarray(state['joint_radians'], dtype=float)
        if angles.shape != (3,) or not np.isfinite(angles).all():
            raise ValueError('Invalid motor angles from joystick server.')
        return angles

    def _run(self):
        connected = None
        while not self.stop.is_set():
            try:
                angles = self.poll()
                if connected is not True:
                    print(f'Connected to {self.url}', flush=True)
                connected = True
                try:
                    self.pending.get_nowait()
                except queue.Empty:
                    pass
                self.pending.put_nowait(angles)
            except (URLError, OSError, ValueError, KeyError, TypeError) as exc:
                if connected is not False:
                    print(f'Waiting for joystick server at {self.url}: {exc}', flush=True)
                connected = False
            self.stop.wait(0.1)

    def latest(self):
        try:
            return self.pending.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
