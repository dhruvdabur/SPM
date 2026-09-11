import argparse
import os
import time

os.environ["GLFW_PLATFORM"] = "x11"
os.environ["XDG_SESSION_TYPE"] = "x11"

import mujoco.viewer
from simulation import Simulation, WebCommands


def main():
    parser = argparse.ArgumentParser(description='Run MuJoCo physics with browser joystick commands.')
    parser.add_argument('--web-url', default='http://127.0.0.1:8000',
                        help='Joystick server address (default: http://127.0.0.1:8000).')
    parser.add_argument('--manual', action='store_true', help='Open the original viewer without browser control.')
    args = parser.parse_args()
    sim = Simulation()
    if args.manual:
        mujoco.viewer.launch(sim.model, sim.data)
        return
    commands = WebCommands(args.web_url)
    commands.thread.start()
    print('Live physics: browser joystick drives motor_1, motor_2, motor_3. Close the window to stop.', flush=True)
    try:
        with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
            while viewer.is_running():
                started = time.monotonic()
                target = commands.latest()
                with viewer.lock():
                    if target is not None:
                        sim.set_target(target)
                    # Advance roughly one display frame of simulation time.
                    for _ in range(max(1, round(1 / (60 * sim.model.opt.timestep)))):
                        sim.step()
                viewer.sync()
                time.sleep(max(0, 1 / 60 - (time.monotonic() - started)))
    except KeyboardInterrupt:
        print('\nViewer closed (Ctrl+C).')
    finally:
        commands.close()


if __name__ == '__main__':
    main()
