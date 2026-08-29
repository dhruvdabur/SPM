import os
import warnings

warnings.filterwarnings("ignore", category=UserWarning)

os.environ["GLFW_PLATFORM"] = "x11"
os.environ["XDG_SESSION_TYPE"] = "x11"

import mujoco
import mujoco.viewer

model = mujoco.MjModel.from_xml_path("models/3rrr.xml")
data = mujoco.MjData(model)

try:
    # Launch interactive viewer window
    mujoco.viewer.launch(model, data)
except KeyboardInterrupt:
    print("\nViewer closed (Ctrl+C).")