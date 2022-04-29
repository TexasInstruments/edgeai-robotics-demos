import sys
import time
from scuttlepy import *

config = '/opt/robot/edgeai-robotics-demos/python/common/scuttlepy/config/scuttle_sk_config.yaml'

try:
    robot = SCUTTLE(config, openLoop=True)
except Exception as e:
    print("Exception: {0}".format(e))
    sys.exit(2)

robot.setMotion([1.0, 0.0])
time.sleep(5)
robot.setMotion([1.0, 1.0])
time.sleep(5)
robot.setMotion([0.0, 1.0])
time.sleep(5)
robot.setMotion([0.0, -1.0])
time.sleep(5)
robot.setMotion([1.0, -1.0])
time.sleep(5)
robot.setMotion([-1.0, -1.0])
time.sleep(5)
robot.setMotion([-1.0, 1.0])
time.sleep(5)
robot.setMotion([-1.0, 0.0])
time.sleep(5)
robot.stop()
del robot

