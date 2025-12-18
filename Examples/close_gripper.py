'''
Run this script using:

python3 close_gripper.py 192.168.1.208
'''

import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
# import pyRobotiqGripper
from Bestman_piper import *
from time import time, sleep
import numpy as np
def main():

    try:
        # Instantiate the robot interface
        bestman = Bestman_piper(obot_name="can0")

        # Clear fault on the robot server if any
        # if bestman.robot.isFault():
        #     log.warn("Fault occurred on the robot server, trying to clear ...")
        #     bestman.robot.clearFault()
        #     time.sleep(2)
        #     if bestman.robot.isFault():
        #         log.error("Fault cannot be cleared, exiting ...")
        #         return
        #     log.info("Fault on the robot server is cleared")

        # Close gripper
        bestman.close_gripper()
        # print(bestman.get_eef_pos())


    except Exception as e:
        # Log any exceptions that occur
        print(str(e))


if __name__ == "__main__":
    main()
