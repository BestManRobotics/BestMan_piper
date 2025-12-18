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

        bestman.gripper_goto(60)## unit mm
        # print(bestman.get_eef_pos())


    except Exception as e:
        # Log any exceptions that occur
        print(str(e))


if __name__ == "__main__":
    main()
