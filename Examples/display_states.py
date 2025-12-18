'''
Run this script using:
python /home/$(whoami)/BestMan_Flexiv/Examples/display_states.py 192.168.2.100 192.168.2.108
'''

import sys
import os
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(os.path.join(parent_dir, 'RoboticsToolBox'))
from Bestman_sim_flexiv import *
import time

def main():
    try:
        # Initialize and clear faults
        bestman = Bestman_piper(obot_name="can0")
        while True:
            print(bestman.robot.GetArmJointMsgs())
            print(bestman.robot.GetAllMotorAngleLimitMaxSpd())
            print(bestman.robot.GetArmEndPoseMsgs())
            print(bestman.robot.GetArmLowSpdInfoMsgs())
            print(bestman.robot.GetArmStatus())
            print(bestman.robot.GetArmGripperMsgs())
            print(bestman.robot.GetArmHighSpdInfoMsgs())
            print(bestman.robot.GetCurrentMotorAngleLimitMaxVel())       
            print(bestman.robot.GetCurrentEndVelAndAccParam())
            print(bestman.robot.GetCrashProtectionLevelFeedback())
            print(bestman.robot.GetGripperTeachingPendantParamFeedback())
            print(bestman.robot.GetCurrentMotorMaxAccLimit())
            print(bestman.robot.GetPiperFirmwareVersion())
            time.sleep(0.5)
    except Exception as e:
        # Print exception error message
        bestman.log.error(str(e))

if __name__ == "__main__":
    main()