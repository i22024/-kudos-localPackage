"#!/usr/bin/env python"
import rospy
from soccer_simu.msg import mcl2_local_result

class priROS():
    def __init__(self):
        pass

    def listener(self):
        rospy.init_node('soccer_simu_local_result_processor', anonymous = False)
        msg = rospy.wait_for_message("soccer_simu_local_result", mcl2_local_result, timeout=None)
        return msg.debug_num

if __name__=='__main__':
    priROS = priROS()

    while True:
        debug_num = priROS.listener()
        print(debug_num)
        

