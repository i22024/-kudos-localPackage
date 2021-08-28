"#!/usr/bin/env python"
# 라이브러리 임포트
import rospy
from fake_op3.msg import kudos_vision_ball_position as kvbp
from fake_op3.msg import kudos_vision_gcm as kvg
from fake_op3.msg import kudos_vision_head_pub as kvhp
from fake_op3.msg import kudos_vision_mcl2_local_result as kvmlr
from fake_op3.msg import kudos_vision_op3_local_mode as kvolm

message_form = {'op3_local_mode':False,
                'start_point_x':0,
                'start_point_y':0,
                'start_point_orien':0}

class priROS():
    def __init__(self):
        pass

    def talker(self, message_form):
        pub = rospy.Publisher('kudos_vision_op3_local_mode', kvolm, queue_size=1)
        message = kvolm()
        message.op3_local_mode = message_form['op3_local_mode']
        message.start_point_x = message_form['start_point_x']
        message.start_point_y = message_form['start_point_y']
        message.start_point_orien = message_form['start_point_orien']
        pub.publish(message)

def when_receive_other_message(ros_data, args):
    priROS = args[0]
    global message_form
    priROS.talker(message_form)

def when_receive_target_message(ros_data, args):
    priROS = args[0]
    global message_form
    if ros_data.vision_go_local == -1:
        message_form["op3_local_mode"] = False
    elif ros_data.vision_go_local == 1:
        message_form["op3_local_mode"] = True
    message_form["start_point_x"] = ros_data.fake_start_point_x
    message_form["start_point_y"] = ros_data.fake_start_point_y
    message_form["start_point_orien"] = ros_data.fake_start_point_orien
    priROS.talker(message_form)


if __name__=='__main__':
    priROS = priROS()
    rospy.init_node('kudos_vision_fake_op3', anonymous = False)
    rospy.Subscriber("kudos_vision_ball_position", kvbp, when_receive_other_message, (priROS, ), queue_size=1)
    rospy.Subscriber("kudos_vision_head_pub", kvhp, when_receive_other_message, (priROS, ), queue_size=1)
    rospy.Subscriber("kudos_vision_mcl2_local_result", kvmlr, when_receive_other_message, (priROS, ), queue_size=1)
    rospy.Subscriber("kudos_vision_gcm", kvg, when_receive_target_message, (priROS, ), queue_size=1)
    rospy.spin()
