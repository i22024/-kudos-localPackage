"#!/usr/bin/env python"
from mlagents_envs.rpc_communicator import RpcCommunicator
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
import numpy as np
import sys
import rospy
from soccer_simu.msg import mcl2_sensor_data
from numpy.core.numeric import empty_like
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import random
from PIL import Image
from tqdm import tqdm
import cv2
import user_function.CustomFuncionFor_mlAgent as CF
import user_function.image_filter as MY_IMAGE_FILTER
import math

import time

game = "vision_simulator.x86_64"
env_path = "./Linux_build/" + game
save_picture_path = "./made_data/"
channel = EngineConfigurationChannel()
channel.set_configuration_parameters(time_scale=1.0, target_frame_rate=60, capture_frame_rate=60)
env = UnityEnvironment(file_name=env_path, side_channels=[channel])
env.reset()
behavior_names = list(env.behavior_specs)
ConversionDataType = CF.ConversionDataType()

AgentsHelper = CF.AgentsHelper(env, string_log=None, ConversionDataType=ConversionDataType)
write_file_name_list_index_instead_of_correct_name = False
list_index_for_ALL = 0
list_index_for_ball = 2
list_index_for_stage = 1
list_index_for_goal1_detection = 3
list_index_for_goal2_detection = 4
list_index_for_goal1_range = 5
list_index_for_goal2_range = 6
list_index_for_top_view = 7

# hsv범위
mask_minimum_condition = np.array([0, 0, 235])
mask_maximum_condition = np.array([180, 10, 255])
# 데이터에서 선을 추출하기 전, 변환할 사이즈와 앞으로 밀어낼 사이즈, ROI사이즈를 결정(픽셀단위)
pre_processing_size = 450
roi_size = {"x_size":350, "y_size":450, "pre_x":100}
push_move = 120
# 팽창과 침식 정도를 결정하는 변수이다.
dilate_power = 5 #팽창강도
erode_power = 7 #침식강도
# 대표 픽셀을 추출할 때 기준 거리를 결정하는 변수이다.
standard_pixel_distance = 40
delete_noise_distance = 120

class priROS():
    def __init__(self):
        pass

    def talker(self, message_form):
        pub = rospy.Publisher('soccer_simu_sensor_data', mcl2_sensor_data, queue_size=1)
        rospy.init_node('soccer_simu_sensor_processor', anonymous = False)
        message = mcl2_sensor_data()
        message.debug_num = message_form['debug_num']
        message.sensor_data_x = message_form['sensor_data_x']
        message.sensor_data_y = message_form['sensor_data_y']
        pub.publish(message)

class useful_function():
    def __init__(self):
        self.pts = np.zeros((4, 2), dtype=np.float32)
        self.pts1 = 0
        self.pts2 = 0

    def get_distance_from_two_points(self, point1, point2):
        # 포인트의 형식은 리스트[x좌표, y좌표]
        distance = math.sqrt(((point1[0] - point2[0]) ** 2) + ((point1[1] - point2[1]) ** 2))
        return distance

    def save_numpy_file(self, append_name, img):
        im = Image.fromarray(img.astype('uint8'), 'RGB')
        im.save(save_picture_path + append_name + '.jpg')

    def perspective(self, top_view_npArr, pre_processing_size):
        empty_size_mul = 5
        cp_size_left_loc = 2
        cp_size_right_loc = 3
        img_shape = np.shape(top_view_npArr)
        top_view_npArr = np.zeros((img_shape[0], img_shape[1]*empty_size_mul, 3), dtype=np.uint8)
        top_view_npArr[: ,int(img_shape[1])*cp_size_left_loc:int(img_shape[1]*cp_size_right_loc), :] = view_npArr
        topLeft = [img_shape[0]*cp_size_left_loc,0]
        bottomRight = [img_shape[0]*empty_size_mul, img_shape[0]]
        topRight = [img_shape[0]*cp_size_right_loc,0]
        bottomLeft = [0, img_shape[0]]
        self.pts1 = np.float32([topLeft, topRight, bottomRight, bottomLeft])
        w1 = abs(bottomRight[0] - bottomLeft[0])
        w2 = abs(topRight[0] - topLeft[0])
        h1 = abs(topRight[1] - bottomRight[1])
        h2 = abs(topLeft[1] - bottomLeft[1])
        width = max([w1, w2])  # 두 좌우 거리간의 최대값이 서류의 폭
        height = max([h1, h2])  # 두 상하 거리간의 최대값이 서류의 높이
        self.pts2 = np.float32([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]])
        mtrx = cv2.getPerspectiveTransform(self.pts1, self.pts2)
        result = cv2.warpPerspective(top_view_npArr, mtrx, (int(width), int(height)))
        result = cv2.resize(result, dsize=(pre_processing_size, pre_processing_size), interpolation=cv2.INTER_AREA)
        result = np.swapaxes(result, 0, 1)
        result = np.flip(result)

        return result
    
    def Image_mask(self, mask_minimum_condition, mask_maximum_condition, roi_size, dilate_power, erode_power):
        hsv_top_view_npArr = cv2.cvtColor(top_view_npArr, cv2.COLOR_BGR2HSV)
        '''
        h2, s2, v2 = cv2.split(hsv_top_view_npArr)
        cv2.imshow('h', h2) # 색상 범위는 (0~180), 하얀색에서는 채도가 제대로 결정되지 않음
        cv2.imshow('s', s2) # 채도 범위는 원색의 강렬함 정도, 0~255사이의 값으로 표현된다. 하얀색에서는 원색의 강렬함 정도가 낮게 표현된
        cv2.imshow('v', v2) # 명도는 색의 밝고 어두운 정도를 표현한다. 0~255사이의 값으로 표현된다. 시뮬상에서는 하얀색과 초록색이 구분되지 않는다.
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        '''
        # 지정한 범위로 마스크를 씌워 하얀색 영역만 검출해낸다.
        masked_hsv_top_view_npArr = cv2.inRange(hsv_top_view_npArr, mask_minimum_condition, mask_maximum_condition)

        #위치 검출기의 범위를 벗어나서 검출되는 선이 발생하지 않도록 ROI를 설정한다.
        masked_hsv_top_view_npArr = masked_hsv_top_view_npArr[:,roi_size["pre_x"]:roi_size["pre_x"]+roi_size["x_size"]]

        # 팽창을 이용하여 라인사이에 존재하는 노이즈를 없애고 침식을 이용해 라인을 얇게 형성한다.
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (dilate_power, dilate_power))
        masked_hsv_top_view_npArr = cv2.dilate(masked_hsv_top_view_npArr, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (erode_power, erode_power))
        masked_hsv_top_view_npArr = cv2.erode(masked_hsv_top_view_npArr, kernel)

        return masked_hsv_top_view_npArr
    
    def get_profit_point(self, indexing_masked, standard_pixel_distance, delete_noise_distance):
        circle_img = np.zeros_like(masked_top_view_npArr, np.uint8)
        point_list = []
        for index in range(len(indexing_masked[0])):
            tmp_point = [indexing_masked[0][index], indexing_masked[1][index]]
            if len(point_list) == 0:
                point_list.append(tmp_point)
            else:
                min_distance = 999
                for point in point_list:
                     distance = self.get_distance_from_two_points(point, tmp_point)
                     if min_distance>distance:
                         min_distance = distance
                if min_distance > standard_pixel_distance:
                    point_list.append(tmp_point)

        final_point_list = point_list.copy()
        total_point_list = []
        for point in final_point_list:
            min_distance = 9999
            point_list.remove(point)
            for target_point in point_list:
                distance = self.get_distance_from_two_points(target_point, point)
                if distance<min_distance:
                    min_distance = distance
            if min_distance<delete_noise_distance:
                total_point_list.append(point)
            point_list.append(point)

        return total_point_list


if __name__ == "__main__":
    priROS = priROS()
    useful_function = useful_function()
    count = 0
    while 1:
        behavior_name = behavior_names[0]
        decision_steps, terminal_steps = env.get_steps(behavior_name)
        vec_observation, vis_observation_list, done = AgentsHelper.getObservation(behavior_name)
        view_npArr = vis_observation_list[list_index_for_stage]
        top_view_npArr = useful_function.perspective(view_npArr, pre_processing_size)
        '''
        cv2.imshow('top_view', top_view_npArr)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        '''
        masked_top_view_npArr = useful_function.Image_mask(mask_minimum_condition, mask_maximum_condition, roi_size, dilate_power, erode_power)
        indexing_masked = np.where(masked_top_view_npArr>254)
        '''
        cv2.imshow('masked_top_view_npArr', masked_top_view_npArr)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        '''

        point_list = useful_function.get_profit_point(indexing_masked, standard_pixel_distance, delete_noise_distance)
        circle_img = np.zeros_like(masked_top_view_npArr, np.uint8)
        for point in point_list:
            cv2.circle(circle_img, (point[1], point[0]), 5, 255)
        '''
        cv2.imshow('circle_img', circle_img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        '''
        message_form = {
            "debug_num":1,
            "sensor_data_x":[],
            "sensor_data_y":[]}
        for point in point_list:
            message_form["sensor_data_y"].append(point[0]-(pre_processing_size//2))
            message_form["sensor_data_x"].append(point[1]+push_move)
        for i in range(100-len(point_list)):
            message_form["sensor_data_x"].append(-100)
            message_form["sensor_data_y"].append(-100)

        priROS.talker(message_form)

        if count<2:
            a = input("사용자의 입력을 기다리는 중입니다.")
            count = count+1

        action = [3, 8, 0, 40]
        actionTuple = ConversionDataType.ConvertList2DiscreteAction(action, behavior_name)
        env.set_actions(behavior_name, actionTuple)
        env.step()
    env.close()