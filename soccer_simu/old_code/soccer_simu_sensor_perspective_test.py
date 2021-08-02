"#!/usr/bin/env python"
from mlagents_envs.rpc_communicator import RpcCommunicator
from mlagents_envs.environment import UnityEnvironment
from mlagents_envs.base_env import ActionTuple
from mlagents_envs.side_channel.engine_configuration_channel import EngineConfigurationChannel
import numpy as np
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import random
from PIL import Image
from tqdm import tqdm
import cv2
import user_function.CustomFuncionFor_mlAgent as CF
import user_function.image_filter as MY_IMAGE_FILTER
import math

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

# 데이터에서 선을 추출하기 전, 변환할 사이즈와 앞으로 밀어낼 사이즈, ROI사이즈를 결정(픽셀단위)
pre_processing_size = 600

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

    def perspective(self, top_view_npArr):
        topLeft = [832,0]  # x+y가 가장 값이 가장 작은게 좌상단 좌표
        bottomRight = [2080, 416]  # x+y가 가장 큰 값이 우하단 좌표
        topRight = [1248,0]  # x-y가 가장 작은 것이 우상단 좌표
        bottomLeft = [0, 416]  # x-y가 가장 큰 값이 좌하단 좌표
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

        return result
        

if __name__ == "__main__":
    useful_function = useful_function()
    while 1:
        behavior_name = behavior_names[0]
        decision_steps, terminal_steps = env.get_steps(behavior_name)
        vec_observation, vis_observation_list, done = AgentsHelper.getObservation(behavior_name)
        view_npArr = vis_observation_list[list_index_for_stage]
        view_npArr_shape = np.shape(view_npArr)
        top_view_npArr = np.zeros((view_npArr_shape[0], view_npArr_shape[1]*5, 3), dtype=np.uint8)
        top_view_npArr[: ,int(view_npArr_shape[1])*2:int(view_npArr_shape[1]*3), :] = view_npArr
        win_name = "scanning"
        rows, cols = top_view_npArr.shape[:2]
        top_view_npArr = useful_function.perspective(top_view_npArr)
        cv2.imshow('scanned', top_view_npArr)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        top_view_npArr = cv2.resize(top_view_npArr, dsize=(pre_processing_size, pre_processing_size), interpolation=cv2.INTER_AREA)
        cv2.imshow('top_view', top_view_npArr)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        action = [3, 0, 0, 40]
        actionTuple = ConversionDataType.ConvertList2DiscreteAction(action, behavior_name)
        env.set_actions(behavior_name, actionTuple)
        env.step()
    env.close()




