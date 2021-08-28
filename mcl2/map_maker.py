'''
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
'''
import numpy as np
import sys
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import matplotlib.pyplot as plt
import struct

class map_maker():
    def __init__(self):
        self.border = 100
        self.play_ground_width = 900
        self.play_ground_height = 600
        self.map_width = 1100
        self.map_height = 800
        self.linethickness = 5
        self.circle_size = 75
        self.basic_field = np.empty((self.map_height, self.map_width), dtype=np.uint8)
        self.unity_field = np.empty((self.map_height, self.map_width, 3), dtype=np.uint8)
        self.mcl_field = np.empty((self.map_height, self.map_width, 3), dtype=np.uint8)
        self.dist_map = np.empty((self.map_height, self.map_width), dtype=np.float64)
        self.generate_data_path = "./bh_play_ground/python_dump_data/"
        self.logo_path = "./bh_play_ground/kudos_logo_img/"
    
    def configuration_basic_field(self):
        self.basic_field[:,:] = 0
        b = self.border
        pw = self.play_ground_width
        ph = self.play_ground_height
        # 중앙원, 가로 대 라인
        cv2.circle(self.basic_field, (pw//2+b, ph//2+b), self.circle_size, 255, self.linethickness)#
        cv2.line(self.basic_field, (b, b), (pw+b, b), 255, self.linethickness)
        cv2.line(self.basic_field, (b,b+ph), (pw+b, ph+b), 255, self.linethickness)
        # 가로 중 라인
        cv2.line(self.basic_field, (b,b+50), (b+100, b+50), 255, self.linethickness)#
        cv2.line(self.basic_field, (b,b+ph-50), (b+100, b+ph-50), 255, self.linethickness)#
        cv2.line(self.basic_field, (pw+b-100, b+50), (pw+b, b+50), 255, self.linethickness)#
        cv2.line(self.basic_field, (pw+b-100, ph+b-50), (pw+b, ph-50+b), 255, self.linethickness)#
        # 세로 라인들
        cv2.line(self.basic_field, (pw//2+b, b), (pw//2+b, ph+b), 255, self.linethickness)#
        cv2.line(self.basic_field, (b+100, b+50), (b+100, ph+b-50), 255, self.linethickness)#
        cv2.line(self.basic_field, (pw+b-100, b+50), (pw+b-100, ph+b-50), 255, self.linethickness)#
        cv2.line(self.basic_field, (b, b), (b, ph+b), 255, self.linethickness)
        cv2.line(self.basic_field, (pw+b, b), (pw+b, ph+b), 255, self.linethickness)
        # 추가 마크 작업
        cv2.circle(self.basic_field, (pw//2+b, ph//2+b), 10, 255, -1)#
        cv2.circle(self.basic_field, (pw//2-b, ph//2+b), 7, 255, -1)#
        cv2.circle(self.basic_field, (pw//2+3*b, ph//2+b), 7, 255, -1)#

    def make_unity_field(self):
        # BGR채널 순서로 관리
        ground_color = np.array([25, 230, 25])
        line_color = np.array([255, 255, 255])
        self.unity_field[:, :, :] = ground_color

        for row_index, row in enumerate(self.basic_field):
            for col_index, col in enumerate(row):
                if col == 255:
                    self.unity_field[row_index, col_index, :] = line_color

    def make_mcl_field(self):
        self.mcl_field = cv2.imread(self.logo_path+"resize_mcl_src.png", cv2.IMREAD_COLOR)
        line_color = np.array([255, 255, 255])
        for row_index, row in enumerate(self.basic_field):
            for col_index, col in enumerate(row):
                if col == 255:
                    self.mcl_field[row_index, col_index, :] = line_color

    def make_distance_map(self):
        out = 255 - self.basic_field.copy()
        self.dist_map = cv2.distanceTransform(out, cv2.DIST_L2, 0)
        self.dist_map = cv2.normalize(self.dist_map, None, 1.0, 0, cv2.NORM_MINMAX, cv2.CV_64FC1)
        norm_dist_map = cv2.normalize(self.dist_map, None, 255, 0, cv2.NORM_MINMAX, cv2.CV_8UC1)
        return norm_dist_map

    def dump_distance_map_bin(self):
        f = open(self.generate_data_path+'bh_dist.bin', 'wb')
        for i in range(self.map_width):
            for j in range(self.map_height):
                data = struct.pack('d', self.dist_map[j][i])
                f.write(data)
        f.close()





if __name__=='__main__':
    map_maker = map_maker()

    map_maker.configuration_basic_field()
    #cv2.imshow('basic_field', map_maker.basic_field)
    plt.imshow(map_maker.basic_field)
    plt.show()
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
    map_maker.make_unity_field()
    #cv2.imshow('unity_field', map_maker.unity_field)
    plt.imshow(map_maker.unity_field)
    plt.show()
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    map_maker.make_mcl_field()
    #cv2.imshow('mcl_field', map_maker.mcl_field)
    plt.imshow(map_maker.mcl_field)
    plt.show()
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    
    norm_dist_map = map_maker.make_distance_map()
    #cv2.imshow("distance_map", norm_dist_map)
    plt.imshow(norm_dist_map)
    plt.show()
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()

    cv2.imwrite(map_maker.generate_data_path+"unity_field.png", map_maker.unity_field)
    cv2.imwrite(map_maker.generate_data_path+"mcl_field.png", map_maker.mcl_field)
    map_maker.dump_distance_map_bin()
