import struct
import numpy as np
import matplotlib
#matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
path = "./bh_play_ground/python_dump_data/dist.bin"
binary_list = []
len_of_data = 880000
with open(path, "rb") as f:
    data = f.read()
    for i in range(len_of_data):
        my_double = struct.unpack('d', data[8*i:8*(i+1)])
        my_double = my_double[0]
        binary_list.append(my_double)

check_bin_img = np.empty((800,1100), dtype=np.float64)
for index_row in range(800):
    for index_col in range(1100):
        check_bin_img[index_row][index_col] = binary_list[800*index_col+index_row]

print(check_bin_img)
plt.imshow(check_bin_img)
plt.show()
