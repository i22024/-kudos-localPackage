import cv2
import numpy as np
import random

def custom_gaussian_filter(src, sigma):
    processed_image = cv2.GaussianBlur(src, (0,0), sigma)
    return processed_image

def custom_motion_blur_filter(src, power):
    size = power
    gen_motion_blur_filter = np.zeros((size, size))
    random_direction = random.randint(0, 3)
    if random_direction == 0:
        gen_motion_blur_filter = np.eye(size)
    elif random_direction == 1:
        gen_motion_blur_filter = np.eye(size)
        gen_motion_blur_filter = np.flip(gen_motion_blur_filter, axis=0)
    elif random_direction == 2:
        gen_motion_blur_filter[int((size - 1) / 2), :] = np.ones(size)
    elif random_direction == 3:
        gen_motion_blur_filter[:, int((size - 1) / 2)] = np.ones(size)
    gen_motion_blur_filter = gen_motion_blur_filter/size
    processed_image = cv2.filter2D(src, -1, gen_motion_blur_filter)
    return processed_image