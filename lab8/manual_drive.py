#!/usr/bin/env python
from eye import *
import os
import cv2
import time
import numpy as np
import ctypes
import datetime
import pygame
from pygame.locals import *

# ================== Parameters ====================
FRAME_RATE = 20

# Manual control
max_speed = 200
min_speed = 0
speed = 0
speed_increment = 20

max_steer = 30
min_steer = -30
steering_angle = 0
steer_increment = 3

# Image recording
Image_Count = 0
Record_Button = 0
Recording_Data = False

New_Image_Save_Path = ""

Script_Path = os.path.dirname(os.path.abspath(__file__))
Image_Path = os.path.join(Script_Path, 'ImageDatasets')

# ================== Image Preprocessing ====================
def Image_Processing(image):
    # First cast raw pointer to POINTER of correct type
    ptr_type = ctypes.POINTER(ctypes.c_uint8 * (240 * 320 * 3))
    image_ptr = ctypes.cast(image, ptr_type)
    
    # Now convert to NumPy array and reshape
    np_img = np.frombuffer(image_ptr.contents, dtype=np.uint8)
    np_img = np_img.reshape((240, 320, 3))
    
    # Crop and resize
    cropped = np_img[120:240, :, :]  # Bottom half
    resized = cv2.resize(cropped, (200, 66))
    return resized

# ================== Manual Control Loop ====================
def Manual_Control():
    global speed, steering_angle, Record_Button, Image_Count
    global Recording_Data, New_Image_Save_Path

    img = CAMGet()
    LCDImageStart(0, 0, 320, 240)
    LCDImage(img)

    processed_img = Image_Processing(img)

    keys = pygame.key.get_pressed()

    # Toggle recording
    if keys[K_r] and (pygame.time.get_ticks() - Record_Button) > 200:
        Record_Button = pygame.time.get_ticks()
        Recording_Data = not Recording_Data
        if Recording_Data:
            now = datetime.datetime.now().strftime("%Y-%m-%d-%H.%M.%S")
            New_Image_Save_Path = os.path.join(Image_Path, f"Manual_Image_{now}")
            os.makedirs(New_Image_Save_Path, exist_ok=True)
            print(f"Recording Started: %s", New_Image_Save_Path)
        else:
            print(f"Recording Stopped")

    # Keyboard control
    if keys[K_w] or keys[K_UP]:
        speed = min(speed + speed_increment, max_speed)
    elif keys[K_s] or keys[K_DOWN]:
        speed = max(speed - speed_increment, min_speed)
    else:
        speed *= 0.9  # gradually slow down

    if keys[K_d] or keys[K_RIGHT]:
        steering_angle = max(steering_angle - steer_increment, min_steer)
    elif keys[K_a] or keys[K_LEFT]:
        steering_angle = min(steering_angle + steer_increment, max_steer)
    else:
        steering_angle *= 0.8  # center steering slowly

    VWSetSpeed(int(speed), int(steering_angle))

    if Recording_Data:
        filename = f"Manual_Image_{Image_Count}_{int(speed)}_{int(steering_angle)}.png"
        print("filename: ", filename)
        path = os.path.join(New_Image_Save_Path, filename)
        cv2.imwrite(path, cv2.cvtColor(processed_img, cv2.COLOR_RGB2BGR))
        Image_Count += 1

# ================== Main Loop ====================
def main():
    CAMInit(QVGA)
    done = 0

    while True:
        LCDMenu("Manual Drive", " ", " ", "END")
        k = KEYGet()
        if k == KEY1:
            LCDMenu("Manual Control", " ", " ", "Exit")
            print("Manual Drive")
            pygame.init()
            pygame.font.init()
            pygame.display.set_mode((400, 100))
            pygame.display.set_caption("Manual Control Window")

            Iterations = 0
            Start_FPS = time.time()

            while True:
                pygame.time.Clock().tick(FRAME_RATE)
                pygame.event.pump()
                Manual_Control()
                Iterations += 1
                k = KEYRead()
                if k == KEY4:
                    End_FPS = time.time()
                    FPS = float(Iterations / (End_FPS - Start_FPS))
                    print(f'Frames/Second : %.2f' % (FPS))
                    break
        elif k == KEY4:
            done = 1
        VWSetSpeed(0, 0)
        if done: break

main()
