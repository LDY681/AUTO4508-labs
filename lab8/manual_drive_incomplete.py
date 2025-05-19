#!/usr/bin/env python
from eye import *
import os
#pip3 install opencv-python
import cv2
import time
import numpy as np
import ctypes
import datetime
#pip3 install pygame
import pygame
from pygame.locals import *

#frame rate
FRAME_RATE=20 

#manual control
#TO DEFINE
max_speed=
min_speed=
speed=
speed_increment = 
max_steer=
min_steer=
steering_angle=
steer_increment = 
#image recording
Image_Count = 0
Record_Button = 0
Recording_Data = False
# Default image format: RGB 
#################################################################
Script_Path = os.path.dirname(os.path.abspath(__file__))
Image_Path = os.path.join(Script_Path, 'ImageDatasets')

def Image_Processing(image):
    #TO DO
    #Output image shape: (200,66,3)
    #TO DO
    return image

def Manual_Control():
    global speed
    global steering_angle
    global Record_Button
    global Image_Count
    global Recording_Data
    global New_Image_Save_Path
    img=CAMGet()
    LCDImageStart(0, 0, 320, 240)
    LCDImage(img)
    img=Image_Processing(img)
    #display_img=img.ctypes.data_as(ctypes.POINTER(ctypes.c_byte))
    #LCDImageStart(0, 0, 200, 66)
    #LCDImage(display_img)
    keys = pygame.key.get_pressed()
    if keys[K_r] and (pygame.time.get_ticks() - Record_Button) > 200:
        Record_Button=pygame.time.get_ticks()
        Recording_Data = not Recording_Data
        if Recording_Data:
            #TO DO
            #Make recording folder
            #example folder name: Manual_Image_2022-05-02-16.51.42
            #TO DO
            os.makedirs(New_Image_Save_Path)

    #TO DO
    #Manual control by keyboard or joystick
    #TO DO

    steering_angle = #TO DO
    speed = #TO DO

    VWSetSpeed(speed,steering_angle)
    if Recording_Data:
        #TO DO
        #save images in the folder "New_Image_Save_Path" with current speed and steering angle in the name
        #example: Manual_Image_35_300_-9_.png
        #TO DO

def main():
    CAMInit(QVGA)
    done=0

    while True:
        LCDMenu("Manual Drive", " ", " ", "END")
        k=KEYGet()
        if k == KEY1:
            LCDMenu("Manual Control", " ", " ", "Exit")
            LCDSetPrintf(18, 0, "Manual Drive")
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
                k=KEYRead()
                if k == KEY4:
                    End_FPS = time.time()
                    FPS = float(Iterations/(End_FPS - Start_FPS))
                    LCDSetPrintf(18, 30, 'Frames/Second : %.2f' %(FPS))
                    break
        elif k == KEY4:
            done = 1
        VWSetSpeed(0,0) # stop
        if done: break

main()
