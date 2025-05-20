#!/usr/bin/env python
from eye import *
#pip3 install tensorflow==2.6.2 
from tensorflow.keras.models import load_model
import os
#pip3 install opencv-python
import cv2
import time
import numpy as np
import ctypes

#Change model name here
Model_Name="PilotNet_2025-05-20-11.22.09.h5"

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

def PilotNet_Drive(model):
    img=CAMGet()
    LCDImageStart(0, 0, 320, 240)
    LCDImage(img)
    img=Image_Processing(img)
    #display_img=img.ctypes.data_as(ctypes.POINTER(ctypes.c_byte))
    #LCDImageStart(0, 0, 200, 66)
    #LCDImage(display_img)
    model_input = np.expand_dims(img, axis=0)
    speed=int(np.round(model.predict(model_input)[0]))
    steering_angle=int(np.round(model.predict(model_input)[1]))
    LCDSetPrintf(19, 30, "Steering angle:= %d    ", steering_angle)
    LCDSetPrintf(19, 60, "Speed:= %d    ", speed)
    VWSetSpeed(speed,steering_angle)

def main():
    CAMInit(QVGA)
    done=0

    while True:
        LCDMenu("PilotNet Drive", " ", "", "END")
        k=KEYGet()
        if k == KEY1:
            LCDMenu("Auto Control", " ", " ", "Exit")
            LCDSetPrintf(18, 0, "PilotNet Drive")
            PilotNet_model=load_model(Model_Name, compile=False)
            
            Iterations = 0
            Start_FPS = time.time()
            while True:
                PilotNet_Drive(PilotNet_model)
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
