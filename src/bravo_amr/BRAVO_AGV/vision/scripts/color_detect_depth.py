#!/usr/bin/env python
""""
Code by: Ricardo Berumen
Color mask for test tube detection with depth camera
"""

import argparse
import sys

import cv2
import numpy as np

from pyorbbecsdk import *
from utils import frame_to_bgr_image

ESC_KEY = 27

def color_detect(img, img_d):
    #Definimos los parametros de la máscara roja
    redHigh=np.array([10,255,255],np.uint8)
    redLow=np.array([0,70,0],np.uint8)
    """ redHigh=np.array([180,255,250],np.uint8)
    redLow=np.array([80,170,65],np.uint8) """
    rmask=[redLow,redHigh]

    #Definimos los parametros de la máscara azul
    blueHigh=np.array([140,220,255],np.uint8)
    blueLow=np.array([100,150,0],np.uint8)
    bmask=[blueLow,blueHigh]

    mask = bmask

    #Creo el kernel para el traking
    kernel = np.ones((5,5),np.uint8)#matriz de 5 por 5 con vals de 8 bits
            
    ttube=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)#cambiamos el frame a una escala HSV

            #defino la máscara
    ttubemask=cv2.inRange(ttube,mask[0],mask[1])

            #limpiamos el ruido de la mascara
    ttubemaskCl=cv2.morphologyEx(ttubemask, cv2.MORPH_OPEN,kernel)

            #visualizamos la máscara
    ttubemaskVis = cv2.bitwise_and(img,img,mask=ttubemaskCl)
    ttubemaskVis_depth = cv2.bitwise_and(img_d,img_d,mask=ttubemaskCl)

            

            #cuadro de detección
    x,y,w,h= cv2.boundingRect(ttubemaskCl)# Obtenemos las coordenadas de los límites de la máscara
    cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),3)#creamos el rectangulo de deteción
    cv2.circle(img, ((x + w // 2), (y + h // 2)), 5, (255, 0, 0), -1)#creamos el circulo del centro del rectángulo
    ttube_center = [(x + w // 2), (y + h // 2)]


            #Visualizamos cada frame...
    cv2.imshow('Original',img)
    cv2.imshow('mask',ttubemask)
    cv2.imshow('maskVis',ttubemaskVis)
    return ttubemask, ttubemaskVis_depth, ttube_center


def main(argv):
    pipeline = Pipeline()
    device = pipeline.get_device()
    device_info = device.get_device_info()
    device_pid = device_info.get_pid()
    config = Config()
    parser = argparse.ArgumentParser()
    parser.add_argument("-m", "--mode",
                        help="align mode, HW=hardware mode,SW=software mode,NONE=disable align",
                        type=str, default='HW')
    parser.add_argument("-s", "--enable_sync", help="enable sync", type=bool, default=True)
    args = parser.parse_args()
    align_mode = args.mode
    enable_sync = args.enable_sync
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = profile_list.get_default_video_stream_profile()
        config.enable_stream(color_profile)
        profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        assert profile_list is not None
        depth_profile = profile_list.get_default_video_stream_profile()
        assert depth_profile is not None
        print("color profile : {}x{}@{}_{}".format(color_profile.get_width(),
                                                   color_profile.get_height(),
                                                   color_profile.get_fps(),
                                                   color_profile.get_format()))
        print("depth profile : {}x{}@{}_{}".format(depth_profile.get_width(),
                                                   depth_profile.get_height(),
                                                   depth_profile.get_fps(),
                                                   depth_profile.get_format()))
        config.enable_stream(depth_profile)
    except Exception as e:
        print(e)
        return
    if align_mode == 'HW':
        if device_pid == 0x066B:
            # Femto Mega does not support hardware D2C, and it is changed to software D2C
            config.set_align_mode(OBAlignMode.SW_MODE)
        else:
            config.set_align_mode(OBAlignMode.HW_MODE)
    elif align_mode == 'SW':
        config.set_align_mode(OBAlignMode.SW_MODE)
    else:
        config.set_align_mode(OBAlignMode.DISABLE)
    if enable_sync:
        try:
            pipeline.enable_frame_sync()
        except Exception as e:
            print(e)
    try:
        pipeline.start(config)
    except Exception as e:
        print(e)
        return
    while True:
        try:
            frames: FrameSet = pipeline.wait_for_frames(100)
            if frames is None:
                continue
            color_frame = frames.get_color_frame()
            if color_frame is None:
                continue
            # covert to RGB format
            color_image = frame_to_bgr_image(color_frame)
            if color_image is None:
                print("failed to convert frame to image")
                continue
            depth_frame = frames.get_depth_frame()
            if depth_frame is None:
                continue

            width = depth_frame.get_width()
            height = depth_frame.get_height()
            scale = depth_frame.get_depth_scale()

            depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
            depth_data = depth_data.reshape((height, width))
            depth_data = depth_data.astype(np.float32) * scale
            depth_image = cv2.normalize(depth_data, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
            depth_image_2 = depth_image.copy()
            ttube_center = [0, 0]
            ttube_bit_mask, ttube_bit_mask_d, ttube_center = color_detect(color_image, depth_image_2)
            # overlay color image on depth image
            depth_image = cv2.addWeighted(color_image, 0.5, depth_image, 0.5, 0)
            cv2.imshow("SyncAlignViewer ", depth_image_2)
            ttube_center_f = np.flip(ttube_center)
            print(ttube_center_f)
            if ttube_center != [0.0, 0.0]:
                depth_center = depth_data[ttube_center_f[0], ttube_center_f[1]]
            #print(depth_data.shape)


            key = cv2.waitKey(1)
            if key == ord('q') or key == ESC_KEY:
                break
        except KeyboardInterrupt:
            break
    pipeline.stop()


if __name__ == "__main__":
    print("Please NOTE: This example is NOT supported by the Gemini 330 series.")
    print("If you want to see the example on Gemini 330 series, please refer to align_filter_viewer.py")
    main(sys.argv[1:])
