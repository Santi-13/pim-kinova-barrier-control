import cv2
import numpy as np

from pyorbbecsdk import Config
from pyorbbecsdk import OBError
from pyorbbecsdk import OBSensorType, OBFormat
from pyorbbecsdk import Pipeline, FrameSet
from pyorbbecsdk import VideoStreamProfile
from utils import frame_to_bgr_image

ESC_KEY = 27


def main():
    config = Config()
    pipeline = Pipeline()
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        try:
            color_profile: VideoStreamProfile = profile_list.get_video_stream_profile(640, 0, OBFormat.RGB, 30)
        except OBError as e:
            print(e)
            color_profile = profile_list.get_default_video_stream_profile()
            print("color profile: ", color_profile)
        config.enable_stream(color_profile)
    except Exception as e:
        print(e)
        return
    pipeline.start(config)
    def nothing(x):
        pass
    cv2.namedWindow("Trackbars")

    cv2.createTrackbar("L-H","Trackbars",0,255,nothing)
    cv2.createTrackbar("L-S","Trackbars",0,255,nothing)
    cv2.createTrackbar("L-V","Trackbars",0,255,nothing)
    cv2.createTrackbar("U-H","Trackbars",0,255,nothing)
    cv2.createTrackbar("U-S","Trackbars",0,255,nothing)
    cv2.createTrackbar("U-V","Trackbars",0,255,nothing)

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
            cv2.imshow("Color Viewer", color_image)
            

            
            frame = color_image
                
            frame = cv2.resize(frame, (640,480))

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            l_h = cv2.getTrackbarPos("L-H","Trackbars")
            l_s = cv2.getTrackbarPos("L-S","Trackbars")
            l_v = cv2.getTrackbarPos("L-V","Trackbars")
            u_h = cv2.getTrackbarPos("U-H","Trackbars")
            u_s = cv2.getTrackbarPos("U-S","Trackbars")
            u_v = cv2.getTrackbarPos("U-V","Trackbars")

            lower = np.array([l_h,l_s,l_v])
            upper = np.array([u_h,u_s,u_v])
                
            mask = cv2.inRange(hsv, lower, upper)

            result = cv2.bitwise_and(frame, frame, mask = mask)
            cv2.imshow("Result", result)

            key = cv2.waitKey(1)
            if key == ord('q') or key == ESC_KEY:
                break
        except KeyboardInterrupt:
            break
    pipeline.stop()


if __name__ == "__main__":
    main()

