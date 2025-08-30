# from openni import openni2

# openni2_path = "/home/evo/Documents/orbbec/Orbbec descargas/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release/" \
#                "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux_x64/" \
#                "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/sdk/libs/"# Debe contener OpenNI2/ y libs/
# openni2.initialize(openni2_path)

# try:
#     dev = openni2.Device.open_any()
#     print("✅ Dispositivo de profundidad abierto correctamente.")
# except Exception as e:
#     print("❌ Error al abrir dispositivo:", e)

# openni2.unload()

import cv2
import numpy as np
from openni import openni2
openni2_path = "/home/evo/Documents/orbbec/Orbbec descargas/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release/" \
               "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux_x64/" \
               "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/sdk/libs/"
openni2.initialize(openni2_path)

dev = openni2.Device.open_any()
depth_stream = dev.create_depth_stream()
depth_stream.start()
cap = cv2.VideoCapture(4)

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    depth_frame = depth_stream.read_frame()
    depth_data = np.ndarray((depth_frame.height, depth_frame.width), dtype=np.uint16,
                            buffer=depth_frame.get_buffer_as_uint16())
    depth_viz = cv2.convertScaleAbs(depth_data, alpha=0.03)
    cv2.imshow("RGB", frame)
    cv2.imshow("Depth", depth_viz)
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()
depth_stream.stop()
openni2.unload()
cv2.destroyAllWindows()

