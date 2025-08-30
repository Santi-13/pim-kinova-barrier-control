# import cv2
# import numpy as np
# from ultralytics import YOLO
# import os
# import torch
# from openni import openni2
# import time

# # === Inicializar OpenNI2 ===
# openni2_path = "/home/evo/Documents/orbbec/Orbbec descargas/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release/" \
#                "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux_x64/" \
#                "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/sdk/libs/"
# openni2.initialize(openni2_path)

# dev = openni2.Device.open_any()
# depth_stream = dev.create_depth_stream()
# depth_stream.start()
# print("Cámara de profundidad inicializada.")

# cam_index = 0
# tries = 30
# while tries > 0:
#     # cap = cv2.VideoCapture(cam_index)
#     cap = cv2.VideoCapture(4)
#     time.sleep(0.2)
#     if cap.isOpened():
#         print("Cámara RGB inicializada.")
#         break
#     else:
#         print(f"Reintentando acceso a cámara RGB (intento {30 - tries})...")
#         cap.release()
#         tries -= 1

# if not cap.isOpened():
#     print("No se pudo acceder a la cámara RGB después de varios intentos.")
#     depth_stream.stop()
#     openni2.unload()
#     exit()



# with np.load("/home/evo/Documents/orbbec/Orbbec descargas/VISION-BRAVO-main/Codigos/camera_calibration2.npz") as X:
#     mtx, dist = X['mtx'], X['dist']
#     fx, fy = mtx[0, 0], mtx[1, 1]
#     cx, cy = mtx[0, 2], mtx[1, 2]



# print("Torch CUDA disponible:", torch.cuda.is_available())
# device = 0 if torch.cuda.is_available() else 'cpu'



# model = YOLO("/home/evo/Documents/orbbec/Orbbec descargas/VISION-BRAVO-main/Codigos/runs/train/EVO7-0.6824/weights/best.pt")
# # === Loop principal ===
# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("Frame no capturado.")
#         continue

#     annotated = frame.copy()
#     results = model.predict(source=frame, device=0, conf=0.75, stream=False, verbose=False)

#     depth_frame = depth_stream.read_frame()
#     depth_data = np.ndarray((depth_frame.height, depth_frame.width), dtype=np.uint16,
#                             buffer=depth_frame.get_buffer_as_uint16())


#     depth_viz = cv2.convertScaleAbs(depth_data, alpha=0.03)
#     depth_viz = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)
#     depth_viz = cv2.flip(depth_viz, 1) 

#     for r in results:
#         boxes = r.boxes
#         if len(boxes) > 0:
#             annotated = r.plot()

#         for box in boxes:
#             x1, y1, x2, y2 = map(int, box.xyxy[0])
#             cx2d = int((x1 + x2) / 2)
#             cy2d = int((y1 + y2) / 2)

#             if 0 <= cy2d < depth_data.shape[0] and 0 <= cx2d < depth_data.shape[1]:
#                 box_size = 3
#                 x_start = max(0, cx2d - box_size // 2)
#                 y_start = max(0, cy2d - box_size // 2)
#                 x_end = min(depth_data.shape[1], cx2d + box_size // 2 + 1)
#                 y_end = min(depth_data.shape[0], cy2d + box_size // 2 + 1)

#                 center_patch = depth_data[y_start:y_end, x_start:x_end]
#                 valid_depths = center_patch[center_patch > 0]

#                 if len(valid_depths) == 0:
#                     continue

#                 depth_mm = np.median(valid_depths)

#                 Z = depth_mm / 10.0
#                 X_coord = (cx2d - cx) * Z / fx
#                 Y_coord = -(cy2d - cy) * Z / fy

#                 print(f"{X_coord:.2f}, {Y_coord:.2f}, {Z:.2f}")


#                 cv2.circle(annotated, (cx2d, cy2d), 5, (0, 255, 0), -1)
#                 cv2.putText(annotated, f"({X_coord:.1f},{Y_coord:.1f},{Z:.1f})cm", (cx2d + 5, cy2d - 5),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


#                 width_px = x2 - x1
#                 height_px = y2 - y1
#                 width_cm = (width_px * Z / fx) - 6
#                 height_cm = (height_px * Z / fy) - 6
#                 print(f"{width_cm:.2f}")
                
                
#                 line_y = int((y1 + y2) / 2)
#                 label_x = int((x1 + x2) / 3) - 40
#                 cv2.line(annotated, (x1, line_y), (x2, line_y), (255, 0, 255), 2)
#                 cv2.putText(annotated, f"{width_cm:.1f} cm", (label_x, line_y - 5),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

#                 # Visualización en la imagen de profundidad
#                 cv2.rectangle(depth_viz, (x_start, y_start), (x_end, y_end), (255, 255, 0), 2)
#                 cv2.putText(depth_viz, "Z centro", (x_start + 2, y_start - 4),
#                             cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
#                 cv2.circle(depth_viz, (cx2d, cy2d), 4, (255, 255, 255), -1)


#     cv2.imshow("Detección 3D", annotated)
#     cv2.imshow("Z visual", depth_viz)

#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         print("RGB:", frame.shape[:2], "DEPTH:", depth_data.shape)
#         break

# cap.release()
# depth_stream.stop()
# openni2.unload()
# cv2.destroyAllWindows()



import cv2
import numpy as np
from ultralytics import YOLO
import torch
from openni import openni2
import time

# === Inicializar OpenNI2 ===
openni2_path = "/home/evo/Documents/orbbec/Orbbec descargas/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release/" \
               "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux_x64/" \
               "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/sdk/libs/"
openni2.initialize(openni2_path)

dev = openni2.Device.open_any()
depth_stream = dev.create_depth_stream()
depth_stream.start()
print("📡 Cámara de profundidad inicializada.")

# === Inicializar cámara RGB ===
cap = cv2.VideoCapture(4)
if not cap.isOpened():
    print("No se pudo abrir la cámara RGB.")
    depth_stream.stop()
    openni2.unload()
    exit()
print("📷 Cámara RGB inicializada.")

# === Calibración ===
with np.load("/home/evo/Documents/orbbec/Orbbec descargas/VISION-BRAVO-main/Codigos/camera_calibration2.npz") as X:
    mtx, dist = X['mtx'], X['dist']
    fx, fy = mtx[0, 0], mtx[1, 1]
    cx, cy = mtx[0, 2], mtx[1, 2]

# === Modelo YOLO ===
device = 0 if torch.cuda.is_available() else 'cpu'
print("Torch  disponible:", torch.cuda.is_available())
model = YOLO("/home/evo/Documents/orbbec/Orbbec descargas/VISION-BRAVO-main/Codigos/runs/train/EVO7-0.6824/weights/best.pt")

# === Crear ventana solo una vez ===
cv2.namedWindow("Detección 3D", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Detección 3D", 960, 720)
cv2.namedWindow("Z visual", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Z visual", 640, 480)

# === Loop principal ===
while True:
    ret, frame = cap.read()
    if not ret:
        print("Frame no capturado.")
        continue

    annotated = frame.copy()
    results = model.predict(source=frame, device=device, conf=0.75, stream=False, verbose=False)

    depth_frame = depth_stream.read_frame()
    depth_data = np.ndarray((depth_frame.height, depth_frame.width), dtype=np.uint16,
                            buffer=depth_frame.get_buffer_as_uint16())
    depth_viz = cv2.convertScaleAbs(depth_data, alpha=0.03)
    depth_viz = cv2.applyColorMap(depth_viz, cv2.COLORMAP_JET)
    depth_viz = cv2.flip(depth_viz, 1)

    for r in results:
        for box in r.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cx2d = int((x1 + x2) / 2)
            cy2d = int((y1 + y2) / 2)

            if 0 <= cy2d < depth_data.shape[0] and 0 <= cx2d < depth_data.shape[1]:
                box_size = 3
                x_start = max(0, cx2d - box_size // 2)
                y_start = max(0, cy2d - box_size // 2)
                x_end = min(depth_data.shape[1], cx2d + box_size // 2 + 1)
                y_end = min(depth_data.shape[0], cy2d + box_size // 2 + 1)

                center_patch = depth_data[y_start:y_end, x_start:x_end]
                valid_depths = center_patch[(center_patch > 0) & (center_patch < 10000)]

                if len(valid_depths) == 0:
                    continue

                depth_mm = np.median(valid_depths)
                Z = depth_mm / 10.0
                X_coord = (cx2d - cx) * Z / fx
                Y_coord = -(cy2d - cy) * Z / fy

                print(f"{X_coord:.1f}, {Y_coord:.1f}, {Z:.1f} ")

                width_px = x2 - x1
                width_cm = (width_px * Z / fx) - 6
                print(width_cm)
                # === Dibujar en imagen RGB
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(annotated, f"({X_coord:.1f},{Y_coord:.1f},{Z:.1f})cm", (x1, y1 - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(annotated, f"{width_cm:.1f} cm", (x1, y2 + 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

                # === Dibujar en profundidad
                cv2.rectangle(depth_viz, (x_start, y_start), (x_end, y_end), (255, 255, 0), 2)
                cv2.putText(depth_viz, "Z centro", (x_start + 2, y_start - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)

    cv2.imshow("Detección 3D", annotated)
    cv2.imshow("Z visual", depth_viz)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        print("✅ Finalizando...")
        break

# === Limpiar ===
cap.release()
depth_stream.stop()
openni2.unload()
cv2.destroyAllWindows()

