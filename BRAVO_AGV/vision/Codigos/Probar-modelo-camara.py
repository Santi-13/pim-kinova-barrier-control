import cv2
from ultralytics import YOLO
import torch
import os

# Configurar para evitar errores de memoria
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
torch.cuda.empty_cache()

# Ruta de tu modelo entrenado
model_path = "C:/BRAVO/flask name.v2i.yolov12/runs/train/EVO7-0.6824/weights/best.pt"

# Cargar el modelo
model = YOLO(model_path)

# Iniciar cámara
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("No se pudo acceder a la cámara.")
    exit()

print("🎥 Cámara activada. Presiona 'q' para salir.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error al leer el frame.")
        break

    # Hacer predicción en el frame
    results = model.predict(source=frame, device=0, conf=0.4, stream=False)

    # Dibujar resultados
    for r in results:
        annotated_frame = r.plot()
        cv2.imshow("Detección en tiempo real - YOLOv12", annotated_frame)

    # Salir con 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Liberar recursos
cap.release()
cv2.destroyAllWindows()
print("✅ Cámara cerrada.")
