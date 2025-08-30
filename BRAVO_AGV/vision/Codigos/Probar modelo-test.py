from ultralytics import YOLO
import os
import torch

# Liberar memoria
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "expandable_segments:True"
torch.cuda.empty_cache()

if __name__ == '__main__':
    model = YOLO("C:/BRAVO/flask name.v2i.yolov12/runs/train/EVO3-0.6694/weights/best.pt")  # ajusta si tu carpeta es otra

    results = model.predict(
        source="C:/BRAVO/flask name.v2i.yolov12/test/images",  # carpeta de imágenes de entrenamiento
        save=True,            # guarda las imágenes con predicciones
        save_txt=False,       # si quieres también guardar los .txt con predicciones
        imgsz=512,            # asegúrate de que sea igual al usado en entrenamiento
        conf=0.25,            # puedes ajustar el umbral de confianza
        device=0,
        name="TEST-MODELO-EVO3-0.6694"# usa la GPU
    )

    print("✅ Predicciones terminadas. Las imágenes con resultados están en la carpeta 'runs/predict'.")
