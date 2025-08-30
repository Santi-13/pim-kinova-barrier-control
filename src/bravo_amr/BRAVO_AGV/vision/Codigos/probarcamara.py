import cv2

print("🎥 Probar cámara...")
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("❌ No se pudo abrir la cámara.")
else:
    ret, frame = cap.read()
    if ret:
        print("✅ Cámara capturó un frame.")
    else:
        print("⚠️ No se pudo leer un frame de la cámara.")
    cap.release()
