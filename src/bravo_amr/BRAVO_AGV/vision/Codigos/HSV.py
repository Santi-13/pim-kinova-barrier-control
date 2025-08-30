import cv2
import numpy as np

# Iniciar cámara
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("No se pudo abrir la cámara.")
    exit()

def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        bgr_color = frame[y, x]
        hsv_color = cv2.cvtColor(np.uint8([[bgr_color]]), cv2.COLOR_BGR2HSV)[0][0]
        print(f"📍 Coordenada ({x}, {y})")
        print(f"   BGR: {bgr_color}")
        print(f"   HSV: {hsv_color}")

        # Mostrar en pantalla también
        label = f"H:{hsv_color[0]} S:{hsv_color[1]} V:{hsv_color[2]}"
        cv2.putText(frame, label, (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.circle(frame, (x, y), 4, (0, 255, 255), -1)

cv2.namedWindow("Click para ver HSV")
cv2.setMouseCallback("Click para ver HSV", click_event)

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    cv2.imshow("Click para ver HSV", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
