import numpy as np

# Ruta del archivo
file_path = "camera_calibration.npz"

# Cargar archivo
calib = np.load(file_path)

# Mostrar los campos disponibles
print("🧾 Contenido del archivo:")
print(calib.files)

# Cargar variables
camera_matrix = calib['camera_matrix']
dist_coeffs = calib['dist_coeffs']
rvecs = calib['rvecs']
tvecs = calib['tvecs']

# Mostrar resumen
print("\n📌 Matriz de cámara (mtx):\n", camera_matrix)
print("\n📌 Coeficientes de distorsión (dist):\n", dist_coeffs)
print(f"\n🔁 {len(rvecs)} vectores de rotación (rvecs)")
print(f"➡️  {len(tvecs)} vectores de traslación (tvecs)")
