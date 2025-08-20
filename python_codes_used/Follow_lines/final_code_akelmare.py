import GUI
import HAL
import cv2.cv2 as cv2
import numpy as np
import time

# CONTROLADORES 
def pdi_angular(err, err_prev, err_acc):
    """P-D-I angular con derivativo filtrado (EMA)."""
    global d_w_filt
    d_raw = err - err_prev
    d_w_filt = 0.7 * d_w_filt + 0.3 * d_raw     # filtro paso-bajos

    err_acc = np.clip(err_acc + err, -I_MAX, I_MAX) 
    w = -(Kp_w * err + Ki_w * err_acc + Kd_w * d_w_filt)
    return w, err_acc


def pd_velocidad(err, err_prev, Kp, Kd, v_min, v_max):
    """P-D de la velocidad lineal."""
    d = err - err_prev
    v = v_max - (Kp * abs(err) + Kd * abs(d))
    return max(v_min, v)


# CONSTANTES Y PARÁMETROS 
MAX_W = 1.2                     # giro máximo 
MIN_W = -1.2

lower_red = np.array([0, 54, 115])
upper_red = np.array([3, 219, 255])

# P-D-I angular
Kp_w = 0.004
Kd_w = 0.040
Ki_w = 0.000001
I_MAX = 3000

# Velocidad en curvas
Kp_v_c = 0.030
Kd_v_c = 0.004
v_min_c = 0
v_max_c = 15

# Velocidad en rectas
Kp_v_r = 0.060
Kd_v_r = 0.006
v_min_r = 18
v_max_r = 38         

# Umbrales
threshold_c = 15    # error_w para modo curva
threshold_r = 55    # suma para modo recta
dv_max = 2          

# Variables 
centroides = []      # 4 últimos centroides
error_list = []      # 3 últimos errores angulares
error_pre_w = 0
d_w_filt = 0
error_acc_w = 0
error_pre_v = 0
w_prev = 0
v_prev = v_min_r
prev_time = time.time()

while True:
    # FPS
    now = time.time()
    fps = 1 / (now - prev_time) if now > prev_time else 0
    prev_time = now

    # Captura + ROI
    image = HAL.getImage()
    h, wi, _ = image.shape
    y0, band = int(h * 0.35), int(h * 0.20)
    roi = np.zeros_like(image)
    roi[y0:y0 + band] = image[y0:y0 + band]

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    # no encuentra la línea
    if not contours:
        HAL.setV(2)
        HAL.setW(1.0 if error_pre_w < 0 else -1.0)
        GUI.showImage(image)
        continue

    # Centroide
    max_c = max(contours, key=cv2.contourArea)
    M = cv2.moments(max_c)
    cx = int(M["m10"] / M["m00"]) if M["m00"] else wi // 2
    cy = int(M["m01"] / M["m00"]) if M["m00"] else h // 2

    centroides.append(cx)
    if len(centroides) > 4:
        centroides.pop(0)
    cx_filt = sum(centroides) / len(centroides)

    # Error angular (desfase +2 px para compensar cámara)
    error_w = cx_filt - wi / 2 + 2

    # ZONA MUERTA
    if abs(error_w) < 3:
        error_w = 0

    # Gestión de lista de errores
    error_list.append(abs(error_w))
    if len(error_list) > 3:
        error_list.pop(0)
    error_sum = sum(error_list)

    # Control angular
    w_cmd, error_acc_w = pdi_angular(error_w, error_pre_w, error_acc_w)
    error_pre_w = error_w

    # Umbral mínimo para w
    if abs(w_cmd) < 0.04:
        w_cmd = 0

    w_cmd = np.clip(w_cmd, MIN_W, MAX_W)
    w = 0.7 * w_prev + 0.3 * w_cmd
    w_prev = w

    # Control de velocidad
    if abs(error_w) > threshold_c:           # Curva
        v_cmd = pd_velocidad(error_w, error_pre_v,
                             Kp_v_c, Kd_v_c, v_min_c, v_max_c)
        error_pre_v = error_w
    elif error_sum <= threshold_r:           # Recta
        v_cmd = pd_velocidad(error_sum, error_pre_v,
                             Kp_v_r, Kd_v_r, v_min_r, v_max_r)
        error_pre_v = error_sum
    else:                                    # Zona intermedia
        v_cmd = v_prev

    v = v_prev + np.clip(v_cmd - v_prev, -dv_max, dv_max)
    v_prev = v

    # Actuadores
    HAL.setV(v)
    HAL.setW(w)

    # Show
    cv2.circle(image, (int(cx_filt), int(cy)), 5, (0, 255, 255), -1)
    cv2.putText(image, f"Er: {error_w:.1f}", (10, 310),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(image, f"W: {w:.2f}",   (10, 340),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    cv2.putText(image, f"V: {v:.1f}",   (10, 370),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    #cv2.putText(image, f"FPS:{fps:.1f}", (10, 400),
    #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    GUI.showImage(image)
