import GUI
import HAL
import cv2.cv2 as cv2
import numpy as np
import time


def pdi_angular(error_w, error_pre_w, error_acc_w, Kp_w, Ki_w, Kd_w):
    d_w = error_w - error_pre_w
    # Término integral: acumula el error
    error_acc_w += error_w
    w = -Kp_w * error_w - Ki_w * error_acc_w - Kd_w * d_w
    return w, error_w, error_acc_w


def pd_velocidad_rectas(error_sum, error_pre_v, Kp_v, Kd_v, min_v, max_v): 
    # Se utiliza la suma de los últimos errores en lugar del error actual
    d_v = error_sum - error_pre_v
    v = max_v - (Kp_v * abs(error_sum) + Kd_v * abs(d_v))
    return max(min_v, v), error_sum   


def pd_velocidad_curvas(error_v, error_pre_v, Kp_v, Kd_v, min_v, max_v): 
    d_v = error_v - error_pre_v
    v = max_v - (Kp_v * abs(error_v) + Kd_v * abs(d_v))
    return max(min_v, v), error_v   


# Variables globales para el control
error_pre_w = 0
error_acc_w = 0
error_pre_v = 0
error_w = 0
error_list = []  # Lista para almacenar los 4 últimos errores

Kp_w = 0.0083
Kd_w = 0.01691 
Ki_w = 0.000003    # Ganancia integral  

# -- Ganancias PD para velocidad en curvas --
Kp_v_c = 0.0408
Kd_v_c = 0.003  

Kp_v_r = 0.08  
Kd_v_r = 0.005  

lower_red = np.array([0, 54, 115])
upper_red = np.array([3, 219, 255])

v_min_r = 17
v_max_r = 28

v_min_c = 4
v_max_c = 17

w = 0
v = 0
threshold_r = 60  # Umbral para la suma de los 6 últimos errores
threshold_c = 12

MAX_W = 1.7   # límite superior de giro (ajústalo a tu gusto)
MIN_W = -1.7  # límite inferior

# Inicializamos el tiempo previo
prev_time = time.time()

while True:
    # Calculamos los fps
    current_time = time.time()
    delta = current_time - prev_time
    fps = 1 / delta if delta > 0 else 0
    prev_time = current_time

    image = HAL.getImage()
    h, wi, c = image.shape
    # Crear una imagen negra del mismo tamaño para la región de interés (ROI)
    roi = np.zeros((h, wi, c), dtype=np.uint8)
    
    # Calcular la franja del horizonte
    band_ = int(h * 0.14)
    hrzn_y_s = int(h * 0.39)  # Comienza en el 10% de la altura
    hrzn_y_e = hrzn_y_s + band_

    # Copiar solo la franja del horizonte en la imagen negra
    roi[hrzn_y_s:hrzn_y_e, :] = image[hrzn_y_s:hrzn_y_e, :]
    
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_red, upper_red)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        if error_w < 0:
            HAL.setV(2)
            HAL.setW(1.5)

        else:
            HAL.setV(2)
            HAL.setW(-1.5)

    else:
        max_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(max_contour)

        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

        else:
            cx, cy = wi // 2, h // 2  # Valores por defecto

        
        # Diferencia entre el centroide y el centro de la imagen

        error_w = cx - (wi // 2) + 2  # Corrección en píxeles
        
        # Actualizar la lista de errores (almacena el valor absoluto)
        error_list.append(abs(error_w))

        if len(error_list) > 3:
            error_list.pop(0)

        error_sum = sum(error_list)
        
        # Control angular
        w, error_pre_w, error_acc_w = pdi_angular(error_w, error_pre_w, error_acc_w, Kp_w, Ki_w, Kd_w)
        
        # Selección de controlador de velocidad:
        # Si la suma de 3 últimos errores menor que umbral PD rectas
        if abs(error_w) > threshold_c:
             v, error_pre_v = pd_velocidad_curvas(error_w, error_pre_v, Kp_v_c, Kd_v_c, v_min_c, v_max_c)
            
        elif error_sum <= threshold_r:
            v, error_pre_v = pd_velocidad_rectas(error_sum, error_pre_v, Kp_v_r, Kd_v_r, v_min_r, v_max_r)
           
        w = max(min(w, MAX_W), MIN_W)

        HAL.setV(v)
        HAL.setW(w)
    
        # Visualización sobre la imagen

        cv2.circle(image, (cx, cy), 5, (0, 255, 255), -1)

    cv2.putText(image, "Er: " + str(error_w), (10, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    cv2.putText(image, "w: " + str(w), (10, 340), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    cv2.putText(image, "v: " + str(v), (10, 370), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

    #cv2.putText(image, "Fps: " + str(round(fps,2)), (540, 310), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
    
    #cv2.line(image, (wi // 2, 0), (wi // 2, h), (255, 0, 0), 2)
    
    GUI.showImage(image)
