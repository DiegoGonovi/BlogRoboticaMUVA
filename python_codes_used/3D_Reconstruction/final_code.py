from HAL import HAL
from GUI import GUI
import cv2 as cv2
import numpy as np
import time

# Procesar las imágenes
imageLeft = HAL.getImage('left') #to get the left image
imageRight = HAL.getImage('right') #to get the right image

imageLeft_gray = cv2.cvtColor(imageLeft, cv2.COLOR_BGR2GRAY)
imageRight_gray = cv2.cvtColor(imageRight, cv2.COLOR_BGR2GRAY)

h_thr_l, _ = cv2.threshold(imageLeft_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
l_thr_l = int(0.5*h_thr_l)

h_thr_r, _ = cv2.threshold(imageRight_gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
l_thr_r = int(0.5*h_thr_r)

img_canny_l = cv2.Canny(imageLeft_gray, l_thr_l, h_thr_l)
img_canny_r = cv2.Canny(imageRight_gray, l_thr_r, h_thr_r)

keypoints_left = np.column_stack(np.where(img_canny_l > 0))
#print('Longitud Keypoints Left', len(keypoints_left))
ky_l_r = keypoints_left
#print('Longitud Keypoints Reducidos Left', len(ky_l_r))

cam_l = HAL.getCameraPosition('left')
cam_r = HAL.getCameraPosition('right')
#print('cam_l:',cam_l)
#print('cam_r:', cam_r)

rect_img = (0, 0, imageRight.shape[1], imageRight.shape[0])
block_size = 5
points = []

while True:

    for y_px, x_px in ky_l_r:
        imageLeft = HAL.getImage('left') #to get the left image
        imageRight = HAL.getImage('right') #to get the right image

        cv2.circle(imageLeft, (x_px, y_px), radius=15, color=(0, 0, 255), thickness=3)
        # Convertir coordenadas gráfico a óptico
        p_opt = HAL.graficToOptical('left', [x_px, y_px, 1])

        # Retroproyectar el punto 2D a 3D 
        p_3D = HAL.backproject('left', p_opt)

        # Recta epipolar (punto, pendiente)
        # Une punto cámara, punto característicos, será la pendiente
        
        m_epi = p_3D[:3] - cam_l
        # Normalizar pendiente
        m_epi /= np.linalg.norm(m_epi)

        # Crear dos puntos 3D a lo largo de la recta epipolar
        p1_3d = (cam_l + 10 * m_epi).tolist() + [1] 
        p2_3d = (cam_l - 10 * m_epi).tolist() + [1]

        # Proyectarlos en la imagen derecha
        p1_2d = HAL.project('right', p1_3d)
        p2_2d = HAL.project('right', p2_3d)

        p1_px_r = HAL.opticalToGrafic('right', p1_2d)[:2]
        p2_px_r = HAL.opticalToGrafic('right', p2_2d)[:2]

        # Cortar recta a solo imagen
        _, pt1, pt2 = cv2.clipLine(rect_img, (int(p1_px_r[0]), int(p1_px_r[1])), (int(p2_px_r[0]), int(p2_px_r[1])))

        mask_epi = np.zeros_like(img_canny_r)
        cv2.line(mask_epi, pt1, pt2, 255, 5) # Grosor 5 px
        
        y_key_r, x_key_r = np.where(mask_epi == 255)

        x_epi_min, x_epi_max = np.min(x_key_r), np.max(x_key_r)
        y_epi_min, y_epi_max = np.min(y_key_r), np.max(y_key_r)

        # ROI
        x_epi_min = max(x_epi_min - block_size//2, 0)
        y_epi_min = max(y_epi_min - block_size//2, 0)
        x_epi_max = min(x_epi_max + block_size//2, imageRight_gray.shape[1]-1)
        y_epi_max = min(y_epi_max + block_size//2, imageRight_gray.shape[0]-1)
        
        # Franja epi
        cv2.rectangle(imageRight, (x_epi_min, y_epi_min), (x_epi_max, y_epi_max), (0, 255, 0), 2)

        # Parche  5x5 en imagen epi derecha
        patch_r = imageRight[y_epi_min:y_epi_max + 1, x_epi_min:x_epi_max + 1]
        
        # Parche  5x5 en imagen izquierda
        patch_l = imageLeft[y_px - block_size//2:y_px + block_size//2 + 1, x_px - block_size//2:x_px + block_size//2 + 1]
        
        # Búsqueda de homólogo
        res = cv2.matchTemplate(patch_r, patch_l, cv2.TM_CCORR_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)

        if max_val > 0.93:
            x_match = x_epi_min + max_loc[0] + block_size//2
            y_match = y_epi_min + max_loc[1] + block_size//2
        
            cv2.circle(imageRight, (x_match, y_match), radius=15, color=(255, 0, 0), thickness=3)  

            p_opt_macth = HAL.graficToOptical('right', [x_match, y_match, 1])
            p_3D_match = HAL.backproject('right', p_opt_macth)
            
            m_match = p_3D_match[:3] - cam_r
            m_match /= np.linalg.norm(m_match)

            v1 = np.array(m_epi[:3])    # Vector dirección recta izquierda
            v2 = np.array(m_match[:3])  # Vector dirección recta derecha

            w0 = cam_l - cam_r

            # t *(v1 *v1) - s*(v1 * v2) = -v1 *w0
            # t *(v1 *v2) - s*(v2 * v2) = -v2 *w0
            a = np.dot(v1, v1)
            b = np.dot(v1, v2)
            c = np.dot(v1, w0)
            e = np.dot(v2, v2)
            f = np.dot(v2, w0)

            # t = (bf-ce)/(ae-bb), s = (af-bc)/(ae-bb)
            denom_ = a*e - b*b

            time.sleep(0.05)

            if abs(denom_) > 1e-5:  # Rectas no paralelas
                t = (b*f - c*e) / denom_
                s = (a*f - b*c) / denom_

                # Calcular puntos más cercanos en ambas rectas
                P_left = cam_l + t * v1
                P_right = cam_r + s * v2

                # El punto 3D estimado es el promedio de los dos puntos más cercanos
                x_match_3D = (P_left + P_right) / 2
                
                x_match_3D = x_match_3D / 100  # mm a dm
                # Distancia entre puntos 
                distance = np.linalg.norm(P_left - P_right)
                distance = distance / 100

                if distance < 44:
                    #print('Punto 3D estimado:', x_match_3D)
                    #print('Error:', distance)

                    avg_color = np.mean(patch_l, axis=(0, 1))

                    px_color_list = [float(c) for c in avg_color]
                    px_color_rgb = tuple(px_color_list[::-1])
                    X_list = [float(i) for i in x_match_3D]  

                    p_final_color = tuple(X_list) + tuple(px_color_rgb)

                    points.append(p_final_color)

                    GUI.showImages(imageLeft,imageRight,True)
                    GUI.ShowNewPoints([p_final_color])
    
    print('Tiempo de espera')
    time.sleep(10)

    

    
