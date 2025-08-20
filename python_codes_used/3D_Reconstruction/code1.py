from HAL import HAL
from GUI import GUI
import cv2 as cv2
import numpy as np
import time

#GUI.ClearAllPoints()

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
print('Longitud Keypoints Reducidos Left', len(ky_l_r))

cam_l = HAL.getCameraPosition('left')
cam_r = HAL.getCameraPosition('right')

#print('Posiciones cámara:', cam_l, cam_r)

block_size = 3
points = []

#print('Dimensiones:', imageLeft.shape[1], imageLeft.shape[0])
rect_img = (0, 0, imageRight.shape[1], imageRight.shape[0])

while True: 
    GUI.showImages(imageLeft,imageRight,True)

    print('entra al for')
    for y_px, x_px in ky_l_r:

        cv2.circle(imageLeft, (x_px, y_px), radius=15, color=(0, 0, 255), thickness=3)

        # Convertir coordenadas gráfico a óptico
        #print('x_px, y_px:', x_px, y_px)
        # HAL usa un origen en la esquina inferior izquierda
        p_opt = HAL.graficToOptical('left', [x_px, y_px, 1])

        # Retroproyectar el punto 2D a 3D 
        p_3D = HAL.backproject('left', p_opt)

        # Recta epipolar (punto, pendiente)
        # Une punto cámara, punto característicos, será la pendiente
        # Pertenece a punto cámara
        m_epi = p_3D[:3] - cam_l

        # Normalizar pendiente
        m_epi /= np.linalg.norm(m_epi)

        # Punto pendiente
        #l_epi = [cam_l, m_epi]

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

        mask_epi = np.zeros_like(imageRight_gray)
        cv2.line(mask_epi, pt1, pt2, 255, 5) # Grosor 5 px
        
        y_key_r, x_key_r = np.where(mask_epi == 255)

        x_epi_min, x_epi_max = np.min(x_key_r), np.max(x_key_r)
        y_epi_min, y_epi_max = np.min(y_key_r), np.max(y_key_r)

        # ROI
        x_epi_min = max(x_epi_min - block_size//2, 0)
        y_epi_min = max(y_epi_min - block_size//2, 0)
        x_epi_max = min(x_epi_max + block_size//2, imageRight_gray.shape[1]-1)
        y_epi_max = min(y_epi_max + block_size//2, imageRight_gray.shape[0]-1)

        # Parche 5x5 
        patch_r = imageRight_gray[y_epi_min:y_epi_max + 1, x_epi_min:x_epi_max + 1]
        
        cv2.rectangle(imageRight, (x_epi_min, y_epi_min), (x_epi_max, y_epi_max), (0, 255, 0), 2)

        patch_l = imageLeft_gray[y_px - block_size//2:y_px + block_size//2 + 1, x_px - block_size//2:x_px + block_size//2 + 1]

        res = cv2.matchTemplate(patch_r, patch_l, cv2.TM_CCOEFF_NORMED)
        _, max_val, _, max_loc = cv2.minMaxLoc(res)

        if max_val > 0.8: 

            x_match = x_epi_min + max_loc[0] + block_size//2
            y_match = y_epi_min + max_loc[1] + block_size//2

            cv2.circle(imageRight, (x_match, y_match), radius=15, color=(0, 0, 255), thickness=3)
        
            p_opt_macth = HAL.graficToOptical('right', [x_match, y_match, 1])
            p_3D_match = HAL.backproject('right', p_opt_macth)

            m_match = p_3D_match[:3] - cam_r

            # Normalizar pendiente
            m_match /= np.linalg.norm(m_match)

            # Formulación del sistema de ecuaciones

            # Para cámara izquierda
            # A1 matriz de proyección aprox
            A1 = np.eye(3) - np.outer(m_epi, m_epi)
            b1 = A1.dot(cam_l)

            # Para cámara derecha 
            # A2 matriz de proyección aprox
            A2 = np.eye(3) - np.outer(m_match, m_match)
            b2 = A2.dot(cam_r)

            # Apilar las ecuaciones
            # A x = b
            A = np.vstack((A1, A2))
            b = np.hstack((b1, b2))

            # Resolver usando np.linalg.lstsq
            X, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

            if residuals < 3: 
                px_color = imageLeft[y_px, x_px]


                X_list = [float(i) for i in X]  # Floats
                px_color_list = [int(clr) for clr in px_color]  # Int
                p_final_color = tuple(X_list) + tuple(px_color_list)
                points.append(p_final_color)
        


            #time.sleep(4)
    

    #points = [(4, 4, 4, 255, 0, 0)]    
    print('Salió del for')
    GUI.ShowNewPoints(points)
    time.sleep(4)    
    #GUI.ShowNewPoints(points)



