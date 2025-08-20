import GUI
import HAL
import pyapriltags
import cv2 as cv2
import numpy as np
import yaml
from pathlib import Path
import time
from math import atan2

# Enter sequential code!

def bb_april_tags(r, image):

    # extract the bounding box (x, y)-coordinates for the AprilTag
    # and convert each of the (x, y)-coordinate pairs to integers
    (ptA, ptB, ptC, ptD) = r.corners
    ptB = (int(ptB[0]), int(ptB[1]))
    ptC = (int(ptC[0]), int(ptC[1]))
    ptD = (int(ptD[0]), int(ptD[1]))
    ptA = (int(ptA[0]), int(ptA[1]))

    # draw the bounding box of the AprilTag detection
    cv2.line(image, ptA, ptB, (255, 0, 255), 2) # morado
    cv2.line(image, ptB, ptC, (0, 255, 0), 2) # verde
    cv2.line(image, ptC, ptD, (255, 255, 0), 2) # azul
    cv2.line(image, ptD, ptA, (0, 215, 255), 2) # amarillo

    # draw the center (x, y)-coordinates of the AprilTag
    (cX, cY) = (int(r.center[0]), int(r.center[1]))
    cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
    
    # draw the tag family on the image
    tagFamily = r.tag_family.decode("utf-8")
    cv2.putText(
        image,
        tagFamily,
        (ptA[0], ptA[1] - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (0, 255, 0),
        2,
    )

def RT_Cam_AprilTag(r, object_pts, matrix_camera, dist_coeffs, image):

    #ptA  Inf izq / ptB  Inf derecha
    #ptC  Superior derecha / ptD  Sup izquierda
    (ptA, ptB, ptC, ptD) = r.corners # Coordendas de esquina en la imagen

    image_pts = r.corners.astype(np.float32)  # corners detección

    # Calcular rotación (rvec) y traslación (tvec)
    _, rvec, tvec = cv2.solvePnP(object_pts, image_pts, matrix_camera, dist_coeffs)

    R, _ = cv2.Rodrigues(rvec)  # Convierte vector de rotación a matriz 3x3

    # Matriz de rotación R respecto cámara
    # rojo=X, verde=Y, azul=Z
    cv2.drawFrameAxes(image, matrix_camera, dist_coeffs, rvec, tvec, 0.3, 5)

    RT = np.eye(4, dtype=np.float32)
    RT[:3, :3] = R
    RT[:3, 3] = tvec.flatten()

    return RT

def project_points(image, object_pts, RT_tag_cam, matrix_camera, dist_coeffs):

    # Obtener rvec y tvec de RT_tag_cam
    rvec, _ = cv2.Rodrigues(RT_tag_cam[:3, :3])
    tvec = RT_tag_cam[:3, 3]

    # Proyectar puntos 3D a imagen
    projected_pts, _ = cv2.projectPoints(object_pts, rvec, tvec, matrix_camera, dist_coeffs)

    # Dibujar puntos proyectados (rojo)
    for pt in projected_pts:
        x, y = int(pt[0][0]), int(pt[0][1])
        cv2.circle(image, (x, y), 8, (0, 0, 255), -1)  # Rojo


def RT_World_Tag(pos_data):

    x, y, z, yaw = pos_data
    cy = np.cos(yaw)
    sy = np.sin(yaw)

    matrix_RT = np.array([
        [cy, -sy, 0, x],
        [sy, cy, 0, y],
        [0, 0, 1, z], 
        [0, 0, 0, 1]
    ], dtype=np.float32)

    return matrix_RT

def matriz_RT_Optical_world(tx=0.0, ty=0.0, tz=0.0):
    theta = -np.pi / 2     # -90° en radianes

    # Rotación 90° alrededor de z
    Rz = np.array([[np.cos(theta), -np.sin(theta), 0],
                   [np.sin(theta),  np.cos(theta), 0],
                   [0,              0,             1]])

    # Rotación -90° alrededor de x
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(theta), -np.sin(theta)],
                   [0, np.sin(theta),  np.cos(theta)]])

    # Rotación compuesta (primero Rz, luego Rx)
    R = Rz @ Rx

    # Matriz homogénea 4×4
    RT = np.eye(4)
    RT[:3, :3] = R
    RT[:3, 3] = [tx, ty, tz]

    return RT


detector = pyapriltags.Detector(searchpath=["apriltags"], families="tag36h11")

image = HAL.getImage()

size = image.shape
focal_length = 1696.8027877807617
center = (size[1] / 2, size[0] / 2)

matrix_camera = np.array([[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]], dtype="double",)

dist_coeffs = np.zeros((4,1))  # Asume sin distorsión

conf = yaml.safe_load(
    Path("/resources/exercises/marker_visual_loc/apriltags_poses.yaml").read_text()
)
tags = conf["tags"]

tag_size = 0.24
half = tag_size / 2

object_pts = np.array([
    [-half,  half, 0],  # ptA: inf-izq
    [ half,  half, 0],  # ptB: inf-der
    [ half, -half, 0],  # ptC: sup-der
    [-half, -half, 0],  # ptD: sup-izq
], dtype=np.float32)

RT_robot_cam = np.array([
    [1, 0, 0, 0.069],
    [0, 1, 0, -0.047],
    [0, 0, 1, 0.107],
    [0, 0, 0, 1]
], dtype=np.float32)

pose_est = np.array([0.0, 0.0, 0.0], dtype=np.float32)

fase = 1
dur_fase = {1: 12.0, 2: 23.0, 3: 20.0, 4: 25.0, 5: 5}
nxt_fase = time.time() + dur_fase[fase]

v_n, w_n = 0.8, 0

while True:
    t = time.time()
    dt = 1.0/23.0  # 0.023 s por iteración

    tags_detect = []

    image = HAL.getImage()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)

    for r in results:

        tag_id = r.tag_id
        bb_april_tags(r, image)

        pos_data = tags.get(str('tag_' + str(tag_id)), {}).get('position', None)

        if pos_data is None:
            continue
        
        # Donde está el tag respecto a la cámara
        RT_tag_cam = RT_Cam_AprilTag(r, object_pts, matrix_camera, 
            dist_coeffs, image)

        # Comporbar que RT es correcta
        project_points(image, object_pts, RT_tag_cam, matrix_camera, dist_coeffs)
        
        # Donde está tag respecto mundo
        RT_tag_world = RT_World_Tag(pos_data)

        # Coger solo el más cercano
        distance_tag_cam = np.linalg.norm(RT_tag_cam[:3, 3])
        tags_detect.append((distance_tag_cam, RT_tag_world, RT_tag_cam, tag_id))
        
    if tags_detect: 
        tags_detect.sort(key=lambda x: x[0])
        # Info tag more close
        _, RT_tag_world, RT_tag_cam, _ = tags_detect[0]

        # Óptica respecto mundo
        RT_optical_world = matriz_RT_Optical_world()

        # Cámara según tag
        RT_cam_tag = np.linalg.inv(RT_tag_cam)

        RT_final = RT_tag_world @ RT_optical_world @ RT_cam_tag @ np.linalg.inv(RT_optical_world) @ RT_robot_cam 

        x_robot, y_robot, _ = RT_final[:3, 3]
        yaw_robot = atan2(RT_final[1,0], RT_final[0,0])
        pose_est = np.array([ x_robot, y_robot,yaw_robot], dtype=np.float32)
    
        #print(x_robot, y_robot)
        #print('Real:', HAL.getOdom().x, HAL.getOdom().y)

    else: # No se detecttan tags
        x_last, y_last, yaw_last = pose_est # Pose antigua
        dx = v_n * dt * np.cos(yaw_last) / 10
        dy = v_n * dt * np.sin(yaw_last) / 10
        d_yaw = w_n * dt * (np.pi/180)

        pose_est = np.array([x_last + dx, y_last + dy, yaw_last + d_yaw],
                            dtype=np.float32)
        


    GUI.showImage(image)      
    GUI.showEstimatedPose((pose_est[0], pose_est[1], pose_est[2]))
    HAL.setV(v_n)
    HAL.setW(w_n)
    #HAL.setW(-0.08)
    # Dinámica bucle de movimiento
    if t >= nxt_fase:
        if fase == 1:
            v_n = 0.4
            w_n = 0.7
            fase = 2
        elif fase == 2:
            v_n = 0.9
            w_n = 0.0
            fase = 3
        elif fase == 3:
            v_n = 0.3
            w_n = 0.8
            fase = 4
        elif fase == 4:
            v_n = 0.9
            w_n = 0
            fase = 5
        elif fase == 5:
            v_n = 0.4
            w_n = 0.6
            fase = 1

        nxt_fase = t + dur_fase[fase]
