import GUI
import HAL
import pyapriltags
import cv2 as cv2
import numpy as np
import yaml
from pathlib import Path

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

    image_pts = np.array([ptA, ptB, ptC, ptD], dtype=np.float32)  # corners detección

    # Calcular rotación (rvec) y traslación (tvec)
    _, rvec, tvec = cv2.solvePnP(object_pts, image_pts, matrix_camera, dist_coeffs)

    R, _ = cv2.Rodrigues(rvec)  # Convierte vector de rotación a matriz 3x3
    # Matriz de rotación R respecto cámara
    # rojo=X, verde=Y, azul=Z
    cv2.drawFrameAxes(image, matrix_camera, dist_coeffs, rvec, tvec, 0.3, 5)

    RT = np.concatenate((R, tvec), axis=1)  # shape (3,4)

    # añade la fila [0 0 0 1] para homogeneizar
    bottom = np.array([[0, 0, 0, 1]], dtype=np.float32)

    return np.vstack((RT, bottom))

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


def RT_World_Optical():
    # Rotación -90° alrededor del eje Z
    Rz = np.array([
        [0, 1, 0],
        [-1, 0, 0],
        [0, 0, 1]
    ], dtype=np.float32)    
    # Rotación -90° alrededor del eje X
    Rx = np.array([
        [1, 0, 0],
        [0, 0, 1],
        [0, -1, 0]
    ], dtype=np.float32)    

    R_optical = Rx @ Rz # Primero Rz, luego Rx
    RT_optical = np.eye(4, dtype=np.float32)
    RT_optical[:3, :3] = R_optical  

    return RT_optical

traslation = [0.069, -0.047, 0.107] # metros, archivo sdf
rotation = np.array([0, 0, 0], dtype=np.float32)
rotation_matrix, _ = cv2.Rodrigues(rotation) 
RT_cam_robot = np.eye(4, dtype=np.float32)
RT_cam_robot[:3, :3] = rotation_matrix
RT_cam_robot[:3, 3] = traslation

detector = pyapriltags.Detector(searchpath=["apriltags"], families="tag36h11")

image = HAL.getImage()

size = image.shape
focal_length = size[1]
center = (size[1] / 2, size[0] / 2)
matrix_camera = np.array([[focal_length, 0, center[0]], [0, focal_length, center[1]], [0, 0, 1]], dtype="double",)
dist_coeffs = np.zeros((4,1))  # Asume sin distorsión


conf = yaml.safe_load(
    Path("/resources/exercises/marker_visual_loc/apriltags_poses.yaml").read_text()
)
tags = conf["tags"]


tag_size = 0.3
half = tag_size / 2

object_pts = np.array([
    [-half,  half, 0],  # ptA: inf-izq
    [ half,  half, 0],  # ptB: inf-der
    [ half, -half, 0],  # ptC: sup-der
    [-half, -half, 0],  # ptD: sup-izq
], dtype=np.float32)



while True:
    image = HAL.getImage()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)

    for r in results:
        HAL.setV(3)
        tag_id = r.tag_id
        bb_april_tags(r, image)
        pos_data = tags.get(str('tag_' + str(tag_id)), {}).get('position', None)

        if pos_data is None:
            continue
        
        # Mundo - óptica
        RT_world_optical = RT_World_Optical()
        # Óptica - mundo
        RT_optical_world = np.linalg.inv(RT_world_optical)

        # Mundo - Tag (mundo)
        RT_world_tag = RT_World_Tag(pos_data)
        # Mundo - Tag (óptica)
        RT_tag_optica = RT_optical_world  @ RT_world_tag
        
        # Cámara (óptica) - Tag (óptica)
        RT_cam_tag = RT_Cam_AprilTag(r, object_pts, matrix_camera, 
            dist_coeffs, image)
        
        # Tag (óptica) - cámara (óptica)
        RT_tag_cam = np.linalg.inv(RT_cam_tag) # Tag cámara

        # Cámara - Mundo (mundo)
        RT_cam_world = RT_optical_world @ RT_tag_cam

        # Mundo - robot
        RT_robot_world = RT_cam_robot @ RT_cam_world

        x_robot, y_robot, z_robot = RT_robot_world[:3, 3]
        print(str('tag_' + str(tag_id)))
        print(x_robot, y_robot, z_robot)
        print('Real:', HAL.getOdom().x, HAL.getOdom().y)
        GUI.showEstimatedPose((x_robot, y_robot, 0))
        GUI.showImage(image)
    # Enter iterative code!
