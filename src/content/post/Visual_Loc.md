---
title: "Autolocalización Visual Basada en Marcadores"
description: "En esta práctica se pretende estimar la posición y orientación de un robot en un entorno 2D mediante la detección de marcadores visuales AprilTags, aplicando técnicas de visión por computadora y transformaciones geométricas."
publishDate: "18 MAY 2025"
tags: [
  "robótica",
  "visión por computadora",
  "localización visual",
  "AprilTags",
  "estimación de pose"
]
coverImage:
  src: "./images_post/AprilTags/portada.png"
  alt: "Reconstrucción 3D mediante marcadores visuales"
  width: 1200  # Sin unidades (se asume px)
  height: 600
draft: false
---

## Sistema de Posicionamiento 🧭
El uso de marcadores fiduciales como AprilTags permite obtener una localización precisa y robusta mediante técnicas de visión artificial. Esta estrategia se basa en la detección de marcadores visuales cuya geometría y posición en el mundo son conocidas de antemano. A partir de su aparición en la imagen captada por la cámara del robot, es posible estimar la pose relativa del marcador respecto a la cámara utilizando algoritmos como solvePnP.

Gracias a esta estimación y a la cadena de transformaciones geométricas, es posible inferir directamente la posición del robot en el entorno global. De esta forma, se logra un sistema de autolocalización visual sin necesidad de construir o cargar un mapa del entorno, , lo que permite una puesta en marcha más directa y flexible en entornos controlados.

La Figura 1, tomada del trabajo de Zhang et al. (2023) sobre localización visual en robótica agrícola, resume el esquema geométrico general para la estimación de la pose de un vehículo móvil. Este planteamiento teórico representa el fundamento del reto abordado en esta práctica.

![Info April Tags](./images_post/AprilTags/info_apriltags_redimensionada.png)
**_Figura 1_**: Descripción del sistema de coordenadas para la localización del robot.\
_[Zhang, Wei & Gong, Liang & Sun, Yefeng & Gao, Bishu & Yu, Chenrui & Liu, Chengliang. (2023). Precise visual positioning of agricultural mobile robots with a fiducial marker reprojection approach. Measurement Science and Technology. 34. 10.1088/1361-6501/ace8b0. ]_

Por tanto, a priori, la estimación de la pose del robot se basa en una cadena de transformaciones que relaciona los distintos sistemas de referencia involucrados.

```math
RT_mundo_robot = RT_mundo_tag·RT_tag_camara·RT_camara_robot
```

## Detección de marcadores AprilTags 🎯

El primer desafío a abordar es la detección de los marcadores AprilTags. Para ello, se emplea el detector _pyapriltags_, capaz de identificar en tiempo real las esquinas de cada marcador y extraer su identificador. Gracias a esta información, es posible acceder a la pose absoluta del marcador en el sistema de coordenadas global obteniendo su posición y orientación en el entorno. 

Para facilitar la depuración visual y comprobar que la detección de marcadores se realiza correctamente, se han añadido varios colores a los bordes de los marcadores detectados.

![Colours April Tags](./images_post/AprilTags/tags_colours.png)

Una vez detectado un marcador válido, y conociendo su geometría real en el mundo, se procede a la obtención de la primera transformación, la pose del marcador respecto a la cámara. Para ello, se necesitan las coordenadas 3D reales de las esquinas del marcador, las coordenadas 2D de esas esquinas obtenidas por el detector y La matriz de parámetros intrínsecos de la cámara. 

Dicha matriz se ha construido a partir de los valores proporcionados por el simulador. 

**Comando: Matriz de intrínsecos.**
```python
    ros2 topic echo /turtlebot3/camera/camera_info
``` 
Con estos parámetros y asumiendo una lente sin distorsión, se resuelve el problema PnP mediante _cv2.solvePnP_, obteniendo los vectores de rotación y traslación que permiten construir la matriz **RT_tag_cam**. 
​

## Vídeo 🎥
1. [Autolocalización visual basada en marcadores apriltags completa.](https://youtu.be/UpFAeQSnzSg)